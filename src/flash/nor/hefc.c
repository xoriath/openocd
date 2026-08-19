// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   HEFC (High-End Flash Controller) NOR flash driver                     *
 *   Copyright (C) 2024 Microchip Technology Inc.                          *
 *                                                                          *
 *   Supports the HEFC flash controller used by:                           *
 *     SAMRH707  (rad-hardened Cortex-M7, single-bank 128 KiB flash)      *
 *     SAMRH71   (rad-hardened Cortex-M7, single-bank 128 KiB flash)      *
 *                                                                          *
 *   Both parts have identical flash geometry; no per-variant table is     *
 *   needed.  There is no silicon-readable DID/CHIPID register for         *
 *   auto-detection, so the probe step simply populates fixed geometry.    *
 *                                                                          *
 *   Register addresses and command codes are derived from the SAMRH707    *
 *   flash-loader source (FlashPrg.c).                                     *
 *                                                                          *
 *   Remaining out-of-scope items:                                         *
 *     - Watchdog/clock Init() steps from FlashPrg.c are intentionally     *
 *       omitted (see hefc_probe comment).                                 *
 *     - SWD-DP IDCODE (DPIDR): only an internal CHIPID register           *
 *       (0x40100000) is available, not a static SWD-DP IDCODE value,      *
 *       so -expected-id remains unset in the TCL configs.                 *
 *     - protect/protect_check assume one SLB/CLB/GLB lock bit per page;   *
 *       this matches CLB/EP/WP already being issued per-page elsewhere    *
 *       in this driver but has not been hardware-validated.               *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>

#define HEFC_BASE                   0x40004000UL

/* HEFC register offsets from HEFC_BASE */
#define HEFC_FMR_OFF                0x00UL  /* Flash Mode Register           */
#define HEFC_FCR_OFF                0x04UL  /* Flash Command Register        */
#define HEFC_FSR_OFF                0x08UL  /* Flash Status Register         */
#define HEFC_FRR_OFF                0x0CUL  /* Flash Result Register         */
#define HEFC_WPMR_OFF               0xE4UL  /* Write Protection Mode Register*/

/* Absolute register addresses */
#define HEFC_FMR_ADDR               (HEFC_BASE + HEFC_FMR_OFF)
#define HEFC_FCR_ADDR               (HEFC_BASE + HEFC_FCR_OFF)
#define HEFC_FSR_ADDR               (HEFC_BASE + HEFC_FSR_OFF)
#define HEFC_WPMR_ADDR              (HEFC_BASE + HEFC_WPMR_OFF)
#define HEFC_FRR_ADDR               (HEFC_BASE + HEFC_FRR_OFF)

/* HEFC_FMR bits */
#define HEFC_FMR_ONE_MASK           0x00010000UL   /* bit 16: ONE wait state     */

/* -------------------------------------------------------------------------
 * HEFC_FCR encoding:
 *   bits[ 7:0]  = FCMD  (8-bit command opcode)
 *   bits[23:8]  = FARG  (16-bit page/sector argument)
 *   bits[31:24] = FKEY  (must be 0x5A to unlock the command)
 * ------------------------------------------------------------------------- */
#define HEFC_FKEY                   0x5AUL
#define HEFC_FCR_CMD(cmd, farg)     \
	((uint32_t)((cmd) | ((uint32_t)(farg) << 8) | (HEFC_FKEY << 24)))

/* HEFC_FCR command opcodes */
#define HEFC_FCMD_WP                0x01UL   /* Write Page                     */
#define HEFC_FCMD_WPL               0x02UL   /* Write Page and Lock            */
#define HEFC_FCMD_EA                0x05UL   /* Erase All                      */
#define HEFC_FCMD_EP                0x06UL   /* Erase Page                     */
#define HEFC_FCMD_EPA               0x07UL   /* Erase Pages (multiple)         */
#define HEFC_FCMD_SLB               0x08UL   /* Set Lock Bit                   */
#define HEFC_FCMD_CLB               0x09UL   /* Clear Lock Bit                 */
#define HEFC_FCMD_GLB               0x0AUL   /* Get Lock Bit                   */

/* HEFC_FSR bit masks */
#define HEFC_FSR_FRDY_MASK          0x00000001UL   /* bit 0:  Flash Ready        */
#define HEFC_FSR_FCMDE_MASK         0x00000002UL   /* bit 1:  Command Error      */
#define HEFC_FSR_FLOCKE_MASK        0x00000004UL   /* bit 2:  Lock Error         */
#define HEFC_FSR_WREER_MASK         0x00000010UL   /* bit 4:  Write Error        */
#define HEFC_FSR_ERRORS_MASK \
	(HEFC_FSR_FCMDE_MASK | HEFC_FSR_FLOCKE_MASK | HEFC_FSR_WREER_MASK)

/* HEFC_WPMR unlock key (ASCII "EFC\0") */
#define HEFC_WPMR_UNLOCK            0x45464300UL

/* =========================================================================
 * Flash geometry — identical for SAMRH707 and SAMRH71
 * ========================================================================= */
#define HEFC_FLASH_ADDR             0x10000000UL   /* Flash base address         */
#define HEFC_FLASH_SIZE             0x00020000UL   /* 128 KiB                    */
#define HEFC_PAGE_SIZE              256U            /* Bytes per page / sector    */
#define HEFC_NUM_PAGES              (HEFC_FLASH_SIZE / HEFC_PAGE_SIZE)  /* 512   */

/* Polling timeout for HEFC_FSR.FRDY (milliseconds) */
#define HEFC_READY_TIMEOUT_MS       1000

/* =========================================================================
 * Private driver state
 * ========================================================================= */
struct hefc_info {
	bool probed;
};

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/**
 * hefc_wait_ready - Poll HEFC_FSR.FRDY until set or timeout expires.
 *
 * Returns ERROR_OK when the flash signals ready, ERROR_FAIL on timeout,
 * or an error code if the register read itself fails.
 */
static int hefc_wait_ready(struct target *target)
{
	int64_t t0 = timeval_ms();
	uint32_t fsr;

	while (true) {
		keep_alive();

		int ret = target_read_u32(target, HEFC_FSR_ADDR, &fsr);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: failed to read FSR");
			return ret;
		}

		if (fsr & HEFC_FSR_FRDY_MASK)
			return ERROR_OK;

		if ((timeval_ms() - t0) > HEFC_READY_TIMEOUT_MS) {
			LOG_ERROR("HEFC: timeout waiting for flash ready "
				  "(FSR=0x%08" PRIx32 ")", fsr);
			return ERROR_FAIL;
		}
	}
}

/**
 * hefc_issue_cmd - Write a command to HEFC_FCR and wait for completion.
 *
 * @fcmd: FCMD opcode (HEFC_FCMD_*)
 * @page: FARG page number
 */
static int hefc_issue_cmd(struct target *target, uint32_t fcmd, uint32_t page)
{
	int ret = target_write_u32(target, HEFC_FCR_ADDR,
				   HEFC_FCR_CMD(fcmd, page));
	if (ret != ERROR_OK) {
		LOG_ERROR("HEFC: failed to write FCR "
			  "(cmd=0x%02" PRIx32 " page=%" PRIu32 ")", fcmd, page);
		return ret;
	}

	return hefc_wait_ready(target);
}

/* =========================================================================
 * Flash bank command handler
 * ========================================================================= */
FLASH_BANK_COMMAND_HANDLER(hefc_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct hefc_info *info = calloc(1, sizeof(*info));
	if (!info)
		return ERROR_FAIL;

	/* If the .cfg passed size == 0, fill in the compiled-in default */
	if (bank->size == 0)
		bank->size = HEFC_FLASH_SIZE;

	bank->driver_priv = info;
	return ERROR_OK;
}

/* =========================================================================
 * Probe
 *
 * No DID/CHIPID register exists on SAMRH707/SAMRH71; geometry is fixed.
 * The probe step populates the sector table with 256-byte sectors and
 * performs the HEFC_WPMR unlock + FMR wait-state setup from FlashPrg.c
 * Init().
 * ========================================================================= */
static int hefc_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct hefc_info *info = bank->driver_priv;

	/* If size was not set in the .cfg, use the compiled-in default */
	if (bank->size == 0)
		bank->size = HEFC_FLASH_SIZE;

	free(bank->sectors);

	/*
	 * Erase granularity: CONFIRMED 256 bytes (1 page) via icd4_cortex-m7.py
	 * / j-link_cortex-m7.py (SAMRH707_DFP scripts/), which compute the EP
	 * (Erase Page) argument as (address & 0x3fff00) -- a 256-byte-aligned
	 * page number -- and issue one EP command per 256-byte page, matching
	 * this driver's per-page approach.  The separate EPA (Erase Pages)
	 * opcode (0x07) is not used by the reference algorithm.
	 */
	bank->num_sectors = bank->size / HEFC_PAGE_SIZE;
	bank->sectors = alloc_block_array(0, HEFC_PAGE_SIZE, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = 0;

	/*
	 * Unlock HEFC write-protect register (ASCII "EFC\0") and configure
	 * one flash wait state (HEFC_FMR bit 16).  These are taken from
	 * FlashPrg.c Init() and are safe to apply on a debug-halted target.
	 *
	 * Watchdog disable / clock setup from reference Init() intentionally
	 * omitted — a debug-halted target reached via SWD/JTAG typically does
	 * not require replicating the embedded boot-time Init() sequence.
	 * If flash programming fails on a live board, consult FlashPrg.c
	 * Init() for the RSWDT_MR, WDT_MR, and PMC register setup that may
	 * be needed (RSWDT/WDT base addresses not independently confirmed,
	 * so they are not included here to avoid introducing unverified
	 * register addresses).
	 */
	int ret = target_write_u32(target, HEFC_WPMR_ADDR, HEFC_WPMR_UNLOCK);
	if (ret != ERROR_OK)
		LOG_WARNING("HEFC: could not write WPMR to unlock write-protect "
			    "(target may not be halted yet — proceeding)");

	ret = target_write_u32(target, HEFC_FMR_ADDR, HEFC_FMR_ONE_MASK);
	if (ret != ERROR_OK)
		LOG_WARNING("HEFC: could not write FMR wait-state configuration");

	info->probed = true;
	return ERROR_OK;
}

static int hefc_auto_probe(struct flash_bank *bank)
{
	struct hefc_info *info = bank->driver_priv;

	if (info->probed)
		return ERROR_OK;

	return hefc_probe(bank);
}

/* =========================================================================
 * Erase
 *
 * Replicates EraseSector() from FlashPrg.c:
 *   1. CLB  — clear lock bit for the page
 *   2. EP   — erase the page
 *   3. Check FSR error bits
 * ========================================================================= */
static int hefc_erase(struct flash_bank *bank, unsigned int first,
		      unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("HEFC: target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (unsigned int s = first; s <= last; s++) {
		uint32_t page = (uint32_t)s;  /* sector index == page number (256 B/page) */

		/* Step 1: Clear lock bit for this page */
		int ret = hefc_issue_cmd(target, HEFC_FCMD_CLB, page);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: CLB command failed for page %" PRIu32, page);
			return ret;
		}

		/* Step 2: Erase the page */
		ret = hefc_issue_cmd(target, HEFC_FCMD_EP, page);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: EP command failed for page %" PRIu32, page);
			return ret;
		}

		/* Step 3: Check for errors in FSR */
		uint32_t fsr;
		ret = target_read_u32(target, HEFC_FSR_ADDR, &fsr);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: failed to read FSR after erase of page "
				  "%" PRIu32, page);
			return ret;
		}
		if (fsr & HEFC_FSR_ERRORS_MASK) {
			LOG_ERROR("HEFC: erase error for page %" PRIu32
				  " (FSR=0x%08" PRIx32 ")", page, fsr);
			return ERROR_FAIL;
		}

		bank->sectors[s].is_erased = 1;
	}

	return ERROR_OK;
}

/* =========================================================================
 * Write
 *
 * Replicates ProgramPage() from FlashPrg.c:
 *   1. Write page data directly into flash-mapped address space via
 *      target_write_memory() (the reference uses direct 32-bit pointer
 *      writes; bus-write semantics are equivalent).
 *   2. Issue WP (Write Page) command to latch the page into NVM.
 *   3. Check FSR error bits.
 *
 * Partial pages are padded with 0xFF (the erased byte value) to fill
 * the 256-byte write unit.
 * ========================================================================= */
static int hefc_write(struct flash_bank *bank, const uint8_t *buffer,
		      uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("HEFC: target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t written = 0;

	while (written < count) {
		uint32_t cur_offset = offset + written;
		uint32_t page       = cur_offset / HEFC_PAGE_SIZE;
		uint32_t page_off   = cur_offset % HEFC_PAGE_SIZE;

		/* Base address of this page in the flash address space */
		uint32_t flash_page_addr = bank->base + (page * HEFC_PAGE_SIZE);

		/* Build a full 256-byte page buffer, padding with 0xFF */
		uint8_t page_buf[HEFC_PAGE_SIZE];
		memset(page_buf, 0xFF, sizeof(page_buf));

		uint32_t chunk = HEFC_PAGE_SIZE - page_off;
		if (chunk > count - written)
			chunk = count - written;

		memcpy(page_buf + page_off, buffer + written, chunk);

		/*
		 * Write page data into the flash-mapped address space.
		 * The reference loader writes 32-bit words directly via a pointer;
		 * target_write_memory() with element size 4 replicates that
		 * behaviour over the debug transport.
		 */
		int ret = target_write_memory(target, flash_page_addr,
					      4, HEFC_PAGE_SIZE / 4, page_buf);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: failed to write data to flash address "
				  "0x%08" PRIx32, flash_page_addr);
			return ret;
		}

		/* Issue WP command to latch the page into NVM */
		ret = hefc_issue_cmd(target, HEFC_FCMD_WP, page);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: WP command failed for page %" PRIu32, page);
			return ret;
		}

		/* Check for errors */
		uint32_t fsr;
		ret = target_read_u32(target, HEFC_FSR_ADDR, &fsr);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: failed to read FSR after write of page "
				  "%" PRIu32, page);
			return ret;
		}
		if (fsr & HEFC_FSR_ERRORS_MASK) {
			LOG_ERROR("HEFC: write error for page %" PRIu32
				  " (FSR=0x%08" PRIx32 ")", page, fsr);
			return ERROR_FAIL;
		}

		written += chunk;
	}

	return ERROR_OK;
}

/* =========================================================================
 * Protect / Protect check
 *
 * SLB (Set Lock Bit) / GLB (Get Lock Bit) address one lock bit per page,
 * consistent with CLB/EP/WP already being issued per-page elsewhere in
 * this driver.  GLB latches one 32-bit word of lock-bit status per FRR
 * read, MSB first page in each word being bit 0; enough words are read
 * to cover every page.
 * ========================================================================= */
static int hefc_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("HEFC: target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (unsigned int s = first; s <= last; s++) {
		int ret = hefc_issue_cmd(target, set ? HEFC_FCMD_SLB : HEFC_FCMD_CLB, s);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: %s command failed for page %u",
					set ? "SLB" : "CLB", s);
			return ret;
		}

		uint32_t fsr;
		ret = target_read_u32(target, HEFC_FSR_ADDR, &fsr);
		if (ret != ERROR_OK)
			return ret;
		if (fsr & HEFC_FSR_ERRORS_MASK) {
			LOG_ERROR("HEFC: %s error for page %u (FSR=0x%08" PRIx32 ")",
					set ? "SLB" : "CLB", s, fsr);
			return ERROR_FAIL;
		}

		bank->sectors[s].is_protected = set ? 1 : 0;
	}

	return ERROR_OK;
}

static int hefc_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int ret = hefc_issue_cmd(target, HEFC_FCMD_GLB, 0);
	if (ret != ERROR_OK) {
		LOG_ERROR("HEFC: GLB command failed");
		return ret;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i += 32) {
		uint32_t word;
		ret = target_read_u32(target, HEFC_FRR_ADDR, &word);
		if (ret != ERROR_OK) {
			LOG_ERROR("HEFC: failed to read FRR during GLB readback");
			return ret;
		}
		for (unsigned int b = 0; b < 32 && (i + b) < bank->num_sectors; b++)
			bank->sectors[i + b].is_protected = (word >> b) & 1;
	}

	return ERROR_OK;
}

/* =========================================================================
 * Driver registration
 * ========================================================================= */
const struct flash_driver hefc_flash = {
	.name               = "hefc",
	.flash_bank_command = hefc_flash_bank_command,
	.erase              = hefc_erase,
	.write              = hefc_write,
	.read               = default_flash_read,
	.probe              = hefc_probe,
	.auto_probe         = hefc_auto_probe,
	.erase_check        = default_flash_blank_check,
	.protect            = hefc_protect,
	.protect_check      = hefc_protect_check,
	.free_driver_priv   = default_flash_free_driver_priv,
};
