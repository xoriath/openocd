// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   PIC32C FCW (Flash Control Word) NOR flash driver                      *
 *   Copyright (C) 2024 Microchip Technology Inc.                          *
 *   Author: Morten Engelhardt Olsen <MortenEngelhardt.Olsen@microchip.com> *
 *                                                                          *
 *   Supports the FCW flash controller used by:                            *
 *     PIC32CZ-CA80, PIC32CZ-CA90, PIC32CZ-CA91  (Group A, CZ-CA family)  *
 *     PIC32CK-SG, PIC32CK-GC                     (Group A, CK family)     *
 *     PIC32CM-GC00, PIC32CM-SG00                  (Group B, CM family)     *
 *                                                                          *
 *   Two register-layout groups exist:                                      *
 *     Group A: CTRLA at FCW+0x00 is the command register; STATUS at +0x18 *
 *     Group B: CTRLOP at FCW+0x04 is the command register; STATUS at +0x1C*
 *                                                                          *
 *   The device variant must be supplied as an extra argument to the        *
 *   'flash bank' command because no silicon-readable DID/CHIPID register   *
 *   exists in this family for auto-detection.                              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "helper/binarybuffer.h"

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include <target/cortex_m.h>
#include <inttypes.h>

/* =========================================================================
 * DSU (Device Service Unit) base and mailbox register offsets
 * Base address 0x44000000 is shared by all PIC32C FCW variants.
 * ========================================================================= */
#define PIC32CFCW_DSU_BASE              0x44000000UL

/* BCC/DAL register offsets from DSU_BASE (same for all variants) */
#define PIC32CFCW_DSU_BCC0_OFF          0x110UL  /* Boot Comm. Channel 0 (debugger cmd)  */
#define PIC32CFCW_DSU_BCC1_OFF          0x114UL  /* Boot Comm. Channel 1 (boot ROM reply) */
#define PIC32CFCW_DSU_DAL_OFF           0x124UL  /* Debug Access Level register           */

/* --- Group A: CZ-CA80/CA90/CA91, CK-SG, CK-GC --- */
#define PIC32CFCW_GRPA_DSU_STATUSA_OFF  0x100UL
#define PIC32CFCW_GRPA_DSU_STATUSB_OFF  0x104UL
#define PIC32CFCW_GRPA_CRSTEXT_MASK     0x00000300UL  /* bits [9:8]: Reset Extension active */
#define PIC32CFCW_GRPA_BCCD1_MASK       0x00000002UL  /* bit 1: BCC1 data ready             */

/* --- Group B: CM-GC00, CM-SG00 --- */
#define PIC32CFCW_GRPB_DSU_STATUSA_OFF  0x104UL
#define PIC32CFCW_GRPB_DSU_STATUSB_OFF  0x108UL
#define PIC32CFCW_GRPB_CRSTEXT_MASK     0x00000100UL  /* bit 8: Reset Extension active      */
#define PIC32CFCW_GRPB_BCCD1_MASK       0x00000002UL  /* bit 1: BCC1 data ready             */

/* =========================================================================
 * FCW peripheral register offsets from FCW peripheral base address
 * ========================================================================= */

/* --- Group A FCW (CZ-CA* base=0x44002000; CK-SG/CK-GC base=0x44004000) --- */
#define PIC32CFCW_GRPA_FCW_CTRLA        0x00UL  /* Command register (CTRLA)             */
#define PIC32CFCW_GRPA_FCW_CTRLB        0x04UL
#define PIC32CFCW_GRPA_FCW_INTFLAG      0x14UL
#define PIC32CFCW_GRPA_FCW_INTFLAG_CLR  0x000030FFUL  /* W1C mask for INTFLAG             */
#define PIC32CFCW_GRPA_FCW_STATUS       0x18UL  /* STATUS register; bit 0 = BUSY        */
#define PIC32CFCW_GRPA_FCW_KEY          0x1CUL
#define PIC32CFCW_GRPA_FCW_ADDR         0x20UL
#define PIC32CFCW_GRPA_FCW_SRCADDR      0x24UL

/* --- Group B FCW (CM-GC00, CM-SG00; base=0x44004000) --- */
/* Note: no register at +0x00 in Group B; command register is CTRLOP at +0x04 */
#define PIC32CFCW_GRPB_FCW_CTRLOP       0x04UL  /* Command register (CTRLOP)            */
#define PIC32CFCW_GRPB_FCW_INTFLAG      0x14UL
#define PIC32CFCW_GRPB_FCW_INTFLAG_CLR  0x000031FFUL  /* W1C mask for INTFLAG             */
#define PIC32CFCW_GRPB_FCW_STATUS       0x1CUL  /* STATUS register; bit 0 = BUSY        */
#define PIC32CFCW_GRPB_FCW_KEY          0x20UL
#define PIC32CFCW_GRPB_FCW_ADDR         0x24UL
#define PIC32CFCW_GRPB_FCW_SRCADDR      0x28UL

/* =========================================================================
 * FCW key and command constants
 * ========================================================================= */
#define PIC32CFCW_KEY_MSB               0x91C32C00UL
#define PIC32CFCW_KEY_WRKEY             0x1UL
#define PIC32CFCW_KEY_CFGKEY            0x4UL
/* Write/erase unlock word = KEY_MSB | KEY_WRKEY = 0x91C32C01 */
#define PIC32CFCW_WRKEY                 (PIC32CFCW_KEY_MSB | PIC32CFCW_KEY_WRKEY)

#define PIC32CFCW_CMD_WRITE_ROW         0x03UL  /* Program one 1 kB row   */
#define PIC32CFCW_CMD_ERASE_PAGE        0x04UL  /* Erase one 1 kB page    */

#define PIC32CFCW_STATUS_BUSY           0x1UL   /* FCW STATUS register bit 0 */

/* =========================================================================
 * Boot ROM / DSU mailbox constants
 * ========================================================================= */
#define PIC32CFCW_DEBUGGER_CMD_EXIT     ((uint32_t)0x444247AAU)  /* Exit reset-extension handshake */
#define PIC32CFCW_BOOTROM_STATUS_BOOTOK ((uint32_t)0x4U)        /* Successful boot reply in BCC1  */

/* =========================================================================
 * Flash geometry
 * ========================================================================= */
#define PIC32CFCW_ROW_SIZE              1024U   /* program/erase granularity (bytes) */

/* =========================================================================
 * Variant descriptor table
 *
 * group_b: true  => Group B register layout (CM-GC00, CM-SG00)
 *          false => Group A register layout (CZ-CA*, CK-SG, CK-GC)
 * fcw_base: FCW peripheral base address for the variant sub-family.
 * cmd_ce_all: chip-erase command opcode -- informational only; chip-erase
 *   is NOT implemented in this driver (requires BootROM HMAC Interactive
 *   Mode unlock, which is out of scope for this initial patch).
 * ========================================================================= */
struct pic32cfcw_variant {
	const char *name;
	bool group_b;
	uint32_t fcw_base;
	uint8_t cmd_ce_all;
	uint32_t ram_scratch; /* fallback SRAM scratch address if no work area is configured */
};

static const struct pic32cfcw_variant pic32cfcw_variants[] = {
	{ "CZCA80",  false, 0x44002000UL, 0xE3, 0x20020000UL },
	{ "CZCA90",  false, 0x44002000UL, 0xE3, 0x20020000UL },
	{ "CZCA91",  false, 0x44002000UL, 0xE3, 0x20020000UL },
	{ "CKSG",    false, 0x44004000UL, 0xE2, 0x20010000UL },
	{ "CKGC",    false, 0x44004000UL, 0xE2, 0x20010000UL },
	{ "CMGC00",  true,  0x44004000UL, 0xE2, 0x20000000UL },
	{ "CMSG00",  true,  0x44004000UL, 0xE2, 0x20000000UL },
};

#define PIC32CFCW_NUM_VARIANTS ARRAY_SIZE(pic32cfcw_variants)

/* =========================================================================
 * Driver-private per-bank data
 *
 * Resolved once in flash_bank_command, cached here so erase/write/probe
 * and the reset-deassert command do not need to re-derive them.
 * ========================================================================= */
struct pic32cfcw_info {
	const struct pic32cfcw_variant *variant;
	bool probed;

	/* FCW peripheral: absolute addresses of key registers */
	uint32_t fcw_cmd;             /* command register: CTRLA (Grp A) or CTRLOP (Grp B) */
	uint32_t fcw_status;          /* STATUS register                                     */
	uint32_t fcw_key;             /* KEY register (write-unlock before each command)     */
	uint32_t fcw_addr;            /* ADDR register (destination flash address)           */
	uint32_t fcw_srcaddr;         /* SRCADDR register (source SRAM address for writes)   */
	uint32_t fcw_intflag;         /* INTFLAG register                                    */
	uint32_t fcw_intflag_clrmask; /* W1C mask for INTFLAG                                */

	/* DSU: absolute addresses of variant-specific STATUS registers */
	uint32_t dsu_statusa;         /* DSU STATUSA register address                        */
	uint32_t dsu_crstext_mask;    /* W1C mask for CRSTEXT bit(s) in STATUSA              */
	uint32_t dsu_statusb;         /* DSU STATUSB register address                        */
	uint32_t dsu_bccd1_mask;      /* mask for BCCD1 bit in STATUSB                       */
};

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/** Resolve all register addresses from the variant descriptor and cache them. */
static void pic32cfcw_fill_addrs(struct pic32cfcw_info *info,
		const struct pic32cfcw_variant *variant)
{
	uint32_t fcw = variant->fcw_base;
	uint32_t dsu = PIC32CFCW_DSU_BASE;

	info->variant = variant;

	if (!variant->group_b) {
		/* Group A: CZ-CA*, CK-SG, CK-GC */
		info->fcw_cmd             = fcw + PIC32CFCW_GRPA_FCW_CTRLA;
		info->fcw_status          = fcw + PIC32CFCW_GRPA_FCW_STATUS;
		info->fcw_key             = fcw + PIC32CFCW_GRPA_FCW_KEY;
		info->fcw_addr            = fcw + PIC32CFCW_GRPA_FCW_ADDR;
		info->fcw_srcaddr         = fcw + PIC32CFCW_GRPA_FCW_SRCADDR;
		info->fcw_intflag         = fcw + PIC32CFCW_GRPA_FCW_INTFLAG;
		info->fcw_intflag_clrmask = PIC32CFCW_GRPA_FCW_INTFLAG_CLR;
		info->dsu_statusa         = dsu + PIC32CFCW_GRPA_DSU_STATUSA_OFF;
		info->dsu_crstext_mask    = PIC32CFCW_GRPA_CRSTEXT_MASK;
		info->dsu_statusb         = dsu + PIC32CFCW_GRPA_DSU_STATUSB_OFF;
		info->dsu_bccd1_mask      = PIC32CFCW_GRPA_BCCD1_MASK;
	} else {
		/* Group B: CM-GC00, CM-SG00 */
		info->fcw_cmd             = fcw + PIC32CFCW_GRPB_FCW_CTRLOP;
		info->fcw_status          = fcw + PIC32CFCW_GRPB_FCW_STATUS;
		info->fcw_key             = fcw + PIC32CFCW_GRPB_FCW_KEY;
		info->fcw_addr            = fcw + PIC32CFCW_GRPB_FCW_ADDR;
		info->fcw_srcaddr         = fcw + PIC32CFCW_GRPB_FCW_SRCADDR;
		info->fcw_intflag         = fcw + PIC32CFCW_GRPB_FCW_INTFLAG;
		info->fcw_intflag_clrmask = PIC32CFCW_GRPB_FCW_INTFLAG_CLR;
		info->dsu_statusa         = dsu + PIC32CFCW_GRPB_DSU_STATUSA_OFF;
		info->dsu_crstext_mask    = PIC32CFCW_GRPB_CRSTEXT_MASK;
		info->dsu_statusb         = dsu + PIC32CFCW_GRPB_DSU_STATUSB_OFF;
		info->dsu_bccd1_mask      = PIC32CFCW_GRPB_BCCD1_MASK;
	}
}

/** Return the variant entry matching @a name, or NULL if not found. */
static const struct pic32cfcw_variant *pic32cfcw_find_variant(const char *name)
{
	for (size_t i = 0; i < PIC32CFCW_NUM_VARIANTS; i++) {
		if (strcmp(pic32cfcw_variants[i].name, name) == 0)
			return &pic32cfcw_variants[i];
	}
	return NULL;
}

/**
 * Poll FCW STATUS bit 0 (BUSY) until clear or @a timeout_ms milliseconds.
 * Returns ERROR_OK when BUSY clears, ERROR_FLASH_OPERATION_FAILED on timeout.
 */
static int pic32cfcw_poll_busy(struct target *target, uint32_t status_addr,
		int timeout_ms)
{
	int64_t ts_start = timeval_ms();
	uint32_t status = PIC32CFCW_STATUS_BUSY;

	do {
		int retval = target_read_u32(target, status_addr, &status);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to read FCW STATUS at 0x%08" PRIx32,
					status_addr);
			return retval;
		}
		if (!(status & PIC32CFCW_STATUS_BUSY))
			return ERROR_OK;
		keep_alive();
	} while (timeval_ms() - ts_start < timeout_ms);

	LOG_ERROR("pic32cfcw: FCW operation timed out "
			"(STATUS=0x%08" PRIx32 " at 0x%08" PRIx32 ")",
			status, status_addr);
	/* TODO: decode individual FCW error bits in STATUS when their exact
	 * bit-field assignments are confirmed in the final datasheet release. */
	return ERROR_FLASH_OPERATION_FAILED;
}

/* =========================================================================
 * Reset-extension / DAL handshake
 *
 * These functions mirror samdl1x_reset_extension / samdl1x_exit_reset_extension
 * in at91samd.c (Phase A) but are parameterised for PIC32C register layouts.
 * ========================================================================= */

/**
 * Assert and deassert SRST to trigger the CPU reset-extension latch.
 *
 * NOTE: This is an approximation.  The strict sequences.xml "ResetExtension"
 * requires exactly 4 SWCLK pulses while nRESET is held low, which cannot be
 * reproduced via the current jtag_add_reset() API.  In practice the SWD
 * reconnect traffic emitted by most J-Link and CMSIS-DAP adapters provides
 * sufficient SWCLK edges for the reset-extension latch to arm.  A follow-up
 * change should add explicit adapter-level SWCLK pulse support for strict
 * sequences.xml compliance (same caveat as the SAML10/L11 implementation).
 */
static int pic32cfcw_reset_extension(struct target *target)
{
	int retval;

	LOG_DEBUG("pic32cfcw: asserting SRST for reset-extension handshake");

	jtag_add_reset(0, 1);  /* assert SRST */
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		LOG_DEBUG("pic32cfcw: failed to assert SRST (%d) -- continuing", retval);

	alive_sleep(5);

	jtag_add_reset(0, 0);  /* deassert SRST */
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		LOG_DEBUG("pic32cfcw: failed to deassert SRST (%d) -- continuing", retval);

	alive_sleep(10);

	return ERROR_OK;
}

/**
 * Exit the reset-extension state via the DSU BCC mailbox handshake.
 * Implements sequences.xml "ExitResetExtension" for the PIC32C FCW family.
 *
 * Unlike SAM L10/L11, PIC32C does NOT require a BCC1!=0 user-page
 * validation pre-check before sending the EXIT command; that check is
 * SAM-L-specific and is therefore omitted here.
 */
static int pic32cfcw_exit_reset_extension(struct target *target,
		const struct pic32cfcw_info *info)
{
	int retval;
	uint32_t statusa;
	uint32_t statusb;
	uint32_t bcc1;
	int64_t ts_start;

	/* 1. Read DSU STATUSA */
	retval = target_read_u32(target, info->dsu_statusa, &statusa);
	if (retval != ERROR_OK) {
		LOG_ERROR("pic32cfcw: failed to read DSU STATUSA at 0x%08" PRIx32,
				info->dsu_statusa);
		return retval;
	}

	/* 2. Verify CRSTEXT is set -- if not, the reset-extension did not latch */
	if (!(statusa & info->dsu_crstext_mask)) {
		LOG_ERROR("pic32cfcw: could not enter reset extension "
				"(CRSTEXT not set; STATUSA=0x%08" PRIx32 ")", statusa);
		return ERROR_FAIL;
	}

	/* 3. W1C clear CRSTEXT to allow boot ROM to continue execution */
	retval = target_write_u32(target, info->dsu_statusa, info->dsu_crstext_mask);
	if (retval != ERROR_OK) {
		LOG_ERROR("pic32cfcw: failed to W1C-clear CRSTEXT in DSU STATUSA");
		return retval;
	}

	/* 4. Allow boot ROM to advance (5 ms) */
	alive_sleep(5);

	/* 5. Send EXIT command via DSU BCC0 */
	retval = target_write_u32(target,
			PIC32CFCW_DSU_BASE + PIC32CFCW_DSU_BCC0_OFF,
			PIC32CFCW_DEBUGGER_CMD_EXIT);
	if (retval != ERROR_OK) {
		LOG_ERROR("pic32cfcw: failed to write EXIT command to DSU BCC0");
		return retval;
	}

	/* 6. Poll DSU STATUSB BCCD1 bit until set (boot ROM placed reply in BCC1).
	 * Timeout: 500 ms. */
	ts_start = timeval_ms();
	do {
		retval = target_read_u32(target, info->dsu_statusb, &statusb);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to read DSU STATUSB at 0x%08" PRIx32,
					info->dsu_statusb);
			return retval;
		}
		if (statusb & info->dsu_bccd1_mask)
			break;
		keep_alive();
	} while (timeval_ms() - ts_start < 500);

	if (!(statusb & info->dsu_bccd1_mask)) {
		LOG_ERROR("pic32cfcw: timeout waiting for BCC1 valid "
				"(STATUSB=0x%08" PRIx32 ")", statusb);
		return ERROR_FAIL;
	}

	/* 7. Read BCC1 and verify boot ROM reports successful boot */
	retval = target_read_u32(target,
			PIC32CFCW_DSU_BASE + PIC32CFCW_DSU_BCC1_OFF, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("pic32cfcw: failed to read DSU BCC1 reply");
		return retval;
	}

	if (bcc1 != PIC32CFCW_BOOTROM_STATUS_BOOTOK) {
		LOG_ERROR("pic32cfcw: boot validation failed "
				"(BCC1=0x%08" PRIx32 ", expected 0x%08" PRIx32 ")",
				bcc1, PIC32CFCW_BOOTROM_STATUS_BOOTOK);
		return ERROR_FAIL;
	}

	/* 8. Halt the core via ARMv7-M/v8-M DHCSR */
	retval = target_write_u32(target, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN);
	if (retval != ERROR_OK) {
		LOG_ERROR("pic32cfcw: failed to write DHCSR to halt core");
		return retval;
	}

	LOG_DEBUG("pic32cfcw: reset-extension exit complete for variant %s",
			info->variant->name);
	return ERROR_OK;
}

/**
 * Enter reset-extension then exit via BCC mailbox handshake ("park" state).
 * Combines pic32cfcw_reset_extension() and pic32cfcw_exit_reset_extension().
 */
static int pic32cfcw_reset_to_park(struct target *target,
		const struct pic32cfcw_info *info)
{
	int retval = pic32cfcw_reset_extension(target);
	if (retval != ERROR_OK)
		return retval;
	return pic32cfcw_exit_reset_extension(target, info);
}

/* =========================================================================
 * Flash driver operations
 * ========================================================================= */

static int pic32cfcw_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct pic32cfcw_info *info = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (unsigned int i = first; i <= last; i++) {
		uint32_t addr = (uint32_t)bank->base + (uint32_t)i * PIC32CFCW_ROW_SIZE;

		/* Unlock FCW for this operation (KEY auto-clears after command) */
		int retval = target_write_u32(target, info->fcw_key, PIC32CFCW_WRKEY);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to write FCW KEY for erase at "
					"0x%08" PRIx32, addr);
			return retval;
		}

		/* Set erase destination address */
		retval = target_write_u32(target, info->fcw_addr, addr);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to write FCW ADDR for erase");
			return retval;
		}

		/* Issue erase-page command */
		retval = target_write_u32(target, info->fcw_cmd, PIC32CFCW_CMD_ERASE_PAGE);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to write FCW CMD for erase");
			return retval;
		}

		/* Wait for FCW BUSY to clear (1 s timeout) */
		retval = pic32cfcw_poll_busy(target, info->fcw_status, 1000);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: erase failed for sector %u (addr 0x%08" PRIx32 ")",
					i, addr);
			return retval;
		}

		bank->sectors[i].is_erased = 1;
	}

	return ERROR_OK;
}

static int pic32cfcw_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct pic32cfcw_info *info = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *scratch = NULL;
	uint8_t row_buf[PIC32CFCW_ROW_SIZE];
	uint32_t bytes_written = 0;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* FCW uses a "direct RAM source" model: SRCADDR must point to SRAM.
	 * Allocate a ROW_SIZE scratch buffer in target working area.  If no
	 * work area is configured (or it is too small), fall back to a fixed
	 * per-variant SRAM address instead of failing outright; this address
	 * is never touched by OpenOCD's own workarea allocator, so as long as
	 * the target application does not use that RAM region concurrently,
	 * this is safe. */
	uint32_t scratch_addr;
	retval = target_alloc_working_area(target, PIC32CFCW_ROW_SIZE, &scratch);
	if (retval != ERROR_OK) {
		LOG_WARNING("pic32cfcw: no work area available (%d); falling back to "
				"fixed scratch address 0x%08" PRIx32, retval,
				info->variant->ram_scratch);
		scratch_addr = info->variant->ram_scratch;
	} else {
		scratch_addr = scratch->address;
	}

	while (bytes_written < count) {
		uint32_t chunk = count - bytes_written;
		if (chunk > PIC32CFCW_ROW_SIZE)
			chunk = PIC32CFCW_ROW_SIZE;

		uint32_t flash_addr = (uint32_t)bank->base + offset + bytes_written;

		/* Pad final partial row with 0xFF (blank flash value) */
		memset(row_buf, 0xFF, sizeof(row_buf));
		memcpy(row_buf, buffer + bytes_written, chunk);

		/* Copy row data into target SRAM working area */
		retval = target_write_buffer(target, scratch_addr,
				PIC32CFCW_ROW_SIZE, row_buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to write row data to SRAM working area");
			goto done;
		}

		/* SRCADDR → ADDR → KEY → CMD → poll BUSY */
		retval = target_write_u32(target, info->fcw_srcaddr, scratch_addr);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to write FCW SRCADDR");
			goto done;
		}

		retval = target_write_u32(target, info->fcw_addr, flash_addr);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to write FCW ADDR");
			goto done;
		}

		retval = target_write_u32(target, info->fcw_key, PIC32CFCW_WRKEY);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to write FCW KEY");
			goto done;
		}

		retval = target_write_u32(target, info->fcw_cmd, PIC32CFCW_CMD_WRITE_ROW);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: failed to write FCW CMD");
			goto done;
		}

		retval = pic32cfcw_poll_busy(target, info->fcw_status, 1000);
		if (retval != ERROR_OK) {
			LOG_ERROR("pic32cfcw: write failed at flash address 0x%08" PRIx32,
					flash_addr);
			goto done;
		}

		bytes_written += chunk;
	}

done:
	target_free_working_area(target, scratch);
	return retval;
}

static int pic32cfcw_probe(struct flash_bank *bank)
{
	struct pic32cfcw_info *info = bank->driver_priv;

	if (!info || !info->variant) {
		LOG_ERROR("pic32cfcw: driver private data not initialised "
				"(check flash bank command)");
		return ERROR_FAIL;
	}

	if (bank->size == 0) {
		LOG_ERROR("pic32cfcw: flash bank size is 0 -- specify "
				"<size> in 'flash bank' command");
		return ERROR_FAIL;
	}

	/* Build sector map: erase and program granularity is ROW_SIZE (1024 B).
	 * Round up if bank->size is not an exact multiple; all currently known
	 * variants have flash sizes that are multiples of ROW_SIZE, but this
	 * keeps a misconfigured .cfg size from silently truncating the map. */
	free(bank->sectors);
	bank->num_sectors = DIV_ROUND_UP(bank->size, PIC32CFCW_ROW_SIZE);
	bank->sectors = alloc_block_array(0, PIC32CFCW_ROW_SIZE, bank->num_sectors);
	if (!bank->sectors) {
		LOG_ERROR("pic32cfcw: failed to allocate sector table");
		return ERROR_FAIL;
	}

	/* CFM (Configuration Flash Memory) special regions are deferred to a
	 * follow-up change (same pattern as UROW/BOCOR deferral in at91samd.c
	 * Phase A).  Only the PFM (Program Flash Memory) bank is modelled here. */

	info->probed = true;

	LOG_INFO("pic32cfcw: probed variant %s -- PFM %u kB at 0x%08" PRIx32
			", %u sectors of %u B",
			info->variant->name,
			(unsigned)(bank->size / 1024U),
			(uint32_t)bank->base,
			(unsigned)bank->num_sectors,
			PIC32CFCW_ROW_SIZE);

	return ERROR_OK;
}

/* =========================================================================
 * flash_bank command handler
 *
 * Usage: flash bank <name> pic32cfcw <base> <size> 0 0 <target> <variant>
 *
 * <variant> selects the register-group (A or B) and FCW base.  It is
 * mandatory because this family has no silicon-readable DID register for
 * auto-detection.
 *
 * Supported variants: CZCA80, CZCA90, CZCA91, CKSG, CKGC, CMGC00, CMSG00
 * ========================================================================= */
FLASH_BANK_COMMAND_HANDLER(pic32cfcw_flash_bank_command)
{
	/* CMD_ARGV layout (all standard flash-bank args are present):
	 *   [0]=bank_name [1]=driver [2]=base [3]=size [4]=chip_w [5]=bus_w
	 *   [6]=target    [7]=variant (driver-specific, mandatory)            */
	if (CMD_ARGC < 8) {
		LOG_ERROR("pic32cfcw: missing variant argument");
		LOG_ERROR("Usage: flash bank <name> pic32cfcw <base> <size> "
				"0 0 <target> <variant>");
		LOG_ERROR("  Supported variants: CZCA80 CZCA90 CZCA91 "
				"CKSG CKGC CMGC00 CMSG00");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	const char *variant_name = CMD_ARGV[7];
	const struct pic32cfcw_variant *variant = pic32cfcw_find_variant(variant_name);
	if (!variant) {
		LOG_ERROR("pic32cfcw: unrecognised variant '%s'", variant_name);
		LOG_ERROR("  Supported variants: CZCA80 CZCA90 CZCA91 "
				"CKSG CKGC CMGC00 CMSG00");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	struct pic32cfcw_info *info = calloc(1, sizeof(*info));
	if (!info) {
		LOG_ERROR("pic32cfcw: out of memory allocating driver private data");
		return ERROR_FAIL;
	}

	pic32cfcw_fill_addrs(info, variant);
	bank->driver_priv = info;

	LOG_DEBUG("pic32cfcw: configured variant %s "
			"(FCW base 0x%08" PRIx32 ", Group %s, "
			"PFM base 0x%08" PRIx32 " size 0x%08" PRIx32 ")",
			variant->name, variant->fcw_base,
			variant->group_b ? "B" : "A",
			(uint32_t)bank->base, bank->size);

	return ERROR_OK;
}

/* =========================================================================
 * Tcl command: pic32cfcw_reset_deassert
 *
 * Called from target event hooks in .cfg files:
 *   $_TARGETNAME configure -event examine-end       { pic32cfcw_reset_deassert }
 *   $_TARGETNAME configure -event reset-deassert-post { pic32cfcw_reset_deassert }
 *
 * Performs the full reset-extension + DSU BCC mailbox handshake and halts
 * the core, placing it in a state ready for flash programming.
 * ========================================================================= */
COMMAND_HANDLER(pic32cfcw_handle_reset_deassert)
{
	struct target *target = get_current_target(CMD_CTX);

	/* Scan flash banks to find a pic32cfcw bank for the current target.
	 * We match by driver name to avoid a forward reference to &pic32cfcw_flash. */
	const struct pic32cfcw_info *info = NULL;
	for (struct flash_bank *b = flash_bank_list(); b; b = b->next) {
		if (b->target == target &&
				strcmp(b->driver->name, "pic32cfcw") == 0) {
			info = b->driver_priv;
			break;
		}
	}

	if (!info) {
		LOG_ERROR("pic32cfcw: no pic32cfcw flash bank found for current target; "
				"cannot perform reset-extension handshake");
		return ERROR_FAIL;
	}

	return pic32cfcw_reset_to_park(target, info);
}

static const struct command_registration pic32cfcw_command_handlers[] = {
	{
		.name = "pic32cfcw_reset_deassert",
		.handler = pic32cfcw_handle_reset_deassert,
		.mode = COMMAND_EXEC,
		.help = "Perform reset-extension and DSU BCC mailbox handshake to exit "
			"reset-extension mode for PIC32C FCW-equipped devices. "
			"Called automatically from target event hooks in the .cfg file.",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver pic32cfcw_flash = {
	.name = "pic32cfcw",
	.commands = pic32cfcw_command_handlers,
	.flash_bank_command = pic32cfcw_flash_bank_command,
	.erase = pic32cfcw_erase,
	.protect = NULL,
	.write = pic32cfcw_write,
	.read = default_flash_read,
	.probe = pic32cfcw_probe,
	.auto_probe = pic32cfcw_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.free_driver_priv = default_flash_free_driver_priv,
};
