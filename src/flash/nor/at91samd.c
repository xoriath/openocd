// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2013 by Andrey Yurovsky                                 *
 *   Andrey Yurovsky <yurovsky@gmail.com>                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "helper/binarybuffer.h"

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include <target/cortex_m.h>

#define SAMD_NUM_PROT_BLOCKS	16
#define SAMD_PAGE_SIZE_MAX	1024

#define SAMD_FLASH			((uint32_t)0x00000000)	/* physical Flash memory */
#define SAMD_USER_ROW		((uint32_t)0x00804000)	/* User Row of Flash */
#define SAMD_PAC1			0x41000000	/* Peripheral Access Control 1 */
#define SAMD_DSU			0x41002000	/* Device Service Unit */
#define SAMD_NVMCTRL		0x41004000	/* Non-volatile memory controller */

#define SAMD_DSU_STATUSA	1		/* STATUS A register, byte */
#define SAMD_DSU_STATUSB	2		/* STATUS B register, byte */
#define SAMD_DSU_DID		0x18		/* Device ID register */
#define SAMD_DSU_CTRL_EXT	0x100		/* CTRL register, external access */
/* DSU boot-ROM mailbox registers -- SAML10/L11 only (Cortex-M23) */
#define SAMD_DSU_BCC0		0x20		/* Boot Communication Channel 0, 32-bit */
#define SAMD_DSU_BCC1		0x24		/* Boot Communication Channel 1, 32-bit */
/* DSU STATUSA bits */
#define SAMD_DSU_STATUSA_CRSTEXT	0x02	/* CPU Reset Phase Extension active */
/* DSU STATUSB bits */
#define SAMD_DSU_STATUSB_DAL_MASK	0x03	/* Debug Access Level field */
#define SAMD_DSU_STATUSB_BOOTROM	0x80	/* Boot ROM active / BCC1 has reply */
/* BCC debugger command codes (values from sequences.xml ResetExtension) */
#define SAMD_DEBUGGER_CMD_EXIT		0x444247AAUL
#define SAMD_DEBUGGER_CMD_IMODE		0x44424755UL
#define SAMD_DEBUGGER_CMD_CHIPERASE	0x444247E3UL
/* BCC1 reply codes (SAML10/L11 Cortex-M23) */
#define SAMD_BCC1_REPLY_OK		0xEC000039UL
#define SAMD_BCC1_REPLY_LOCKED		0xEC000022UL
#define SAMD_BCC1_REPLY_IMODE_OK	0xEC000020UL
#define SAMD_BCC1_REPLY_ERASE_BUSY	0xEC000024UL
#define SAMD_BCC1_REPLY_ERASE_OK	0xEC000021UL
/* PIC32CM-PL10 BootROM reply code after DEBUGGER_CMD_EXIT */
#define SAMD_PL10_BOOTROM_STATUS_BOOTOK   0x4UL

/* PIC32CM-LE/LS "ALL" (full-chip, key-gated) erase command code, distinct
 * from SAMD_DEBUGGER_CMD_CHIPERASE (0xE3, the no-key non-secure-only variant
 * shared with SAML10/L11's chip-erase). */
#define SAMD_DEBUGGER_CMD_CHIPERASE_ALL	0x444247E2UL
/* Default CEKEY value used when BOCOR has not been custom-provisioned
 * (matches the MPLAB X debugger scripts' "x.erase.key" default). */
#define SAMD_LE_LS_CHIPERASE_DEFAULT_KEY	0xFFFFFFFFUL

/* PIC32CM-PL10 BootROM reply/status codes for the interactive-mode chip-erase
 * challenge/response handshake (distinct small-integer domain from the
 * 0xECxxxxxx codes used by SAML10/L11/LE/LS). */
#define SAMD_PL10_STATUS_CMD_VALID	0x5UL
#define SAMD_PL10_STATUS_CHALLENGE	0xBUL
#define SAMD_PL10_STATUS_OK		0x9UL

#define SAMD_NVMCTRL_CTRLA		0x00	/* NVM control A register */
#define SAMD_NVMCTRL_CTRLB		0x04	/* NVM control B register */
#define SAMD_NVMCTRL_PARAM		0x08	/* NVM parameters register */
#define SAMD_NVMCTRL_INTFLAG	0x14	/* NVM Interrupt Flag Status & Clear */
#define SAMD_NVMCTRL_STATUS		0x18	/* NVM status register */
#define SAMD_NVMCTRL_ADDR		0x1C	/* NVM address register */
#define SAMD_NVMCTRL_LOCK		0x20	/* NVM Lock section register */

#define SAMD_CMDEX_KEY		0xA5UL
#define SAMD_NVM_CMD(n)		((SAMD_CMDEX_KEY << 8) | (n & 0x7F))

/* NVMCTRL commands.  See Table 20-4 in 42129F–SAM–10/2013 */
#define SAMD_NVM_CMD_ER		0x02		/* Erase Row */
#define SAMD_NVM_CMD_WP		0x04		/* Write Page */
#define SAMD_NVM_CMD_EAR	0x05		/* Erase Auxiliary Row */
#define SAMD_NVM_CMD_WAP	0x06		/* Write Auxiliary Page */
#define SAMD_NVM_CMD_LR		0x40		/* Lock Region */
#define SAMD_NVM_CMD_UR		0x41		/* Unlock Region */
#define SAMD_NVM_CMD_SPRM	0x42		/* Set Power Reduction Mode */
#define SAMD_NVM_CMD_CPRM	0x43		/* Clear Power Reduction Mode */
#define SAMD_NVM_CMD_PBC	0x44		/* Page Buffer Clear */
#define SAMD_NVM_CMD_SSB	0x45		/* Set Security Bit */
#define SAMD_NVM_CMD_INVALL	0x46		/* Invalidate all caches */

/* NVMCTRL bits */
#define SAMD_NVM_CTRLB_MANW 0x80

/* NVMCTRL_INTFLAG bits */
#define SAMD_NVM_INTFLAG_READY 0x01

/* Known identifiers */
#define SAMD_PROCESSOR_M0	0x01
#define SAMD_FAMILY_D		0x00
#define SAMD_FAMILY_L		0x01
#define SAMD_FAMILY_C		0x02
#define SAMD_SERIES_20		0x00
#define SAMD_SERIES_21		0x01
#define SAMD_SERIES_22		0x02
#define SAMD_SERIES_10		0x02
#define SAMD_SERIES_11		0x03
#define SAMD_SERIES_09		0x04

/* Cortex-M23 processor ID (SAML10/L11 family) */
#define SAMD_PROCESSOR_M23	0x02
/* Series identifiers for SAM L10/L11 (under FAMILY_L + PROCESSOR_M23).
 * Note: SAMD_SERIES_L11 (0x03) and SAMD_SERIES_L10 (0x04) happen to alias
 * SAMD_SERIES_11 (0x03) and SAMD_SERIES_09 (0x04) numerically, but there is
 * no collision because samd_find_family() matches the full (processor, family,
 * series) triple. */
#define SAMD_SERIES_L11		0x03	/* SAM L11, under FAMILY_L + PROCESSOR_M23 */
#define SAMD_SERIES_L10		0x04	/* SAM L10, under FAMILY_L + PROCESSOR_M23 */

/* PIC32CM-PL10 placeholder family/series identifiers.
 * PIC32CM-PL10's DSU_DID uses different fields (PNDID/PNMID/VER) that do NOT map to
 * the classic PROCESSOR/FAMILY/SERIES scheme.  These placeholder values are never
 * matched by samd_find_family() via DID lookup -- identification uses forced_family
 * set by the mandatory variant-name Tcl argument in flash_bank_command. */
#define SAMD_FAMILY_PL    0x1F  /* placeholder, not a real DID field value */
#define SAMD_SERIES_PL10  0x3F  /* placeholder, not a real DID field value */

/* PIC32CM-GV placeholder family/series identifiers.
 * PIC32CM-GV's real DSU_DID decodes to the SAME (processor, family, series)
 * triple as SAMD20 (M0+, FAMILY_D, SERIES_20), so it cannot get its own
 * samd_families[] row without colliding with the existing SAMD20 entry.
 * This placeholder is NEVER matched by samd_find_family() via DID lookup;
 * it only anchors a row for the forced_family override mechanism. */
#define SAMD_FAMILY_CMGV  0x1E  /* placeholder for PIC32CM-GV  */
#define SAMD_SERIES_CMGV  0x3E

/* PIC32CM-JH/MC real family/series identifiers.
 * Both decode to FAMILY_C with SERIES values (6, 7) that are unused by any
 * existing SAMC row, so DID auto-probe works for these families. */
#define SAMD_SERIES_CMJH  0x06
#define SAMD_SERIES_CMMC  0x07

/* PIC32CM-LE/LS real family/series identifiers (FAMILY_L, PROCESSOR_M23).
 * LE decodes to a single SERIES (5).  LS spans three SERIES values (5, 6, 7)
 * because different flash-size sub-families use different DSU_DID SERIES
 * fields; all three rows point at the same pic32cmls_parts[] table.
 * SERIES 5 is shared between LE and LS's 1216-size SKUs: both variants have
 * an identical DSU_DID for that flash size, so auto-probe cannot tell them
 * apart by DID alone.  This is harmless for flashing since geometry and the
 * DAL reset-extension protocol are identical either way; only the part name
 * reported in the log may be wrong. */
#define SAMD_SERIES_CMLE     0x05
#define SAMD_SERIES_CMLS_5   0x05
#define SAMD_SERIES_CMLS_6   0x06
#define SAMD_SERIES_CMLS_7   0x07

/* SAMHA0/SAMHA1 placeholder family/series identifiers.
 * Note: SAMDA1 and SAMR30/34/35 (SAMR3x) are already fully supported via the existing
 * samd21_parts[] and saml21_parts[] tables under the SAMD21/SAML21 family rows
 * (SAMDA1 at DEVSEL 0x29-0x31 and 0x64-0x6C; SAMR30 at 0x1E-0x1F, SAMR34/R35 at 0x28+).
 * They do not need a forced-override entry here.
 * These values are NEVER matched by samd_find_family() via DID lookup; they exist
 * solely to anchor rows in samd_families[] for the forced_family override mechanism.
 * Values chosen from unused range (0x17-0x18 / 0x37-0x38) to avoid collisions with
 * real DID fields and with existing placeholder macros (0x1A-0x1F already used). */
#define SAMD_FAMILY_HA0   0x18  /* placeholder for SAMHA0                  */
#define SAMD_SERIES_HA0   0x38
#define SAMD_FAMILY_HA1   0x17  /* placeholder for SAMHA1                  */
#define SAMD_SERIES_HA1   0x37

/* Device ID macros */
#define SAMD_GET_PROCESSOR(id) (id >> 28)
#define SAMD_GET_FAMILY(id) (((id >> 23) & 0x1F))
#define SAMD_GET_SERIES(id) (((id >> 16) & 0x3F))
#define SAMD_GET_DEVSEL(id) (id & 0xFF)

/* Bits to mask out lockbits in user row */
#define NVMUSERROW_LOCKBIT_MASK 0x0000FFFFFFFFFFFFULL

struct samd_part {
	uint8_t id;
	const char *name;
	uint32_t flash_kb;
	uint32_t ram_kb;
};

/* Known SAMD09 parts. DID reset values missing in RM, see
 * https://github.com/avrxml/asf/blob/master/sam0/utils/cmsis/samd09/include/ */
static const struct samd_part samd09_parts[] = {
	{ 0x0, "SAMD09D14A", 16, 4 },
	{ 0x7, "SAMD09C13A", 8, 4 },
};

/* Known SAMD10 parts */
static const struct samd_part samd10_parts[] = {
	{ 0x0, "SAMD10D14AMU", 16, 4 },
	{ 0x1, "SAMD10D13AMU", 8, 4 },
	{ 0x2, "SAMD10D12AMU", 4, 4 },
	{ 0x3, "SAMD10D14ASU", 16, 4 },
	{ 0x4, "SAMD10D13ASU", 8, 4 },
	{ 0x5, "SAMD10D12ASU", 4, 4 },
	{ 0x6, "SAMD10C14A", 16, 4 },
	{ 0x7, "SAMD10C13A", 8, 4 },
	{ 0x8, "SAMD10C12A", 4, 4 },
};

/* Known SAMD11 parts */
static const struct samd_part samd11_parts[] = {
	{ 0x0, "SAMD11D14AM", 16, 4 },
	{ 0x1, "SAMD11D13AMU", 8, 4 },
	{ 0x2, "SAMD11D12AMU", 4, 4 },
	{ 0x3, "SAMD11D14ASS", 16, 4 },
	{ 0x4, "SAMD11D13ASU", 8, 4 },
	{ 0x5, "SAMD11D12ASU", 4, 4 },
	{ 0x6, "SAMD11C14A", 16, 4 },
	{ 0x7, "SAMD11C13A", 8, 4 },
	{ 0x8, "SAMD11C12A", 4, 4 },
	{ 0x9, "SAMD11D14AU", 16, 4 },
};

/* Known SAMD20 parts. See Table 12-8 in 42129F–SAM–10/2013 */
static const struct samd_part samd20_parts[] = {
	{ 0x0, "SAMD20J18A", 256, 32 },
	{ 0x1, "SAMD20J17A", 128, 16 },
	{ 0x2, "SAMD20J16A", 64, 8 },
	{ 0x3, "SAMD20J15A", 32, 4 },
	{ 0x4, "SAMD20J14A", 16, 2 },
	{ 0x5, "SAMD20G18A", 256, 32 },
	{ 0x6, "SAMD20G17A", 128, 16 },
	{ 0x7, "SAMD20G16A", 64, 8 },
	{ 0x8, "SAMD20G15A", 32, 4 },
	{ 0x9, "SAMD20G14A", 16, 2 },
	{ 0xA, "SAMD20E18A", 256, 32 },
	{ 0xB, "SAMD20E17A", 128, 16 },
	{ 0xC, "SAMD20E16A", 64, 8 },
	{ 0xD, "SAMD20E15A", 32, 4 },
	{ 0xE, "SAMD20E14A", 16, 2 },
};

/* Known SAMD21 parts. */
static const struct samd_part samd21_parts[] = {
	{ 0x0, "SAMD21J18A", 256, 32 },
	{ 0x1, "SAMD21J17A", 128, 16 },
	{ 0x2, "SAMD21J16A", 64, 8 },
	{ 0x3, "SAMD21J15A", 32, 4 },
	{ 0x4, "SAMD21J14A", 16, 2 },
	{ 0x5, "SAMD21G18A", 256, 32 },
	{ 0x6, "SAMD21G17A", 128, 16 },
	{ 0x7, "SAMD21G16A", 64, 8 },
	{ 0x8, "SAMD21G15A", 32, 4 },
	{ 0x9, "SAMD21G14A", 16, 2 },
	{ 0xA, "SAMD21E18A", 256, 32 },
	{ 0xB, "SAMD21E17A", 128, 16 },
	{ 0xC, "SAMD21E16A", 64, 8 },
	{ 0xD, "SAMD21E15A", 32, 4 },
	{ 0xE, "SAMD21E14A", 16, 2 },

    /* SAMR21 parts have integrated SAMD21 with a radio */
	{ 0x18, "SAMR21G19A", 256, 32 }, /* with 512k of serial flash */
	{ 0x19, "SAMR21G18A", 256, 32 },
	{ 0x1A, "SAMR21G17A", 128, 32 },
	{ 0x1B, "SAMR21G16A",  64, 16 },
	{ 0x1C, "SAMR21E18A", 256, 32 },
	{ 0x1D, "SAMR21E17A", 128, 32 },
	{ 0x1E, "SAMR21E16A",  64, 16 },

    /* SAMD21 B Variants (Table 3-7 from rev I of datasheet) */
	{ 0x20, "SAMD21J16B", 64, 8 },
	{ 0x21, "SAMD21J15B", 32, 4 },
	{ 0x23, "SAMD21G16B", 64, 8 },
	{ 0x24, "SAMD21G15B", 32, 4 },
	{ 0x26, "SAMD21E16B", 64, 8 },
	{ 0x27, "SAMD21E15B", 32, 4 },

	/* SAMD21 D and L Variants (from Errata)
	   http://ww1.microchip.com/downloads/en/DeviceDoc/
	   SAM-D21-Family-Silicon-Errata-and-DataSheet-Clarification-DS80000760D.pdf */
	{ 0x55, "SAMD21E16BU", 64, 8 },
	{ 0x56, "SAMD21E15BU", 32, 4 },
	{ 0x57, "SAMD21G16L", 64, 8 },
	{ 0x3E, "SAMD21E16L", 64, 8 },
	{ 0x3F, "SAMD21E15L", 32, 4 },
	{ 0x62, "SAMD21E16CU", 64, 8 },
	{ 0x63, "SAMD21E15CU", 32, 4 },
	{ 0x92, "SAMD21J17D", 128, 16 },
	{ 0x93, "SAMD21G17D", 128, 16 },
	{ 0x94, "SAMD21E17D", 128, 16 },
	{ 0x95, "SAMD21E17DU", 128, 16 },
	{ 0x96, "SAMD21G17L", 128, 16 },
	{ 0x97, "SAMD21E17L", 128, 16 },

	/* Known SAMDA1 parts.
	   SAMD-A1 series uses the same series identifier like the SAMD21
	   taken from http://ww1.microchip.com/downloads/en/DeviceDoc/40001895A.pdf (pages 14-17) */
	{ 0x29, "SAMDA1J16A", 64, 8 },
	{ 0x2A, "SAMDA1J15A", 32, 4 },
	{ 0x2B, "SAMDA1J14A", 16, 4 },
	{ 0x2C, "SAMDA1G16A", 64, 8 },
	{ 0x2D, "SAMDA1G15A", 32, 4 },
	{ 0x2E, "SAMDA1G14A", 16, 4 },
	{ 0x2F, "SAMDA1E16A", 64, 8 },
	{ 0x30, "SAMDA1E15A", 32, 4 },
	{ 0x31, "SAMDA1E14A", 16, 4 },
	{ 0x64, "SAMDA1J16B", 64, 8 },
	{ 0x65, "SAMDA1J15B", 32, 4 },
	{ 0x66, "SAMDA1J14B", 16, 4 },
	{ 0x67, "SAMDA1G16B", 64, 8 },
	{ 0x68, "SAMDA1G15B", 32, 4 },
	{ 0x69, "SAMDA1G14B", 16, 4 },
	{ 0x6A, "SAMDA1E16B", 64, 8 },
	{ 0x6B, "SAMDA1E15B", 32, 4 },
	{ 0x6C, "SAMDA1E14B", 16, 4 },
};

/* Known SAML21 parts. */
static const struct samd_part saml21_parts[] = {
	{ 0x00, "SAML21J18A", 256, 32 },
	{ 0x01, "SAML21J17A", 128, 16 },
	{ 0x02, "SAML21J16A", 64, 8 },
	{ 0x05, "SAML21G18A", 256, 32 },
	{ 0x06, "SAML21G17A", 128, 16 },
	{ 0x07, "SAML21G16A", 64, 8 },
	{ 0x0A, "SAML21E18A", 256, 32 },
	{ 0x0B, "SAML21E17A", 128, 16 },
	{ 0x0C, "SAML21E16A", 64, 8 },
	{ 0x0D, "SAML21E15A", 32, 4 },
	{ 0x0F, "SAML21J18B", 256, 32 },
	{ 0x10, "SAML21J17B", 128, 16 },
	{ 0x11, "SAML21J16B", 64, 8 },
	{ 0x14, "SAML21G18B", 256, 32 },
	{ 0x15, "SAML21G17B", 128, 16 },
	{ 0x16, "SAML21G16B", 64, 8 },
	{ 0x19, "SAML21E18B", 256, 32 },
	{ 0x1A, "SAML21E17B", 128, 16 },
	{ 0x1B, "SAML21E16B", 64, 8 },
	{ 0x1C, "SAML21E15B", 32, 4 },

    /* SAMR30 parts have integrated SAML21 with a radio */
	{ 0x1E, "SAMR30G18A", 256, 32 },
	{ 0x1F, "SAMR30E18A", 256, 32 },

    /* SAMR34/R35 parts have integrated SAML21 with a lora radio */
	{ 0x28, "SAMR34J18", 256, 40 },
	{ 0x29, "SAMR34J17", 128, 24 },
	{ 0x2A, "SAMR34J16", 64, 12 },
	{ 0x2B, "SAMR35J18", 256, 40 },
	{ 0x2C, "SAMR35J17", 128, 24 },
	{ 0x2D, "SAMR35J16", 64, 12 },
};

/* Known SAML22 parts. */
static const struct samd_part saml22_parts[] = {
	{ 0x00, "SAML22N18A", 256, 32 },
	{ 0x01, "SAML22N17A", 128, 16 },
	{ 0x02, "SAML22N16A", 64, 8 },
	{ 0x05, "SAML22J18A", 256, 32 },
	{ 0x06, "SAML22J17A", 128, 16 },
	{ 0x07, "SAML22J16A", 64, 8 },
	{ 0x0A, "SAML22G18A", 256, 32 },
	{ 0x0B, "SAML22G17A", 128, 16 },
	{ 0x0C, "SAML22G16A", 64, 8 },
};

/* Known SAML10 parts (Cortex-M23 with TrustZone-M). */
static const struct samd_part saml10_parts[] = {
	{ 0x00, "SAM L10E16A", 64, 16 },
	{ 0x01, "SAM L10E15A", 32, 8  },
	{ 0x02, "SAM L10E14A", 16, 4  },
	{ 0x03, "SAM L10D16A", 64, 16 },
	{ 0x04, "SAM L10D15A", 32, 8  },
	{ 0x05, "SAM L10D14A", 16, 4  },
};

/* Known SAML11 parts (Cortex-M23 with TrustZone-M). */
static const struct samd_part saml11_parts[] = {
	{ 0x00, "SAM L11E16A", 64, 16 },
	{ 0x01, "SAM L11E15A", 32, 8  },
	{ 0x02, "SAM L11E14A", 16, 8  },
	{ 0x03, "SAM L11D16A", 64, 16 },
	{ 0x04, "SAM L11D15A", 32, 8  },
	{ 0x05, "SAM L11D14A", 16, 8  },
};

/* Known PIC32CM-PL10 parts (Cortex-M0+ with DSU boot-ROM BCC mailbox).
 * No DID auto-probe: the variant must be specified via the mandatory Tcl
 * variant-name argument to flash bank (see samd_flash_bank_command).
 * id = DEVSEL byte from DSU_DID (0x0BA0_xx53). */
static const struct samd_part pic32cmpl_parts[] = {
	{ 0x00, "PIC32CM6408PL10028",  64,  8 },
	{ 0x01, "PIC32CM6408PL10032",  64,  8 },
	{ 0x02, "PIC32CM6408PL10048",  64,  8 },
	{ 0x03, "PIC32CM6408PL10064",  64,  8 },
	{ 0x04, "PIC32CM3204PL10020",  32,  4 },
	{ 0x05, "PIC32CM3204PL10028",  32,  4 },
	{ 0x06, "PIC32CM3204PL10032",  32,  4 },
	{ 0x0A, "PIC32CM1216PL10028", 128, 16 },
	{ 0x0B, "PIC32CM1216PL10032", 128, 16 },
	{ 0x0C, "PIC32CM1216PL10048", 128, 16 },
	{ 0x0D, "PIC32CM1216PL10064", 128, 16 },
};

/* Known PIC32CM-LE parts (Cortex-M23, BCC-mailbox DAL reset extension).
 * DSU layout is identical to SAML10/L11; reuses saml1x_dsu_layout directly.
 * id = real DID DEVSEL byte; DID auto-probe works for this family.
 * Chip erase uses the "ALL" BootROM command with the factory-default CEKEY;
 * see samd_bcc_chip_erase_all().  Devices with a custom-provisioned CEKEY
 * will fail chip-erase with a clear error (use MPLAB IPE in that case).
 * DID for the 1216 variants collides with the equivalent PIC32CM-LS parts;
 * harmless for flashing since geometry is identical either way. */
static const struct samd_part pic32cmle_parts[] = {
	{ 0x0A, "PIC32CM1216LE00032", 128, 16 },
	{ 0x0B, "PIC32CM1216LE00048", 128, 16 },
	{ 0x06, "PIC32CM2532LE00048", 256, 32 },
	{ 0x05, "PIC32CM2532LE00064", 256, 32 },
	{ 0x04, "PIC32CM2532LE00100", 256, 32 },
	{ 0x02, "PIC32CM5164LE00048", 512, 64 },
	{ 0x01, "PIC32CM5164LE00064", 512, 64 },
	{ 0x00, "PIC32CM5164LE00100", 512, 64 },
};

/* Known PIC32CM-LS parts (Cortex-M23, BCC-mailbox DAL reset extension).
 * DSU layout is identical to SAML10/L11; reuses saml1x_dsu_layout directly.
 * id = real DID DEVSEL byte; DID auto-probe works via the three
 * SAMD_SERIES_CMLS_* rows in samd_families[] (one per flash-size group).
 * TODO: PIC32CM-LS has dual NVMCTRL instances selected by DAL (DSU_STATUSB
 * bits[1:0]); this DAL-aware NVMCTRL offset selection is not implemented, so
 * flash operations only work correctly at DAL=full-access.
 * Chip erase uses the "ALL" BootROM command with the factory-default CEKEY;
 * see samd_bcc_chip_erase_all().  Devices with a custom-provisioned CEKEY
 * will fail chip-erase with a clear error (use MPLAB IPE in that case).
 * LS60 variants add hardware crypto/PKC (Trust Platform); their DEVSEL bytes
 * overlap with the LS00 5164 entries but live under a different SERIES. */
static const struct samd_part pic32cmls_parts[] = {
	{ 0x0A, "PIC32CM1216LS00032", 128, 16 },
	{ 0x0B, "PIC32CM1216LS00048", 128, 16 },
	{ 0x06, "PIC32CM2532LS00048", 256, 32 },
	{ 0x05, "PIC32CM2532LS00064", 256, 32 },
	{ 0x04, "PIC32CM2532LS00100", 256, 32 },
	{ 0x02, "PIC32CM5164LS00048", 512, 64 },
	{ 0x01, "PIC32CM5164LS00064", 512, 64 },
	{ 0x00, "PIC32CM5164LS00100", 512, 64 },
	{ 0x02, "PIC32CM5164LS60048", 512, 64 },
	{ 0x01, "PIC32CM5164LS60064", 512, 64 },
	{ 0x00, "PIC32CM5164LS60100", 512, 64 },
};

/* Known PIC32CM-GV parts (Cortex-M0+, legacy DSU path, no BCC mailbox).
 * Real DID collides with SAMD20's, so this family has no DID auto-probe;
 * variant must be specified via the mandatory Tcl variant-name arg.
 * id values are internal placeholders, not real DID DEVSEL values. */
static const struct samd_part pic32cmgv_parts[] = {
	{ 0x0, "PIC32CM1602GV00032", 16, 2 },
	{ 0x1, "PIC32CM1602GV00048", 16, 2 },
	{ 0x2, "PIC32CM1602GV00064", 16, 2 },
	{ 0x3, "PIC32CM3204GV00032", 32, 4 },
	{ 0x4, "PIC32CM3204GV00048", 32, 4 },
	{ 0x5, "PIC32CM3204GV00064", 32, 4 },
};

/* Known PIC32CM-JH parts (Cortex-M0+, legacy DSU path, no BCC mailbox).
 * id = real DID DEVSEL byte; DID auto-probe works for this family. */
static const struct samd_part pic32cmjh_parts[] = {
	{ 0x16, "PIC32CM2532JH00032", 256, 32 },
	{ 0x13, "PIC32CM2532JH00048", 256, 32 },
	{ 0x10, "PIC32CM2532JH00064", 256, 32 },
	{ 0x0D, "PIC32CM2532JH00100", 256, 32 },
	{ 0x15, "PIC32CM5164JH00032", 512, 64 },
	{ 0x14, "PIC32CM5164JH00048", 512, 64 },
	{ 0x0F, "PIC32CM5164JH00064", 512, 64 },
	{ 0x0E, "PIC32CM5164JH00100", 512, 64 },
	{ 0x07, "PIC32CM2532JH01032", 256, 32 },
	{ 0x06, "PIC32CM2532JH01048", 256, 32 },
	{ 0x05, "PIC32CM2532JH01064", 256, 32 },
	{ 0x04, "PIC32CM2532JH01100", 256, 32 },
	{ 0x03, "PIC32CM5164JH01032", 512, 64 },
	{ 0x02, "PIC32CM5164JH01048", 512, 64 },
	{ 0x01, "PIC32CM5164JH01064", 512, 64 },
	{ 0x00, "PIC32CM5164JH01100", 512, 64 },
	{ 0x0B, "PIC32CM1216JH01032", 128, 16 },
	{ 0x0A, "PIC32CM1216JH01048", 128, 16 },
	{ 0x1D, "PIC32CM3204JH00032",  32,  4 },
	{ 0x1B, "PIC32CM3204JH00048",  32,  4 },
	{ 0x19, "PIC32CM3204JH00064",  32,  4 },
	{ 0x1C, "PIC32CM6408JH00032",  64,  8 },
	{ 0x1A, "PIC32CM6408JH00048",  64,  8 },
	{ 0x18, "PIC32CM6408JH00064",  64,  8 },
};

/* Known PIC32CM-MC parts (Cortex-M0+, legacy DSU path, no BCC mailbox).
 * id = real DID DEVSEL byte; DID auto-probe works for this family. */
static const struct samd_part pic32cmmc_parts[] = {
	{ 0x00, "PIC32CM1216MC00032", 128, 16 },
	{ 0x06, "PIC32CM1216MC00048", 128, 16 },
	{ 0x01, "PIC32CM6408MC00032",  64,  8 },
	{ 0x07, "PIC32CM6408MC00048",  64,  8 },
};

/* Known SAMHA0 parts requiring forced-variant-name override.
 * Cortex-M0+, legacy DSU path, no BCC mailbox.
 * No DID auto-probe: variant must be specified via the mandatory Tcl variant-name arg.
 * id values are internal placeholders, not real DID DEVSEL values. */
static const struct samd_part samha0_parts[] = {
	{ 0x0, "ATSAMHA0E14AB",  16,  4 },
	{ 0x1, "ATSAMHA0E15AB",  32,  4 },
	{ 0x2, "ATSAMHA0E16AB",  64,  8 },
	{ 0x3, "ATSAMHA0G14AB",  16,  4 },
	{ 0x4, "ATSAMHA0G15AB",  32,  4 },
	{ 0x5, "ATSAMHA0G16AB",  64,  8 },
	{ 0x6, "ATSAMHA0G17AB", 128, 16 },
};

/* Known SAMHA1 parts requiring forced-variant-name override.
 * Cortex-M0+, legacy DSU path, no BCC mailbox.
 * No DID auto-probe: variant must be specified via the mandatory Tcl variant-name arg.
 * id values are internal placeholders, not real DID DEVSEL values. */
static const struct samd_part samha1_parts[] = {
	{ 0x0, "ATSAMHA1G14A",   16,  4 },
	{ 0x1, "ATSAMHA1G15A",   32,  4 },
	{ 0x2, "ATSAMHA1G16A",   64,  8 },
	{ 0x3, "ATSAMHA1E14AB",  16,  4 },
	{ 0x4, "ATSAMHA1E15AB",  32,  4 },
	{ 0x5, "ATSAMHA1E16AB",  64,  8 },
	{ 0x6, "ATSAMHA1G14AB",  16,  4 },
	{ 0x7, "ATSAMHA1G15AB",  32,  4 },
	{ 0x8, "ATSAMHA1G16AB",  64,  8 },
	{ 0x9, "ATSAMHA1G17AB", 128, 16 },
};

/* Known SAMC20 parts. */
static const struct samd_part samc20_parts[] = {
	{ 0x00, "SAMC20J18A", 256, 32 },
	{ 0x01, "SAMC20J17A", 128, 16 },
	{ 0x02, "SAMC20J16A", 64, 8 },
	{ 0x03, "SAMC20J15A", 32, 4 },
	{ 0x05, "SAMC20G18A", 256, 32 },
	{ 0x06, "SAMC20G17A", 128, 16 },
	{ 0x07, "SAMC20G16A", 64, 8 },
	{ 0x08, "SAMC20G15A", 32, 4 },
	{ 0x0A, "SAMC20E18A", 256, 32 },
	{ 0x0B, "SAMC20E17A", 128, 16 },
	{ 0x0C, "SAMC20E16A", 64, 8 },
	{ 0x0D, "SAMC20E15A", 32, 4 },
	{ 0x20, "SAMC20N18A", 256, 32 },
	{ 0x21, "SAMC20N17A", 128, 16 },
};

/* Known SAMC21 parts. */
static const struct samd_part samc21_parts[] = {
	{ 0x00, "SAMC21J18A", 256, 32 },
	{ 0x01, "SAMC21J17A", 128, 16 },
	{ 0x02, "SAMC21J16A", 64, 8 },
	{ 0x03, "SAMC21J15A", 32, 4 },
	{ 0x05, "SAMC21G18A", 256, 32 },
	{ 0x06, "SAMC21G17A", 128, 16 },
	{ 0x07, "SAMC21G16A", 64, 8 },
	{ 0x08, "SAMC21G15A", 32, 4 },
	{ 0x0A, "SAMC21E18A", 256, 32 },
	{ 0x0B, "SAMC21E17A", 128, 16 },
	{ 0x0C, "SAMC21E16A", 64, 8 },
	{ 0x0D, "SAMC21E15A", 32, 4 },
	{ 0x20, "SAMC21N18A", 256, 32 },
	{ 0x21, "SAMC21N17A", 128, 16 },
};

/**
 * Per-family DSU extended-block register layout and protocol parameters.
 * Only families with has_bootrom_dal=true carry a non-NULL dsu_layout pointer
 * (SAML10/L11 and PIC32CM-PL10).
 */
struct samd_dsu_layout {
	uint32_t statusa_addr;        /* absolute address of DSU STATUSA */
	uint32_t statusb_addr;        /* absolute address of DSU STATUSB */
	uint32_t bcc0_addr;           /* absolute address of DSU BCC0 */
	uint32_t bcc1_addr;           /* absolute address of DSU BCC1 */
	uint32_t dal_addr;            /* dedicated DAL register addr; 0 = derive from statusb & 0x3 */
	uint32_t crstext_mask;        /* CRSTEXT bit(s) in STATUSA */
	uint32_t brext_mask;          /* BREXT bit(s) in STATUSA */
	uint32_t bootrom_ready_mask;  /* bit(s) in STATUSB indicating BCC1 reply is ready */
	uint32_t bootok_reply;        /* expected BCC1 value after DEBUGGER_CMD_EXIT success */
	uint32_t locked_reply;        /* BCC1 value indicating locked chip (0 = not distinguished) */
	bool statusab_is_32bit;       /* true = use target_read/write_u32, false = u8 */
	bool chip_erase_supported;    /* false = chip-erase requires an unsupported protocol */
};

/* Each family of parts contains a parts table in the DEVSEL field of DID.  The
 * processor ID, family ID, and series ID are used to determine which exact
 * family this is and then we can use the corresponding table. */
struct samd_family {
	uint8_t processor;
	uint8_t family;
	uint8_t series;
	const struct samd_part *parts;
	size_t num_parts;
	uint64_t nvm_userrow_res_mask; /* protect bits which are reserved, 0 -> protect */
	bool has_bootrom_dal;          /* true: needs DSU BCC mailbox reset-extension handshake */
	const struct samd_dsu_layout *dsu_layout; /* NULL for families without has_bootrom_dal */
	bool needs_ram_xn_clear;       /* true: clear IDAU_RXN in the user row after reset-extension
					* exit (TrustZone parts only: SAML11, PIC32CM-LS) */
};

/* DSU register layout for SAML10/L11 (Cortex-M23, M23 extended DSU block).
 * Addresses verified against sequences.xml:
 *   STATUSA=0x41002101, STATUSB=0x41002102, BCC0=0x41002120, BCC1=0x41002124.
 * NOTE: the legacy-8-bit-DSU-block offsets (SAMD_DSU_STATUSA=1, SAMD_DSU_STATUSB=2,
 * SAMD_DSU_BCC0=0x20, SAMD_DSU_BCC1=0x24) produce addresses 0x100 too low and must
 * NOT be used for SAML10/L11 register access. */
static const struct samd_dsu_layout saml1x_dsu_layout = {
	.statusa_addr         = SAMD_DSU + 0x101,  /* 0x41002101 */
	.statusb_addr         = SAMD_DSU + 0x102,  /* 0x41002102 */
	.bcc0_addr            = SAMD_DSU + 0x120,  /* 0x41002120 */
	.bcc1_addr            = SAMD_DSU + 0x124,  /* 0x41002124 */
	.dal_addr             = 0,                  /* DAL derived from statusb & SAMD_DSU_STATUSB_DAL_MASK */
	.crstext_mask         = SAMD_DSU_STATUSA_CRSTEXT,  /* 0x02 */
	.brext_mask           = 0x20,              /* BREXT bit in M23 extended STATUSA */
	.bootrom_ready_mask   = SAMD_DSU_STATUSB_BOOTROM,  /* 0x80 */
	.bootok_reply         = SAMD_BCC1_REPLY_OK,        /* 0xEC000039UL */
	.locked_reply         = SAMD_BCC1_REPLY_LOCKED,    /* 0xEC000022UL */
	.statusab_is_32bit    = false,
	.chip_erase_supported = true,
};

/* DSU register layout for PIC32CM-PL10 (Cortex-M0+ with extended DSU block).
 * Addresses and masks from PIC32CM-PL10 sequences.xml.
 * STATUSA and STATUSB are 32-bit registers here (unlike SAML10/L11 which are 8-bit). */
static const struct samd_dsu_layout pic32cmpl_dsu_layout = {
	.statusa_addr         = SAMD_DSU + 0x104,  /* 0x41002104, 32-bit */
	.statusb_addr         = SAMD_DSU + 0x108,  /* 0x41002108, 32-bit */
	.bcc0_addr            = SAMD_DSU + 0x110,  /* 0x41002110 */
	.bcc1_addr            = SAMD_DSU + 0x114,  /* 0x41002114 */
	.dal_addr             = SAMD_DSU + 0x124,  /* 0x41002124, dedicated DAL register */
	.crstext_mask         = 0x100,             /* CRSTEXT bit in 32-bit STATUSA */
	.brext_mask           = 0x10000,           /* BREXT bit in 32-bit STATUSA */
	.bootrom_ready_mask   = 0x2,               /* BCCD1 bit in 32-bit STATUSB */
	.bootok_reply         = SAMD_PL10_BOOTROM_STATUS_BOOTOK,  /* 0x4UL */
	.locked_reply         = 0,                 /* locked condition not separately identified */
	.statusab_is_32bit    = true,
	.chip_erase_supported = true,
};

/* Known SAMD families */
static const struct samd_family samd_families[] = {
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_20,
		samd20_parts, ARRAY_SIZE(samd20_parts),
		0xFFFF01FFFE01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_21,
		samd21_parts, ARRAY_SIZE(samd21_parts),
		0xFFFF01FFFE01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_09,
		samd09_parts, ARRAY_SIZE(samd09_parts),
		0xFFFF01FFFE01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_10,
		samd10_parts, ARRAY_SIZE(samd10_parts),
		0xFFFF01FFFE01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_11,
		samd11_parts, ARRAY_SIZE(samd11_parts),
		0xFFFF01FFFE01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_L, SAMD_SERIES_21,
		saml21_parts, ARRAY_SIZE(saml21_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_L, SAMD_SERIES_22,
		saml22_parts, ARRAY_SIZE(saml22_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_C, SAMD_SERIES_20,
		samc20_parts, ARRAY_SIZE(samc20_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_C, SAMD_SERIES_21,
		samc21_parts, ARRAY_SIZE(samc21_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	/* TODO: verify user-row reserved-bit mask for SAML10/L11 (UROW/BOCOR split
	 * not yet modeled -- reusing L/C-family mask as a conservative placeholder) */
	{ SAMD_PROCESSOR_M23, SAMD_FAMILY_L, SAMD_SERIES_L10,
		saml10_parts, ARRAY_SIZE(saml10_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = true, .dsu_layout = &saml1x_dsu_layout },
	{ SAMD_PROCESSOR_M23, SAMD_FAMILY_L, SAMD_SERIES_L11,
		saml11_parts, ARRAY_SIZE(saml11_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = true, .dsu_layout = &saml1x_dsu_layout,
		.needs_ram_xn_clear = true },
	/* PIC32CM-PL10: Cortex-M0+ with DSU boot-ROM BCC mailbox.
	 * DSU_DID uses PNDID/PNMID/VER fields, not the classic PROCESSOR/FAMILY/SERIES
	 * scheme, so this row is never matched by samd_find_family() via DID lookup.
	 * The part is identified by the forced_family mechanism (variant-name Tcl arg).
	 * Fuse config is FUSES_BOOTCFG at 0x0D000400, not a plain NVMUSERROW, so the
	 * mask below (from USER_WORD_IMPLEMENTED_MASK) is informational only. */
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_PL, SAMD_SERIES_PL10,
		pic32cmpl_parts, ARRAY_SIZE(pic32cmpl_parts),
		0x3FE0000000000000ULL,
		.has_bootrom_dal = true, .dsu_layout = &pic32cmpl_dsu_layout },
	/* PIC32CM-LE: Cortex-M23, BCC-mailbox DAL handshake, reuses saml1x_dsu_layout.
	 * Real DID: FAMILY_L, SERIES 5 -- DID auto-probe works.  (1216-size SKUs
	 * share an identical DID with PIC32CM-LS's 1216 SKUs; see note above.) */
	{ SAMD_PROCESSOR_M23, SAMD_FAMILY_L, SAMD_SERIES_CMLE,
		pic32cmle_parts, ARRAY_SIZE(pic32cmle_parts),
		0xFFFFFC0001FF0040ULL,
		.has_bootrom_dal = true, .dsu_layout = &saml1x_dsu_layout },
	/* PIC32CM-LS: Cortex-M23, BCC-mailbox DAL handshake, reuses saml1x_dsu_layout.
	 * Real DID spans three SERIES values depending on flash size; all three
	 * rows share the same pic32cmls_parts[] table.  DID auto-probe works.
	 * NOTE: dual NVMCTRL instances (DAL-gated offset) not implemented; see parts table. */
	{ SAMD_PROCESSOR_M23, SAMD_FAMILY_L, SAMD_SERIES_CMLS_5,
		pic32cmls_parts, ARRAY_SIZE(pic32cmls_parts),
		0xFFFFE40001FF0040ULL,
		.has_bootrom_dal = true, .dsu_layout = &saml1x_dsu_layout,
		.needs_ram_xn_clear = true },
	{ SAMD_PROCESSOR_M23, SAMD_FAMILY_L, SAMD_SERIES_CMLS_6,
		pic32cmls_parts, ARRAY_SIZE(pic32cmls_parts),
		0xFFFFE40001FF0040ULL,
		.has_bootrom_dal = true, .dsu_layout = &saml1x_dsu_layout,
		.needs_ram_xn_clear = true },
	{ SAMD_PROCESSOR_M23, SAMD_FAMILY_L, SAMD_SERIES_CMLS_7,
		pic32cmls_parts, ARRAY_SIZE(pic32cmls_parts),
		0xFFFFE40001FF0040ULL,
		.has_bootrom_dal = true, .dsu_layout = &saml1x_dsu_layout,
		.needs_ram_xn_clear = true },
	/* PIC32CM-GV: Cortex-M0+, legacy DSU path, no BCC.
	 * Placeholder PROCESSOR/FAMILY/SERIES -- never matched by DID auto-probe.
	 * (Real DID collides with SAMD20's; see SAMD_FAMILY_CMGV comment above.) */
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_CMGV, SAMD_SERIES_CMGV,
		pic32cmgv_parts, ARRAY_SIZE(pic32cmgv_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	/* PIC32CM-JH: Cortex-M0+, legacy DSU path, no BCC.
	 * Real DID: FAMILY_C, SERIES 6 -- DID auto-probe works. */
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_C, SAMD_SERIES_CMJH,
		pic32cmjh_parts, ARRAY_SIZE(pic32cmjh_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	/* PIC32CM-MC: Cortex-M0+, legacy DSU path, no BCC.
	 * Real DID: FAMILY_C, SERIES 7 -- DID auto-probe works. */
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_C, SAMD_SERIES_CMMC,
		pic32cmmc_parts, ARRAY_SIZE(pic32cmmc_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	/* SAMHA0: Cortex-M0+, legacy DSU path, no BCC.
	 * Placeholder PROCESSOR/FAMILY/SERIES -- never matched by DID auto-probe. */
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_HA0, SAMD_SERIES_HA0,
		samha0_parts, ARRAY_SIZE(samha0_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
	/* SAMHA1: Cortex-M0+, legacy DSU path, no BCC.
	 * Placeholder PROCESSOR/FAMILY/SERIES -- never matched by DID auto-probe. */
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_HA1, SAMD_SERIES_HA1,
		samha1_parts, ARRAY_SIZE(samha1_parts),
		0xFFFF03FFFC01FF77ULL,
		.has_bootrom_dal = false, .dsu_layout = NULL },
};

struct samd_info {
	uint32_t page_size;
	int num_pages;
	int sector_size;
	int prot_block_size;

	bool probed;
	bool has_bootrom_dal;                     /* needs DSU BCC mailbox reset-extension handshake */
	const struct samd_family *family;         /* resolved family (from DID or forced_family) */
	const struct samd_part *forced_part;      /* non-NULL: variant-name arg bypassed DID probe */
	const struct samd_family *forced_family;  /* non-NULL: variant-name arg bypassed DID probe */
	bool cekey_set;                           /* true: cekey[] was supplied on the flash bank line */
	uint32_t cekey[4];                        /* PIC32CM-LE/LS custom CEKEY (from BOCOR provisioning) */
	struct target *target;
};


/**
 * Gives the family structure to specific device id.
 * @param id The id of the device.
 * @return On failure NULL, otherwise a pointer to the structure.
 */
static const struct samd_family *samd_find_family(uint32_t id)
{
	uint8_t processor = SAMD_GET_PROCESSOR(id);
	uint8_t family = SAMD_GET_FAMILY(id);
	uint8_t series = SAMD_GET_SERIES(id);

	for (unsigned int i = 0; i < ARRAY_SIZE(samd_families); i++) {
		if (samd_families[i].processor == processor &&
			samd_families[i].series == series &&
			samd_families[i].family == family)
			return &samd_families[i];
	}

	return NULL;
}

/**
 * Gives the part structure to specific device id.
 * @param id The id of the device.
 * @return On failure NULL, otherwise a pointer to the structure.
 */
static const struct samd_part *samd_find_part(uint32_t id)
{
	uint8_t devsel = SAMD_GET_DEVSEL(id);
	const struct samd_family *family = samd_find_family(id);
	if (!family)
		return NULL;

	for (unsigned int i = 0; i < family->num_parts; i++) {
		if (family->parts[i].id == devsel)
			return &family->parts[i];
	}

	return NULL;
}

static int samd_protect_check(struct flash_bank *bank)
{
	int res;
	uint16_t lock;

	res = target_read_u16(bank->target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_LOCK, &lock);
	if (res != ERROR_OK)
		return res;

	/* Lock bits are active-low */
	for (unsigned int prot_block = 0; prot_block < bank->num_prot_blocks; prot_block++)
		bank->prot_blocks[prot_block].is_protected = !(lock & (1u<<prot_block));

	return ERROR_OK;
}

static int samd_get_flash_page_info(struct target *target,
		uint32_t *sizep, int *nump)
{
	int res;
	uint32_t param;

	res = target_read_u32(target, SAMD_NVMCTRL + SAMD_NVMCTRL_PARAM, &param);
	if (res == ERROR_OK) {
		/* The PSZ field (bits 18:16) indicate the page size bytes as 2^(3+n)
		 * so 0 is 8KB and 7 is 1024KB. */
		if (sizep)
			*sizep = (8 << ((param >> 16) & 0x7));
		/* The NVMP field (bits 15:0) indicates the total number of pages */
		if (nump)
			*nump = param & 0xFFFF;
	} else {
		LOG_ERROR("Couldn't read NVM Parameters register");
	}

	return res;
}

static int samd_probe(struct flash_bank *bank)
{
	uint32_t id = 0;
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;
	const struct samd_part *part;
	const struct samd_family *family;

	if (chip->probed)
		return ERROR_OK;

	if (chip->forced_part && chip->forced_family) {
		/* PIC32CM-PL10 and similar: variant-name arg bypasses DID auto-probe */
		part = chip->forced_part;
		family = chip->forced_family;
	} else {
		res = target_read_u32(bank->target, SAMD_DSU + SAMD_DSU_DID, &id);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read Device ID register");
			return res;
		}

		part = samd_find_part(id);
		if (!part) {
			LOG_ERROR("Couldn't find part corresponding to DID %08" PRIx32, id);
			return ERROR_FAIL;
		}

		family = samd_find_family(id);
	}

	bank->size = part->flash_kb * 1024;

	res = samd_get_flash_page_info(bank->target, &chip->page_size,
			&chip->num_pages);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	/* Sanity check: the total flash size in the DSU should match the page size
	 * multiplied by the number of pages. */
	if (bank->size != chip->num_pages * chip->page_size) {
		LOG_WARNING("SAMD: bank size doesn't match NVM parameters. "
				"Identified %" PRIu32 "KB Flash but NVMCTRL reports %u %" PRIu32 "B pages",
				part->flash_kb, chip->num_pages, chip->page_size);
	}

	/* Erase granularity = 1 row = 4 pages */
	chip->sector_size = chip->page_size * 4;

	/* Allocate the sector table */
	bank->num_sectors = chip->num_pages / 4;
	bank->sectors = alloc_block_array(0, chip->sector_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	/* 16 protection blocks per device */
	chip->prot_block_size = bank->size / SAMD_NUM_PROT_BLOCKS;

	/* Allocate the table of protection blocks */
	bank->num_prot_blocks = SAMD_NUM_PROT_BLOCKS;
	bank->prot_blocks = alloc_block_array(0, chip->prot_block_size, bank->num_prot_blocks);
	if (!bank->prot_blocks)
		return ERROR_FAIL;

	samd_protect_check(bank);

	/* Done */
	chip->probed = true;
	chip->family = family;

	/* Record whether this target requires the DSU boot-ROM DAL handshake */
	chip->has_bootrom_dal = (family != NULL && family->has_bootrom_dal);

	LOG_INFO("SAMD MCU: %s (%" PRIu32 "KB Flash, %" PRIu32 "KB RAM)", part->name,
			part->flash_kb, part->ram_kb);

	return ERROR_OK;
}

static int samd_check_error(struct target *target)
{
	int ret, ret2;
	uint8_t intflag;
	uint16_t status;
	int timeout_ms = 1000;
	int64_t ts_start = timeval_ms();

	do {
		ret = target_read_u8(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_INTFLAG, &intflag);
		if (ret != ERROR_OK) {
			LOG_ERROR("Can't read NVM intflag");
			return ret;
		}
		if (intflag & SAMD_NVM_INTFLAG_READY)
			break;
		keep_alive();
	} while (timeval_ms() - ts_start < timeout_ms);

	if (!(intflag & SAMD_NVM_INTFLAG_READY)) {
		LOG_ERROR("SAMD: NVM programming timed out");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	ret = target_read_u16(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_STATUS, &status);
	if (ret != ERROR_OK) {
		LOG_ERROR("Can't read NVM status");
		return ret;
	}

	if ((status & 0x001C) == 0)
		return ERROR_OK;

	if (status & (1 << 4)) { /* NVME */
		LOG_ERROR("SAMD: NVM Error");
		ret = ERROR_FLASH_OPERATION_FAILED;
	}

	if (status & (1 << 3)) { /* LOCKE */
		LOG_ERROR("SAMD: NVM lock error");
		ret = ERROR_FLASH_PROTECTED;
	}

	if (status & (1 << 2)) { /* PROGE */
		LOG_ERROR("SAMD: NVM programming error");
		ret = ERROR_FLASH_OPER_UNSUPPORTED;
	}

	/* Clear the error conditions by writing a one to them */
	ret2 = target_write_u16(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_STATUS, status);
	if (ret2 != ERROR_OK)
		LOG_ERROR("Can't clear NVM error conditions");

	return ret;
}

static int samd_issue_nvmctrl_command(struct target *target, uint16_t cmd)
{
	int res;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Issue the NVM command */
	/* 32-bit write is used to ensure atomic operation on ST-Link */
	res = target_write_u32(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLA, SAMD_NVM_CMD(cmd));
	if (res != ERROR_OK)
		return res;

	/* Check to see if the NVM command resulted in an error condition. */
	return samd_check_error(target);
}

/**
 * Erases a flash-row at the given address.
 * @param target Pointer to the target structure.
 * @param address The address of the row.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int samd_erase_row(struct target *target, uint32_t address)
{
	int res;

	/* Set an address contained in the row to be erased */
	res = target_write_u32(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_ADDR, address >> 1);

	/* Issue the Erase Row command to erase that row. */
	if (res == ERROR_OK)
		res = samd_issue_nvmctrl_command(target,
				address == SAMD_USER_ROW ? SAMD_NVM_CMD_EAR : SAMD_NVM_CMD_ER);

	if (res != ERROR_OK)  {
		LOG_ERROR("Failed to erase row containing %08" PRIx32, address);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/**
 * Returns the bitmask of reserved bits in register.
 * @param target Pointer to the target structure.
 * @param mask Bitmask, 0 -> value stays untouched.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int samd_get_reservedmask(struct target *target, uint64_t *mask)
{
	int res;
	/* Get the devicetype */
	uint32_t id;
	res = target_read_u32(target, SAMD_DSU + SAMD_DSU_DID, &id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Device ID register");
		return res;
	}
	const struct samd_family *family;
	family = samd_find_family(id);
	if (!family) {
		LOG_ERROR("Couldn't determine device family");
		return ERROR_FAIL;
	}
	*mask = family->nvm_userrow_res_mask;
	return ERROR_OK;
}

static int read_userrow(struct target *target, uint64_t *userrow)
{
	int res;
	uint8_t buffer[8];

	res = target_read_memory(target, SAMD_USER_ROW, 4, 2, buffer);
	if (res != ERROR_OK)
		return res;

	*userrow = target_buffer_get_u64(target, buffer);
	return ERROR_OK;
}

/**
 * Modify the contents of the User Row in Flash. The User Row itself
 * has a size of one page and contains a combination of "fuses" and
 * calibration data. Bits which have a value of zero in the mask will
 * not be changed. Up to now devices only use the first 64 bits.
 * @param target Pointer to the target structure.
 * @param value_input The value to write.
 * @param value_mask Bitmask, 0 -> value stays untouched.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int samd_modify_user_row_masked(struct target *target,
		uint64_t value_input, uint64_t value_mask)
{
	int res;
	uint32_t nvm_ctrlb;
	bool manual_wp = true;

	/* Retrieve the MCU's page size, in bytes. This is also the size of the
	 * entire User Row. */
	uint32_t page_size;
	res = samd_get_flash_page_info(target, &page_size, NULL);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	/* Make sure the size is sane. */
	assert(page_size <= SAMD_PAGE_SIZE_MAX &&
		page_size >= sizeof(value_input));

	uint8_t buf[SAMD_PAGE_SIZE_MAX];
	/* Read the user row (comprising one page) by words. */
	res = target_read_memory(target, SAMD_USER_ROW, 4, page_size / 4, buf);
	if (res != ERROR_OK)
		return res;

	uint64_t value_device;
	res = read_userrow(target, &value_device);
	if (res != ERROR_OK)
		return res;
	uint64_t value_new = (value_input & value_mask) | (value_device & ~value_mask);

	/* We will need to erase before writing if the new value needs a '1' in any
	 * position for which the current value had a '0'.  Otherwise we can avoid
	 * erasing. */
	if ((~value_device) & value_new) {
		res = samd_erase_row(target, SAMD_USER_ROW);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't erase user row");
			return res;
		}
	}

	/* Modify */
	target_buffer_set_u64(target, buf, value_new);

	/* Write the page buffer back out to the target. */
	res = target_write_memory(target, SAMD_USER_ROW, 4, page_size / 4, buf);
	if (res != ERROR_OK)
		return res;

	/* Check if we need to do manual page write commands */
	res = target_read_u32(target, SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLB, &nvm_ctrlb);
	if (res == ERROR_OK)
		manual_wp = (nvm_ctrlb & SAMD_NVM_CTRLB_MANW) != 0;
	else {
		LOG_ERROR("Read of NVM register CTRKB failed.");
		return ERROR_FAIL;
	}
	if (manual_wp) {
		/* Trigger flash write */
		res = samd_issue_nvmctrl_command(target, SAMD_NVM_CMD_WAP);
	} else {
		res = samd_check_error(target);
	}

	return res;
}

/**
 * Modifies the user row register to the given value.
 * @param target Pointer to the target structure.
 * @param value The value to write.
 * @param startb The bit-offset by which the given value is shifted.
 * @param endb The bit-offset of the last bit in value to write.
 * @return On success ERROR_OK, on failure an errorcode.
 */
static int samd_modify_user_row(struct target *target, uint64_t value,
		uint8_t startb, uint8_t endb)
{
	uint64_t mask = 0;
	int i;
	for (i = startb ; i <= endb ; i++)
		mask |= ((uint64_t)1) << i;

	return samd_modify_user_row_masked(target, value << startb, mask);
}

static int samd_protect(struct flash_bank *bank, int set,
		unsigned int first, unsigned int last)
{
	int res = ERROR_OK;

	/* We can issue lock/unlock region commands with the target running but
	 * the settings won't persist unless we're able to modify the LOCK regions
	 * and that requires the target to be halted. */
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (unsigned int prot_block = first; prot_block <= last; prot_block++) {
		if (set != bank->prot_blocks[prot_block].is_protected) {
			/* Load an address that is within this protection block (we use offset 0) */
			res = target_write_u32(bank->target,
							SAMD_NVMCTRL + SAMD_NVMCTRL_ADDR,
							bank->prot_blocks[prot_block].offset >> 1);
			if (res != ERROR_OK)
				goto exit;

			/* Tell the controller to lock that block */
			res = samd_issue_nvmctrl_command(bank->target,
					set ? SAMD_NVM_CMD_LR : SAMD_NVM_CMD_UR);
			if (res != ERROR_OK)
				goto exit;
		}
	}

	/* We've now applied our changes, however they will be undone by the next
	 * reset unless we also apply them to the LOCK bits in the User Page.  The
	 * LOCK bits start at bit 48, corresponding to Sector 0 and end with bit 63,
	 * corresponding to Sector 15.  A '1' means unlocked and a '0' means
	 * locked.  See Table 9-3 in the SAMD20 datasheet for more details. */

	res = samd_modify_user_row(bank->target,
			set ? (uint64_t)0 : (uint64_t)UINT64_MAX,
			48 + first, 48 + last);
	if (res != ERROR_OK)
		LOG_WARNING("SAMD: protect settings were not made persistent!");

	res = ERROR_OK;

exit:
	samd_protect_check(bank);

	return res;
}

static int samd_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (samd_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* For each sector to be erased */
	for (unsigned int s = first; s <= last; s++) {
		res = samd_erase_row(bank->target, bank->sectors[s].offset);
		if (res != ERROR_OK) {
			LOG_ERROR("SAMD: failed to erase sector %d at 0x%08" PRIx32, s, bank->sectors[s].offset);
			return res;
		}
	}

	return ERROR_OK;
}


static int samd_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int res;
	uint32_t nvm_ctrlb;
	uint32_t address;
	uint32_t pg_offset;
	uint32_t nb;
	uint32_t nw;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;
	uint8_t *pb = NULL;
	bool manual_wp;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (samd_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* Check if we need to do manual page write commands */
	res = target_read_u32(bank->target, SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLB, &nvm_ctrlb);

	if (res != ERROR_OK)
		return res;

	if (nvm_ctrlb & SAMD_NVM_CTRLB_MANW)
		manual_wp = true;
	else
		manual_wp = false;

	res = samd_issue_nvmctrl_command(bank->target, SAMD_NVM_CMD_PBC);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: %d", __func__, __LINE__);
		return res;
	}

	while (count) {
		nb = chip->page_size - offset % chip->page_size;
		if (count < nb)
			nb = count;

		address = bank->base + offset;
		pg_offset = offset % chip->page_size;

		if (offset % 4 || (offset + nb) % 4) {
			/* Either start or end of write is not word aligned */
			if (!pb) {
				pb = malloc(chip->page_size);
				if (!pb)
					return ERROR_FAIL;
			}

			/* Set temporary page buffer to 0xff and overwrite the relevant part */
			memset(pb, 0xff, chip->page_size);
			memcpy(pb + pg_offset, buffer, nb);

			/* Align start address to a word boundary */
			address -= offset % 4;
			pg_offset -= offset % 4;
			assert(pg_offset % 4 == 0);

			/* Extend length to whole words */
			nw = (nb + offset % 4 + 3) / 4;
			assert(pg_offset + 4 * nw <= chip->page_size);

			/* Now we have original data extended by 0xff bytes
			 * to the nearest word boundary on both start and end */
			res = target_write_memory(bank->target, address, 4, nw, pb + pg_offset);
		} else {
			assert(nb % 4 == 0);
			nw = nb / 4;
			assert(pg_offset + 4 * nw <= chip->page_size);

			/* Word aligned data, use direct write from buffer */
			res = target_write_memory(bank->target, address, 4, nw, buffer);
		}
		if (res != ERROR_OK) {
			LOG_ERROR("%s: %d", __func__, __LINE__);
			goto free_pb;
		}

		/* Devices with errata 13134 have automatic page write enabled by default
		 * For other devices issue a write page CMD to the NVM
		 * If the page has not been written up to the last word
		 * then issue CMD_WP always */
		if (manual_wp || pg_offset + 4 * nw < chip->page_size) {
			res = samd_issue_nvmctrl_command(bank->target, SAMD_NVM_CMD_WP);
		} else {
			/* Access through AHB is stalled while flash is being programmed */
			usleep(200);

			res = samd_check_error(bank->target);
		}

		if (res != ERROR_OK) {
			LOG_ERROR("%s: write failed at address 0x%08" PRIx32, __func__, address);
			goto free_pb;
		}

		/* We're done with the page contents */
		count -= nb;
		offset += nb;
		buffer += nb;
	}

free_pb:
	free(pb);
	return res;
}

/* TODO: SAML10/L11 items not yet implemented:
 *  - DAL-aware NVMCTRL base-address selection (NVMCTRL sits at a different
 *    offset in secure vs non-secure alias space on L11)
 *  - UROW / BOCOR dual-bank user-row modeling for L11 TrustZone fuses
 *  - Refine reserved-bit mask in samd_families[] for L10/L11
 */

/**
 * Trigger the SAML10/L11 DSU reset-phase-extension entry.
 *
 * The boot ROM enters reset-extension mode when it detects >= 4 SWCLK edges
 * while nRESET is asserted (sequences.xml "ResetExtension").  We assert SRST
 * via the adapter, which causes the SWD layer to issue a reconnect sequence
 * (line-reset + JTAG-to-SWD switch + read IDCODE) that supplies the required
 * clock edges.
 *
 * NOTE: exact per-pulse SWCLK control while reset is held is not achievable
 * through jtag_add_reset() alone; the normal SWD reconnect traffic provides
 * sufficient edges for most J-Link/CMSIS-DAP adapters.  Hardware validation
 * is needed; a follow-up change should add adapter-level explicit SWCLK pulse
 * support for strict sequences.xml compliance.
 */
/**
 * Read DSU STATUSA via the access width specified in the layout (u8 or u32).
 * @param out  Receives the register value (zero-extended to 32 bits for u8 layouts).
 */
static int samd_dsu_read_statusa(struct target *target,
		const struct samd_dsu_layout *layout, uint32_t *out)
{
	if (layout->statusab_is_32bit) {
		return target_read_u32(target, layout->statusa_addr, out);
	} else {
		uint8_t val;
		int r = target_read_u8(target, layout->statusa_addr, &val);
		if (r == ERROR_OK)
			*out = val;
		return r;
	}
}

/**
 * Write DSU STATUSA via the access width specified in the layout (u8 or u32).
 */
static int samd_dsu_write_statusa(struct target *target,
		const struct samd_dsu_layout *layout, uint32_t val)
{
	if (layout->statusab_is_32bit)
		return target_write_u32(target, layout->statusa_addr, val);
	else
		return target_write_u8(target, layout->statusa_addr, (uint8_t)val);
}

/**
 * Read DSU STATUSB via the access width specified in the layout (u8 or u32).
 */
static int samd_dsu_read_statusb(struct target *target,
		const struct samd_dsu_layout *layout, uint32_t *out)
{
	if (layout->statusab_is_32bit) {
		return target_read_u32(target, layout->statusb_addr, out);
	} else {
		uint8_t val;
		int r = target_read_u8(target, layout->statusb_addr, &val);
		if (r == ERROR_OK)
			*out = val;
		return r;
	}
}

/**
 * Assert SRST for the DSU BCC mailbox reset-extension handshake.
 * Used by all families that have has_bootrom_dal=true.
 */
static int samd_bcc_reset_extension(struct target *target)
{
	int retval;
	(void)target;  /* target not used directly; SRST is asserted via the JTAG subsystem */

	LOG_DEBUG("SAMD BCC: asserting SRST for reset-extension handshake");

	jtag_add_reset(0, 1);	/* assert SRST */
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		LOG_DEBUG("SAMD BCC: failed to assert SRST (%d) -- continuing", retval);

	alive_sleep(5);

	jtag_add_reset(0, 0);	/* deassert SRST */
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		LOG_DEBUG("SAMD BCC: failed to deassert SRST (%d) -- continuing", retval);

	alive_sleep(10);

	return ERROR_OK;
}

/**
 * Complete the DSU BCC mailbox reset-extension exit handshake.
 * Implements sequences.xml "ExitResetExtension" for SAML10/L11 and PIC32CM-PL10.
 * Register addresses, widths, and masks are taken from @a layout.
 *
 * On success, *dal_out (if non-NULL) is set to the DAL value.
 */
static int samd_bcc_exit_reset_extension(struct target *target,
		const struct samd_dsu_layout *layout, uint8_t *dal_out)
{
	int retval;
	uint32_t statusa, statusb;
	uint32_t bcc1;
	int timeout_ms;
	int64_t ts_start;

	/* 1. Read STATUSA */
	retval = samd_dsu_read_statusa(target, layout, &statusa);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAMD BCC: failed to read DSU STATUSA");
		return retval;
	}

	/* 2. Verify CRSTEXT is set */
	if (!(statusa & layout->crstext_mask)) {
		LOG_ERROR("SAMD BCC: could not enter reset extension (CRSTEXT not set)");
		return ERROR_FAIL;
	}

	/* 3. W1C clear CRSTEXT so the boot ROM continues */
	retval = samd_dsu_write_statusa(target, layout, layout->crstext_mask);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAMD BCC: failed to clear CRSTEXT in DSU STATUSA");
		return retval;
	}

	/* 4. Allow boot ROM to advance (5 ms) */
	alive_sleep(5);

	/* 5. Check for boot-ROM-active status and user-page validation result */
	retval = samd_dsu_read_statusb(target, layout, &statusb);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAMD BCC: failed to read DSU STATUSB");
		return retval;
	}

	if (statusb & layout->bootrom_ready_mask) {
		retval = target_read_u32(target, layout->bcc1_addr, &bcc1);
		if (retval != ERROR_OK) {
			LOG_ERROR("SAMD BCC: failed to read BCC1");
			return retval;
		}
		if (bcc1 != 0) {
			LOG_ERROR("SAMD BCC: User page validation failed "
				"(BCC1=0x%08" PRIx32 ")", bcc1);
			return ERROR_FAIL;
		}
	}

	/* 6. Send EXIT command via BCC0 */
	retval = target_write_u32(target, layout->bcc0_addr, SAMD_DEBUGGER_CMD_EXIT);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAMD BCC: failed to write EXIT command to BCC0");
		return retval;
	}

	/* 7. Poll STATUSB bootrom_ready_mask until set (BCC1 reply ready) */
	timeout_ms = 500;
	ts_start = timeval_ms();
	do {
		retval = samd_dsu_read_statusb(target, layout, &statusb);
		if (retval != ERROR_OK) {
			LOG_ERROR("SAMD BCC: failed to read DSU STATUSB");
			return retval;
		}
		if (statusb & layout->bootrom_ready_mask)
			break;
		keep_alive();
	} while (timeval_ms() - ts_start < timeout_ms);

	if (!(statusb & layout->bootrom_ready_mask)) {
		LOG_ERROR("SAMD BCC: timeout waiting for boot ROM reply after EXIT command");
		return ERROR_FAIL;
	}

	/* 8. Read and validate BCC1 reply */
	retval = target_read_u32(target, layout->bcc1_addr, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAMD BCC: failed to read BCC1 reply");
		return retval;
	}

	if (layout->locked_reply != 0 && bcc1 == layout->locked_reply) {
		LOG_ERROR("SAMD BCC: Chip is locked (BCC1=0x%08" PRIx32 ")", bcc1);
		return ERROR_FAIL;
	}
	if (bcc1 != layout->bootok_reply) {
		LOG_ERROR("SAMD BCC: Boot validation failed "
			"(BCC1=0x%08" PRIx32 ", expected 0x%08" PRIx32 ")",
			bcc1, layout->bootok_reply);
		return ERROR_FAIL;
	}

	/* 9. Halt the core */
	retval = target_write_u32(target, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAMD BCC: failed to write DHCSR to halt core");
		return retval;
	}

	/* 10. Read DAL */
	uint32_t dal;
	if (layout->dal_addr != 0) {
		/* Dedicated DAL register (e.g. PIC32CM-PL10) */
		retval = target_read_u32(target, layout->dal_addr, &dal);
		if (retval != ERROR_OK) {
			LOG_ERROR("SAMD BCC: failed to read DAL register");
			return retval;
		}
	} else {
		/* DAL encoded in STATUSB (e.g. SAML10/L11) */
		retval = samd_dsu_read_statusb(target, layout, &statusb);
		if (retval != ERROR_OK) {
			LOG_ERROR("SAMD BCC: failed to read DSU STATUSB for DAL");
			return retval;
		}
		dal = statusb & SAMD_DSU_STATUSB_DAL_MASK;
	}

	if (dal_out)
		*dal_out = (uint8_t)dal;
	LOG_INFO("SAMD BCC DAL=%u", (unsigned int)dal);

	return ERROR_OK;
}

/**
 * Assert reset-extension and then exit via BCC handshake (park state).
 * Combines samd_bcc_reset_extension() + samd_bcc_exit_reset_extension().
 */
static int samd_bcc_reset_to_park(struct target *target,
		const struct samd_dsu_layout *layout, uint8_t *dal_out)
{
	int retval = samd_bcc_reset_extension(target);
	if (retval != ERROR_OK)
		return retval;
	return samd_bcc_exit_reset_extension(target, layout, dal_out);
}

/**
 * Clear the IDAU_RXN (RAM eXecute Never) fuse bit in the user row, if set.
 * This bit lives at bit 11 of user-row WORD_1 (absolute bit 43 of the
 * 64-bit user row) on TrustZone-capable M23 parts (SAML11, PIC32CM-LS).
 * Clearing a fuse bit from 1 to 0 never requires erasing the row first,
 * so this is safe to call unconditionally on every reset-extension exit.
 */
static int samd_clear_ram_execute_never(struct target *target)
{
	return samd_modify_user_row(target, 0, 43, 43);
}

/**
 * Chip-erase a SAML10/L11 via the DSU boot-ROM BCC mailbox.
 * Implements sequences.xml "FlashEraseChip".
 * Uses saml1x_dsu_layout for all register addresses (SAML10/L11 only).
 */
static int samdl1x_chip_erase(struct target *target)
{
	int retval;
	uint8_t statusa_u8;
	uint32_t bcc1;
	int timeout_ms;
	int64_t ts_start;

	/* 1. Enter reset extension */
	retval = samd_bcc_reset_extension(target);
	if (retval != ERROR_OK)
		return retval;

	/* 2. Read STATUSA (u8), verify CRSTEXT is set.
	 * Uses corrected SAML10/L11 address 0x41002101 from saml1x_dsu_layout. */
	retval = target_read_u8(target, saml1x_dsu_layout.statusa_addr, &statusa_u8);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAML10/L11: failed to read DSU STATUSA");
		return retval;
	}
	if (!(statusa_u8 & saml1x_dsu_layout.crstext_mask)) {
		LOG_ERROR("SAML10/L11: could not enter reset extension (CRSTEXT not set)");
		return ERROR_FAIL;
	}

	/* 3. W1C clear CRSTEXT, delay 5 ms */
	retval = target_write_u8(target, saml1x_dsu_layout.statusa_addr,
			(uint8_t)saml1x_dsu_layout.crstext_mask);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAML10/L11: failed to clear CRSTEXT");
		return retval;
	}
	alive_sleep(5);

	/* 4. Enter interactive mode */
	retval = target_write_u32(target, saml1x_dsu_layout.bcc0_addr,
			SAMD_DEBUGGER_CMD_IMODE);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAML10/L11: failed to write IMODE command to BCC0");
		return retval;
	}
	retval = target_read_u32(target, saml1x_dsu_layout.bcc1_addr, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAML10/L11: failed to read BCC1 after IMODE command");
		return retval;
	}
	if (bcc1 != SAMD_BCC1_REPLY_IMODE_OK) {
		LOG_ERROR("SAML10/L11: failed to enter command loop "
			"(BCC1=0x%08" PRIx32 ")", bcc1);
		return ERROR_FAIL;
	}

	/* 5. Issue chip-erase command */
	retval = target_write_u32(target, saml1x_dsu_layout.bcc0_addr,
			SAMD_DEBUGGER_CMD_CHIPERASE);
	if (retval != ERROR_OK) {
		LOG_ERROR("SAML10/L11: failed to write CHIPERASE command to BCC0");
		return retval;
	}

	/* 6a. Wait for erase to start (BCC1 == ERASE_BUSY) */
	timeout_ms = 500;
	ts_start = timeval_ms();
	do {
		retval = target_read_u32(target, saml1x_dsu_layout.bcc1_addr, &bcc1);
		if (retval != ERROR_OK) {
			LOG_ERROR("SAML10/L11: failed to read BCC1 during erase-start poll");
			return retval;
		}
		if (bcc1 == SAMD_BCC1_REPLY_ERASE_BUSY)
			break;
		keep_alive();
	} while (timeval_ms() - ts_start < timeout_ms);

	if (bcc1 != SAMD_BCC1_REPLY_ERASE_BUSY) {
		LOG_ERROR("SAML10/L11: chip erase did not start "
			"(BCC1=0x%08" PRIx32 ")", bcc1);
		return ERROR_FAIL;
	}

	/* 6b. Wait for erase to complete (BCC1 changes from ERASE_BUSY) */
	timeout_ms = 10000;
	ts_start = timeval_ms();
	do {
		retval = target_read_u32(target, saml1x_dsu_layout.bcc1_addr, &bcc1);
		if (retval != ERROR_OK) {
			LOG_ERROR("SAML10/L11: failed to read BCC1 during erase-complete poll");
			return retval;
		}
		if (bcc1 != SAMD_BCC1_REPLY_ERASE_BUSY)
			break;
		keep_alive();
	} while (timeval_ms() - ts_start < timeout_ms);

	if (bcc1 == SAMD_BCC1_REPLY_ERASE_BUSY) {
		LOG_ERROR("SAML10/L11: chip erase timed out");
		return ERROR_FAIL;
	}
	if (bcc1 != SAMD_BCC1_REPLY_ERASE_OK) {
		LOG_ERROR("SAML10/L11: chip erase failed (BCC1=0x%08" PRIx32 ")", bcc1);
		return ERROR_FAIL;
	}

	/* 7. Return to park state (equivalent to sequences.xml Sequence(ResetToPark)) */
	return samd_bcc_reset_to_park(target, &saml1x_dsu_layout, NULL);
}

/**
 * Poll BCC1 until it equals @a expect or @a timeout_ms elapses.
 */
static int samd_bcc_poll_bcc1(struct target *target, uint32_t bcc1_addr,
		uint32_t expect, int timeout_ms, uint32_t *last_out)
{
	int retval;
	uint32_t bcc1 = 0;
	int64_t ts_start = timeval_ms();

	do {
		retval = target_read_u32(target, bcc1_addr, &bcc1);
		if (retval != ERROR_OK)
			return retval;
		if (bcc1 == expect)
			break;
		keep_alive();
	} while (timeval_ms() - ts_start < timeout_ms);

	if (last_out)
		*last_out = bcc1;
	return (bcc1 == expect) ? ERROR_OK : ERROR_FAIL;
}

/**
 * Full-chip erase for PIC32CM-LE/LS via the "ALL" (key-gated) BootROM
 * command (0xE2).  Protocol confirmed from the MPLAB X DFP debugger
 * scripts (icd4_cortex-m23.py, PIC32CM-LE_DFP/PIC32CM-LS_DFP): the same
 * reset-extension/IMODE-entry handshake as samdl1x_chip_erase(), plus a
 * 4-word key sent via BCC0 with a per-word BCCD0-clear handshake.  The
 * key is a static per-device value stored in BOCOR (not a computed HMAC
 * or challenge response); this implementation uses the documented
 * factory-default key (all 0xFFFFFFFF words), which is what an
 * unprovisioned/default device expects.  If a device has been
 * provisioned with a custom CEKEY, this erase will fail with a clear
 * error rather than silently doing the wrong thing.
 */
static int samd_bcc_chip_erase_all(struct target *target,
		const struct samd_dsu_layout *layout, const uint32_t key[4])
{
	int retval;
	uint8_t statusa_u8;
	uint32_t statusb, bcc1;

	/* 1. Enter reset extension */
	retval = samd_bcc_reset_extension(target);
	if (retval != ERROR_OK)
		return retval;

	/* 2. Read STATUSA (8-bit for SAML10/L11-layout parts), verify CRSTEXT */
	retval = target_read_u8(target, layout->statusa_addr, &statusa_u8);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-LE/LS: failed to read DSU STATUSA");
		return retval;
	}
	if (!(statusa_u8 & layout->crstext_mask)) {
		LOG_ERROR("PIC32CM-LE/LS: could not enter reset extension (CRSTEXT not set)");
		return ERROR_FAIL;
	}

	/* 3. W1C clear CRSTEXT, delay 5 ms */
	retval = target_write_u8(target, layout->statusa_addr, (uint8_t)layout->crstext_mask);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-LE/LS: failed to clear CRSTEXT");
		return retval;
	}
	alive_sleep(5);

	/* 4. If the BootROM already has a reply pending, it indicates a prior
	 * user-page validation failure -- bail out rather than continuing. */
	retval = samd_dsu_read_statusb(target, layout, &statusb);
	if (retval != ERROR_OK)
		return retval;
	if (statusb & layout->bootrom_ready_mask) {
		retval = target_read_u32(target, layout->bcc1_addr, &bcc1);
		if (retval != ERROR_OK)
			return retval;
		if (bcc1 != 0) {
			LOG_ERROR("PIC32CM-LE/LS: user page validation failed "
				"(BCC1=0x%08" PRIx32 ")", bcc1);
			return ERROR_FAIL;
		}
	}

	/* 5. Enter interactive mode */
	retval = target_write_u32(target, layout->bcc0_addr, SAMD_DEBUGGER_CMD_IMODE);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-LE/LS: failed to write IMODE command to BCC0");
		return retval;
	}
	retval = target_read_u32(target, layout->bcc1_addr, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-LE/LS: failed to read BCC1 after IMODE command");
		return retval;
	}
	if (bcc1 != SAMD_BCC1_REPLY_IMODE_OK) {
		LOG_ERROR("PIC32CM-LE/LS: failed to enter command loop "
			"(BCC1=0x%08" PRIx32 ")", bcc1);
		return ERROR_FAIL;
	}

	/* 6. Issue the "ALL" chip-erase command */
	retval = target_write_u32(target, layout->bcc0_addr, SAMD_DEBUGGER_CMD_CHIPERASE_ALL);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-LE/LS: failed to write chip-erase command to BCC0");
		return retval;
	}
	retval = target_read_u32(target, layout->bcc1_addr, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-LE/LS: failed to read BCC1 after chip-erase command");
		return retval;
	}
	if (bcc1 != SAMD_BCC1_REPLY_ERASE_BUSY) {
		LOG_ERROR("PIC32CM-LE/LS: chip-erase command rejected "
			"(BCC1=0x%08" PRIx32 ")", bcc1);
		return ERROR_FAIL;
	}

	/* 7. Send the 4-word CEKEY, one word at a time, waiting for the
	 * BootROM to clear BCCD0 (bit 0x40 of STATUSB) after each word. */
	for (int i = 0; i < 4; i++) {
		retval = target_write_u32(target, layout->bcc0_addr, key[i]);
		if (retval != ERROR_OK) {
			LOG_ERROR("PIC32CM-LE/LS: failed to write CEKEY word %d", i);
			return retval;
		}

		int64_t ts_start = timeval_ms();
		do {
			retval = samd_dsu_read_statusb(target, layout, &statusb);
			if (retval != ERROR_OK)
				return retval;
			if (!(statusb & 0x40))
				break;
			keep_alive();
		} while (timeval_ms() - ts_start < 500);
		if (statusb & 0x40) {
			LOG_ERROR("PIC32CM-LE/LS: BootROM did not accept CEKEY word %d "
				"(wrong key for this device?)", i);
			return ERROR_FAIL;
		}
	}

	/* 8. Wait for the erase to finish (BCC1 changes away from ERASE_BUSY).
	 * samd_bcc_poll_bcc1() only waits for a specific value, so poll manually
	 * here since we're waiting for the value to change away from one. */
	int64_t ts_start = timeval_ms();
	do {
		retval = target_read_u32(target, layout->bcc1_addr, &bcc1);
		if (retval != ERROR_OK)
			return retval;
		if (bcc1 != SAMD_BCC1_REPLY_ERASE_BUSY && bcc1 != 0)
			break;
		keep_alive();
	} while (timeval_ms() - ts_start < 30000);

	if (bcc1 != SAMD_BCC1_REPLY_ERASE_OK) {
		LOG_ERROR("PIC32CM-LE/LS: chip erase failed (BCC1=0x%08" PRIx32 ")", bcc1);
		return ERROR_FAIL;
	}

	/* 9. Return to park state */
	return samd_bcc_reset_to_park(target, layout, NULL);
}

/**
 * Full-chip erase for PIC32CM-PL10 via the interactive-mode CMD_CE_ALL
 * (0xE3) BootROM command.  Protocol confirmed from the MPLAB X DFP
 * debugger scripts (icd4_cortex-m0plus.py, PIC32CM-PL_DFP): unlike
 * SAML10/L11/LE/LS, PL10 uses a small-integer status-code domain
 * (SAMD_PL10_STATUS_*) instead of the 0xECxxxxxx codes, and its
 * "challenge/response" step is a fixed, non-secret handshake -- four
 * reads of BCC1 (values discarded) followed by eight literal zero
 * writes to BCC0, each gated on a STATUSB bit rather than any
 * device-specific value.  No key or HMAC is involved.
 */
static int pic32cmpl_chip_erase(struct target *target)
{
	int retval;
	uint32_t statusa, statusb, bcc1;
	const struct samd_dsu_layout *layout = &pic32cmpl_dsu_layout;

	/* 1. Enter reset extension */
	retval = samd_bcc_reset_extension(target);
	if (retval != ERROR_OK)
		return retval;

	/* 2. Read STATUSA (32-bit for PIC32CM-PL10), verify CRSTEXT, clear it */
	retval = samd_dsu_read_statusa(target, layout, &statusa);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-PL10: failed to read DSU STATUSA");
		return retval;
	}
	if (!(statusa & layout->crstext_mask)) {
		LOG_ERROR("PIC32CM-PL10: could not enter reset extension (CRSTEXT not set)");
		return ERROR_FAIL;
	}
	retval = samd_dsu_write_statusa(target, layout, layout->crstext_mask);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-PL10: failed to clear CRSTEXT");
		return retval;
	}
	alive_sleep(5);

	/* 3. Enter interactive mode; PL10 replies with CMD_VALID (0x5), not
	 * the IMODE_OK (0xEC000020) code used by SAML10/L11/LE/LS. */
	retval = target_write_u32(target, layout->bcc0_addr, SAMD_DEBUGGER_CMD_IMODE);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-PL10: failed to write IMODE command to BCC0");
		return retval;
	}
	retval = samd_bcc_poll_bcc1(target, layout->bcc1_addr, SAMD_PL10_STATUS_CMD_VALID,
			5000, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-PL10: failed to enter interactive mode "
			"(BCC1=0x%08" PRIx32 ")", bcc1);
		return retval;
	}

	/* 4. Issue CMD_CE_ALL; wait for the CHALLENGE status */
	retval = target_write_u32(target, layout->bcc0_addr, SAMD_DEBUGGER_CMD_CHIPERASE);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-PL10: failed to write chip-erase command to BCC0");
		return retval;
	}
	retval = samd_bcc_poll_bcc1(target, layout->bcc1_addr, SAMD_PL10_STATUS_CHALLENGE,
			5000, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-PL10: chip-erase command rejected "
			"(BCC1=0x%08" PRIx32 ")", bcc1);
		return retval;
	}

	/* 5. Fixed, non-secret challenge/response handshake: drain 4 words
	 * from BCC1 (values unused), then send 8 literal zero words. */
	for (int i = 0; i < 4; i++) {
		int64_t ts_start = timeval_ms();
		do {
			retval = samd_dsu_read_statusb(target, layout, &statusb);
			if (retval != ERROR_OK)
				return retval;
			if (statusb & layout->bootrom_ready_mask)
				break;
			keep_alive();
		} while (timeval_ms() - ts_start < 5000);
		if (!(statusb & layout->bootrom_ready_mask)) {
			LOG_ERROR("PIC32CM-PL10: BootROM did not send full challenge string");
			return ERROR_FAIL;
		}
		retval = target_read_u32(target, layout->bcc1_addr, &bcc1); /* discarded */
		if (retval != ERROR_OK)
			return retval;
	}
	for (int i = 0; i < 8; i++) {
		int64_t ts_start = timeval_ms();
		do {
			retval = samd_dsu_read_statusb(target, layout, &statusb);
			if (retval != ERROR_OK)
				return retval;
			if (!(statusb & 0x1))
				break;
			keep_alive();
		} while (timeval_ms() - ts_start < 5000);
		if (statusb & 0x1) {
			LOG_ERROR("PIC32CM-PL10: BootROM did not accept challenge response");
			return ERROR_FAIL;
		}
		retval = target_write_u32(target, layout->bcc0_addr, 0);
		if (retval != ERROR_OK)
			return retval;
	}

	/* 6. Wait for command acceptance, then for completion */
	retval = samd_bcc_poll_bcc1(target, layout->bcc1_addr, SAMD_PL10_STATUS_CMD_VALID,
			10000, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-PL10: chip erase not accepted "
			"(BCC1=0x%08" PRIx32 ")", bcc1);
		return retval;
	}
	retval = samd_bcc_poll_bcc1(target, layout->bcc1_addr, SAMD_PL10_STATUS_OK,
			60000, &bcc1);
	if (retval != ERROR_OK) {
		LOG_ERROR("PIC32CM-PL10: chip erase failed or timed out "
			"(BCC1=0x%08" PRIx32 ")", bcc1);
		return retval;
	}

	/* 7. Return to park state */
	return samd_bcc_reset_to_park(target, layout, NULL);
}

FLASH_BANK_COMMAND_HANDLER(samd_flash_bank_command)
{
	if (bank->base != SAMD_FLASH) {
		LOG_ERROR("Address " TARGET_ADDR_FMT
				" invalid bank address (try 0x%08" PRIx32
				"[at91samd series] )",
				bank->base, SAMD_FLASH);
		return ERROR_FAIL;
	}

	struct samd_info *chip;
	chip = calloc(1, sizeof(*chip));
	if (!chip) {
		LOG_ERROR("No memory for flash bank chip info");
		return ERROR_FAIL;
	}

	chip->target = bank->target;
	chip->probed = false;
	chip->forced_part = NULL;
	chip->forced_family = NULL;

	/* Optional 8th argument: variant name for parts with no DID auto-probe
	 * (e.g. PIC32CM-PL10 whose DSU_DID uses non-standard field encoding).
	 * When supplied, DID-based family/part lookup in samd_probe() is bypassed. */
	if (CMD_ARGC >= 8) {
		const char *variant = CMD_ARGV[7];
		bool found = false;

		/* Search all forced-override parts tables for the requested variant name.
		 * Each entry in forced_tbls[] corresponds to a family whose parts are
		 * identified by variant-name override rather than DID auto-probe.
		 * The matching samd_families[] row is located by its parts pointer. */
		static const struct samd_part * const forced_tbls[] = {
			pic32cmpl_parts, pic32cmle_parts, pic32cmls_parts,
			pic32cmgv_parts, pic32cmjh_parts, pic32cmmc_parts,
			samha0_parts, samha1_parts,
		};
		static const size_t forced_tbl_sizes[] = {
			ARRAY_SIZE(pic32cmpl_parts), ARRAY_SIZE(pic32cmle_parts),
			ARRAY_SIZE(pic32cmls_parts), ARRAY_SIZE(pic32cmgv_parts),
			ARRAY_SIZE(pic32cmjh_parts), ARRAY_SIZE(pic32cmmc_parts),
			ARRAY_SIZE(samha0_parts), ARRAY_SIZE(samha1_parts),
		};
		for (size_t t = 0; t < ARRAY_SIZE(forced_tbls) && !chip->forced_part; t++) {
			for (size_t i = 0; i < forced_tbl_sizes[t]; i++) {
				if (strcmp(forced_tbls[t][i].name, variant) == 0) {
					chip->forced_part = &forced_tbls[t][i];
					for (size_t fi = 0; fi < ARRAY_SIZE(samd_families); fi++) {
						if (samd_families[fi].parts == forced_tbls[t]) {
							chip->forced_family = &samd_families[fi];
							found = true;
							break;
						}
					}
					break;
				}
			}
		}

		if (!found) {
			LOG_ERROR("at91samd: unknown variant name '%s' "
				"(PL10 e.g.: PIC32CM6408PL10028..PIC32CM1216PL10064; "
				"LE e.g.: PIC32CM1216LE00032..PIC32CM5164LE00100; "
				"LS e.g.: PIC32CM1216LS00032..PIC32CM5164LS60100; "
				"GV e.g.: PIC32CM1602GV00032..PIC32CM3204GV00064; "
				"JH e.g.: PIC32CM2532JH00032..PIC32CM5164JH01100; "
				"MC e.g.: PIC32CM1216MC00032..PIC32CM6408MC00048; "
				"SAMHA0 e.g.: ATSAMHA0E14AB..ATSAMHA0G17AB; "
				"SAMHA1 e.g.: ATSAMHA1G14A..ATSAMHA1G17AB; "
				"or another registered variant name)", variant);
			free(chip);
			return ERROR_FAIL;
		}
		LOG_INFO("at91samd: using forced variant '%s' (DID auto-probe bypassed)",
			variant);

		/* Optional 4 more arguments: a custom CEKEY for PIC32CM-LE/LS full
		 * chip-erase, for devices whose BOCOR was provisioned with a
		 * non-default key.  If not supplied, chip-erase uses the factory
		 * default (all 0xFFFFFFFF).  Ignored for families that don't use a
		 * CEKEY. */
		if (CMD_ARGC >= 12) {
			for (int i = 0; i < 4; i++) {
				int retval = parse_u32(CMD_ARGV[8 + i], &chip->cekey[i]);
				if (retval != ERROR_OK) {
					LOG_ERROR("at91samd: invalid CEKEY word '%s'", CMD_ARGV[8 + i]);
					free(chip);
					return retval;
				}
			}
			chip->cekey_set = true;
			LOG_INFO("at91samd: using custom CEKEY for chip-erase");
		}
	}

	bank->driver_priv = chip;

	return ERROR_OK;
}

/**
 * Find the samd_family for a target.
 *
 * First tries DID-based lookup (works for all classic SAMD/SAMC/SAML parts).
 * Falls back to the forced_family stored in the flash bank's driver_priv when
 * DID lookup fails or produces no match -- this handles PIC32CM-PL10 and other
 * parts whose DSU_DID does not use the classic PROCESSOR/FAMILY/SERIES encoding.
 */
static const struct samd_family *samd_get_family_for_target(struct target *target)
{
	uint32_t did = 0;
	if (target_read_u32(target, SAMD_DSU + SAMD_DSU_DID, &did) == ERROR_OK) {
		const struct samd_family *family = samd_find_family(did);
		if (family)
			return family;
	}

	/* DID lookup failed -- try forced_family from flash bank (PIC32CM-PL10 etc.) */
	for (struct flash_bank *b = flash_bank_list(); b; b = b->next) {
		if (b->target != target)
			continue;
		if (!b->driver || strcmp(b->driver->name, "at91samd") != 0)
			continue;
		const struct samd_info *chip = b->driver_priv;
		if (chip && chip->forced_family)
			return chip->forced_family;
	}

	return NULL;
}

COMMAND_HANDLER(samd_handle_chip_erase_command)
{
	struct target *target = get_current_target(CMD_CTX);
	int res = ERROR_FAIL;

	if (target) {
		const struct samd_family *family = samd_get_family_for_target(target);

		if (family && family->has_bootrom_dal) {
			/* BCC mailbox device: check if chip-erase is supported */
			if (!family->dsu_layout->chip_erase_supported) {
				command_print(CMD, "chip-erase not supported for this device");
				return ERROR_FAIL;
			}
			/* Dispatch to the correct BootROM chip-erase protocol.  All of
			 * these share the has_bootrom_dal/saml1x_dsu_layout machinery
			 * for reset-extension, but the actual erase command sequence
			 * differs per family (see the function doc comments). */
			if (family->parts == pic32cmpl_parts) {
				res = pic32cmpl_chip_erase(target);
			} else if (family->parts == pic32cmle_parts || family->parts == pic32cmls_parts) {
				static const uint32_t default_key[4] = {
					SAMD_LE_LS_CHIPERASE_DEFAULT_KEY, SAMD_LE_LS_CHIPERASE_DEFAULT_KEY,
					SAMD_LE_LS_CHIPERASE_DEFAULT_KEY, SAMD_LE_LS_CHIPERASE_DEFAULT_KEY,
				};
				const uint32_t *key = default_key;

				/* Use a custom CEKEY if one was supplied on the flash bank
				 * command line (see samd_flash_bank_command). */
				for (struct flash_bank *b = flash_bank_list(); b; b = b->next) {
					if (b->target != target || !b->driver ||
							strcmp(b->driver->name, "at91samd") != 0)
						continue;
					const struct samd_info *chip = b->driver_priv;
					if (chip && chip->cekey_set)
						key = chip->cekey;
					break;
				}

				res = samd_bcc_chip_erase_all(target, family->dsu_layout, key);
			} else {
				res = samdl1x_chip_erase(target);
			}

			if (res == ERROR_OK)
				command_print(CMD, "chip erase completed");
			else
				command_print(CMD, "chip erase failed");
			return res;
		}

		/* Legacy path (SAMD/SAMC/SAML21/SAML22):
		 * Enable access to the DSU by disabling the write protect bit */
		target_write_u32(target, SAMD_PAC1, (1<<1));
		/* intentionally without error checking - not accessible on secured chip */

		/* Tell the DSU to perform a full chip erase.  It takes about 240ms to
		 * perform the erase. */
		res = target_write_u8(target, SAMD_DSU + SAMD_DSU_CTRL_EXT, (1<<4));
		if (res == ERROR_OK)
			command_print(CMD, "chip erase started");
		else
			command_print(CMD, "write to DSU CTRL failed");
	}

	return res;
}

COMMAND_HANDLER(samd_handle_set_security_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC < 1 || (CMD_ARGC >= 1 && (strcmp(CMD_ARGV[0], "enable")))) {
		command_print(CMD, "supply the \"enable\" argument to proceed.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		res = samd_issue_nvmctrl_command(target, SAMD_NVM_CMD_SSB);

		/* Check (and clear) error conditions */
		if (res == ERROR_OK)
			command_print(CMD, "chip secured on next power-cycle");
		else
			command_print(CMD, "failed to secure chip");
	}

	return res;
}

COMMAND_HANDLER(samd_handle_eeprom_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		if (CMD_ARGC >= 1) {
			int val = atoi(CMD_ARGV[0]);
			uint32_t code;

			if (val == 0)
				code = 7;
			else {
				/* Try to match size in bytes with corresponding size code */
				for (code = 0; code <= 6; code++) {
					if (val == (2 << (13 - code)))
						break;
				}

				if (code > 6) {
					command_print(CMD, "Invalid EEPROM size.  Please see "
							"datasheet for a list valid sizes.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}

			res = samd_modify_user_row(target, code, 4, 6);
		} else {
			uint16_t val;
			res = target_read_u16(target, SAMD_USER_ROW, &val);
			if (res == ERROR_OK) {
				uint32_t size = ((val >> 4) & 0x7); /* grab size code */

				if (size == 0x7)
					command_print(CMD, "EEPROM is disabled");
				else {
					/* Otherwise, 6 is 256B, 0 is 16KB */
					command_print(CMD, "EEPROM size is %u bytes",
							(2 << (13 - size)));
				}
			}
		}
	}

	return res;
}

COMMAND_HANDLER(samd_handle_nvmuserrow_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (CMD_ARGC > 2) {
			command_print(CMD, "Too much Arguments given.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (CMD_ARGC > 0) {
			if (target->state != TARGET_HALTED) {
				LOG_ERROR("Target not halted.");
				return ERROR_TARGET_NOT_HALTED;
			}

			uint64_t mask;
			res = samd_get_reservedmask(target, &mask);
			if (res != ERROR_OK) {
				LOG_ERROR("Couldn't determine the mask for reserved bits.");
				return ERROR_FAIL;
			}
			mask &= NVMUSERROW_LOCKBIT_MASK;

			uint64_t value;
			COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], value);

			if (CMD_ARGC == 2) {
				uint64_t mask_temp;
				COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], mask_temp);

				mask &= mask_temp;
			}
			res = samd_modify_user_row_masked(target, value, mask);
			if (res != ERROR_OK)
				return res;
		}

		/* read register */
		uint64_t value;
		res = read_userrow(target, &value);
		if (res == ERROR_OK)
			command_print(CMD, "NVMUSERROW: 0x%016"PRIX64, value);
		else
			LOG_ERROR("NVMUSERROW could not be read.");
	}
	return res;
}

COMMAND_HANDLER(samd_handle_bootloader_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		/* Retrieve the MCU's page size, in bytes. */
		uint32_t page_size;
		res = samd_get_flash_page_info(target, &page_size, NULL);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't determine Flash page size");
			return res;
		}

		if (CMD_ARGC >= 1) {
			int val = atoi(CMD_ARGV[0]);
			uint32_t code;

			if (val == 0)
				code = 7;
			else {
				/* Try to match size in bytes with corresponding size code */
				for (code = 0; code <= 6; code++) {
					if ((unsigned int)val == (2UL << (8UL - code)) * page_size)
						break;
				}

				if (code > 6) {
					command_print(CMD, "Invalid bootloader size.  Please "
							"see datasheet for a list valid sizes.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

			}

			res = samd_modify_user_row(target, code, 0, 2);
		} else {
			uint16_t val;
			res = target_read_u16(target, SAMD_USER_ROW, &val);
			if (res == ERROR_OK) {
				uint32_t size = (val & 0x7); /* grab size code */
				uint32_t nb;

				if (size == 0x7)
					nb = 0;
				else
					nb = (2 << (8 - size)) * page_size;

				/* There are 4 pages per row */
				command_print(CMD, "Bootloader size is %" PRIu32 " bytes (%" PRIu32 " rows)",
					   nb, (uint32_t)(nb / (page_size * 4)));
			}
		}
	}

	return res;
}



COMMAND_HANDLER(samd_handle_reset_deassert)
{
	struct target *target = get_current_target(CMD_CTX);
	int retval = ERROR_OK;
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	/* If the target has been unresponsive before, try to re-establish
	 * communication now - CPU is held in reset by DSU, DAP is working */
	if (!target_was_examined(target))
		target_examine_one(target);
	target_poll(target);

	/* In case of sysresetreq, debug retains state set in cortex_m_assert_reset()
	 * so we just release reset held by DSU
	 *
	 * n_RESET (srst) clears the DP, so reenable debug and set vector catch here
	 *
	 * After vectreset DSU release is not needed however makes no harm
	 */
	if (target->reset_halt && (jtag_reset_config & RESET_HAS_SRST)) {
		retval = target_write_u32(target, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, DCB_DEMCR,
				TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
		/* do not return on error here, releasing DSU reset is more important */
	}

	/* Determine whether this target requires the full DSU boot-ROM BCC mailbox
	 * handshake to release reset extension (SAML10/L11, PIC32CM-PL10, ...).
	 * The BCC handshake is unconditional HW behaviour -- it is independent of
	 * the reset_halt / SRST configuration above. */
	const struct samd_family *family = samd_get_family_for_target(target);
	bool has_dal = (family != NULL && family->has_bootrom_dal);

	if (has_dal) {
		/* BCC mailbox device: perform handshake to exit reset extension */
		int retval2 = samd_bcc_reset_to_park(target, family->dsu_layout, NULL);
		if (retval2 != ERROR_OK)
			return retval2;

		if (family->needs_ram_xn_clear) {
			retval2 = samd_clear_ram_execute_never(target);
			if (retval2 != ERROR_OK)
				LOG_WARNING("at91samd: failed to clear IDAU_RXN in user row "
						"(RAM may still be execute-never)");
		}
	} else {
		/* Legacy SAMD/SAMC/SAML21/SAML22: W1C clear CPU Reset Phase Extension bit */
		int retval2 = target_write_u8(target,
				SAMD_DSU + SAMD_DSU_STATUSA, (1 << 1));
		if (retval2 != ERROR_OK)
			return retval2;
	}

	return retval;
}

static const struct command_registration at91samd_exec_command_handlers[] = {
	{
		.name = "dsu_reset_deassert",
		.handler = samd_handle_reset_deassert,
		.mode = COMMAND_EXEC,
		.help = "Deassert internal reset held by DSU.",
		.usage = "",
	},
	{
		.name = "chip-erase",
		.handler = samd_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase the entire Flash by using the Chip-"
			"Erase feature in the Device Service Unit (DSU).",
		.usage = "",
	},
	{
		.name = "set-security",
		.handler = samd_handle_set_security_command,
		.mode = COMMAND_EXEC,
		.help = "Secure the chip's Flash by setting the Security Bit. "
			"This makes it impossible to read the Flash contents. "
			"The only way to undo this is to issue the chip-erase "
			"command.",
		.usage = "'enable'",
	},
	{
		.name = "eeprom",
		.usage = "[size_in_bytes]",
		.handler = samd_handle_eeprom_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the EEPROM size setting, stored in the User Row. "
			"Please see Table 20-3 of the SAMD20 datasheet for allowed values. "
			"Changes are stored immediately but take affect after the MCU is "
			"reset.",
	},
	{
		.name = "bootloader",
		.usage = "[size_in_bytes]",
		.handler = samd_handle_bootloader_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the bootloader size, stored in the User Row. "
			"Please see Table 20-2 of the SAMD20 datasheet for allowed values. "
			"Changes are stored immediately but take affect after the MCU is "
			"reset.",
	},
	{
		.name = "nvmuserrow",
		.usage = "[value] [mask]",
		.handler = samd_handle_nvmuserrow_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the nvmuserrow register. It is 64 bit wide "
			"and located at address 0x804000. Use the optional mask argument "
			"to prevent changes at positions where the bitvalue is zero. "
			"For security reasons the lock- and reserved-bits are masked out "
			"in background and therefore cannot be changed.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration at91samd_command_handlers[] = {
	{
		.name = "at91samd",
		.mode = COMMAND_ANY,
		.help = "at91samd flash command group",
		.usage = "",
		.chain = at91samd_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver at91samd_flash = {
	.name = "at91samd",
	.commands = at91samd_command_handlers,
	.flash_bank_command = samd_flash_bank_command,
	.erase = samd_erase,
	.protect = samd_protect,
	.write = samd_write,
	.read = default_flash_read,
	.probe = samd_probe,
	.auto_probe = samd_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = samd_protect_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
