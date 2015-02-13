/*
 * Copyright (C) 2014, Broadcom Corporation. All Rights Reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <asm-generic/unaligned.h>

/***********************************************************************
 * Definitions
 ***********************************************************************/
#define DRV_NAME                        "iproc_nand"

/*
 * iProc NAND flash commands
 */
#define CMD_PAGE_READ                   0x01
#define CMD_SPARE_AREA_READ             0x02
#define CMD_STATUS_READ                 0x03
#define CMD_PROGRAM_PAGE                0x04
#define CMD_PROGRAM_SPARE_AREA          0x05
#define CMD_COPY_BACK                   0x06
#define CMD_DEVICE_ID_READ              0x07
#define CMD_BLOCK_ERASE                 0x08
#define CMD_FLASH_RESET                 0x09
#define CMD_BLOCKS_LOCK                 0x0a
#define CMD_BLOCKS_LOCK_DOWN            0x0b
#define CMD_BLOCKS_UNLOCK               0x0c
#define CMD_READ_BLOCKS_LOCK_STATUS     0x0d
#define CMD_PARAMETER_READ              0x0e
#define CMD_PARAMETER_CHANGE_COL        0x0f
#define CMD_LOW_LEVEL_OP                0x10
#define CMD_PAGE_READ_MULTI		0x11
#define CMD_STATUS_READ_MULTI		0x12
#define CMD_PROGRAM_PAGE_MULTI		0x13
#define CMD_PROGRAM_PAGE_MULTI_CACHE	0x14
#define CMD_BLOCK_ERASE_MULTI		0x15

/*
 * NAND controller register offset
 */
#define NCREG_REVISION                 0x000	/* Revision */
#define NCREG_CMD_START                0x004	/* Command Start */
#define NCREG_CMD_EXT_ADDRESS          0x008	/* Command Extended Address */
#define NCREG_CMD_ADDRESS              0x00c	/* Command Address */
#define NCREG_CMD_END_ADDRESS          0x010	/* Command End Address */
#define NCREG_INTFC_STATUS             0x014	/* Interface Status */
#define NCREG_CS_NAND_SELECT           0x018	/* EBI CS Select */
#define NCREG_CS_NAND_XOR              0x01c	/* EBI CS Address XOR with
						   1FC0 Control */
#define NCREG_LL_OP                    0x020	/* Low Level Operation */
#define NCREG_MPLANE_BASE_EXT_ADDRESS  0x024	/* Multiplane base extended
						   address */
#define NCREG_MPLANE_BASE_ADDRESS      0x028	/* Multiplane base address */
#define NCREG_ACC_CONTROL_CS0          0x050	/* Access Control CS0 */
#define NCREG_CONFIG_CS0               0x054	/* Config CS0 */
#define NCREG_TIMING_1_CS0             0x058	/* Timing Parameters 1 CS0 */
#define NCREG_TIMING_2_CS0             0x05c	/* Timing Parameters 2 CS0 */
#define NCREG_ACC_CONTROL_CS1          0x060	/* Access Control CS1 */
#define NCREG_CONFIG_CS1               0x064	/* Config CS1 */
#define NCREG_TIMING_1_CS1             0x068	/* Timing Parameters 1 CS1 */
#define NCREG_TIMING_2_CS1             0x06c	/* Timing Parameters 2 CS1 */
#define NCREG_CORR_STAT_THRESHOLD      0x0c0	/* Correctable Error Reporting
						   Threshold */
#define NCREG_BLK_WR_PROTECT           0x0c8	/* Block Write Protect Enable
						   and Size for EBI_CS0b */
#define NCREG_MULTIPLANE_OPCODES_1     0x0cc	/* Multiplane Customized
						   Opcodes */
#define NCREG_MULTIPLANE_OPCODES_2     0x0d0	/* Multiplane Customized
						   Opcodes */
#define NCREG_MULTIPLANE_CTRL          0x0d4	/* Multiplane Control */
#define NCREG_UNCORR_ERROR_COUNT       0x0fc	/* Total Uncorrectable Error
						   Count */
#define NCREG_CORR_ERROR_COUNT         0x100	/* Correctable Error Count */
#define NCREG_READ_ERROR_COUNT         0x104	/* Total Correctable Error
						   Count */
#define NCREG_BLOCK_LOCK_STATUS        0x108	/* Block Lock Status */
#define NCREG_ECC_CORR_EXT_ADDR        0x10c	/* ECC Correctable Error
						   Extended Address */
#define NCREG_ECC_CORR_ADDR            0x110	/* ECC Correctable Error
						   Address */
#define NCREG_ECC_UNC_EXT_ADDR         0x114	/* ECC Uncorrectable Error
						   Extended Address */
#define NCREG_ECC_UNC_ADDR             0x118	/* ECC Uncorrectable Error
						   Address */
#define NCREG_FLASH_READ_EXT_ADDR      0x11c	/* Read Data Extended Address */
#define NCREG_FLASH_READ_ADDR          0x120	/* Read Data Address */
#define NCREG_PROGRAM_PAGE_EXT_ADDR    0x124	/* Page Program Extended
						   Address */
#define NCREG_PROGRAM_PAGE_ADDR        0x128	/* Page Program Address */
#define NCREG_COPY_BACK_EXT_ADDR       0x12c	/* Copy Back Extended Address */
#define NCREG_COPY_BACK_ADDR           0x130	/* Copy Back Address */
#define NCREG_BLOCK_ERASE_EXT_ADDR     0x134	/* Block Erase Extended
						   Address */
#define NCREG_BLOCK_ERASE_ADDR         0x138	/* Block Erase Address */
#define NCREG_INV_READ_EXT_ADDR        0x13c	/* Invalid Data Extended
						   Address */
#define NCREG_INV_READ_ADDR            0x140	/* Invalid Data Address */
#define NCREG_INIT_STATUS              0x144	/* Initialization status */
#define NCREG_ONFI_STATUS              0x148	/* ONFI Status */
#define NCREG_ONFI_DEBUG_DATA          0x14c	/* ONFI Debug Data */
#define NCREG_SEMAPHORE                0x150	/* Semaphore */
#define NCREG_FLASH_DEVICE_ID          0x194	/* Device ID */
#define NCREG_FLASH_DEVICE_ID_EXT      0x198	/* Extended Device ID */
#define NCREG_LL_RDDATA                0x19c	/* Low Level Read Data */
#define NCREG_SPARE_AREA_READ_OFS_0    0x200	/* Spare Area Read Bytes */
#define NCREG_SPARE_AREA_WRITE_OFS_0   0x280	/* Spare Area Write Bytes */
#define NCREG_FLASH_CACHE_BASE         0x400	/* Cache Buffer Access */
#define NCREG_INTERRUPT_BASE           0xf00	/* Interrupt Base Address */

/*
 * Required NAND controller register fields
 */
#define NCFLD_CMD_START_OPCODE_SHIFT                            24
#define NCFLD_INTFC_STATUS_FLASH_STATUS_MASK                    0x000000FF
#define NCFLD_CS_NAND_SELECT_AUTO_DEVID_CONFIG                  0x40000000
#define NCFLD_CS_NAND_SELECT_WP                                 0x20000000
#define NCFLD_CS_NAND_SELECT_DIRECT_ACCESS_CS_MASK              0x000000FF
#define NCFLD_CS_NAND_XOR_CS_MASK                               0x000000FF
#define NCFLD_CONFIG_CS0_BLOCK_SIZE_MASK                        0x70000000
#define NCFLD_CONFIG_CS0_BLOCK_SIZE_SHIFT                       28
#define NCFLD_CONFIG_CS0_DEVICE_SIZE_MASK                       0x0f000000
#define NCFLD_CONFIG_CS0_DEVICE_SIZE_SHIFT                      24
#define NCFLD_CONFIG_CS0_DEVICE_WIDTH_MASK                      0x00800000
#define NCFLD_CONFIG_CS0_DEVICE_WIDTH_SHIFT                     23
#define NCFLD_CONFIG_CS0_PAGE_SIZE_MASK                         0x00300000
#define NCFLD_CONFIG_CS0_PAGE_SIZE_SHIFT                        20
#define NCFLD_CONFIG_CS0_FUL_ADR_BYTES_MASK                     0x00070000
#define NCFLD_CONFIG_CS0_FUL_ADR_BYTES_SHIFT                    16
#define NCFLD_CONFIG_CS0_COL_ADR_BYTES_MASK                     0x00007000
#define NCFLD_CONFIG_CS0_COL_ADR_BYTES_SHIFT                    12
#define NCFLD_CONFIG_CS0_BLK_ADR_BYTES_MASK                     0x00000700
#define NCFLD_CONFIG_CS0_BLK_ADR_BYTES_SHIFT                    8
#define NCFLD_ACC_CONTROL_CS0_RD_ECC_EN_MASK                    0x80000000
#define NCFLD_ACC_CONTROL_CS0_RD_ECC_EN_SHIFT                   31
#define NCFLD_ACC_CONTROL_CS0_WR_ECC_EN_MASK                    0x40000000
#define NCFLD_ACC_CONTROL_CS0_WR_ECC_EN_SHIFT                   30
#define NCFLD_ACC_CONTROL_CS0_FAST_PGM_RDIN_MASK                0x10000000
#define NCFLD_ACC_CONTROL_CS0_FAST_PGM_RDIN_SHIFT               28
#define NCFLD_ACC_CONTROL_CS0_RD_ERASED_ECC_EN_MASK             0x08000000
#define NCFLD_ACC_CONTROL_CS0_RD_ERASED_ECC_EN_SHIFT            27
#define NCFLD_ACC_CONTROL_CS0_PARTIAL_PAGE_EN_MASK              0x04000000
#define NCFLD_ACC_CONTROL_CS0_PARTIAL_PAGE_EN_SHIFT             26
#define NCFLD_ACC_CONTROL_CS0_PAGE_HIT_EN_MASK                  0x01000000
#define NCFLD_ACC_CONTROL_CS0_PAGE_HIT_EN_SHIFT                 24
#define NCFLD_ACC_CONTROL_CS0_ECC_LEVEL_MASK                    0x001f0000
#define NCFLD_ACC_CONTROL_CS0_ECC_LEVEL_SHIFT                   16
#define NCFLD_ACC_CONTROL_CS0_SECTOR_SIZE_1K_MASK               0x00000080
#define NCFLD_ACC_CONTROL_CS0_SECTOR_SIZE_1K_SHIFT              7
#define NCFLD_ACC_CONTROL_CS0_SPARE_AREA_SIZE_MASK              0x0000007f
#define NCFLD_ACC_CONTROL_CS0_SPARE_AREA_SIZE_SHIFT             0
#define NCFLD_CORR_STAT_THRESHOLD_CS0_MASK                      0x0000003f
#define NCFLD_CORR_STAT_THRESHOLD_CS0_SHIFT                     0
#define NCFLD_CORR_STAT_THRESHOLD_CS1_MASK                      0x00000fc0
#define NCFLD_CORR_STAT_THRESHOLD_CS1_SHIFT                     6
#define NCFLD_INIT_STATUS_INIT_SUCCESS				0x20000000

/*
 * IDM NAND register offset
 */
#define IDMREG_IO_CONTROL_DIRECT				0x408
#define IDMREG_IO_STATUS					0x500
#define IDMREG_RESET_CONTROL					0x800

/*
 * Required IDM NAND IO Control register fields
 */
#define IDMFLD_NAND_IO_CONTROL_DIRECT_AXI_BE_MODE              (1UL << 28)
#define IDMFLD_NAND_IO_CONTROL_DIRECT_APB_LE_MODE              (1UL << 24)
#define IDMFLD_NAND_IO_CONTROL_DIRECT_IRQ_SHIFT                2

/*
 * Interrupts
 */
#define NCINTR_NP_READ                                          0
#define NCINTR_BLKERA                                           1
#define NCINTR_CPYBK                                            2
#define NCINTR_PGMPG                                            3
#define NCINTR_CTLRDY                                           4
#define NCINTR_RBPIN                                            5
#define NCINTR_UNC                                              6
#define NCINTR_CORR                                             7

/* 512B flash cache in the NAND controller HW */
#define FC_SHIFT            9U
#define FC_BYTES            512U
#define FC_WORDS            (FC_BYTES >> 2)
#define FC(x)               (NCREG_FLASH_CACHE_BASE + ((x) << 2))

/* 64B oob cache in the NAND controller HW */
#define MAX_CONTROLLER_OOB_BYTES        64
#define MAX_CONTROLLER_OOB_WORDS        (MAX_CONTROLLER_OOB_BYTES >> 2)

/* Default ECC correction status threshold percentage: 60% of ECC strength */
#define DEFAULT_CORR_STATUS_THRESHOLD_PERCENT	60

/*
 * Register access macros - NAND flash controller
 */
#define NAND_REG_RD(x)		readl(ctrl.nand_regs + (x))
#define NAND_REG_WR(x, y)	writel((y), ctrl.nand_regs + (x))
#define NAND_REG_CLR(x, y)	NAND_REG_WR((x), NAND_REG_RD(x) & ~(y))
#define NAND_REG_SET(x, y)	NAND_REG_WR((x), NAND_REG_RD(x) | (y))

/*
 * IRQ operations
 */
#define NAND_ENABLE_IRQ(bit)						     \
	writel(readl(ctrl.nand_idm_io_ctrl_direct_reg) |		     \
	       (1UL << ((bit) + IDMFLD_NAND_IO_CONTROL_DIRECT_IRQ_SHIFT)),   \
	       ctrl.nand_idm_io_ctrl_direct_reg)

#define NAND_DISABLE_IRQ(bit)						     \
	writel(readl(ctrl.nand_idm_io_ctrl_direct_reg) &		     \
	       ~(1UL << ((bit) + IDMFLD_NAND_IO_CONTROL_DIRECT_IRQ_SHIFT)),  \
	       ctrl.nand_idm_io_ctrl_direct_reg)

#define NAND_ACK_IRQ(bit) writel(1, ((u32 *)ctrl.nand_intr_regs) + (bit))

#define NAND_TEST_IRQ(bit) (readl(((u32 *)ctrl.nand_intr_regs) + (bit)) & 1)

/*
 * Data access macros for endianness
 */
#ifdef __LITTLE_ENDIAN
#define NAND_BEGIN_DATA_ACCESS()					\
	writel(readl(ctrl.nand_idm_io_ctrl_direct_reg) |		\
	       IDMFLD_NAND_IO_CONTROL_DIRECT_APB_LE_MODE,		\
	       ctrl.nand_idm_io_ctrl_direct_reg)
#define NAND_END_DATA_ACCESS()						\
	writel(readl(ctrl.nand_idm_io_ctrl_direct_reg) &		\
	       ~IDMFLD_NAND_IO_CONTROL_DIRECT_APB_LE_MODE,		\
	       ctrl.nand_idm_io_ctrl_direct_reg)
#else				/* !__LITTLE_ENDIAN */
#define NAND_BEGIN_DATA_ACCESS()
#define NAND_END_DATA_ACCESS()
#endif				/* !__LITTLE_ENDIAN */

/*
 * Misc NAND controller configuration/status macros
 */
#define NC_REG_CONFIG(cs) (NCREG_CONFIG_CS0 + ((cs) << 4))

#define WR_CONFIG(cs, field, val) do {					\
	u32 reg = NC_REG_CONFIG(cs), contents = NAND_REG_RD(reg);	\
	contents &= ~(NCFLD_CONFIG_CS0_##field##_MASK);			\
	contents |= (val) << NCFLD_CONFIG_CS0_##field##_SHIFT;		\
	NAND_REG_WR(reg, contents);					\
} while (0)

#define RD_CONFIG(cs, field)						    \
	((NAND_REG_RD(NC_REG_CONFIG(cs)) & NCFLD_CONFIG_CS0_##field##_MASK) \
	>> NCFLD_CONFIG_CS0_##field##_SHIFT)

#define NC_REG_ACC_CONTROL(cs) (NCREG_ACC_CONTROL_CS0 + ((cs) << 4))

#define WR_ACC_CONTROL(cs, field, val) do {				\
	u32 reg = NC_REG_ACC_CONTROL(cs), contents = NAND_REG_RD(reg);	\
	contents &= ~(NCFLD_ACC_CONTROL_CS0_##field##_MASK);		\
	contents |= (val) << NCFLD_ACC_CONTROL_CS0_##field##_SHIFT;	\
	NAND_REG_WR(reg, contents);					\
} while (0)

#define RD_ACC_CONTROL(cs, field)					\
	((NAND_REG_RD(NC_REG_ACC_CONTROL(cs)) &				\
	 NCFLD_ACC_CONTROL_CS0_##field##_MASK)				\
	 >> NCFLD_ACC_CONTROL_CS0_##field##_SHIFT)

#define CORR_ERROR_COUNT (NAND_REG_RD(NCREG_CORR_ERROR_COUNT))
#define UNCORR_ERROR_COUNT (NAND_REG_RD(NCREG_UNCORR_ERROR_COUNT))

#define WR_CORR_THRESH(cs, val) do {					   \
	u32 contents = NAND_REG_RD(NCREG_CORR_STAT_THRESHOLD);		   \
	u32 shift = NCFLD_CORR_STAT_THRESHOLD_CS1_SHIFT * (cs);		   \
	contents &= ~(NCFLD_CORR_STAT_THRESHOLD_CS0_MASK << shift);	   \
	contents |= ((val) & NCFLD_CORR_STAT_THRESHOLD_CS0_MASK) << shift; \
	NAND_REG_WR(NCREG_CORR_STAT_THRESHOLD, contents);		   \
} while (0)

#define NC_REG_TIMING1(cs) (NCREG_TIMING_1_CS0 + ((cs) << 4))
#define NC_REG_TIMING2(cs) (NCREG_TIMING_2_CS0 + ((cs) << 4))

#define NAND_STRAP_TYPE							     \
	((readl(ctrl.nand_strap_regs) & ctrl.data->strap_type_bitfield.mask) \
	>> ctrl.data->strap_type_bitfield.shift)
#define NAND_STRAP_PAGE							     \
	((readl(ctrl.nand_strap_regs) & ctrl.data->strap_page_bitfield.mask) \
	>> ctrl.data->strap_page_bitfield.shift)

/*
 * Internal structures
 */

struct nand_strap_type {
	uint8_t sector_1k;
	uint8_t ecc_level;
	uint16_t spare_size;
};

struct nand_strap_bitfield {
	uint32_t mask;
	uint32_t shift;
};

#define ONFI_TIMING_MODES	6
struct nand_timing {
	u32 timing1;
	u32 timing2;
};

struct nand_ctrl_data {
	uint32_t chip_select_max;
	struct nand_strap_bitfield strap_type_bitfield;
	struct nand_strap_bitfield strap_page_bitfield;
	struct nand_strap_bitfield strap_width_bitfield;
	struct nand_strap_type strap_types[16];
	uint32_t strap_page_sizes[4];
	struct nand_timing onfi_tmode[ONFI_TIMING_MODES];
};

/*
 * This flag controls if WP stays on between erase/write
 * commands to mitigate flash corruption due to power glitches.
 * Values:
 * WP_NOT_USED: WP is not used or not available
 * WP_SET_BY_DEFAULT: WP is set by default, cleared for erase/write operations
 * WP_ALWAYS_CLEARED: WP is always cleared
 */
enum iproc_nand_wp_mode {
	WP_NOT_USED = 0,
	WP_SET_BY_DEFAULT = 1,
	WP_ALWAYS_CLEARED = 2,
};

struct iproc_nand_controller {
	struct nand_hw_control controller;
	int irq;
	int cmd_pending;
	struct completion done;
	int boot_inited;
	int hw_auto_init;
	/*
	 * This flag indicates that existing data cache should not be used.
	 * When PAGE_HIT is enabled and a read operation is performed from
	 * the same address, the controller is not reading from the NAND,
	 * considering that data is already available in the controller data
	 * cache. In that case the correctable or uncorrectable interrupts
	 * are not asserted, so the read would appear successful.
	 * This flag will be used to temporary disable PAGE_HIT to force the
	 * data to be read again form the NAND and have correct ECC results.
	 */
	int data_cache_invalid;

	/*
	 * The percentage of the BCH ECC correction capability
	 * from which correctable errors will be reported.
	 */
	int corr_threshold_percent;

	void *nand_regs;
	void *nand_intr_regs;
	void *nand_idm_regs;
	void *nand_idm_io_ctrl_direct_reg;
	void *nand_strap_regs;

	int strap_type;
	int strap_page_size;

	unsigned int max_cs;

	/*
	 * This flag controls NAND interface timing.
	 * Values:
	 * -1:    use current timing register values
	 * [0-5]: change timing register values to comply with ONFI
	 *        timing mode [0-5]
	 */
	int tmode;

	enum iproc_nand_wp_mode wp_mode;

	struct nand_ctrl_data *data;
};

enum iproc_nand_ecc_code {
	ECC_CODE_BCH,
	ECC_CODE_HAMMING,
};

struct iproc_nand_cfg {
	u64 device_size;
	unsigned int block_size;
	unsigned int page_size;
	unsigned int spare_area_size;
	unsigned int device_width;
	unsigned int col_adr_bytes;
	unsigned int blk_adr_bytes;
	unsigned int ful_adr_bytes;
	unsigned int sector_size_1k;
	unsigned int ecc_level;
	enum iproc_nand_ecc_code ecc_code;
};

struct iproc_nand_host {
	u32 buf[FC_WORDS];
	struct nand_chip chip;
	struct mtd_info mtd;
	struct platform_device *pdev;
	int cs;
	unsigned int last_cmd;
	unsigned int last_byte;
	u64 last_addr;
	struct iproc_nand_cfg hwcfg;
};

static struct nand_ecclayout iproc_nand_oob_layout;

/*
 * Static function prototypes
 */
static void iproc_nand_ctrl_release(void);
static int iproc_nand_ctrl_setup(struct platform_device *pdev);

/*
 * Global variables
 */

static struct iproc_nand_controller ctrl;

/* Maximum BCH ECC level for 512B and 1024B sectors */
static const uint8_t iproc_max_bch_ecc_level[2] = { 17, 20 };

/* BCH ECC bytes required per 512B */
static const uint8_t iproc_bch_ecc_bytes[] = {
	0, 2, 4, 6, 7, 9, 11, 13, 14, 16, 18, 20, 21, 23, 25,
	27, 28, 30, 32, 34, 35
};

#if defined(CONFIG_ARCH_BCM_CYGNUS)
/* Cygnus specific data */
static const struct nand_ctrl_data cygnus_nand_ctrl_data = {
	.chip_select_max = 2,
	.strap_type_bitfield = {
		.mask = 0x000f0000,
		.shift = 16,
	},
	.strap_page_bitfield = {
		.mask = 0x00300000,
		.shift = 20,
	},
	.strap_width_bitfield = {
		.mask = 0x01000000,
		.shift = 24,
	},
	.strap_types = {
		/* sector_1k, ecc_level, spare_size */
		{ 0,  0, 16 },
		{ 0,  1, 16 },
		{ 0,  4, 16 },
		{ 0,  8, 16 },
		{ 0,  8, 27 },
		{ 0, 12, 27 },
		{ 1, 12, 27 },
		{ 1, 15, 27 },
		{ 1, 20, 45 },
	},
	.strap_page_sizes = {1024, 2048, 4096, 8192},
	/*
	 * iProc NAND timing configurations for ONFI timing modes [0-5]
	 *
	 * Clock tick = 10ns
	 * Multiplier:
	 * x1: tWP tWH tRP tREH tCLH tALH
	 * x2: tCS tADL tWB tWHR
	 */
	.onfi_tmode = {
		/*
		 * ONFI timing mode 0 :
		 * tWC=100ns tWP=50ns tWH=30ns
		 * tRC=100ns tRP=50ns tREH=30ns
		 * tCS=70ns tCLH=20ns tALH=20ns tADL=200ns
		 * tWB=200ns tWHR=120ns tREA=40ns
		 */
		{
			.timing1 = 0x6565435b,
			.timing2 = 0x00001e85,
		},
		/*
		 * ONFI timing mode 1 :
		 * tWC=45 tWP=25ns tWH=15ns
		 * tRC=50 tRP=25ns tREH=15ns
		 * tCS=35ns tCLH=10ns tALH=10ns tADL=100ns
		 * tWB=100ns tWHR=80ns tREA=30ns
		 */
		{
			.timing1 = 0x33333236,
			.timing2 = 0x00001064,
		},
		/*
		 * ONFI timing mode 2 :
		 * tWC=35ns tWP=17ns tWH=15ns
		 * tRC=35ns tRP=17ns tREH=15ns
		 * tCS=25ns tCLH=10ns tALH=10ns tADL=100ns
		 * tWB=100ns tWHR=80ns tREA=25ns
		 */
		{
			.timing1 = 0x32322226,
			.timing2 = 0x00001063,
		},
		/*
		 * ONFI timing mode 3 :
		 * tWC=30ns tWP=15ns tWH=10ns
		 * tRC=30ns tRP=15ns tREH=10ns
		 * tCS=25ns tCLH=5ns tALH=5ns tADL=100ns
		 * tWB=100ns tWHR=60ns tREA=20ns
		 */
		{
			.timing1 = 0x22222126,
			.timing2 = 0x00001043,
		},
		/*
		 * ONFI timing mode 4 :
		 * tWC=25ns tWP=12ns tWH=10ns
		 * tRC=25ns tRP=12ns tREH=10ns
		 * tCS=20ns tCLH=5ns tALH=5ns tADL=70ns
		 * tWB=100ns tWHR=60ns tREA=20ns
		 */
		{
			.timing1 = 0x21212114,
			.timing2 = 0x00001042,
		},
		/*
		 * ONFI timing mode 5 :
		 * tWC=20ns tWP=10ns tWH=7ns
		 * tRC=20ns tRP=10ns tREH=7ns
		 * tCS=15ns tCLH=5ns tALH=5ns tADL=70ns
		 * tWB=100ns tWHR=60ns tREA=16ns
		 */
		{
			.timing1 = 0x11111114,
			.timing2 = 0x00001042,
		},
	},
};
#endif

#if defined(CONFIG_ARCH_BCM_NSP)
/* Northstar-plus specific data */
static const struct nand_ctrl_data nsp_nand_ctrl_data = {
	.chip_select_max = 1,
	.strap_type_bitfield = {
		.mask = 0x0000f000,
		.shift = 12,
	},
	.strap_page_bitfield = {
		.mask = 0x00000c00,
		.shift = 10,
	},
	.strap_types = {
		/* sector_1k, ecc_level, spare_size */
		{ 0,  0, 16 },
		{ 0, 15, 16 }, /* Hamming ECC */
		{ 0,  4, 16 },
		{ 0,  8, 16 },
		{ 0,  8, 27 },
		{ 0, 12, 27 },
		{ 1, 12, 27 },
		{ 1, 15, 27 },
		{ 1, 20, 45 },
	},
	.strap_page_sizes = {2048, 2048, 4096, 8192},
	/*
	 * iProc NAND timing configurations for ONFI timing modes [0-5]
	 *
	 * Clock tick = 4ns
	 * Multiplier:
	 * x1: tWP tWH tRP tREH tCLH tALH
	 * x4: tCS tADL tWB tWHR
	 */
	.onfi_tmode = {
		/*
		 * ONFI timing mode 0 :
		 * tWC=100ns tWP=50ns tWH=30ns
		 * tRC=100ns tRP=50ns tREH=30ns
		 * tCS=70ns tCLH=20ns tALH=20ns tADL=200ns
		 * tWB=200ns tWHR=120ns tREA=40ns
		 */
		{
			.timing1 = 0xfafa558d,
			.timing2 = 0x00001a85,
		},
		/*
		 * ONFI timing mode 1 :
		 * tWC=45 tWP=25ns tWH=15ns
		 * tRC=50 tRP=25ns tREH=15ns
		 * tCS=35ns tCLH=10ns tALH=10ns tADL=100ns
		 * tWB=100ns tWHR=80ns tREA=30ns
		 */
		{
			.timing1 = 0x85853347,
			.timing2 = 0x00000e64,
		},
		/*
		 * ONFI timing mode 2 :
		 * tWC=35ns tWP=17ns tWH=15ns
		 * tRC=35ns tRP=17ns tREH=15ns
		 * tCS=25ns tCLH=10ns tALH=10ns tADL=100ns
		 * tWB=100ns tWHR=80ns tREA=25ns
		 */
		{
			.timing1 = 0x54542347,
			.timing2 = 0x00000e63,
		},
		/*
		 * ONFI timing mode 3 :
		 * tWC=30ns tWP=15ns tWH=10ns
		 * tRC=30ns tRP=15ns tREH=10ns
		 * tCS=25ns tCLH=5ns tALH=5ns tADL=100ns
		 * tWB=100ns tWHR=60ns tREA=20ns
		 */
		{
			.timing1 = 0x44442237,
			.timing2 = 0x00000e43,
		},
		/*
		 * ONFI timing mode 4 :
		 * tWC=25ns tWP=12ns tWH=10ns
		 * tRC=25ns tRP=12ns tREH=10ns
		 * tCS=20ns tCLH=5ns tALH=5ns tADL=70ns
		 * tWB=100ns tWHR=60ns tREA=20ns
		 */
		{
			.timing1 = 0x43432235,
			.timing2 = 0x00000e42,
		},
		/*
		 * ONFI timing mode 5 :
		 * tWC=20ns tWP=10ns tWH=7ns
		 * tRC=20ns tRP=10ns tREH=7ns
		 * tCS=15ns tCLH=5ns tALH=5ns tADL=70ns
		 * tWB=100ns tWHR=60ns tREA=16ns
		 */
		{
			.timing1 = 0x32321225,
			.timing2 = 0x00000e42,
		},
	},
};
#endif

/***********************************************************************
 * Internal support functions
 ***********************************************************************/

static void iproc_nand_wp(struct mtd_info *mtd, int wp)
{
	if (ctrl.wp_mode == WP_SET_BY_DEFAULT) {
		static int old_wp = -1;
		if (old_wp != wp) {
			pr_debug("%s: WP %s\n", __func__, wp ? "on" : "off");
			old_wp = wp;
		}
		if (wp) {
			NAND_REG_SET(NCREG_CS_NAND_SELECT,
				     NCFLD_CS_NAND_SELECT_WP);
		} else {
			NAND_REG_CLR(NCREG_CS_NAND_SELECT,
				     NCFLD_CS_NAND_SELECT_WP);
		}
	}
}

/* Helper functions for reading and writing OOB registers */
static inline unsigned char oob_reg_read(int offs)
{
	if (offs >= MAX_CONTROLLER_OOB_BYTES)
		return 0x77;

	/* APB read is big endian */
	return NAND_REG_RD(NCREG_SPARE_AREA_READ_OFS_0 + (offs & ~0x03))
	    >> (24 - ((offs & 0x03) << 3));
}

static irqreturn_t iproc_nand_irq(int irq, void *data)
{
	if (NAND_TEST_IRQ(NCINTR_CTLRDY)) {
		NAND_ACK_IRQ(NCINTR_CTLRDY);
		if (ctrl.cmd_pending) {
			/*
			 * If the direct access region (eg. 0x1c000000 on NS)
			 * is accessed, IRQ handler will also be called with
			 * NCINTR_CTLRDY asserted.
			 * Thus we need to filter these events by
			 * ctrl.cmd_pending, or ctrl.done will be mistakenly
			 * set and cause incorrect result for the following
			 * command.
			 * We actually should avoid direct access to the
			 * mapped region when NAND driver is running.
			 */
			complete(&ctrl.done);
		}
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static void iproc_nand_send_cmd(int cmd)
{
	pr_debug("%s: native cmd %d addr_lo 0x%lx\n", __func__, cmd,
		 (unsigned long)NAND_REG_RD(NCREG_CMD_ADDRESS));
	BUG_ON(ctrl.cmd_pending != 0);
	ctrl.cmd_pending = cmd;
	mb();
	NAND_REG_WR(NCREG_CMD_START, cmd << NCFLD_CMD_START_OPCODE_SHIFT);
}

/***********************************************************************
 * NAND MTD API: read/program/erase
 ***********************************************************************/

static void
iproc_nand_cmd_ctrl(struct mtd_info *mtd, int dat, unsigned int ctrl)
{
	/* intentionally left blank */
}

static int iproc_nand_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct nand_chip *chip = mtd->priv;
	struct iproc_nand_host *host = chip->priv;

	pr_debug("%s: native cmd %d\n", __func__, ctrl.cmd_pending);
	if (ctrl.cmd_pending &&
	    wait_for_completion_timeout(&ctrl.done, HZ / 10) <= 0) {
		dev_err(&host->pdev->dev,
			"timeout waiting for command %u (%ld)\n",
			host->last_cmd,
			(unsigned long)NAND_REG_RD(NCREG_CMD_START) >> 24);
		dev_err(&host->pdev->dev,
			"irq status %08lx, intfc status %08lx\n",
			(unsigned long)NAND_TEST_IRQ(NCINTR_CTLRDY),
			(unsigned long)NAND_REG_RD(NCREG_INTFC_STATUS));
	}
	ctrl.cmd_pending = 0;
	return NAND_REG_RD(NCREG_INTFC_STATUS) &
	    NCFLD_INTFC_STATUS_FLASH_STATUS_MASK;
}

static void
iproc_nand_cmdfunc(struct mtd_info *mtd, unsigned command,
		   int column, int page_addr)
{
	struct nand_chip *chip = mtd->priv;
	struct iproc_nand_host *host = chip->priv;
	u64 addr = (u64) page_addr << chip->page_shift;
	int native_cmd = 0;

	if (command == NAND_CMD_READID || command == NAND_CMD_PARAM)
		addr = (u64) column;

	pr_debug("%s: cmd 0x%x addr 0x%llx\n", __func__, command,
		 (unsigned long long)addr);
	host->last_cmd = command;
	host->last_byte = 0;
	host->last_addr = addr;

	switch (command) {
	case NAND_CMD_RESET:
		native_cmd = CMD_FLASH_RESET;
		break;
	case NAND_CMD_STATUS:
		native_cmd = CMD_STATUS_READ;
		break;
	case NAND_CMD_READID:
		native_cmd = CMD_DEVICE_ID_READ;
		break;
	case NAND_CMD_READOOB:
		native_cmd = CMD_SPARE_AREA_READ;
		break;
	case NAND_CMD_ERASE1:
		native_cmd = CMD_BLOCK_ERASE;
		iproc_nand_wp(mtd, 0);
		break;
	case NAND_CMD_PARAM:
		native_cmd = CMD_PARAMETER_READ;
		break;
	default:
		return;
	}

	NAND_REG_WR(NCREG_CMD_EXT_ADDRESS,
		    (host->cs << 16) | ((addr >> 32) & 0xffff));
	NAND_REG_WR(NCREG_CMD_ADDRESS, addr & 0xffffffff);

	iproc_nand_send_cmd(native_cmd);
	iproc_nand_waitfunc(mtd, chip);

	if (command == NAND_CMD_ERASE1)
		iproc_nand_wp(mtd, 1);
}

static void iproc_nand_select_chip(struct mtd_info *mtd, int cs)
{
	struct nand_chip *chip = mtd->priv;
	struct iproc_nand_host *host = chip->priv;

	if (cs < ctrl.max_cs) {
		pr_debug("%s: cs %d\n", __func__, cs);
		host->cs = cs;
	} else {
		dev_warn(&host->pdev->dev, "invalid cs %d ignored\n", cs);
	}
}

static uint8_t iproc_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd->priv;
	struct iproc_nand_host *host = chip->priv;
	uint8_t ret = 0;

	switch (host->last_cmd) {
	case NAND_CMD_READID:
		if (host->last_byte < 4)
			ret = NAND_REG_RD(NCREG_FLASH_DEVICE_ID) >>
			    (24 - (host->last_byte << 3));
		else if (host->last_byte < 8)
			ret = NAND_REG_RD(NCREG_FLASH_DEVICE_ID_EXT) >>
			    (56 - (host->last_byte << 3));
		break;

	case NAND_CMD_READOOB:
		ret = oob_reg_read(host->last_byte);
		break;

	case NAND_CMD_STATUS:
		ret = NAND_REG_RD(NCREG_INTFC_STATUS) &
		    NCFLD_INTFC_STATUS_FLASH_STATUS_MASK;
		if (ctrl.wp_mode == WP_SET_BY_DEFAULT) {
			/* Hide WP status from MTD */
			ret |= NAND_STATUS_WP;
		}
		break;

	case NAND_CMD_PARAM:
		if (host->last_byte < FC_BYTES)
			ret = NAND_REG_RD(FC(host->last_byte >> 2)) >>
			    (24 - ((host->last_byte & 0x03) << 3));
		break;
	}

	pr_debug("%s: byte = 0x%02x\n", __func__, ret);
	host->last_byte++;

	return ret;
}

static void iproc_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;

	for (i = 0; i < len; i++, buf++)
		*buf = iproc_nand_read_byte(mtd);
}

static void iproc_nand_cache_read(u32 **buf, u8 **oob, int oob_bytes)
{
	int i;
	u32 w = 0;

	pr_debug("%s buf %p oob %p oob_bytes %d\n", __func__,
		 *buf, *oob, oob_bytes);

	if (likely(*buf)) {
		NAND_BEGIN_DATA_ACCESS();
		for (i = 0; i < FC_WORDS; i++, (*buf)++)
			**buf = NAND_REG_RD(FC(i));
		NAND_END_DATA_ACCESS();
	}

	if (*oob && oob_bytes > 0) {
		for (i = 0; i < oob_bytes; i++, (*oob)++) {
			if ((i & 0x3) == 0)
				w = NAND_REG_RD(NCREG_SPARE_AREA_READ_OFS_0 +
						i);
			/* APB read is big endian */
			**oob = w >> (24 - ((i & 0x03) << 3));
		}
	}
}

/*
 * This function counts the 0 bits in a sector data and oob bytes.
 * If the count result is less than a threshold it considers that
 * the sector is an erased sector and it sets all its data and oob
 * bits to 1.
 * The threshold is set to half of the ECC strength to allow for
 * a sector that also has some bits stuck on 1 to be correctable.
 */
static int erased_sector(struct mtd_info *mtd, u8 *buf, u8 *oob,
			 unsigned int *bitflips)
{
	struct nand_chip *chip = mtd->priv;
	struct iproc_nand_host *host = chip->priv;
	int data_bytes = 512 << host->hwcfg.sector_size_1k;
	int oob_bytes = host->hwcfg.spare_area_size <<
				host->hwcfg.sector_size_1k;
	/* Set the bitflip threshold to half of the BCH ECC strength */
	int threshold = (host->hwcfg.ecc_level <<
				host->hwcfg.sector_size_1k) / 2;
	int counter = 0;
	int i;

	pr_debug("%s buf %p oob %p\n", __func__, buf, oob);

	if (host->hwcfg.ecc_code != ECC_CODE_BCH)
		return 0;

	/* Count bitflips in OOB first */
	for (i = 0; i < oob_bytes; i++) {
		counter += hweight8(~oob[i]);
		if (counter > threshold)
			return 0;
	}

	/* Count bitflips in data */
	for (i = 0; i < data_bytes; i++) {
		counter += hweight8(~buf[i]);
		if (counter > threshold)
			return 0;
	}

	/* Clear data and oob */
	memset(buf, 0xFF, data_bytes);
	memset(oob, 0xFF, oob_bytes);

	*bitflips = counter;
	return 1;
}

static int
iproc_nand_read(struct mtd_info *mtd,
		struct nand_chip *chip, u64 addr, unsigned int trans,
		u32 *buf, u8 *oob)
{
	struct iproc_nand_host *host = chip->priv;
	u64 start_addr = addr;
	int i;
	int oob_bytes;
	unsigned int max_bitflips, bitflips;
	unsigned int corr_error_count, uncorr_error_count;
	u32 *sector_buf = buf;
	u8 *sector_oob = oob;

	pr_debug("%s %llx -> %p (trans %x)\n", __func__,
		 (unsigned long long)addr, buf, trans);

	BUG_ON(!oob);

	NAND_ACK_IRQ(NCINTR_UNC);
	NAND_ACK_IRQ(NCINTR_CORR);
	max_bitflips = 0;
	corr_error_count = 0;
	uncorr_error_count = 0;

	NAND_REG_WR(NCREG_CMD_EXT_ADDRESS,
		    (host->cs << 16) | ((addr >> 32) & 0xffff));

	for (i = 0; i < trans; i++, addr += FC_BYTES) {
		if (!host->hwcfg.sector_size_1k || ((i & 0x1) == 0)) {
			sector_buf = buf;
			sector_oob = oob;
		}

		NAND_REG_WR(NCREG_CMD_ADDRESS, addr & 0xffffffff);

		if (ctrl.data_cache_invalid) {
			if ((i == 0) && RD_ACC_CONTROL(host->cs, PAGE_HIT_EN))
				/*
				 * Temporary disable the PAGE_HIT to force
				 * data to be read from NAND.
				 */
				WR_ACC_CONTROL(host->cs, PAGE_HIT_EN, 0);
			else
				ctrl.data_cache_invalid = 0;
		}

		/* SPARE_AREA_READ does not use ECC, so just use PAGE_READ */
		iproc_nand_send_cmd(CMD_PAGE_READ);
		iproc_nand_waitfunc(mtd, chip);

		/* OOB bytes per sector */
		oob_bytes = (mtd->oobsize / trans) <<
				host->hwcfg.sector_size_1k;
		/* OOB bytes per 512B transfer */
		if (host->hwcfg.sector_size_1k && (i & 0x01))
			oob_bytes = max(0, oob_bytes -
					MAX_CONTROLLER_OOB_BYTES);
		oob_bytes = min(oob_bytes, MAX_CONTROLLER_OOB_BYTES);

		iproc_nand_cache_read(&buf, &oob, oob_bytes);

		if (ctrl.data_cache_invalid) {
			/* Re-enable PAGE_HIT if it was temporary disabled */
			WR_ACC_CONTROL(host->cs, PAGE_HIT_EN, 1);
			ctrl.data_cache_invalid = 0;
		}

		if (buf && (!host->hwcfg.sector_size_1k || (i & 0x1))) {
			/* Check uncorrectable errors */
			if (NAND_TEST_IRQ(NCINTR_UNC)) {
				if (erased_sector(mtd,
						  (u8 *)sector_buf,
						  sector_oob,
						  &bitflips)) {
					corr_error_count += bitflips;
					if (bitflips > max_bitflips)
						max_bitflips = bitflips;
				} else {
					uncorr_error_count += 1;
				}
				NAND_ACK_IRQ(NCINTR_UNC);
				ctrl.data_cache_invalid = 1;
			}
			/* Check correctable errors */
			if (NAND_TEST_IRQ(NCINTR_CORR)) {
				bitflips = CORR_ERROR_COUNT;
				corr_error_count += bitflips;
				if (bitflips > max_bitflips)
					max_bitflips = bitflips;
				NAND_ACK_IRQ(NCINTR_CORR);
				ctrl.data_cache_invalid = 1;
			}
		}
	}
	if (uncorr_error_count) {
		dev_warn(&host->pdev->dev,
			 "%d uncorrectable errors at 0x%llx\n",
			 uncorr_error_count,
			 (unsigned long long)start_addr);
		mtd->ecc_stats.failed += uncorr_error_count;
		/* NAND layer expects zero on ECC errors */
		return 0;
	}
	if (max_bitflips) {
		pr_debug("%s: corrected %d bit errors at 0x%llx\n",
			 __func__, max_bitflips,
			 (unsigned long long)start_addr);
		mtd->ecc_stats.corrected += corr_error_count;
		return max_bitflips;
	}

	return 0;
}

static int iproc_nand_read_page(struct mtd_info *mtd,
				struct nand_chip *chip,
				uint8_t *buf,
				int oob_required,
				int page)
{
	struct iproc_nand_host *host = chip->priv;

	BUG_ON(!buf);

	return iproc_nand_read(mtd, chip, host->last_addr,
			       mtd->writesize >> FC_SHIFT, (u32 *) buf,
			       (u8 *) chip->oob_poi);
}

static int iproc_nand_read_page_raw(struct mtd_info *mtd,
				    struct nand_chip *chip,
				    uint8_t *buf,
				    int oob_required,
				    int page)
{
	struct iproc_nand_host *host = chip->priv;
	int ret;

	BUG_ON(!buf);

	WR_ACC_CONTROL(host->cs, RD_ECC_EN, 0);
	ret = iproc_nand_read(mtd, chip, host->last_addr,
			      mtd->writesize >> FC_SHIFT,
			      (u32 *) buf,
			      (u8 *) chip->oob_poi);
	WR_ACC_CONTROL(host->cs, RD_ECC_EN, 1);
	return ret;
}

static int
iproc_nand_read_oob(struct mtd_info *mtd,
		    struct nand_chip *chip, int page)
{
	return iproc_nand_read(mtd, chip, (u64) page << chip->page_shift,
			       mtd->writesize >> FC_SHIFT,
			       NULL, (u8 *) chip->oob_poi);
}

static int
iproc_nand_read_oob_raw(struct mtd_info *mtd,
			struct nand_chip *chip, int page)
{
	struct iproc_nand_host *host = chip->priv;
	int ret;

	WR_ACC_CONTROL(host->cs, RD_ECC_EN, 0);
	ret = iproc_nand_read(mtd, chip, (u64) page << chip->page_shift,
			      mtd->writesize >> FC_SHIFT, NULL,
			      (u8 *) chip->oob_poi);
	WR_ACC_CONTROL(host->cs, RD_ECC_EN, 1);
	return ret;
}

static int
iproc_nand_read_subpage(struct mtd_info *mtd,
			struct nand_chip *chip, uint32_t data_offs,
			uint32_t readlen, uint8_t *bufpoi, int page)
{
	int start_sector, end_sector;
	int steps;
	int data_col_addr;
	int sector_size;
	int oob_offs;
	struct iproc_nand_host *host = chip->priv;

	sector_size = 512 << host->hwcfg.sector_size_1k;
	start_sector = data_offs / sector_size;
	end_sector = (data_offs + readlen - 1) / sector_size;
	steps = (end_sector - start_sector + 1) << host->hwcfg.sector_size_1k;
	oob_offs = start_sector * (host->hwcfg.spare_area_size <<
				   host->hwcfg.sector_size_1k);

	data_col_addr = start_sector * sector_size;

	return iproc_nand_read(mtd, chip, host->last_addr + data_col_addr,
			       steps, (u32 *)(bufpoi + data_col_addr),
			       (u8 *)(chip->oob_poi + oob_offs));
}

static void iproc_nand_cache_write(const u32 **buf, u8 **oob, int oob_bytes)
{
	int i;
	u32 w = 0;

	pr_debug("%s buf %p oob %p oob_bytes %d\n", __func__,
		 *buf, *oob, oob_bytes);

	if (*buf) {
		NAND_BEGIN_DATA_ACCESS();
		for (i = 0; i < FC_WORDS; i++, (*buf)++)
			NAND_REG_WR(FC(i), **buf);
		NAND_END_DATA_ACCESS();
	} else {
		for (i = 0; i < FC_WORDS; i++)
			NAND_REG_WR(FC(i), 0xffffffff);
	}

	if (*oob) {
		for (i = 0; i < oob_bytes; i++, (*oob)++) {
			w <<= 8;
			w |= **oob;
			if ((i & 0x3) == 0x3)
				NAND_REG_WR(NCREG_SPARE_AREA_WRITE_OFS_0 +
					    (i & ~0x3), w);
		} /* fill the remaining OOB bytes with 0xFF */
		for (i = oob_bytes; i < MAX_CONTROLLER_OOB_BYTES; i++) {
			w <<= 8;
			w |= 0xFF;
			if ((i & 0x3) == 0x3)
				NAND_REG_WR(NCREG_SPARE_AREA_WRITE_OFS_0 +
					    (i & ~0x3), w);
		}
	} else {
		for (i = 0; i < MAX_CONTROLLER_OOB_WORDS; i++)
			NAND_REG_WR(NCREG_SPARE_AREA_WRITE_OFS_0 + (i << 2),
				    0xffffffff);
	}
}

static int
iproc_nand_write(struct mtd_info *mtd,
		 struct nand_chip *chip, u64 addr, const u32 *buf, u8 *oob)
{
	struct iproc_nand_host *host = chip->priv;
	unsigned int trans = mtd->writesize >> FC_SHIFT;
	unsigned int i;
	int status;
	int oob_bytes;
	int ret = 0;

	pr_debug("%s %llx <- %p\n", __func__, (unsigned long long)addr, buf);

	if (unlikely((u32) buf & 0x03)) {
		dev_warn(&host->pdev->dev, "unaligned buffer: %p\n", buf);
		buf = (u32 *) ((u32) buf & ~0x03);
	}

	iproc_nand_wp(mtd, 0);

	NAND_REG_WR(NCREG_CMD_EXT_ADDRESS,
		    (host->cs << 16) | ((addr >> 32) & 0xffff));

	for (i = 0; i < trans; i++, addr += FC_BYTES) {

		/* full address MUST be set before populating FC */
		NAND_REG_WR(NCREG_CMD_ADDRESS, addr & 0xffffffff);

		oob_bytes = 0;
		if (oob) {
			/* OOB bytes per sector */
			oob_bytes = (mtd->oobsize / trans) <<
					host->hwcfg.sector_size_1k;
			/* OOB bytes per 512B transfer */
			if (host->hwcfg.sector_size_1k && (i & 0x01))
				oob_bytes = max(0, oob_bytes -
						MAX_CONTROLLER_OOB_BYTES);
			oob_bytes = min(oob_bytes, MAX_CONTROLLER_OOB_BYTES);
		}
		iproc_nand_cache_write(&buf, &oob, oob_bytes);

		/* we cannot use SPARE_AREA_PROGRAM when PARTIAL_PAGE_EN=0 */
		iproc_nand_send_cmd(CMD_PROGRAM_PAGE);
		status = iproc_nand_waitfunc(mtd, chip);

		if (status & NAND_STATUS_FAIL) {
			dev_warn(&host->pdev->dev, "program failed at %llx\n",
				 (unsigned long long)addr);
			ret = -EIO;
			break;
		}
	}
	iproc_nand_wp(mtd, 1);
	return ret;
}

static int iproc_nand_write_page(struct mtd_info *mtd,
				 struct nand_chip *chip,
				 const uint8_t *buf,
				 int oob_required)
{
	struct iproc_nand_host *host = chip->priv;

	BUG_ON(!buf);

	return iproc_nand_write(mtd, chip, host->last_addr, (u32 *) buf,
				oob_required ? (u8 *) chip->oob_poi : NULL);
}

static int iproc_nand_write_page_raw(struct mtd_info *mtd,
				     struct nand_chip *chip,
				     const uint8_t *buf,
				     int oob_required)
{
	int ret;
	struct iproc_nand_host *host = chip->priv;

	BUG_ON(!buf);

	WR_ACC_CONTROL(host->cs, WR_ECC_EN, 0);
	ret = iproc_nand_write(mtd, chip, host->last_addr, (u32 *) buf,
			       oob_required ? (u8 *) chip->oob_poi : NULL);
	WR_ACC_CONTROL(host->cs, WR_ECC_EN, 1);
	return ret;
}

static int
iproc_nand_write_oob(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	return iproc_nand_write(mtd, chip, (u64) page << chip->page_shift, NULL,
				(u8 *) chip->oob_poi);
}

static int
iproc_nand_write_oob_raw(struct mtd_info *mtd, struct nand_chip *chip, int page)
{
	struct iproc_nand_host *host = chip->priv;
	int ret;

	WR_ACC_CONTROL(host->cs, WR_ECC_EN, 0);
	ret = iproc_nand_write(mtd, chip, (u64) page << chip->page_shift, NULL,
			       (u8 *) chip->oob_poi);
	WR_ACC_CONTROL(host->cs, WR_ECC_EN, 1);
	return ret;
}

/***********************************************************************
 * Per-CS setup (1 NAND device)
 ***********************************************************************/

static const unsigned int block_sizes[] = { 8, 16, 128, 256, 512, 1024, 2048 };
static const unsigned int page_sizes[] = { 512, 2048, 4096, 8192 };

static void
iproc_nand_set_cfg(struct iproc_nand_host *host, struct iproc_nand_cfg *cfg)
{
	int i, found;

	for (i = 0, found = 0; i < ARRAY_SIZE(block_sizes); i++)
		if ((block_sizes[i] << 10) == cfg->block_size) {
			WR_CONFIG(host->cs, BLOCK_SIZE, i);
			found = 1;
		}
	if (!found)
		dev_warn(&host->pdev->dev, "invalid block size %u\n",
			 cfg->block_size);

	for (i = 0, found = 0; i < ARRAY_SIZE(page_sizes); i++)
		if (page_sizes[i] == cfg->page_size) {
			WR_CONFIG(host->cs, PAGE_SIZE, i);
			found = 1;
		}
	if (!found)
		dev_warn(&host->pdev->dev, "invalid page size %u\n",
			 cfg->page_size);

	if (fls64(cfg->device_size) < 23)
		dev_warn(&host->pdev->dev, "invalid device size 0x%llx\n",
			 (unsigned long long)cfg->device_size);

	if (cfg->ecc_code == ECC_CODE_BCH) {
		if ((cfg->ecc_level >
		     iproc_max_bch_ecc_level[cfg->sector_size_1k]) ||
		    (iproc_bch_ecc_bytes[cfg->ecc_level] >
		     cfg->spare_area_size))
			dev_warn(&host->pdev->dev,
				 "invalid BCH ECC configuration: %u/%u\n",
				 cfg->ecc_level << cfg->sector_size_1k,
				 cfg->sector_size_1k ? 1024 : 512);
	}

	if (cfg->ecc_code == ECC_CODE_HAMMING) {
		if (!((cfg->ecc_level == 15) &&
		      (cfg->spare_area_size == 16) &&
		      (cfg->sector_size_1k == 0)))
			dev_warn(&host->pdev->dev,
				 "invalid HAMMING ECC configuration\n");
	}

	WR_CONFIG(host->cs, DEVICE_SIZE, fls64(cfg->device_size) - 23);
	WR_CONFIG(host->cs, DEVICE_WIDTH, cfg->device_width == 16 ? 1 : 0);
	WR_CONFIG(host->cs, COL_ADR_BYTES, cfg->col_adr_bytes);
	WR_CONFIG(host->cs, BLK_ADR_BYTES, cfg->blk_adr_bytes);
	WR_CONFIG(host->cs, FUL_ADR_BYTES, cfg->ful_adr_bytes);

	WR_ACC_CONTROL(host->cs, SPARE_AREA_SIZE, cfg->spare_area_size);
	WR_ACC_CONTROL(host->cs, SECTOR_SIZE_1K, cfg->sector_size_1k);

	WR_ACC_CONTROL(host->cs, ECC_LEVEL, cfg->ecc_level);
}

static void
iproc_nand_get_cfg(struct iproc_nand_host *host, struct iproc_nand_cfg *cfg)
{
	cfg->block_size = RD_CONFIG(host->cs, BLOCK_SIZE);
	cfg->device_size = (4ULL << 20) << RD_CONFIG(host->cs, DEVICE_SIZE);
	cfg->page_size = RD_CONFIG(host->cs, PAGE_SIZE);
	cfg->device_width = RD_CONFIG(host->cs, DEVICE_WIDTH) ? 16 : 8;
	cfg->col_adr_bytes = RD_CONFIG(host->cs, COL_ADR_BYTES);
	cfg->blk_adr_bytes = RD_CONFIG(host->cs, BLK_ADR_BYTES);
	cfg->ful_adr_bytes = RD_CONFIG(host->cs, FUL_ADR_BYTES);
	cfg->spare_area_size = RD_ACC_CONTROL(host->cs, SPARE_AREA_SIZE);
	cfg->sector_size_1k = RD_ACC_CONTROL(host->cs, SECTOR_SIZE_1K);
	cfg->ecc_level = RD_ACC_CONTROL(host->cs, ECC_LEVEL);

	if (cfg->block_size < ARRAY_SIZE(block_sizes))
		cfg->block_size = block_sizes[cfg->block_size] << 10;
	else
		cfg->block_size = 128 << 10;

	if (cfg->page_size < ARRAY_SIZE(page_sizes))
		cfg->page_size = page_sizes[cfg->page_size];
	else
		cfg->page_size = 2048;

	/* Special case: using Hamming code */
	if ((cfg->ecc_level == 15) && (cfg->spare_area_size == 16) &&
	    (cfg->sector_size_1k == 0))
		cfg->ecc_code = ECC_CODE_HAMMING;
	else
		cfg->ecc_code = ECC_CODE_BCH;
}

static void iproc_nand_print_cfg(struct iproc_nand_cfg *cfg)
{
	pr_info("NAND %u-bit %lluMiB total, %uKiB blocks, %u%s pages\n"
		"     %ubit/%uB %s-ECC %uB/512B OOB\n",
		cfg->device_width,
		(unsigned long long)cfg->device_size >> 20,
		cfg->block_size >> 10,
		cfg->page_size >= 1024 ? cfg->page_size >> 10 : cfg->page_size,
		cfg->page_size >= 1024 ? "KiB" : "B",
		cfg->ecc_code == ECC_CODE_HAMMING ?
			1 : cfg->ecc_level << cfg->sector_size_1k,
		512 << cfg->sector_size_1k,
		cfg->ecc_code == ECC_CODE_HAMMING ? "HAMMING" : "BCH",
		cfg->spare_area_size);
}

static int iproc_nand_setup_dev(struct iproc_nand_host *host)
{
	struct mtd_info *mtd = &host->mtd;
	struct nand_chip *chip = &host->chip;
	struct iproc_nand_cfg orig_cfg, new_cfg;
	struct nand_oobfree *free = iproc_nand_oob_layout.oobfree;

	uint8_t steps;
	uint8_t eccbytes;
	uint8_t eccstrength;
	int threshold;

	iproc_nand_get_cfg(host, &orig_cfg);
	host->hwcfg = orig_cfg;

	memset(&new_cfg, 0, sizeof(new_cfg));
	new_cfg.device_size = mtd->size;
	new_cfg.block_size = mtd->erasesize;
	new_cfg.page_size = mtd->writesize;
	new_cfg.spare_area_size = mtd->oobsize / (mtd->writesize >> FC_SHIFT);
	new_cfg.device_width = (chip->options & NAND_BUSWIDTH_16) ? 16 : 8;
	new_cfg.col_adr_bytes = 2;

	if (mtd->writesize > 512)
		if (mtd->size >= (256 << 20))
			new_cfg.blk_adr_bytes = 3;
		else
			new_cfg.blk_adr_bytes = 2;
	else if (mtd->size >= (64 << 20))
		new_cfg.blk_adr_bytes = 3;
	else
		new_cfg.blk_adr_bytes = 2;
	new_cfg.ful_adr_bytes = new_cfg.blk_adr_bytes + new_cfg.col_adr_bytes;

	pr_debug("%s: mtd configuration\n"
		 "\tdevice_size 0x%llx\n"
		 "\tblock_size 0x%x\n"
		 "\tpage_size 0x%x\n"
		 "\tdevice_width 0x%x\n"
		 "\tcol_adr_bytes 0x%x\n"
		 "\tblk_adr_bytes 0x%x\n"
		 "\tspare_area_size 0x%x\n"
		 "\tecc_level 0x%x\n"
		 "\tsector_size_1k 0x%x\n",
		 __func__,
		 new_cfg.device_size,
		 new_cfg.block_size,
		 new_cfg.page_size,
		 new_cfg.device_width,
		 new_cfg.col_adr_bytes,
		 new_cfg.blk_adr_bytes,
		 new_cfg.spare_area_size,
		 new_cfg.ecc_level,
		 new_cfg.sector_size_1k);

	/* Check settings inherited from bootloader */
	if (ctrl.boot_inited) {

		pr_debug("%s: bootloader configuration\n"
			 "\tdevice_size 0x%llx\n"
			 "\tblock_size 0x%x\n"
			 "\tpage_size 0x%x\n"
			 "\tdevice_width 0x%x\n"
			 "\tcol_adr_bytes 0x%x\n"
			 "\tblk_adr_bytes 0x%x\n"
			 "\tspare_area_size 0x%x\n"
			 "\tecc_level 0x%x\n"
			 "\tsector_size_1k 0x%x\n",
			 __func__,
			 orig_cfg.device_size,
			 orig_cfg.block_size,
			 orig_cfg.page_size,
			 orig_cfg.device_width,
			 orig_cfg.col_adr_bytes,
			 orig_cfg.blk_adr_bytes,
			 orig_cfg.spare_area_size,
			 orig_cfg.ecc_level,
			 orig_cfg.sector_size_1k);
		/* Check basic device attributes first */
		if (orig_cfg.device_size != new_cfg.device_size ||
		    orig_cfg.block_size != new_cfg.block_size ||
		    orig_cfg.page_size != new_cfg.page_size ||
		    orig_cfg.device_width != new_cfg.device_width ||
		    orig_cfg.col_adr_bytes != new_cfg.col_adr_bytes ||
		    orig_cfg.blk_adr_bytes != new_cfg.blk_adr_bytes ||
		    orig_cfg.ful_adr_bytes != new_cfg.ful_adr_bytes ||
		    orig_cfg.ecc_level == 0 ||
		    ((orig_cfg.ecc_code == ECC_CODE_BCH) &&
		     (orig_cfg.ecc_level >
		      iproc_max_bch_ecc_level[orig_cfg.sector_size_1k])) ||
		    orig_cfg.spare_area_size > new_cfg.spare_area_size ||
		    ((orig_cfg.ecc_code == ECC_CODE_BCH) &&
		     (iproc_bch_ecc_bytes[orig_cfg.ecc_level] >
		      orig_cfg.spare_area_size))) {

			ctrl.boot_inited = 0;
			pr_info(DRV_NAME ": invalid bootloader settings\n");

		} else {
			/* Bootloader has initialized the flash correctly. */
			new_cfg = orig_cfg;
			pr_info(DRV_NAME ": following bootloader settings\n");
		}
	}

	/* Decide ECC settings ourselves if it's not initialized before */
	if (!ctrl.boot_inited) {
		pr_info(DRV_NAME " straps: page 0x%x type 0x%x\n",
			ctrl.strap_page_size, ctrl.strap_type);

		/* Check if strap settings are valid */
		if (ctrl.strap_type > 0 &&
		    ctrl.data->strap_page_sizes[ctrl.strap_page_size] ==
		    new_cfg.page_size &&
		    ctrl.data->strap_types[ctrl.strap_type].spare_size <=
		    new_cfg.spare_area_size) {
			/* It's valid, follow the strap settings */
			new_cfg.spare_area_size =
			    ctrl.data->strap_types[ctrl.strap_type].spare_size;
			new_cfg.sector_size_1k =
			    ctrl.data->strap_types[ctrl.strap_type].sector_1k;
			new_cfg.ecc_level =
			    ctrl.data->strap_types[ctrl.strap_type].ecc_level;
			if (ctrl.strap_page_size == 0) {
				new_cfg.blk_adr_bytes = 2;
				new_cfg.ful_adr_bytes = 4;
			} else {
				new_cfg.blk_adr_bytes = 3;
				new_cfg.ful_adr_bytes = 5;
			}

			/* Special case: using Hamming code */
			if ((new_cfg.ecc_level == 15) &&
			    (new_cfg.spare_area_size == 16) &&
			    (new_cfg.sector_size_1k == 0))
				new_cfg.ecc_code = ECC_CODE_HAMMING;
			else
				new_cfg.ecc_code = ECC_CODE_BCH;

			pr_info(DRV_NAME ": following strap settings\n");

		} else {

			/*
			 * Strap settings are not valid,
			 * decide the settings on our own
			 */

			/* Trying to fit with available strap settings */
			new_cfg.spare_area_size =
			    new_cfg.spare_area_size >= 27 ? 27 : 16;
			new_cfg.sector_size_1k = 0;
			new_cfg.ecc_code = ECC_CODE_BCH;
			if (new_cfg.spare_area_size == 27) {
				new_cfg.ecc_level = 12;
				new_cfg.sector_size_1k =
				    (new_cfg.page_size >= 2048) ? 1 : 0;
			} else if (chip->badblockpos ==
				   NAND_SMALL_BADBLOCK_POS) {
				new_cfg.ecc_level = 4;
			} else {
				new_cfg.ecc_level = 8;
			}

			pr_info(DRV_NAME
				": overriding invalid strap settings\n");
		}

		iproc_nand_set_cfg(host, &new_cfg);
		host->hwcfg = new_cfg;
	}

	iproc_nand_print_cfg(&new_cfg);

	WR_ACC_CONTROL(host->cs, RD_ECC_EN, 1);
	WR_ACC_CONTROL(host->cs, WR_ECC_EN, 1);
	WR_ACC_CONTROL(host->cs, FAST_PGM_RDIN, 0);
	WR_ACC_CONTROL(host->cs, RD_ERASED_ECC_EN, 0);
	WR_ACC_CONTROL(host->cs, PARTIAL_PAGE_EN, 0);
	WR_ACC_CONTROL(host->cs, PAGE_HIT_EN, 1);

	if (new_cfg.ecc_code == ECC_CODE_BCH) {
		/* threshold = ceil(ECC-strength * percentage) */
		threshold = ((new_cfg.ecc_level << new_cfg.sector_size_1k) *
			     ctrl.corr_threshold_percent + 99) / 100;
		WR_CORR_THRESH(host->cs, threshold);
		pr_info(DRV_NAME
		       ": ECC correction status threshold set to %d bit\n",
		       threshold);
	} else {
		WR_CORR_THRESH(host->cs, 1);
	}

	mb();

	/* Adjust MTD oobsize according to the configuration */
	mtd->oobsize = new_cfg.spare_area_size * (mtd->writesize >> FC_SHIFT);

	/* Adjust ECC layout for storing usb OOB data */
	free->length = 0;
	steps = mtd->writesize >> FC_SHIFT;
	if (new_cfg.ecc_code == ECC_CODE_HAMMING) {
		eccbytes = 3;
		eccstrength = 1;
	} else {
		eccbytes = iproc_bch_ecc_bytes[new_cfg.ecc_level];
		eccstrength = new_cfg.ecc_level << new_cfg.sector_size_1k;
	}

	/*
	 * These are not really used.
	 * We still prepare them for safety.
	 */
	iproc_nand_oob_layout.eccbytes = eccbytes * steps;
	chip->ecc.bytes = eccbytes;
	chip->ecc.strength = eccstrength;
	chip->ecc.size = 512 << new_cfg.sector_size_1k;

	/* Create oobfree for storing user OOB data */
	if (new_cfg.spare_area_size > eccbytes) {
		unsigned int spare_size;
		uint8_t i, cnt;

		spare_size =
		    new_cfg.spare_area_size << new_cfg.sector_size_1k;
		eccbytes <<= new_cfg.sector_size_1k;
		steps >>= new_cfg.sector_size_1k;
		if (steps > MTD_MAX_OOBFREE_ENTRIES)
			steps = MTD_MAX_OOBFREE_ENTRIES;
		for (i = 0, cnt = 0;
		     i < steps && cnt < MTD_MAX_OOBFREE_ENTRIES; i++) {

			if (new_cfg.ecc_code == ECC_CODE_HAMMING) {
				/*
				 * Hamming code: ECC bytes are 6~8;
				 * First part here.
				 */
				free->offset = i * spare_size;
				free->length = 6;

			} else {
				/* BCH: ECC bytes at the bottom */
				free->offset = i * spare_size;
				free->length = spare_size - eccbytes;
			}

			/* Reserve the first two bytes of the page */
			if (i == 0) {
				if (free->length <= 2) {
					/*
					 * Don't claim this entry if
					 * less than 2 bytes
					 */
					continue;
				}
				free->offset += 2;
				free->length -= 2;
			}

			if (new_cfg.ecc_code == ECC_CODE_HAMMING) {
				/* Hamming code: the 2nd free part */
				free++;
				cnt++;
				if (cnt < MTD_MAX_OOBFREE_ENTRIES) {
					free->offset =
					    i * spare_size + 9;
					free->length = 7;
				} else {
					/* The structure limits us. */
					break;
				}
			}

			free++;
			cnt++;
		}
		if (cnt < MTD_MAX_OOBFREE_ENTRIES) {
			/* Terminater */
			free->length = 0;
		}

		/* Print out oob space information */
		free = iproc_nand_oob_layout.oobfree;
		if (free->length) {
			spare_size = 0;
			while (free->length) {
				spare_size += free->length;
				free++;
			}
			pr_info(DRV_NAME
				": user oob per page: %u bytes (%u steps)\n",
				spare_size, (int)steps);
		}
	}

	if (iproc_nand_oob_layout.oobfree[0].length == 0)
		pr_info(DRV_NAME ": no oob space available\n");

	return 0;
}

static void iproc_nand_timing_setup(struct iproc_nand_host *host)
{
	struct nand_chip *chip = &host->chip;
	int onfi_tmode;	/* bit mask of supported onfi timing modes */

	/*
	 * ctrl.tmode has the configured tmode upper limit [0-5]
	 * or is -1 to indicate power-on default timing
	 */
	if (ctrl.tmode < 0 || ctrl.tmode >= ONFI_TIMING_MODES)
		return;

	if (chip->onfi_version) {
		onfi_tmode = le16_to_cpu(get_unaligned(
				(u16 *)&chip->onfi_params.async_timing_mode));
		if ((onfi_tmode == 0) || (onfi_tmode & ~0x3F)) {
			pr_info(DRV_NAME
				": invalid ONFI timing mode ignored 0x%x\n",
				onfi_tmode);
		} else {
			/*
			 * select the maximum supported ONFI timing mode
			 * that is lower than the configured limit
			 */
			while (ctrl.tmode > 0) {
				if (onfi_tmode & (1 << ctrl.tmode))
					break;
				ctrl.tmode--;
			}
		}
	}

	NAND_REG_WR(NC_REG_TIMING1(host->cs),
		    ctrl.data->onfi_tmode[ctrl.tmode].timing1);
	NAND_REG_WR(NC_REG_TIMING2(host->cs),
		    ctrl.data->onfi_tmode[ctrl.tmode].timing2);
	pr_info(DRV_NAME ": timing mode %d\n", ctrl.tmode);
}

static int iproc_nand_reboot_notifier(struct notifier_block *n,
				      unsigned long state,
				      void *cmd)
{
	struct mtd_info *mtd;

	mtd = container_of(n, struct mtd_info, reboot_notifier);
	/*
	 * FIXME:
	 *
	 * Waiting for nand_shutdown to be merged to mainline
	 */
	/* nand_shutdown(mtd); */
	return NOTIFY_DONE;
}

static int iproc_nand_probe(struct platform_device *pdev)
{
	struct iproc_nand_host *host;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	int ret = 0;

	pr_debug("%s: id %d\n", __func__, pdev->id);

	ret = iproc_nand_ctrl_setup(pdev);
	if (ret)
		return ret;

	host = kzalloc(sizeof(*host), GFP_KERNEL);
	if (!host) {
		dev_err(&pdev->dev, "can't allocate memory\n");
		ret = -ENOMEM;
		goto err2;
	}

	host->cs = 0;

	mtd = &host->mtd;
	chip = &host->chip;
	host->pdev = pdev;
	platform_set_drvdata(pdev, host);

	chip->priv = host;
	mtd->priv = chip;
	mtd->name = "iproc_nand";
	mtd->owner = THIS_MODULE;
	mtd->dev.parent = &pdev->dev;
	mtd->reboot_notifier.notifier_call = iproc_nand_reboot_notifier;
	/*
	 * Set mtd bitflip threshold to 1 as the desired threshold will
	 * be set in the controller register.
	 */
	mtd->bitflip_threshold = 1;

	chip->IO_ADDR_R = (void *)0xdeadbeef;
	chip->IO_ADDR_W = (void *)0xdeadbeef;

	chip->cmd_ctrl = iproc_nand_cmd_ctrl;
	chip->cmdfunc = iproc_nand_cmdfunc;
	chip->waitfunc = iproc_nand_waitfunc;
	chip->read_byte = iproc_nand_read_byte;
	chip->read_buf = iproc_nand_read_buf;
	if (ctrl.max_cs > 1)
		chip->select_chip = iproc_nand_select_chip;

	chip->ecc.mode = NAND_ECC_HW;
	chip->ecc.layout = &iproc_nand_oob_layout;
	chip->ecc.read_page = iproc_nand_read_page;
	chip->ecc.read_subpage = iproc_nand_read_subpage;
	chip->ecc.write_page = iproc_nand_write_page;
	chip->ecc.read_page_raw = iproc_nand_read_page_raw;
	chip->ecc.write_page_raw = iproc_nand_write_page_raw;

	chip->ecc.write_oob_raw = iproc_nand_write_oob_raw;
	chip->ecc.read_oob_raw = iproc_nand_read_oob_raw;

	chip->ecc.read_oob = (void *)iproc_nand_read_oob;
	chip->ecc.write_oob = iproc_nand_write_oob;

	chip->controller = &ctrl.controller;

	if (nand_scan_ident(mtd, ctrl.max_cs, NULL)) {
		ret = -ENXIO;
		goto err1;
	}

	chip->options |= NAND_SUBPAGE_READ |
			 NAND_NO_SUBPAGE_WRITE;

	chip->bbt_options |= NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;

	iproc_nand_timing_setup(host);

	if (iproc_nand_setup_dev(host) || nand_scan_tail(mtd)) {
		ret = -ENXIO;
		goto err1;
	}

	ret = mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);
	if (ret)
		goto err1;

	ret = register_reboot_notifier(&mtd->reboot_notifier);
	if (ret) {
		nand_release(mtd);
		goto err1;
	}

	pr_info(DRV_NAME ": NAND controller driver is loaded\n");
	return 0;

 err1:
	platform_set_drvdata(pdev, NULL);
	kfree(host);
 err2:
	iproc_nand_ctrl_release();
	return ret;
}

static int iproc_nand_remove(struct platform_device *pdev)
{
	struct iproc_nand_host *host = dev_get_drvdata(&pdev->dev);
	struct mtd_info *mtd = &host->mtd;

	unregister_reboot_notifier(&mtd->reboot_notifier);
	nand_release(mtd);
	platform_set_drvdata(pdev, NULL);
	kfree(host);
	iproc_nand_ctrl_release();

	return 0;
}

static const struct of_device_id iproc_nand_of_match[] = {
#if defined(CONFIG_ARCH_BCM_CYGNUS)
	{
		.compatible = "brcm,iproc-nand-cygnus",
		.data = (void *)&cygnus_nand_ctrl_data
	},
#endif
#if defined(CONFIG_ARCH_BCM_NSP)
	{
		.compatible = "brcm,iproc-nand-nsp",
		.data = (void *)&nsp_nand_ctrl_data
	},
#endif
	{},
};
MODULE_DEVICE_TABLE(of, iproc_nand_of_match);

/***********************************************************************
 * Platform driver setup (per controller)
 ***********************************************************************/
static struct platform_driver iproc_nand_driver = {
	.probe = iproc_nand_probe,
	.remove = iproc_nand_remove,
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = iproc_nand_of_match,
		   },
};
module_platform_driver(iproc_nand_driver);

static int iproc_nand_ctrl_setup(struct platform_device *pdev)
{
	int err = -ENODEV;
	struct device_node *dn = pdev->dev.of_node;
	const struct of_device_id *match;
	struct resource res;
	int timeout;
	int len;

	match = of_match_device(iproc_nand_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "can't find DT configuration\n");
		return -ENODEV;
	}
	ctrl.data = (struct nand_ctrl_data *)match->data;

	init_completion(&ctrl.done);
	spin_lock_init(&ctrl.controller.lock);
	init_waitqueue_head(&ctrl.controller.wq);
	ctrl.cmd_pending = 0;
	ctrl.boot_inited = 0;

	/* Initialize registers and IRQ */
	ctrl.nand_regs = NULL;
	ctrl.nand_intr_regs = NULL;
	ctrl.nand_idm_regs = NULL;
	ctrl.nand_idm_io_ctrl_direct_reg = NULL;
	ctrl.nand_strap_regs = NULL;
	ctrl.nand_regs = (void *)of_iomap(dn, 0);
	if (!ctrl.nand_regs) {
		dev_err(&pdev->dev, "can't iomap nand_regs\n");
		err = -EIO;
		goto err;
	}
	ctrl.nand_intr_regs = ctrl.nand_regs + NCREG_INTERRUPT_BASE;
	ctrl.nand_idm_regs = (void *)of_iomap(dn, 1);
	if (!ctrl.nand_idm_regs) {
		dev_err(&pdev->dev, "can't iomap nand_idm_regs\n");
		err = -EIO;
		goto err;
	}
	ctrl.nand_idm_io_ctrl_direct_reg = ctrl.nand_idm_regs +
						IDMREG_IO_CONTROL_DIRECT;
	if (of_address_to_resource(dn, 2, &res)) {
		/*
		 * When no strap_regs is configured in DT
		 * we asume that straps are available in
		 * NAND_IDM_IDM_IO_STATUS register
		 */
		ctrl.nand_strap_regs = ctrl.nand_idm_regs + IDMREG_IO_STATUS;
	} else {
		ctrl.nand_strap_regs =
			(void *)ioremap(res.start, resource_size(&res));
		if (!ctrl.nand_strap_regs) {
			dev_err(&pdev->dev, "can't iomap nand_strap_regs\n");
			err = -EIO;
			goto err;
		}
	}
	ctrl.strap_type = NAND_STRAP_TYPE;
	ctrl.strap_page_size = NAND_STRAP_PAGE;

	ctrl.irq = irq_of_parse_and_map(dn, 0);
	if (ctrl.irq == NO_IRQ) {
		dev_err(&pdev->dev, "can't parse and map interrupt\n");
		err = -EIO;
		goto err;
	}

	if (of_property_read_u32(dn, "#chip-selects", &ctrl.max_cs)) {
		dev_warn(&pdev->dev,
			 "missing #chip-selects property (default to 1)\n");
		ctrl.max_cs = 1;
	}

	if (ctrl.max_cs == 0 || ctrl.max_cs > ctrl.data->chip_select_max) {
		dev_warn(&pdev->dev,
			 "invalid #chip-selects property (default to 1)\n");
		ctrl.max_cs = 1;
	}

	/* Get write protect mode property */
	ctrl.wp_mode = WP_SET_BY_DEFAULT;
	of_property_read_u32(dn, "wp-mode", &ctrl.wp_mode);
	if (ctrl.wp_mode > WP_ALWAYS_CLEARED) {
		dev_warn(&pdev->dev,
			 "invalid wp-mode property %d (ignored)\n",
			 ctrl.wp_mode);
		ctrl.wp_mode = WP_SET_BY_DEFAULT;
	}

	/* Get timing mode property */
	ctrl.tmode = -1; /* Use default timing configuration */
	of_property_read_u32(dn, "timing-mode", &ctrl.tmode);
	if (ctrl.tmode >= ONFI_TIMING_MODES) {
		dev_warn(&pdev->dev,
			 "invalid timing-mode property %d (ignored)\n",
			 ctrl.tmode);
		ctrl.tmode = -1;
	}

	/* Get hw-auto-init property */
	ctrl.hw_auto_init = 0;
	if (of_find_property(dn, "hw-auto-init", &len))
		ctrl.hw_auto_init = 1;

	/* Get ECC correction status percentage property */
	ctrl.corr_threshold_percent = DEFAULT_CORR_STATUS_THRESHOLD_PERCENT;
	of_property_read_u32(dn, "corr-threshold-percent",
			     &ctrl.corr_threshold_percent);
	if (ctrl.corr_threshold_percent > 100) {
		dev_warn(&pdev->dev,
			 "invalid corr-threshold-percent property"
			 " %d (ignored)\n",
			 ctrl.corr_threshold_percent);
		ctrl.corr_threshold_percent =
			DEFAULT_CORR_STATUS_THRESHOLD_PERCENT;
	}

	pr_debug("%s: nand_regs %p\n", __func__, ctrl.nand_regs);
	pr_debug("%s: nand_intr_regs %p\n", __func__, ctrl.nand_intr_regs);
	pr_debug("%s: nand_idm_regs %p\n", __func__, ctrl.nand_idm_regs);
	pr_debug("%s: nand_idm_io_ctrl_direct_reg %p\n", __func__,
			ctrl.nand_idm_io_ctrl_direct_reg);
	pr_debug("%s: nand_strap_regs %p\n", __func__, ctrl.nand_strap_regs);
	pr_debug("%s: irq %d\n", __func__, ctrl.irq);
	pr_debug("%s: max_cs %d\n", __func__, ctrl.max_cs);
	pr_debug("%s: wp_mode %d\n", __func__, ctrl.wp_mode);
	pr_debug("%s: hw_auto_init %d\n", __func__, ctrl.hw_auto_init);

	if (ctrl.hw_auto_init) {
		/* Reset the NAND controller */
		writel(1, ctrl.nand_idm_regs + IDMREG_RESET_CONTROL);
		udelay(1);
		writel(0, ctrl.nand_idm_regs + IDMREG_RESET_CONTROL);
		udelay(10);

		/* Execute controller auto-init */
		NAND_REG_CLR(NCREG_CS_NAND_SELECT,
			     NCFLD_CS_NAND_SELECT_AUTO_DEVID_CONFIG);
		NAND_REG_SET(NCREG_CS_NAND_SELECT,
			     NCFLD_CS_NAND_SELECT_AUTO_DEVID_CONFIG);
		timeout = 100000;
		ctrl.boot_inited = 1;
		while (timeout > 0) {
			udelay(10);
			if (NCFLD_INIT_STATUS_INIT_SUCCESS &
			    NAND_REG_RD(NCREG_INIT_STATUS)) {
				pr_info(DRV_NAME ": auto-init success\n");
				break;
			}
			timeout -= 10;
		}
		if (timeout <= 0) {
			ctrl.boot_inited = 0;
			pr_info(DRV_NAME ": auto-init failed\n");
		}
		pr_debug("%s: auto-init status 0x%x\n", __func__,
			 NAND_REG_RD(NCREG_INIT_STATUS));
	} else {
		/* Check if auto-init was done due to strap_nand_flash */
		if (NAND_REG_RD(NCREG_CS_NAND_SELECT) &
		    NCFLD_CS_NAND_SELECT_AUTO_DEVID_CONFIG)
			ctrl.boot_inited = 1;
		/* Check if controller was initialized by bootloader */
		if (NAND_REG_RD(NCREG_SEMAPHORE) == 0xFF)
			ctrl.boot_inited = 1;
	}

	/* Perform basic controller initialization */
	NAND_REG_CLR(NCREG_CS_NAND_SELECT,
		     NCFLD_CS_NAND_SELECT_AUTO_DEVID_CONFIG);
	NAND_REG_CLR(NCREG_CS_NAND_SELECT,
		     NCFLD_CS_NAND_SELECT_DIRECT_ACCESS_CS_MASK);
	NAND_REG_CLR(NCREG_CS_NAND_XOR, NCFLD_CS_NAND_XOR_CS_MASK);
	if (ctrl.wp_mode == WP_ALWAYS_CLEARED) {
		/* Permanently remove write-protection */
		NAND_REG_CLR(NCREG_CS_NAND_SELECT, NCFLD_CS_NAND_SELECT_WP);
	}

	/* Attach IRQ handler */
	NAND_ACK_IRQ(NCINTR_CTLRDY);
	NAND_ENABLE_IRQ(NCINTR_CTLRDY);
	err = request_irq((unsigned int)ctrl.irq, iproc_nand_irq, 0,
			  DRV_NAME, &ctrl);
	if (err < 0) {
		dev_err(&pdev->dev, "unable to allocate IRQ (error %d)\n", err);
		goto err;
	}

	return 0;

 err:
	NAND_DISABLE_IRQ(NCINTR_CTLRDY);
	if (ctrl.nand_strap_regs) {
		if (ctrl.nand_strap_regs !=
				ctrl.nand_idm_regs + IDMREG_IO_STATUS)
			iounmap(ctrl.nand_strap_regs);
		ctrl.nand_strap_regs = NULL;
	}
	ctrl.nand_idm_io_ctrl_direct_reg = NULL;
	if (ctrl.nand_idm_regs) {
		iounmap(ctrl.nand_idm_regs);
		ctrl.nand_idm_regs = NULL;
	}
	ctrl.nand_intr_regs = NULL;
	if (ctrl.nand_regs) {
		iounmap(ctrl.nand_regs);
		ctrl.nand_regs = NULL;
	}
	return err;
}

static void iproc_nand_ctrl_release(void)
{
	NAND_DISABLE_IRQ(NCINTR_CTLRDY);
	free_irq(ctrl.irq, &ctrl);
	if (ctrl.nand_strap_regs) {
		if (ctrl.nand_strap_regs !=
				ctrl.nand_idm_regs + IDMREG_IO_STATUS)
			iounmap(ctrl.nand_strap_regs);
		ctrl.nand_strap_regs = NULL;
	}
	ctrl.nand_idm_io_ctrl_direct_reg = NULL;
	if (ctrl.nand_idm_regs) {
		iounmap(ctrl.nand_idm_regs);
		ctrl.nand_idm_regs = NULL;
	}
	ctrl.nand_intr_regs = NULL;
	if (ctrl.nand_regs) {
		iounmap(ctrl.nand_regs);
		ctrl.nand_regs = NULL;
	}
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("NAND driver for iProc chips");
