/*
 * Copyright (C) 2014-2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "cygnus-ssp.h"

#define I2S0  0
#define I2S1  1
#define I2S2  2
#define SPDIF 3

#define CYGNUS_TDM_RATE \
		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define PLL_NDIV_FRACT_MAX  (BIT(20)-1)   /* 20 bits max */

/* Begin register offset defines */
#define I2S_OUT_0_FCI_ID 0x180
#define I2S_OUT_1_FCI_ID 0x181
#define I2S_OUT_2_FCI_ID 0x182

#define AUD_MISC_SEROUT_OE_REG_BASE  0x01c
#define AUD_MISC_SEROUT_MCLK_OE  0x8
#define AUD_MISC_SEROUT_LRCK_OE  0x4
#define AUD_MISC_SEROUT_SCLK_OE  0x2
#define AUD_MISC_SEROUT_SDAT_OE  0x1

/* AUD_FMM_BF_CTRL_xxx regs */
#define BF_DST_CFG0_OFFSET  0x100
#define BF_DST_CFG1_OFFSET  0x104
#define BF_DST_CFG2_OFFSET  0x108

#define BF_DST_CTRL0_OFFSET 0x130
#define BF_DST_CTRL1_OFFSET 0x134
#define BF_DST_CTRL2_OFFSET 0x138

#define BF_SRC_CFG0_OFFSET  0x148
#define BF_SRC_CFG1_OFFSET  0x14c
#define BF_SRC_CFG2_OFFSET  0x150
#define BF_SRC_CFG3_OFFSET  0x154

#define BF_SRC_CTRL0_OFFSET 0x1c0
#define BF_SRC_CTRL1_OFFSET 0x1c4
#define BF_SRC_CTRL2_OFFSET 0x1c8
#define BF_SRC_CTRL3_OFFSET 0x1cc

#define BF_SRC_GRP0_OFFSET  0x1fc
#define BF_SRC_GRP1_OFFSET  0x200
#define BF_SRC_GRP2_OFFSET  0x204
#define BF_SRC_GRP3_OFFSET  0x208

/* AUD_FMM_IOP_OUT_I2S_xxx regs */
#define OUT_I2S_0_STREAM_CFG_OFFSET 0xa00
#define OUT_I2S_0_CFG_OFFSET        0xa04
#define OUT_I2S_0_MCLK_CFG_OFFSET   0xa0c

#define OUT_I2S_1_STREAM_CFG_OFFSET 0xa40
#define OUT_I2S_1_CFG_OFFSET        0xa44
#define OUT_I2S_1_MCLK_CFG_OFFSET   0xa4c

#define OUT_I2S_2_STREAM_CFG_OFFSET 0xa80
#define OUT_I2S_2_CFG_OFFSET        0xa84
#define OUT_I2S_2_MCLK_CFG_OFFSET   0xa8c

/* AUD_FMM_IOP_OUT_SPDIF_xxx regs */
#define SPDIF_STREAM_CFG_OFFSET  0xac0
#define SPDIF_CTRL_OFFSET        0xac4
#define SPDIF_FORMAT_CFG_OFFSET  0xad8
#define SPDIF_MCLK_CFG_OFFSET    0xadc

/* AUD_FMM_IOP_PLL_0_xxx regs */
#define IOP_PLL_0_MACRO_OFFSET    0xb00
#define IOP_PLL_0_MDIV_Ch0_OFFSET 0xb14
#define IOP_PLL_0_MDIV_Ch1_OFFSET 0xb18
#define IOP_PLL_0_MDIV_Ch2_OFFSET 0xb1c

#define IOP_PLL_0_ACTIVE_MDIV_Ch0_OFFSET 0xb30
#define IOP_PLL_0_ACTIVE_MDIV_Ch1_OFFSET 0xb34
#define IOP_PLL_0_ACTIVE_MDIV_Ch2_OFFSET 0xb38

/* AUD_FMM_IOP_xxx regs */
#define IOP_PLL_0_CONTROL_OFFSET     0xb04
#define IOP_PLL_0_USER_NDIV_OFFSET   0xb08
#define IOP_PLL_0_ACTIVE_NDIV_OFFSET 0xb20
#define IOP_PLL_0_RESET_OFFSET       0xb5c
#define IOP_NCO_0_CONTROL_OFFSET     0xb80
#define IOP_NCO_1_CONTROL_OFFSET     0xbc0

/* AUD_FMM_IOP_IN_I2S_xxx regs */
#define IN_I2S_0_STREAM_CFG_OFFSET 0xc00
#define IN_I2S_0_CFG_OFFSET        0xc04
#define IN_I2S_1_STREAM_CFG_OFFSET 0xc40
#define IN_I2S_1_CFG_OFFSET        0xc44
#define IN_I2S_2_STREAM_CFG_OFFSET 0xc80
#define IN_I2S_2_CFG_OFFSET        0xc84

/* End register offset defines */


/* AUD_FMM_IOP_OUT_I2S_x_MCLK_CFG_0_REG */
#define I2S_OUT_MCLKRATE_SHIFT 16
#define I2S_OUT_MCLKRATE_MASK  (0xF << I2S_OUT_MCLKRATE_SHIFT)

/* AUD_FMM_IOP_OUT_I2S_x_MCLK_CFG_REG */
#define I2S_OUT_PLLCLKSEL_SHIFT  0
#define I2S_OUT_PLLCLKSEL_MASK  (0xF << I2S_OUT_PLLCLKSEL_SHIFT)

/* AUD_FMM_IOP_OUT_I2S_x_STREAM_CFG */
#define I2S_OUT_STREAM_ENA  31
#define I2S_OUT_STREAM_CFG_GROUP_ID  20
#define I2S_OUT_STREAM_CFG_CHANNEL_GROUPING  24

/* AUD_FMM_IOP_IN_I2S_x_CAP */
#define I2S_IN_STREAM_CFG_CAP_ENA   31
#define I2S_IN_STREAM_CFG_0_GROUP_ID 4

/* AUD_FMM_IOP_OUT_I2S_x_I2S_CFG_REG */
#define I2S_OUT_CFGX_CLK_ENA         0
#define I2S_OUT_CFGX_DATA_ENABLE     1
#define I2S_OUT_CFGX_DATA_ALIGNMENT  6
#define I2S_OUT_CFGX_BITS_PER_SLOT  13
#define I2S_OUT_CFGX_VALID_SLOT     14
#define I2S_OUT_CFGX_FSYNC_WIDTH    18
#define I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32 26
#define I2S_OUT_CFGX_SLAVE_MODE     30
#define I2S_OUT_CFGX_TDM_MODE       31

/* AUD_FMM_BF_CTRL_SOURCECH_CFGx_REG */
#define BF_SRC_CFGX_SFIFO_ENA              0
#define BF_SRC_CFGX_BUFFER_PAIR_ENABLE     1
#define BF_SRC_CFGX_SAMPLE_CH_MODE         2
#define BF_SRC_CFGX_SFIFO_SZ_DOUBLE        5
#define BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY  10
#define BF_SRC_CFGX_BIT_RES               20
#define BF_SRC_CFGX_PROCESS_SEQ_ID_VALID  31

/* AUD_FMM_BF_CTRL_DESTCH_CFGx_REG */
#define BF_DST_CFGX_CAP_ENA              0
#define BF_DST_CFGX_BUFFER_PAIR_ENABLE   1
#define BF_DST_CFGX_DFIFO_SZ_DOUBLE      2
#define BF_DST_CFGX_NOT_PAUSE_WHEN_FULL 11
#define BF_DST_CFGX_FCI_ID              12
#define BF_DST_CFGX_CAP_MODE            24
#define BF_DST_CFGX_PROC_SEQ_ID_VALID   31

/* AUD_FMM_IOP_OUT_SPDIF_xxx */
#define SPDIF_0_OUT_DITHER_ENA     3
#define SPDIF_0_OUT_STREAM_ENA    31

/* AUD_FMM_IOP_NCO_xxx */
#define IOP_NCO_NUMERATOR        16
#define IOP_NCO_CONTROL_RESET    12
#define IOP_NCO_CONTROL_FREE_RUN  4

/* AUD_FMM_IOP_PLL_0_USER */
#define IOP_PLL_0_USER_NDIV_FRAC   10

/* AUD_FMM_IOP_PLL_0_ACTIVE */
#define IOP_PLL_0_ACTIVE_NDIV_FRAC 10


#define INIT_SSP_REGS(num) { \
		.i2s_stream_cfg = OUT_I2S_ ##num## _STREAM_CFG_OFFSET, \
		.i2s_cap_stream_cfg = IN_I2S_ ##num## _STREAM_CFG_OFFSET, \
		.i2s_cfg = OUT_I2S_ ##num## _CFG_OFFSET, \
		.i2s_cap_cfg = IN_I2S_ ##num## _CFG_OFFSET, \
		.i2s_mclk_cfg = OUT_I2S_ ##num## _MCLK_CFG_OFFSET, \
		.bf_destch_ctrl = BF_DST_CTRL ##num## _OFFSET, \
		.bf_destch_cfg = BF_DST_CFG ##num## _OFFSET, \
		.bf_sourcech_ctrl = BF_SRC_CTRL ##num## _OFFSET, \
		.bf_sourcech_cfg = BF_SRC_CFG ##num## _OFFSET \
}


static int group_id[CYGNUS_MAX_PLAYBACK_PORTS] = {0, 1, 2, 3};

/*
 * Choose one of the following valid mclk rates:
 * -----------------------------------------
 * macro pll_ch0   pll_ch1    pll_ch2
 * ----- -------  ----------  ----------
 * 0   4,096,000   8,192,000  16,384,000
 * 1   5,644,800  11,289,600  22,579,200
 * 2   6,144,000  12,288,000  24,576,000
 * 3  12,288,000  24,576,000  49,152,000
 * 4  22,579,200  45,158,400  90,316,800
 * 5  24,576,000  49,152,000  98,304,000
 * -----------------------------------------
 *
 * Use this table to look up the "macro" setting for the audio pll block.
 * There are 6 macro settings that produce some common mclk frequencies.
 * The pll has 3 output channels (1x, 2x, and 4x).
 */
struct pll_macro_entry {
	u32 mclk;
	u32 macro;
	u32 pll_ch_num;
};

static const struct pll_macro_entry pll_predef_mclk[] = {
	{ 4096000, 0, 0},
	{ 8192000, 0, 1},
	{16384000, 0, 2},

	{ 5644800, 1, 0},
	{11289600, 1, 1},
	{22579200, 1, 2},

	{ 6144000, 2, 0},
	{12288000, 2, 1},
	{24576000, 2, 2},

	{12288000, 3, 0},
	{24576000, 3, 1},
	{49152000, 3, 2},

	{22579200, 4, 0},
	{45158400, 4, 1},
	{90316800, 4, 2},

	{24576000, 5, 0},
	{49152000, 5, 1},
	{98304000, 5, 2},
};


struct _nco_clk_coeff {
	u32 mclk;
	u32 sample_inc;
	u32 numer;
	u32 denom;
	u32 phase_inc;
};

static const struct _nco_clk_coeff nco_clk_coeff[] = {
	{2048000,  0xD, 0x02F, 0x100, 0x029F16},
	{4096000,  0x6, 0x12F, 0x200, 0x053E2D},
	{6144000,  0x4, 0x065, 0x100, 0x07DD44},
	{8192000,  0x3, 0x12F, 0x400, 0x0A7C5A},
	{12288000, 0x2, 0x065, 0x200, 0x0FBA88},
	{24576000, 0x1, 0x065, 0x400, 0x1F7510},
	{49152000, 0x0, 0x465, 0x800, 0x3EEA20},

	{2822400,  0x9, 0x06F, 0x0C4, 0x039CD8},
	{5644800,  0x4, 0x133, 0x188, 0x0739B0},
	{11289600, 0x2, 0x133, 0x310, 0x0E7360},
	{22579200, 0x1, 0x133, 0x620, 0x1CE6C0},
	{45158400, 0x0, 0x753, 0xC40, 0x39CD81},
};
/*
 * Use this relationship to derive the sampling rate (lrclk)
 * lrclk = (mclk) / ((2*mclk_to_sclk_ratio) * (32 * SCLK))).
 *
 * Use mclk, macro and pll_ch from the table above
 *
 * Valid SCLK = 0/1/2/4/8/12
 *
 * mclk_to_sclk_ratio = number of MCLK per SCLK. Division is twice the
 * value programmed in this field.
 * Valid mclk_to_sclk_ratio = 1 through to 15
 *
 * eg: To set lrclk = 48khz, set mclk = 12288000, mclk_to_sclk_ratio = 2,
 * SCLK = 64
 */
struct _ssp_clk_coeff {
	u32 mclk;
	u32 rate;
	u32 sclk_rate;
	u32 mclk_rate;
};

static const struct _ssp_clk_coeff ssp_clk_coeff[] = {
	/* 8k */
	{ 4096000, 8000,  64,  4},
	{ 6144000, 8000,  64,  6},
	{ 8192000, 8000,  64,  8},
	{12288000, 8000,  64, 12},

	{ 4096000, 8000, 128,  2},
	{ 6144000, 8000, 128,  3},
	{ 8192000, 8000, 128,  4},
	{12288000, 8000, 128,  6},
	{16384000, 8000, 128,  8},
	{24576000, 8000, 128, 12},

	{ 4096000, 8000, 256,  1},
	{ 8192000, 8000, 256,  2},
	{12288000, 8000, 256,  3},
	{16384000, 8000, 256,  4},
	{24576000, 8000, 256,  6},
	{49152000, 8000, 256, 12},

	{ 8192000, 8000, 512,  1},
	{16384000, 8000, 512,  2},
	{24576000, 8000, 512,  3},
	{49152000, 8000, 512,  6},
	{98304000, 8000, 512, 12},

	/* 16k */
	{ 4096000, 16000,  64,  2},
	{ 6144000, 16000,  64,  3},
	{ 8192000, 16000,  64,  4},
	{12288000, 16000,  64,  6},
	{16384000, 16000,  64,  8},
	{24576000, 16000,  64, 12},

	{ 4096000, 16000, 128,  1},
	{ 8192000, 16000, 128,  2},
	{12288000, 16000, 128,  3},
	{16384000, 16000, 128,  4},
	{24576000, 16000, 128,  6},
	{49152000, 16000, 128, 12},

	{ 8192000, 16000, 256,  1},
	{16384000, 16000, 256,  2},
	{24576000, 16000, 256,  3},
	{49152000, 16000, 256,  6},
	{98304000, 16000, 256, 12},

	{16384000, 16000, 512,  1},
	{49152000, 16000, 512,  3},
	{98304000, 16000, 512,  6},

	/* 32k */
	{ 4096000, 32000,  64,  1},
	{ 8192000, 32000,  64,  2},
	{12288000, 32000,  64,  3},
	{16384000, 32000,  64,  4},
	{24576000, 32000,  64,  6},
	{49152000, 32000,  64, 12},

	{ 8192000, 32000, 128,  1},
	{16384000, 32000, 128,  2},
	{24576000, 32000, 128,  3},
	{49152000, 32000, 128,  6},
	{98304000, 32000, 128, 12},

	{16384000, 32000, 256,  1},
	{49152000, 32000, 256,  3},
	{98304000, 32000, 256,  6},

	{98304000, 32000, 512,  3},

	/* 44.1k */
	{ 5644800, 44100,  64, 1},
	{11289600, 44100,  64, 2},
	{22579200, 44100,  64, 4},
	{45158400, 44100,  64, 8},

	{11289600, 44100, 128, 1},
	{22579200, 44100, 128, 2},
	{45158400, 44100, 128, 4},
	{90316800, 44100, 128, 8},

	{22579200, 44100, 256, 1},
	{45158400, 44100, 256, 2},
	{90316800, 44100, 256, 4},

	{45158400, 44100, 512, 1},
	{90316800, 44100, 512, 2},

	/* 48k */
	{ 6144000, 48000,  64, 1},
	{12288000, 48000,  64, 2},
	{24576000, 48000,  64, 4},
	{49152000, 48000,  64, 8},

	{12288000, 48000, 128, 1},
	{24576000, 48000, 128, 2},
	{49152000, 48000, 128, 4},
	{98304000, 48000, 128, 8},

	{24576000, 48000, 256, 1},
	{49152000, 48000, 256, 2},
	{98304000, 48000, 256, 4},

	{49152000, 48000, 512, 1},
	{98304000, 48000, 512, 2},

	/* 88.2k */
	{11289600, 88200, 64, 1},
	{22579200, 88200, 64, 2},
	{45158400, 88200, 64, 4},
	{90316800, 88200, 64, 8},

	/* 96k */
	{12288000, 96000, 64, 1},
	{24576000, 96000, 64, 2},
	{49152000, 96000, 64, 4},
	{98304000, 96000, 64, 8},

	/* 176.4k */
	{22579200, 176400, 64, 1},
	{45158400, 176400, 64, 2},
	{90316800, 176400, 64, 4},

	/* 192k */
	{24576000, 192000,  64, 1},
	{49152000, 192000,  64, 2},
	{98304000, 192000,  64, 4},

	{49152000, 192000, 128, 1},
};

static struct cygnus_aio_port *cygnus_dai_get_portinfo(struct snd_soc_dai *dai)
{
	struct cygnus_audio *cygaud = snd_soc_dai_get_drvdata(dai);

	return &cygaud->portinfo[dai->id];
}

static void audio_pll0_init(void __iomem *audio_io)
{
	/* Set clock channel post divider ratio to 0x24 */
	writel(0x24, audio_io + IOP_PLL_0_MDIV_Ch0_OFFSET);
	writel(0x24, audio_io + IOP_PLL_0_MDIV_Ch1_OFFSET);
	writel(0x24, audio_io + IOP_PLL_0_MDIV_Ch2_OFFSET);
	/* Disable and enable digital and analog PLL */
	writel(0x3, audio_io + IOP_PLL_0_RESET_OFFSET);
	writel(0x0, audio_io + IOP_PLL_0_RESET_OFFSET);
}

static int audio_ssp_init_portregs(struct cygnus_aio_port *aio)
{
	u32 value, fci_id;

	if ((aio->portnum == I2S0) || (aio->portnum == I2S1)
			|| (aio->portnum == I2S2)) {
		/* FCI_ID */
		value = readl(aio->audio + aio->regs.i2s_stream_cfg);
		value &= ~0xff003ff;

		/* Set Group ID */
		writel(group_id[0], aio->audio + BF_SRC_GRP0_OFFSET);
		writel(group_id[1], aio->audio + BF_SRC_GRP1_OFFSET);
		writel(group_id[2], aio->audio + BF_SRC_GRP2_OFFSET);
		writel(group_id[3], aio->audio + BF_SRC_GRP3_OFFSET);

		/* Configure the AUD_FMM_IOP_OUT_I2S_x_I2S_CFG reg */
		value |= group_id[aio->portnum] << I2S_OUT_STREAM_CFG_GROUP_ID;
		value |= aio->portnum;
		value |= aio->channel_grouping <<
			I2S_OUT_STREAM_CFG_CHANNEL_GROUPING;
		writel(value, aio->audio + aio->regs.i2s_stream_cfg);

		/* Configure the AUD_FMM_BF_CTRL_SOURCECH_CFGX reg */
		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value &= ~BIT(BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY);
		value |= BIT(BF_SRC_CFGX_SFIFO_SZ_DOUBLE);
		value |= BIT(BF_SRC_CFGX_PROCESS_SEQ_ID_VALID);
		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);

		/* Configure the AUD_FMM_IOP_IN_I2S_x_CAP_STREAM_CFG_0 reg */
		value = readl(aio->audio + aio->regs.i2s_cap_stream_cfg);
		value &= ~0xf0;
		value |= group_id[aio->portnum] << I2S_IN_STREAM_CFG_0_GROUP_ID;
		writel(value, aio->audio + aio->regs.i2s_cap_stream_cfg);

		/* Configure the AUD_FMM_BF_CTRL_DESTCH_CFGX_REG_BASE reg */
		fci_id = I2S_OUT_0_FCI_ID + aio->portnum;

		value = readl(aio->audio + aio->regs.bf_destch_cfg);
		value |= BIT(BF_DST_CFGX_DFIFO_SZ_DOUBLE);
		value &= ~BIT(BF_DST_CFGX_NOT_PAUSE_WHEN_FULL);
		value |= (fci_id << BF_DST_CFGX_FCI_ID);
		value |= BIT(BF_DST_CFGX_PROC_SEQ_ID_VALID);
		writel(value, aio->audio + aio->regs.bf_destch_cfg);

	} else if (aio->portnum == SPDIF) {
		writel(group_id[3], aio->audio + BF_SRC_GRP3_OFFSET);

		value = readl(aio->audio + SPDIF_CTRL_OFFSET);
		value |= BIT(SPDIF_0_OUT_DITHER_ENA);
		writel(value, aio->audio + SPDIF_CTRL_OFFSET);

		value = readl(aio->audio + SPDIF_STREAM_CFG_OFFSET);
		value &= ~0x3ff;
		value |= 3;
		value |= BIT(SPDIF_0_OUT_STREAM_ENA);
		writel(value, aio->audio + SPDIF_STREAM_CFG_OFFSET);

		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value &= ~BIT(BF_SRC_CFGX_NOT_PAUSE_WHEN_EMPTY);
		value |= BIT(BF_SRC_CFGX_SFIFO_SZ_DOUBLE);
		value |= BIT(BF_SRC_CFGX_PROCESS_SEQ_ID_VALID);
		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
	} else {
		pr_err("Port not supported\n");
		return -EINVAL;
	}

	return 0;
}

static int audio_ssp_in_enable(struct cygnus_aio_port *aio, bool enable)
{
	u32 value;

	if (enable) {
		value = readl(aio->audio + aio->regs.bf_destch_cfg);
		value |= BIT(BF_DST_CFGX_CAP_ENA);
		writel(value, aio->audio + aio->regs.bf_destch_cfg);

		writel(0x1, aio->audio + aio->regs.bf_destch_ctrl);

		value = readl(aio->audio + aio->regs.i2s_cfg);
		value |= BIT(I2S_OUT_CFGX_CLK_ENA);
		writel(value, aio->audio + aio->regs.i2s_cfg);

		value = readl(aio->audio + aio->regs.i2s_cap_stream_cfg);
		value |= BIT(I2S_IN_STREAM_CFG_CAP_ENA);
		writel(value, aio->audio + aio->regs.i2s_cap_stream_cfg);
		aio->streams_on++;
	} else {
		value = readl(aio->audio + aio->regs.i2s_cap_stream_cfg);
		value &= ~BIT(I2S_IN_STREAM_CFG_CAP_ENA);
		writel(value, aio->audio + aio->regs.i2s_cap_stream_cfg);
		if (aio->streams_on == 1) {
			value = readl(aio->audio + aio->regs.i2s_cfg);
			value &= ~BIT(I2S_OUT_CFGX_CLK_ENA);
			writel(value, aio->audio + aio->regs.i2s_cfg);
		}

		writel(0x0, aio->audio + aio->regs.bf_destch_ctrl);

		value = readl(aio->audio + aio->regs.bf_destch_cfg);
		value &= ~BIT(BF_DST_CFGX_CAP_ENA);
		writel(value, aio->audio + aio->regs.bf_destch_cfg);
		aio->streams_on--;
	}

	return 0;
}

static int audio_ssp_out_enable(struct cygnus_aio_port *aio, bool enable)
{
	u32 value;

	if (aio->portnum < SPDIF) {
		if (enable) {
			value = readl(aio->audio + aio->regs.i2s_stream_cfg);
			value |= BIT(I2S_OUT_STREAM_ENA);
			writel(value, aio->audio + aio->regs.i2s_stream_cfg);

			writel(1, aio->audio + aio->regs.bf_sourcech_ctrl);

			value = readl(aio->audio + aio->regs.i2s_cfg);
			value |= BIT(I2S_OUT_CFGX_CLK_ENA);
			value |= BIT(I2S_OUT_CFGX_DATA_ENABLE);
			writel(value, aio->audio + aio->regs.i2s_cfg);

			value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
			value |= BIT(BF_SRC_CFGX_SFIFO_ENA);
			writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
			aio->streams_on++;
		} else {
			value = readl(aio->audio + aio->regs.i2s_stream_cfg);
			value &= ~BIT(I2S_OUT_STREAM_ENA);
			writel(value, aio->audio + aio->regs.i2s_stream_cfg);
			writel(0, aio->audio + aio->regs.bf_sourcech_ctrl);

			value = readl(aio->audio + aio->regs.i2s_cfg);
			if (aio->streams_on == 1) {
				value &= ~BIT(I2S_OUT_CFGX_CLK_ENA);
				value &= ~BIT(I2S_OUT_CFGX_DATA_ENABLE);
			} else {
				value &= ~BIT(I2S_OUT_CFGX_DATA_ENABLE);
			}
			writel(value, aio->audio + aio->regs.i2s_cfg);

			value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
			value &= ~BIT(BF_SRC_CFGX_SFIFO_ENA);
			writel(value, aio->audio + aio->regs.bf_sourcech_cfg);

			aio->streams_on--;
		}
	} else if (aio->portnum == SPDIF) {
		if (enable) {
			value = readl(aio->audio + SPDIF_FORMAT_CFG_OFFSET);
			value |= 0x3;
			writel(value, aio->audio + SPDIF_FORMAT_CFG_OFFSET);

			writel(1, aio->audio + aio->regs.bf_sourcech_ctrl);

			value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
			value |= BIT(BF_SRC_CFGX_SFIFO_ENA);
			writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
		} else {
			value = readl(aio->audio + SPDIF_FORMAT_CFG_OFFSET);
			value &= ~0x3;
			writel(value, aio->audio + SPDIF_FORMAT_CFG_OFFSET);
			writel(0, aio->audio + aio->regs.bf_sourcech_ctrl);

			value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
			value &= ~BIT(BF_SRC_CFGX_SFIFO_ENA);
			writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
		}
	} else {
		pr_err("Port not supported\n");
		return -EINVAL;
	}

	return 0;
}

static int audio_ssp_source_bitres(struct cygnus_aio_port *aio, unsigned bits)
{
	u32 mask = 0x1f;
	u32 value = 0;
	u32 shift = BF_SRC_CFGX_BIT_RES;

	if ((bits == 8) || (bits == 16) || (bits == 32)) {
		value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
		value &= ~(mask << shift);

		/* 32 bit mode is coded as 0 */
		if ((bits == 8) || (bits == 16))
			value |= (bits << shift);

		writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
		return 0;
	}

	pr_err("Bit resolution not supported %d\n", bits);
	return -EINVAL;
}

static int audio_ssp_dst_bitres(struct cygnus_aio_port *aio, unsigned bits)
{
	u32 value;
	u32 shift = BF_DST_CFGX_CAP_MODE;

	value = readl(aio->audio + aio->regs.bf_destch_cfg);
	if (bits == 16) {
		value |= 1 << shift;
		writel(value, aio->audio + aio->regs.bf_destch_cfg);
	} else if (bits == 32) {
		value &= ~(1 << shift);
		writel(value, aio->audio + aio->regs.bf_destch_cfg);
	} else {
		pr_err("Bit resolution not supported %d\n", bits);
		return -EINVAL;
	}

	return 0;
}

static int nco_configure_mclk(void __iomem *audio_base, int nco_id, int mclk)
{
	u32 value;
	int i;
	bool found = 0;
	int offset;
	int select = -1;
	const struct _nco_clk_coeff *p_entry = NULL;

	if (mclk % 256) {
		pr_err("%s mclk must be divisible by 256\n", __func__);
		return -EINVAL;
	}

	if (nco_id == 0) {
		offset = IOP_NCO_0_CONTROL_OFFSET;
		select = 3;
	} else {
		offset = IOP_NCO_1_CONTROL_OFFSET;
		select = 4;
	}

	for (i = 0; i < ARRAY_SIZE(nco_clk_coeff); i++) {
		p_entry = &nco_clk_coeff[i];

		if (p_entry->mclk == mclk) {
			found = 1;
			break;
		}
	}
	if (!found) {
		pr_err("No valid match found in nco_clk_coeff array\n");
		return -EINVAL;
	}

	pr_debug("mclk = %d, i = %d, denom = %d\n", mclk, i, p_entry->denom);

	/* Assert Reset rate manager loops */
	value = readl(audio_base + offset);
	value |= 1 << IOP_NCO_CONTROL_FREE_RUN;
	writel(value, audio_base + offset);

	/* set denominator */
	writel(p_entry->denom, audio_base + offset + 4);

	/* set numerator and sample_inc */
	value = p_entry->numer << IOP_NCO_NUMERATOR;
	value |= p_entry->sample_inc;
	writel(value, audio_base + offset + 8);

	/* set phase_inc */
	writel(p_entry->phase_inc, audio_base + offset + 12);

	pr_debug("NCO Control    = 0x%x\n", readl(audio_base + offset + 0));
	pr_debug("NCO Denom      = 0x%x\n", readl(audio_base + offset + 4));
	pr_debug("NCO sample_inc = 0x%x\n", readl(audio_base + offset + 8));
	pr_debug("NCO phase_inc  = 0x%x\n", readl(audio_base + offset + 12));

	return select;
}

static int pll_configure_mclk(void __iomem *audio_base, u32 mclk)
{
	int i = 0;
	bool found = 0;
	const struct pll_macro_entry *p_entry;

	for (i = 0; i < ARRAY_SIZE(pll_predef_mclk); i++) {
		p_entry = &pll_predef_mclk[i];

		if (p_entry->mclk == mclk) {
			found = 1;
			break;
		}
	}
	if (!found) {
		pr_err("%s No valid mclk freq (%u) found!\n", __func__, mclk);
		return -EINVAL;
	}

	writel(p_entry->macro, audio_base + IOP_PLL_0_MACRO_OFFSET);
	pr_debug("PLL MACRO = %d\n", p_entry->macro);

	return p_entry->pll_ch_num;
}

static int cygnus_ssp_set_clocks(struct cygnus_aio_port *aio)
{
	u32 value, i = 0;
	u32 mask = 0xF;
	u32 sclk;
	bool found = 0;
	const struct _ssp_clk_coeff *p_entry = NULL;

	if (!aio->lrclk) {
		pr_err("First set lrclk through hw_params()\n");
		return -EINVAL;
	}

	if (!aio->bitrate) {
		pr_err("%s Use .set_clkdiv() to set bitrate\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(ssp_clk_coeff); i++) {
		p_entry = &ssp_clk_coeff[i];
		if ((p_entry->rate == aio->lrclk) &&
				(p_entry->sclk_rate == aio->bitrate) &&
				(p_entry->mclk == aio->mclk)) {
			found = 1;
			break;
		}
	}
	if (!found) {
		pr_err("No valid match found in ssp_clk_coeff array\n");
		return -EINVAL;
	}

	sclk = aio->bitrate;
	if (sclk == 512)
		sclk = 0;
	/* sclks_per_1fs_div = sclk cycles/32 */
	sclk /= 32;
	/* Set sclk rate */
	if (aio->portnum != SPDIF) {
		/* Set number of bitclks per frame */
		value = readl(aio->audio + aio->regs.i2s_cfg);
		value &= ~(mask << I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32);
		value |= sclk << I2S_OUT_CFGX_SCLKS_PER_1FS_DIV32;
		writel(value, aio->audio + aio->regs.i2s_cfg);
		pr_debug("SCLS_PER_1FS_DIV32 = 0x%x\n", value);
	}

	/* Set MCLK_RATE ssp port (spdif and ssp are the same) */
	value = readl(aio->audio + aio->regs.i2s_mclk_cfg);
	value &= ~(I2S_OUT_MCLKRATE_MASK);
	value |= (p_entry->mclk_rate << I2S_OUT_MCLKRATE_SHIFT);
	writel(value, aio->audio + aio->regs.i2s_mclk_cfg);

	pr_debug("mclk cfg reg = 0x%x\n", value);
	pr_debug("bits per frame = %d, mclk = %d Hz, lrclk = %d Hz\n",
			aio->bitrate, aio->mclk, aio->lrclk);
	return 0;
}

static int cygnus_ssp_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);
	int rate, value;
	int ret = 0;

	pr_debug("%s port = %d\n", __func__, aio->portnum);
	pr_debug("params_channels %d\n", params_channels(params));
	pr_debug("rate %d\n", params_rate(params));
	pr_debug("format %d\n", params_format(params));

	rate = params_rate(params);
	if (aio->mode == CYGNUS_SSPMODE_TDM) {
		if ((rate == 192000) && (params_channels(params) > 4)) {
			pr_err("Cannot run %d channels at %dHz\n",
				params_channels(params), rate);
			return -EINVAL;
		}
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Configure channels as mono or stereo */
		if (params_channels(params) == 1) {
			value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
			value |= (1 << BF_SRC_CFGX_SAMPLE_CH_MODE);
			value &= ~(1 << BF_SRC_CFGX_BUFFER_PAIR_ENABLE);
			writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
		} else {
			value = readl(aio->audio + aio->regs.bf_sourcech_cfg);
			value &= ~(1 << BF_SRC_CFGX_SAMPLE_CH_MODE);
			writel(value, aio->audio + aio->regs.bf_sourcech_cfg);
		}
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S8:
			audio_ssp_source_bitres(aio, 8);
			break;

		case SNDRV_PCM_FORMAT_S16_LE:
			audio_ssp_source_bitres(aio, 16);
			break;

		case SNDRV_PCM_FORMAT_S32_LE:
		case SNDRV_PCM_FORMAT_S24_LE:
			audio_ssp_source_bitres(aio, 32);
			break;

		default:
			return -EINVAL;
		}
	} else {
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			audio_ssp_dst_bitres(aio, 16);
			break;

		case SNDRV_PCM_FORMAT_S32_LE:
		case SNDRV_PCM_FORMAT_S24_LE:
			audio_ssp_dst_bitres(aio, 32);
			break;

		default:
			return -EINVAL;
		}
	}

	aio->lrclk = rate;

	pr_debug("%s clksrc = %d\n", __func__, aio->clksrc);

	switch (aio->clksrc) {
	case CYGNUS_SSP_CLKSRC_NCO_0:
	case CYGNUS_SSP_CLKSRC_NCO_1:
	case CYGNUS_SSP_CLKSRC_PLL:
		ret = cygnus_ssp_set_clocks(aio);
		break;

	default:
		pr_err("clksrc is invalid. Use .set_sysclk to set.\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
 * This function sets if the ssp should use uses the pll or NCO and will
 * set the mclk frequency for that clock
 */
static int cygnus_ssp_set_sysclk(struct snd_soc_dai *dai,
			int clk_id, unsigned int freq, int dir)
{
	int sel;
	u32 value;
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);

	pr_debug("%s Enter port = %d\n", __func__, aio->portnum);

	switch (clk_id) {
	case CYGNUS_SSP_CLKSRC_NCO_0:
		sel = nco_configure_mclk(aio->audio, 0, freq);
		break;

	case CYGNUS_SSP_CLKSRC_NCO_1:
		sel = nco_configure_mclk(aio->audio, 1, freq);
		break;

	case CYGNUS_SSP_CLKSRC_PLL:
		sel = pll_configure_mclk(aio->audio, freq);
		break;

	default:
		pr_err("clksrc is not valid\n");
		return -EINVAL;
	}

	if (sel < 0) {
		pr_err("%s Setting mclk failed.\n", __func__);
		return -EINVAL;
	}

	aio->mclk = freq;
	aio->clksrc = clk_id;

	pr_debug("%s Setting MCLKSEL to %d\n", __func__, sel);
	value = readl(aio->audio + aio->regs.i2s_mclk_cfg);
	value &= ~(I2S_OUT_PLLCLKSEL_MASK);
	value |= (sel << I2S_OUT_PLLCLKSEL_SHIFT);
	writel(value, aio->audio + aio->regs.i2s_mclk_cfg);

	return 0;
}

static int cygnus_ssp_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);

	pr_debug("%s  port = %d\n", __func__, aio->portnum);
	snd_soc_dai_set_dma_data(dai, substream, aio);

	return 0;
}

static int cygnus_ssp_dai_set_clkdiv(struct snd_soc_dai *cpu_dai,
				int div_id, int div)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);

	pr_debug("%s Enter\n", __func__);

	if (div_id != CYGNUS_SSP_FRAMEBITS_DIV)
		return -EINVAL;

	/* Can only run 64 bits per frame in i2s mode */
	if (aio->mode == CYGNUS_SSPMODE_I2S)
		return -EINVAL;

	if ((div != 128) && (div != 256) && (div != 512)) {
		pr_err("In TDM mode, bits per frame should be 128/256/512\n");
		return -EINVAL;
	}

	aio->bitrate = div;

	return 0;
}

/*
 * Bit    Update  Notes
 * 31     Yes     TDM Mode        (1 = TDM, 0 = i2s)
 * 30     Yes     Slave Mode	  (1 = Slave, 0 = Master)
 * 29:26  No      Sclks per frame
 * 25:18  Yes     FS Width
 * 17:14  No      Valid Slots
 * 13     Yes     Bits		  (1 = 16 bits, 0 = 32 bits)
 * 12:08  Yes     Bits per samp
 * 07     Yes     Justifcation    (1 = LSB, 0 = MSB)
 * 06     Yes     Alignment       (1 = Delay 1 clk, 0 = no delay
 * 05     Yes     SCLK polarity   (1 = Rising, 0 = Falling)
 * 04     Yes     LRCLK Polarity  (1 = High for left, 0 = Low for left)
 * 03:02  Yes     Reserved - write as zero
 * 01     No      Data Enable
 * 00     No      CLK Enable
 */
#define I2S_IN_CFG_REG_UPDATE_MASK   0x3C03C003

/* Input cfg is same as output, but the FS width is not a valid field */
#define I2S_OUT_CFG_REG_UPDATE_MASK  (I2S_IN_CFG_REG_UPDATE_MASK | 0x03FC0000)

static int cygnus_ssp_set_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);
	u32 ssp_curcfg;
	u32 ssp_newcfg;
	u32 val;
	u32 mask;

	pr_debug("%s Enter\n", __func__);

	if (aio->portnum == SPDIF)
		return -EINVAL;

	ssp_newcfg = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	/* Set the SSP up as slave */
	case SND_SOC_DAIFMT_CBM_CFM:
		ssp_newcfg |= (1 << I2S_OUT_CFGX_SLAVE_MODE);
		aio->slave = 1;
		break;
	/* Set the SSP up as master */
	case SND_SOC_DAIFMT_CBS_CFS:
		ssp_newcfg &= ~(1 << I2S_OUT_CFGX_SLAVE_MODE);
		aio->slave = 0;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
		return -EINVAL;
	case SND_SOC_DAIFMT_I2S:
		ssp_newcfg |= (1 << I2S_OUT_CFGX_DATA_ALIGNMENT);
		ssp_newcfg |= (1 << I2S_OUT_CFGX_FSYNC_WIDTH);
		break;

	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		ssp_newcfg |= (1 << I2S_OUT_CFGX_TDM_MODE);

		/* DSP_A = data after FS, DSP_B = data during FS */
		if (SND_SOC_DAIFMT_DSP_A)
			ssp_newcfg |= (1 << I2S_OUT_CFGX_DATA_ALIGNMENT);

		ssp_newcfg |= (1 << I2S_OUT_CFGX_FSYNC_WIDTH);
		break;

	default:
		return -EINVAL;
	}

	/*
	 * SSP out cfg.
	 * Retain bits we do not want to update, then OR in new bits
	 */
	ssp_curcfg = readl(aio->audio + aio->regs.i2s_cfg);
	ssp_newcfg = (ssp_curcfg & I2S_IN_CFG_REG_UPDATE_MASK) | ssp_newcfg;
	writel(ssp_newcfg, aio->audio + aio->regs.i2s_cfg);

	/*
	 * SSP in cfg.
	 * Retain bits we do not want to update, then OR in new bits
	 */
	ssp_curcfg = readl(aio->audio + aio->regs.i2s_cap_cfg);
	ssp_newcfg = (ssp_curcfg & I2S_OUT_CFG_REG_UPDATE_MASK) | ssp_newcfg;
	writel(ssp_newcfg, aio->audio + aio->regs.i2s_cap_cfg);

	if ((0 <= aio->portnum) && (aio->portnum <= 2)) {
		val = readl(aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);

		/*
		 * Configure the word clk and bit clk as output or tristate
		 * Each port has 4 bits for controlling its pins.
		 * Shift the mask based upon port number.
		 */
		mask = AUD_MISC_SEROUT_LRCK_OE | AUD_MISC_SEROUT_SCLK_OE;
		mask = mask << (aio->portnum * 4);
		if (aio->slave)
			val |= mask;
		else
			val &= ~mask;

		pr_debug("%s  Set OE bits 0x%x\n", __func__, val);
		writel(val, aio->audio + AUD_MISC_SEROUT_OE_REG_BASE);
	}

	return 0;
}

static int cygnus_ssp_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);

	pr_debug("%s cmd %d at port = %d\n", __func__, cmd, aio->portnum);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			audio_ssp_out_enable(aio, 1);
			break;

		case SNDRV_PCM_TRIGGER_STOP:
			audio_ssp_out_enable(aio, 0);
			break;

		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			break;

		default:
			return -EINVAL;
		}
	} else {

		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			audio_ssp_in_enable(aio, 1);
			break;

		case SNDRV_PCM_TRIGGER_STOP:
			audio_ssp_in_enable(aio, 0);
			break;
		}
	}

	return 0;
}

static int cygnus_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
	unsigned int tx_mask, unsigned int rx_mask, int slots, int slot_width)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);
	u32 value;

	pr_debug("==> %s\n", __func__);

	if ((slots < 0) || (slots > 16))
		return -EINVAL;

	/* Slot value must be even */
	if (slots % 2)
		return -EINVAL;

	/* We encode 16 slots as 0 in the reg */
	if (slots == 16)
		slots  = 0;

	value = readl(aio->audio + aio->regs.i2s_cap_cfg);
	value &= ~(0xF << I2S_OUT_CFGX_VALID_SLOT);
	value |= (slots << I2S_OUT_CFGX_VALID_SLOT);
	writel(value, aio->audio + aio->regs.i2s_cap_cfg);

	value = readl(aio->audio + aio->regs.i2s_cfg);
	value &= ~(0xF << I2S_OUT_CFGX_VALID_SLOT);
	value |= (slots << I2S_OUT_CFGX_VALID_SLOT);
	writel(value, aio->audio + aio->regs.i2s_cfg);

	return 0;
}

int cygnus_ssp_get_mode(struct snd_soc_dai *cpu_dai)
{
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(cpu_dai);

	return aio->mode;
}
EXPORT_SYMBOL(cygnus_ssp_get_mode);


static void pll_fract_tweak_set(void __iomem *audio_base, u32 value)
{
	/*
	 * Read ACTIVE PLL registers for current values
	 * Write new values to the USER PLL registers
	 * Transition PLL Control to update the active PLL registers with user
	 * PLL registers
	 */
	u32 ndiv, mdiv0, mdiv1, mdiv2;

	ndiv = readl(audio_base + IOP_PLL_0_ACTIVE_NDIV_OFFSET);
	mdiv0 = readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch0_OFFSET);
	mdiv1 = readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch1_OFFSET);
	mdiv2 = readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch2_OFFSET);

	ndiv &= 0x3ff;
	ndiv |= value << IOP_PLL_0_USER_NDIV_FRAC;

	writel(7, audio_base + IOP_PLL_0_MACRO_OFFSET);
	writel(0, audio_base + IOP_PLL_0_CONTROL_OFFSET);
	writel(mdiv0, audio_base + IOP_PLL_0_MDIV_Ch0_OFFSET);
	writel(mdiv1, audio_base + IOP_PLL_0_MDIV_Ch1_OFFSET);
	writel(mdiv2, audio_base + IOP_PLL_0_MDIV_Ch2_OFFSET);
	writel(ndiv, audio_base + IOP_PLL_0_USER_NDIV_OFFSET);
	writel(1, audio_base + IOP_PLL_0_CONTROL_OFFSET);
}

static u32 pll_fract_tweak_get(void __iomem *audio_base)
{
	u32 ndiv_fract, ndiv_int, value;

	value = readl(audio_base + IOP_PLL_0_USER_NDIV_OFFSET);
	ndiv_fract = value >> IOP_PLL_0_USER_NDIV_FRAC;
	ndiv_int = value & 0x3FF;
	pr_debug("\nuser fract = %d, user int = %d\n", ndiv_fract, ndiv_int);

	value = readl(audio_base + IOP_PLL_0_ACTIVE_NDIV_OFFSET);
	ndiv_fract = value >> IOP_PLL_0_ACTIVE_NDIV_FRAC;
	ndiv_int = value & 0x3FF;
	pr_debug("\nactive fract = %d, active int = %d\n",
		ndiv_fract, ndiv_int);
	pr_debug("\nuser mdiv0 = %d, mdiv1 = %d, mdiv2 = %d\n",
		readl(audio_base + IOP_PLL_0_MDIV_Ch0_OFFSET),
		readl(audio_base + IOP_PLL_0_MDIV_Ch1_OFFSET),
		readl(audio_base + IOP_PLL_0_MDIV_Ch2_OFFSET));

	pr_debug("\nactive mdiv0 = %d, mdiv1 = %d, mdiv2 = %d\n",
		readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch0_OFFSET),
		readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch1_OFFSET),
		readl(audio_base + IOP_PLL_0_ACTIVE_MDIV_Ch2_OFFSET));

	return ndiv_fract;
}

/*
 * pll_tweak_get - read the pll fractional setting.
 *   kcontrol: The control for the speaker gain.
 *   ucontrol: The value that needs to be updated.
 */
static int pll_tweak_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);

	pr_debug("Enter %s\n", __func__);

	ucontrol->value.integer.value[0] = pll_fract_tweak_get(aio->audio);

	return 0;
}

/*
 * pll_tweak_put - set the pll fractional setting.
 *   kcontrol: The control for the pll tweak.
 *   ucontrol: The value that needs to be set.
 */
static int pll_tweak_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dai *dai = snd_kcontrol_chip(kcontrol);
	struct cygnus_aio_port *aio = cygnus_dai_get_portinfo(dai);
	int value;

	value = ucontrol->value.integer.value[0];
	if (value > PLL_NDIV_FRACT_MAX) {
		pr_err("Invalid range. (0 - %lu)\n", PLL_NDIV_FRACT_MAX);
		return -EINVAL;
	}

	pr_debug("Enter %s with value %d\n", __func__, value);
	pll_fract_tweak_set(aio->audio, value);

	return 0;
}

static const struct snd_kcontrol_new pll_tweak_controls[] = {
	SOC_SINGLE_EXT("PLL Tweak", 0, 0, PLL_NDIV_FRACT_MAX, 0,
	pll_tweak_get, pll_tweak_put),
};

int cygnus_ssp_add_pll_tweak_controls(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	return snd_soc_add_dai_controls(cpu_dai,
				pll_tweak_controls,
				ARRAY_SIZE(pll_tweak_controls));
}
EXPORT_SYMBOL(cygnus_ssp_add_pll_tweak_controls);


static const struct snd_soc_dai_ops cygnus_ssp_dai_ops = {
	.startup	= cygnus_ssp_startup,
	.trigger	= cygnus_ssp_trigger,
	.hw_params	= cygnus_ssp_hw_params,
	.set_fmt	= cygnus_ssp_set_fmt,
	.set_sysclk	= cygnus_ssp_set_sysclk,
	.set_clkdiv	= cygnus_ssp_dai_set_clkdiv,
	.set_tdm_slot	= cygnus_set_dai_tdm_slot,
};

static struct snd_soc_dai_driver cygnus_tdm_dai_template = {
	.playback = {
		.channels_min = 2,
		.channels_max = 16,
		.rates = CYGNUS_TDM_RATE | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
				SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 16,
		.rates = CYGNUS_TDM_RATE | SNDRV_PCM_RATE_192000,
		.formats =  SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &cygnus_ssp_dai_ops,
};

static struct snd_soc_dai_driver cygnus_i2s_dai_template = {
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = CYGNUS_TDM_RATE | SNDRV_PCM_RATE_88200 |
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S8 |
				SNDRV_PCM_FMTBIT_S16_LE |
				SNDRV_PCM_FMTBIT_S24_LE |
				SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = CYGNUS_TDM_RATE | SNDRV_PCM_RATE_88200 |
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			SNDRV_PCM_RATE_192000,
		.formats =  SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &cygnus_ssp_dai_ops,
};

static struct snd_soc_dai_driver cygnus_ssp_dai[CYGNUS_MAX_PORTS];

static const struct snd_soc_component_driver cygnus_ssp_component = {
	.name		= "cygnus-audio",
};

static const struct of_device_id cygnus_ssp_of_match[] = {
	{ .compatible = "brcm,cygnus-audio" },
	{},
};
/*
 * Return < 0 if error
 * Return 0 if disabled
 * Return 1 if enabled and node is parsed successfully
 */
static int parse_ssp_child_node(struct platform_device *pdev,
				struct device_node *dn,
				struct cygnus_audio *cygaud,
				struct snd_soc_dai_driver *p_dai)
{
	struct cygnus_aio_port *aio;
	const char *mode;
	const char *channel_grp;
	const char *port_status;
	struct cygnus_ssp_regs ssp_regs[3];
	int rawval;
	int portnum = -1;
	int frame_bits;

	if ((of_property_read_string(dn, "port-status", &port_status)) != 0) {
		dev_err(&pdev->dev, "Missing port-status property\n");
		return -EINVAL;
	}

	/* If not enabled return 1 */
	if (strcmp(port_status, "enabled") != 0)
		return 1;

	if (of_property_read_u32(dn, "ssp-port-id", &rawval) != 0) {
		dev_err(&pdev->dev, "%s: invalid ssp-port-id\n", __func__);
		return -EINVAL;
	}

	if (rawval == 0)
		portnum = I2S0;
	else if (rawval == 1)
		portnum = I2S1;
	else if (rawval == 2)
		portnum = I2S2;
	else if (rawval == 3)
		portnum = SPDIF;
	else
		return -EINVAL;

	if ((of_property_read_string(dn, "mode", &mode)) != 0) {
		dev_err(&pdev->dev, "Missing mode property\n");
		return -EINVAL;
	}

	if (of_property_read_string(dn, "channel-group", &channel_grp) != 0) {
		dev_err(&pdev->dev, "Missing channel_group property\n");
		return -EINVAL;
	}

	aio = &cygaud->portinfo[portnum];

	aio->audio = cygaud->audio;
	aio->clksrc = -1;
	aio->portnum = portnum;

	if ((portnum == I2S0) || (portnum == I2S1) || (portnum == I2S2)) {
		ssp_regs[I2S0] = (struct cygnus_ssp_regs) INIT_SSP_REGS(0);
		ssp_regs[I2S1] = (struct cygnus_ssp_regs) INIT_SSP_REGS(1);
		ssp_regs[I2S2] = (struct cygnus_ssp_regs) INIT_SSP_REGS(2);

		aio->regs = ssp_regs[portnum];

		if (strstr(mode, "i2s")) {
			*p_dai = cygnus_i2s_dai_template;
			aio->mode = CYGNUS_SSPMODE_I2S;
		} else if (strstr(mode, "tdm")) {
			*p_dai = cygnus_tdm_dai_template;
			aio->mode = CYGNUS_SSPMODE_TDM;
		} else {
			return -EINVAL;
		}

	} else { /* SPDIF case */
		aio->regs.bf_sourcech_cfg = BF_SRC_CFG3_OFFSET;
		aio->regs.bf_sourcech_ctrl = BF_SRC_CTRL3_OFFSET;
		aio->regs.i2s_mclk_cfg = SPDIF_MCLK_CFG_OFFSET;
		aio->regs.i2s_stream_cfg = SPDIF_STREAM_CFG_OFFSET;

		*p_dai = cygnus_i2s_dai_template;

		/* For the purposes of this code SPDIF can be I2S mode */
		aio->mode = CYGNUS_SSPMODE_I2S;
	}

	if (of_property_read_string(dn, "dai-name", &p_dai->name) != 0) {
		dev_err(&pdev->dev, "Missing dai-name property\n");
		return -EINVAL;
	}

	if (aio->mode == CYGNUS_SSPMODE_TDM) {
		const char *propname = "tdm-bits-per-frame";

		if (of_property_read_u32(dn, propname, &frame_bits)) {
			dev_err(&pdev->dev, "%s: %s not found\n",
					__func__, propname);
			return -EINVAL;
		}

		if ((frame_bits != 128) && (frame_bits != 256)
						&& (frame_bits != 512)) {
			dev_err(&pdev->dev, "In TDM mode, frame bits should be 128/256/512\n");
			return -EINVAL;
		}

		aio->bitrate = frame_bits;
	} else {
		aio->bitrate = 64; /* I2S must be 64 */
	}

	/* Handle the channel grouping */
	if (portnum == I2S0) {
		if (strstr(channel_grp, "2_0")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x1;
		} else if (strstr(channel_grp, "3_1")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x3;
		} else if (strstr(channel_grp, "5_1")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x7;
		} else {
			dev_err(&pdev->dev, "Invalid channel grouping\n");
			return -EINVAL;
		}
	}
	if (portnum == I2S1) {
		if (strstr(channel_grp, "2_0")) {
			group_id[portnum] = 1;
			aio->channel_grouping = 0x1;
		} else if (strstr(channel_grp, "3_1")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x3;
		} else if (strstr(channel_grp, "5_1")) {
			group_id[portnum] = 0;
			aio->channel_grouping = 0x7;
		} else {
			dev_err(&pdev->dev, "Invalid channel grouping\n");
			return -EINVAL;
		}
	}
	if (portnum == I2S2) {
		if (strstr(channel_grp, "2_0")) {
			group_id[I2S2] = 2;
			aio->channel_grouping = 0x1;
		} else if (strstr(channel_grp, "5_1")) {
			group_id[I2S2] = 0;
			aio->channel_grouping = 0x7;
		} else {
			dev_err(&pdev->dev, "Invalid channel grouping\n");
			return -EINVAL;
		}
	}
	if (portnum == SPDIF) {
		group_id[SPDIF] = 3;
		aio->channel_grouping = 0x1;
	}

	dev_dbg(&pdev->dev, "%s portnum = %d\n", __func__, aio->portnum);
	aio->streams_on = 0;
	audio_ssp_init_portregs(aio);
	return 0;
}

static int cygnus_ssp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child_node;
	struct resource *res = pdev->resource;
	struct cygnus_audio *cygaud;
	int err = -EINVAL;
	int node_count;
	int active_port_count;

	if (!of_match_device(cygnus_ssp_of_match, dev)) {
		dev_err(dev, "Failed to find ssp controller\n");
		return -ENODEV;
	}

	cygaud = devm_kzalloc(dev, sizeof(struct cygnus_audio), GFP_KERNEL);
	if (!cygaud)
		return -ENOMEM;

	dev_set_drvdata(dev, cygaud);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cygaud->audio = devm_ioremap_resource(dev, res);
	if (IS_ERR(cygaud->audio)) {
		dev_err(dev, "audio_io ioremap failed\n");
		return PTR_ERR(cygaud->audio);
	}

	audio_pll0_init(cygaud->audio);

	node_count = 0;
	node_count = of_get_child_count(pdev->dev.of_node);
	if ((node_count < 1) || (node_count > CYGNUS_MAX_PORTS)) {
		dev_err(dev, "Incorrct number of child nodes\n");
		return -EINVAL;
	}

	active_port_count = 0;
	for_each_available_child_of_node(pdev->dev.of_node, child_node) {
		err = parse_ssp_child_node(pdev, child_node, cygaud,
					&cygnus_ssp_dai[active_port_count]);

		/* negative is err, 0 is active and good, 1 is disabled */
		if (err < 0)
			return err;
		else if (err == 0)
			active_port_count++;
	}

	dev_dbg(dev, "Registering %d DAIs\n", active_port_count);
	err = devm_snd_soc_register_component(dev, &cygnus_ssp_component,
				cygnus_ssp_dai, active_port_count);
	if (err) {
		dev_err(dev, "snd_soc_register_dai failed\n");
		return err;
	}

	cygaud->irq_num = platform_get_irq(pdev, 0);
	if (cygaud->irq_num <= 0) {
		dev_err(dev, "platform_get_irq failed\n");
		return cygaud->irq_num;
	}

	err = cygnus_soc_platform_register(dev, cygaud);
	if (err) {
		dev_err(dev, "platform reg error %d\n", err);
		return err;
	}

	return 0;
}

static int cygnus_ssp_remove(struct platform_device *pdev)
{
	cygnus_soc_platform_unregister(&pdev->dev);

	return 0;
}

static struct platform_driver cygnus_ssp_driver = {
	.probe		= cygnus_ssp_probe,
	.remove		= cygnus_ssp_remove,
	.driver		= {
		.name	= "cygnus-ssp",
		.of_match_table = cygnus_ssp_of_match,
	},
};

module_platform_driver(cygnus_ssp_driver)

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Cygnus ASoC SSP Interface");
