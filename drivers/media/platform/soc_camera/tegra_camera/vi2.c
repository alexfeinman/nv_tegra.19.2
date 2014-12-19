/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/ktime.h>
#include <linux/platform_device.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/tegra_v4l2_camera.h>

#include <mach/clk.h>
#include <mach/../../iomap.h>

#include "nvhost_syncpt.h"
#include "common.h"

#define PORT_OVERRIDE 0

#define TEGRA_SYNCPT_CSI_WAIT_TIMEOUT                   40

#define TEGRA_VI_SYNCPT_CSI_A				NVSYNCPT_VI_0_3
#define TEGRA_VI_SYNCPT_CSI_B				NVSYNCPT_VI_1_3

#define TEGRA_VI_CFG_VI_INCR_SYNCPT			0x000
#define TEGRA_VI_CFG_VI_INCR_SYNCPT_CNTRL		0x004
#define TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR		0x008
#define TEGRA_VI_CFG_CTXSW				0x020
#define TEGRA_VI_CFG_INTSTATUS				0x024
#define TEGRA_VI_CFG_PWM_CONTROL			0x038
#define TEGRA_VI_CFG_PWM_HIGH_PULSE			0x03c
#define TEGRA_VI_CFG_PWM_LOW_PULSE			0x040
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_A			0x044
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_B			0x048
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_C			0x04c
#define TEGRA_VI_CFG_PWM_SELECT_PULSE_D			0x050
#define TEGRA_VI_CFG_VGP1				0x064
#define TEGRA_VI_CFG_VGP2				0x068
#define TEGRA_VI_CFG_VGP3				0x06c
#define TEGRA_VI_CFG_VGP4				0x070
#define TEGRA_VI_CFG_VGP5				0x074
#define TEGRA_VI_CFG_VGP6				0x078
#define TEGRA_VI_CFG_INTERRUPT_MASK			0x08c
#define TEGRA_VI_CFG_INTERRUPT_TYPE_SELECT		0x090
#define TEGRA_VI_CFG_INTERRUPT_POLARITY_SELECT		0x094
#define TEGRA_VI_CFG_INTERRUPT_STATUS			0x098
#define TEGRA_VI_CFG_VGP_SYNCPT_CONFIG			0x0ac
#define TEGRA_VI_CFG_VI_SW_RESET			0x0b4
#define TEGRA_VI_CFG_CG_CTRL				0x0b8
#define TEGRA_VI_CFG_VI_MCCIF_FIFOCTRL			0x0e4
#define TEGRA_VI_CFG_TIMEOUT_WCOAL_VI			0x0e8
#define TEGRA_VI_CFG_DVFS				0x0f0
#define TEGRA_VI_CFG_RESERVE				0x0f4
#define TEGRA_VI_CFG_RESERVE_1				0x0f8

#define TEGRA_VI_CSI_0_SW_RESET				0x100
#define TEGRA_VI_CSI_0_SINGLE_SHOT			0x104
#define TEGRA_VI_CSI_0_SINGLE_SHOT_STATE_UPDATE		0x108
#define TEGRA_VI_CSI_0_IMAGE_DEF			0x10c
#define TEGRA_VI_CSI_0_RGB2Y_CTRL			0x110
#define TEGRA_VI_CSI_0_MEM_TILING			0x114
#define TEGRA_VI_CSI_0_CSI_IMAGE_SIZE			0x118
#define TEGRA_VI_CSI_0_CSI_IMAGE_SIZE_WC		0x11c
#define TEGRA_VI_CSI_0_CSI_IMAGE_DT			0x120
#define TEGRA_VI_CSI_0_SURFACE0_OFFSET_MSB		0x124
#define TEGRA_VI_CSI_0_SURFACE0_OFFSET_LSB		0x128
#define TEGRA_VI_CSI_0_SURFACE1_OFFSET_MSB		0x12c
#define TEGRA_VI_CSI_0_SURFACE1_OFFSET_LSB		0x130
#define TEGRA_VI_CSI_0_SURFACE2_OFFSET_MSB		0x134
#define TEGRA_VI_CSI_0_SURFACE2_OFFSET_LSB		0x138
#define TEGRA_VI_CSI_0_SURFACE0_BF_OFFSET_MSB		0x13c
#define TEGRA_VI_CSI_0_SURFACE0_BF_OFFSET_LSB		0x140
#define TEGRA_VI_CSI_0_SURFACE1_BF_OFFSET_MSB		0x144
#define TEGRA_VI_CSI_0_SURFACE1_BF_OFFSET_LSB		0x148
#define TEGRA_VI_CSI_0_SURFACE2_BF_OFFSET_MSB		0x14c
#define TEGRA_VI_CSI_0_SURFACE2_BF_OFFSET_LSB		0x150
#define TEGRA_VI_CSI_0_SURFACE0_STRIDE			0x154
#define TEGRA_VI_CSI_0_SURFACE1_STRIDE			0x158
#define TEGRA_VI_CSI_0_SURFACE2_STRIDE			0x15c
#define TEGRA_VI_CSI_0_SURFACE_HEIGHT0			0x160
#define TEGRA_VI_CSI_0_ISPINTF_CONFIG			0x164
#define TEGRA_VI_CSI_0_ERROR_STATUS			0x184
#define TEGRA_VI_CSI_0_ERROR_INT_MASK			0x188
#define TEGRA_VI_CSI_0_WD_CTRL				0x18c
#define TEGRA_VI_CSI_0_WD_PERIOD			0x190

#define TEGRA_VI_CSI_1_SW_RESET				0x200
#define TEGRA_VI_CSI_1_SINGLE_SHOT			0x204
#define TEGRA_VI_CSI_1_SINGLE_SHOT_STATE_UPDATE		0x208
#define TEGRA_VI_CSI_1_IMAGE_DEF			0x20c
#define TEGRA_VI_CSI_1_RGB2Y_CTRL			0x210
#define TEGRA_VI_CSI_1_MEM_TILING			0x214
#define TEGRA_VI_CSI_1_CSI_IMAGE_SIZE			0x218
#define TEGRA_VI_CSI_1_CSI_IMAGE_SIZE_WC		0x21c
#define TEGRA_VI_CSI_1_CSI_IMAGE_DT			0x220
#define TEGRA_VI_CSI_1_SURFACE0_OFFSET_MSB		0x224
#define TEGRA_VI_CSI_1_SURFACE0_OFFSET_LSB		0x228
#define TEGRA_VI_CSI_1_SURFACE1_OFFSET_MSB		0x22c
#define TEGRA_VI_CSI_1_SURFACE1_OFFSET_LSB		0x230
#define TEGRA_VI_CSI_1_SURFACE2_OFFSET_MSB		0x234
#define TEGRA_VI_CSI_1_SURFACE2_OFFSET_LSB		0x238
#define TEGRA_VI_CSI_1_SURFACE0_BF_OFFSET_MSB		0x23c
#define TEGRA_VI_CSI_1_SURFACE0_BF_OFFSET_LSB		0x240
#define TEGRA_VI_CSI_1_SURFACE1_BF_OFFSET_MSB		0x244
#define TEGRA_VI_CSI_1_SURFACE1_BF_OFFSET_LSB		0x248
#define TEGRA_VI_CSI_1_SURFACE2_BF_OFFSET_MSB		0x24c
#define TEGRA_VI_CSI_1_SURFACE2_BF_OFFSET_LSB		0x250
#define TEGRA_VI_CSI_1_SURFACE0_STRIDE			0x254
#define TEGRA_VI_CSI_1_SURFACE1_STRIDE			0x258
#define TEGRA_VI_CSI_1_SURFACE2_STRIDE			0x25c
#define TEGRA_VI_CSI_1_SURFACE_HEIGHT0			0x260
#define TEGRA_VI_CSI_1_ISPINTF_CONFIG			0x264
#define TEGRA_VI_CSI_1_ERROR_STATUS			0x284
#define TEGRA_VI_CSI_1_ERROR_INT_MASK			0x288
#define TEGRA_VI_CSI_1_WD_CTRL				0x28c
#define TEGRA_VI_CSI_1_WD_PERIOD			0x290

#define TEGRA_CSI_CSI_CAP_CIL				0x808
#define TEGRA_CSI_CSI_CAP_CSI				0x818
#define TEGRA_CSI_CSI_CAP_PP				0x828
#define TEGRA_CSI_INPUT_STREAM_A_CONTROL		0x838
#define TEGRA_CSI_PIXEL_STREAM_A_CONTROL0		0x83c
#define TEGRA_CSI_PIXEL_STREAM_A_CONTROL1		0x840
#define TEGRA_CSI_PIXEL_STREAM_A_GAP			0x844
#define TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND		0x848
#define TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME		0x84c
#define TEGRA_CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK	0x850
#define TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS		0x854
#define TEGRA_CSI_CSI_SW_SENSOR_A_RESET			0x858
#define TEGRA_CSI_INPUT_STREAM_B_CONTROL		0x86c
#define TEGRA_CSI_PIXEL_STREAM_B_CONTROL0		0x870
#define TEGRA_CSI_PIXEL_STREAM_B_CONTROL1		0x874
#define TEGRA_CSI_PIXEL_STREAM_B_GAP			0x878
#define TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND		0x87c
#define TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME		0x880
#define TEGRA_CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK	0x884
#define TEGRA_CSI_CSI_PIXEL_PARSER_B_STATUS		0x888
#define TEGRA_CSI_CSI_SW_SENSOR_B_RESET			0x88c
#define TEGRA_CSI_PHY_CIL_COMMAND			0x908
#define TEGRA_CSI_CIL_PAD_CONFIG0			0x90c

#define TEGRA_CSI_CILA_PAD_CONFIG0			0x92c
#define TEGRA_CSI_CILA_PAD_CONFIG1			0x930
#define TEGRA_CSI_PHY_CILA_CONTROL0			0x934
#define TEGRA_CSI_CSI_CIL_A_INTERRUPT_MASK		0x938
#define TEGRA_CSI_CSI_CIL_A_STATUS			0x93c
#define TEGRA_CSI_CSI_CILA_STATUS			0x940
#define TEGRA_CSI_CIL_A_ESCAPE_MODE_COMMAND		0x944
#define TEGRA_CSI_CIL_A_ESCAPE_MODE_DATA		0x948
#define TEGRA_CSI_CSICIL_SW_SENSOR_A_RESET		0x94c

#define TEGRA_CSI_CILB_PAD_CONFIG0			0x960
#define TEGRA_CSI_CILB_PAD_CONFIG1			0x964
#define TEGRA_CSI_PHY_CILB_CONTROL0			0x968
#define TEGRA_CSI_CSI_CIL_B_INTERRUPT_MASK		0x96c
#define TEGRA_CSI_CSI_CIL_B_STATUS			0x970
#define TEGRA_CSI_CSI_CILB_STATUS			0x974
#define TEGRA_CSI_CIL_B_ESCAPE_MODE_COMMAND		0x978
#define TEGRA_CSI_CIL_B_ESCAPE_MODE_DATA		0x97c
#define TEGRA_CSI_CSICIL_SW_SENSOR_B_RESET		0x980

#define TEGRA_CSI_CILC_PAD_CONFIG0			0x994
#define TEGRA_CSI_CILC_PAD_CONFIG1			0x998
#define TEGRA_CSI_PHY_CILC_CONTROL0			0x99c
#define TEGRA_CSI_CSI_CIL_C_INTERRUPT_MASK		0x9a0
#define TEGRA_CSI_CSI_CIL_C_STATUS			0x9a4
#define TEGRA_CSI_CSI_CILC_STATUS			0x9a8
#define TEGRA_CSI_CIL_C_ESCAPE_MODE_COMMAND		0x9ac
#define TEGRA_CSI_CIL_C_ESCAPE_MODE_DATA		0x9b0
#define TEGRA_CSI_CSICIL_SW_SENSOR_C_RESET		0x9b4

#define TEGRA_CSI_CILD_PAD_CONFIG0			0x9c8
#define TEGRA_CSI_CILD_PAD_CONFIG1			0x9cc
#define TEGRA_CSI_PHY_CILD_CONTROL0			0x9d0
#define TEGRA_CSI_CSI_CIL_D_INTERRUPT_MASK		0x9d4
#define TEGRA_CSI_CSI_CIL_D_STATUS			0x9d8
#define TEGRA_CSI_CSI_CILD_STATUS			0x9dc
#define TEGRA_CSI_CIL_D_ESCAPE_MODE_COMMAND		0x9ec
#define TEGRA_CSI_CIL_D_ESCAPE_MODE_DATA		0x9f0
#define TEGRA_CSI_CSICIL_SW_SENSOR_D_RESET		0x9f4

#define TEGRA_CSI_CILE_PAD_CONFIG0			0xa08
#define TEGRA_CSI_CILE_PAD_CONFIG1			0xa0c
#define TEGRA_CSI_PHY_CILE_CONTROL0			0xa10
#define TEGRA_CSI_CSI_CIL_E_INTERRUPT_MASK		0xa14
#define TEGRA_CSI_CSI_CIL_E_STATUS			0xa18
#define TEGRA_CSI_CIL_E_ESCAPE_MODE_COMMAND		0xa1c
#define TEGRA_CSI_CIL_E_ESCAPE_MODE_DATA		0xa20
#define TEGRA_CSI_CSICIL_SW_SENSOR_E_RESET		0xa24

#define TEGRA_CSI_PATTERN_GENERATOR_CTRL_A		0xa68
#define TEGRA_CSI_PG_BLANK_A				0xa6c
#define TEGRA_CSI_PG_PHASE_A				0xa70
#define TEGRA_CSI_PG_RED_FREQ_A				0xa74
#define TEGRA_CSI_PG_RED_FREQ_RATE_A			0xa78
#define TEGRA_CSI_PG_GREEN_FREQ_A			0xa7c
#define TEGRA_CSI_PG_GREEN_FREQ_RATE_A			0xa80
#define TEGRA_CSI_PG_BLUE_FREQ_A			0xa84
#define TEGRA_CSI_PG_BLUE_FREQ_RATE_A			0xa88

#define TEGRA_CSI_PATTERN_GENERATOR_CTRL_B		0xa9c
#define TEGRA_CSI_PG_BLANK_B				0xaa0
#define TEGRA_CSI_PG_PHASE_B				0xaa4
#define TEGRA_CSI_PG_RED_FREQ_B				0xaa8
#define TEGRA_CSI_PG_RED_FREQ_RATE_B			0xaac
#define TEGRA_CSI_PG_GREEN_FREQ_B			0xab0
#define TEGRA_CSI_PG_GREEN_FREQ_RATE_B			0xab4
#define TEGRA_CSI_PG_BLUE_FREQ_B			0xab8
#define TEGRA_CSI_PG_BLUE_FREQ_RATE_B			0xabc

#define TEGRA_CSI_DPCM_CTRL_A				0xad0
#define TEGRA_CSI_DPCM_CTRL_B				0xad4
#define TEGRA_CSI_STALL_COUNTER				0xae8
#define TEGRA_CSI_CSI_READONLY_STATUS			0xaec
#define TEGRA_CSI_CSI_SW_STATUS_RESET			0xaf0
#define TEGRA_CSI_CLKEN_OVERRIDE			0xaf4
#define TEGRA_CSI_DEBUG_CONTROL				0xaf8
#define TEGRA_CSI_DEBUG_COUNTER_0			0xafc
#define TEGRA_CSI_DEBUG_COUNTER_1			0xb00
#define TEGRA_CSI_DEBUG_COUNTER_2			0xb04

static int vi2_port_is_valid(int port)
{
	return (((port) >= TEGRA_CAMERA_PORT_CSI_A) &&
		((port) <= TEGRA_CAMERA_PORT_CSI_E));
}

/* Clock settings for camera */
static struct tegra_camera_clk vi2_clks0[] = {
	{
		.name = "vi",
		.freq = 408000000,
		.use_devname = 1,
	},
	{
		.name = "vi_sensor",
		.freq = 24000000,
	},
	{
		.name = "csi",
		.freq = 408000000,
		.use_devname = 1,
	},
	{
		.name = "isp",
		.freq = 0,
	},
	{
		.name = "csus",
		.freq = 0,
		.use_devname = 1,
	},
	{
		.name = "sclk",
		.freq = 80000000,
	},
	{
		.name = "emc",
		.freq = 300000000,
	},
	{
		.name = "cilab",
		.freq = 102000000,
		.use_devname = 1,
	},
	/* Always put "p11_d" at the end */
	{
		.name = "pll_d",
		.freq = 927000000,
	},
};

static struct tegra_camera_clk vi2_clks1[] = {
	{
		.name = "vi",
		.freq = 408000000,
		.use_devname = 1,
	},
	{
		.name = "vi_sensor",
		.freq = 24000000,
	},
	{
		.name = "csi",
		.freq = 408000000,
		.use_devname = 1,
	},
	{
		.name = "isp",
		.freq = 0,
	},
	{
		.name = "sclk",
		.freq = 80000000,
	},
	{
		.name = "emc",
		.freq = 300000000,
	},
	{
		.name = "cilcd",
		.freq = 0,
		.use_devname = 1,
	},
	{
		.name = "cile",
		.freq = 0,
		.use_devname = 1,
	},
	/* Always put "p11_d" at the end */
	{
		.name = "pll_d",
		.freq = 927000000,
	},
};

static int clk_enabled;

#define MAX_DEVID_LENGTH	16

static void __iomem *mipi_cal = IO_ADDRESS(TEGRA_MIPI_CAL_BASE);
static u32 mipi_cal_read(unsigned long reg)
{
        return readl(mipi_cal + reg);
}

static void mipi_cal_write(u32 val, unsigned long reg)
{
        writel_relaxed(val, mipi_cal + reg);
}

static void tegra_vi_mipical_calibrate(struct tegra_camera_dev* cam, int id)
{

#define MIPI_CAL_MIPI_CAL_CTRL_0                0x0
#define MIPI_CAL_CIL_MIPI_CAL_STATUS_0          0x8
#define MIPI_CAL_CIL_MIPI_CAL_STATUS2_0          0xc
#define MIPI_CAL_CILA_MIPI_CAL_CONFIG_0         0x14
#define MIPI_CAL_CILB_MIPI_CAL_CONFIG_0         0x18
#define MIPI_CAL_CILC_MIPI_CAL_CONFIG_0         0x1c
#define MIPI_CAL_CILD_MIPI_CAL_CONFIG_0         0x20
#define MIPI_CAL_CILE_MIPI_CAL_CONFIG_0         0x24
#define MIPI_CAL_DSIA_MIPI_CAL_CONFIG_0         0x38
#define MIPI_CAL_DSIB_MIPI_CAL_CONFIG_0         0x3c
#define MIPI_CAL_DSIC_MIPI_CAL_CONFIG_0         0x40
#define MIPI_CAL_DSID_MIPI_CAL_CONFIG_0         0x44
#define	MIPI_CAL_MIPI_BIAS_PAD_CFG0_0			0x58
#define	MIPI_CAL_MIPI_BIAS_PAD_CFG1_0			0x58
#define MIPI_CAL_MIPI_BIAS_PAD_CFG2_0			0x60
#define	MIPI_CAL_DSIA_MIPI_CAL_CONFIG_2_0		0x64
#define MIPI_CAL_DSIB_MIPI_CAL_CONFIG_2_0		0x68
#define MIPI_CAL_CILC_MIPI_CAL_CONFIG_2_0		0x6c
#define MIPI_CAL_CILD_MIPI_CAL_CONFIG_2_0		0x70
#define MIPI_CAL_CSIE_MIPI_CAL_CONFIG_2_0		0x74
#define MIPI_CAL_CILC_MIPI_CAL_CONFIG_2_0		0x6c


#define MIPI_CAL_SELA							21
#define MIPI_CAL_SELB							21
#define MIPI_CAL_SELC							21
#define MIPI_CAL_SELD							21
#define MIPI_CAL_SELE							21
#define MIPI_CAL_CLKSELC						21
#define MIPI_CAL_CLKSELD						21
#define MIPI_CAL_CLKSELE						21
#define MIPI_CAL_SELDSIA						21
#define MIPI_CAL_SELDSIB						21
#define MIPI_CAL_CLKSELDSIA						21
#define MIPI_CAL_CLKSELDSIB						21
#define MIPI_BIAS_PAD_E_VCLAMP_REF				0
#define	PAD_PDVREG								1
#define	MIPI_CAL_STARTCAL						0
#define	MIPI_CAL_AUTOCAL_EN						1

#define REG(r, m, v) mipi_cal_write((mipi_cal_read(r) & ~(1 << m)) | (v << m), r);
#define PREG(r) printk(#r ": %08x\n", mipi_cal_read(r));

	ktime_t t = ktime_get();
	u32 cal_status = 0;

	if ( cam->clk_mipi_cal )
		clk_prepare_enable(cam->clk_mipi_cal);
	if ( cam->clk_72mhz )
		clk_prepare_enable(cam->clk_72mhz);
		
	clk_enabled = 1;
		
	//clk_set_rate(cam->clk_72mhz, 72000000);
	//clk_set_rate(cam->clk_mipi_cal, 68000000);

//printk("-------------------\nClearing Status Register\n-------------------\n");
mipi_cal_write(0xF1F10000, MIPI_CAL_CIL_MIPI_CAL_STATUS_0);


//printk("-------------------\nSetting SEL for CSIA/B. Clearing DSI/CSICDE\n-------------------\n");

REG(MIPI_CAL_CILA_MIPI_CAL_CONFIG_0,MIPI_CAL_SELA,0)    // CSIA pad
REG(MIPI_CAL_CILB_MIPI_CAL_CONFIG_0,MIPI_CAL_SELB,0)    // CSIB pad
REG(MIPI_CAL_CILC_MIPI_CAL_CONFIG_0,MIPI_CAL_SELC,1)    // CSID/DSIC pad
REG(MIPI_CAL_CILD_MIPI_CAL_CONFIG_0,MIPI_CAL_SELD,1)    // CSID/DSID pad
REG(MIPI_CAL_CILC_MIPI_CAL_CONFIG_2_0,MIPI_CAL_CLKSELC,1) // CSIC CLK/DSIC CLK pad
REG(MIPI_CAL_CILD_MIPI_CAL_CONFIG_2_0,MIPI_CAL_CLKSELD,1) // CSID CLK/DSID CLK pad
REG(MIPI_CAL_CILE_MIPI_CAL_CONFIG_0,MIPI_CAL_SELE,0)    // CSIE pad
REG(MIPI_CAL_CSIE_MIPI_CAL_CONFIG_2_0,MIPI_CAL_CLKSELE,0)    // CSIE CLK pad

//PREG(MIPI_CAL_CILA_MIPI_CAL_CONFIG_0)
//PREG(MIPI_CAL_CILB_MIPI_CAL_CONFIG_0)
//PREG(MIPI_CAL_CILC_MIPI_CAL_CONFIG_0)
//PREG(MIPI_CAL_CILD_MIPI_CAL_CONFIG_0)
//PREG(MIPI_CAL_CILE_MIPI_CAL_CONFIG_0)
//PREG(MIPI_CAL_DSIA_MIPI_CAL_CONFIG_0)
//PREG(MIPI_CAL_DSIB_MIPI_CAL_CONFIG_0)

REG(MIPI_CAL_DSIA_MIPI_CAL_CONFIG_0,MIPI_CAL_SELDSIA,0) // DSIA pad
REG(MIPI_CAL_DSIB_MIPI_CAL_CONFIG_0,MIPI_CAL_SELDSIB,0) // DSIB pad
REG(MIPI_CAL_DSIA_MIPI_CAL_CONFIG_2_0,MIPI_CAL_CLKSELDSIA,0) // DSIA CLK pad
REG(MIPI_CAL_DSIB_MIPI_CAL_CONFIG_2_0,MIPI_CAL_CLKSELDSIB,0) // DSIB CLK pad


//printk("----------------------\nProgramming MIPI Pad Controls \n----------------------\n");
REG(MIPI_CAL_MIPI_BIAS_PAD_CFG0_0, MIPI_BIAS_PAD_E_VCLAMP_REF, 1)
REG (MIPI_CAL_MIPI_BIAS_PAD_CFG2_0, PAD_PDVREG, 0)

//printf("-------------------\nSetting MIPI CAL CTRL.\n-------------------\n");
//reg(MIPI_CAL_MIPI_CAL_CTRL_0,MIPI_CAL_NOISE_FLT, 10) //
//reg(MIPI_CAL_MIPI_CAL_CTRL_0,MIPI_CAL_CLKEN_OVR, 1)  //
//reg list "MIPI_CAL_MIPI_CAL_CTRL_0" "*"


//printk("-------------------\nAsserting STARTCAL.\n-------------------\n");
//PREG(MIPI_CAL_CIL_MIPI_CAL_STATUS_0)
//REG(MIPI_CAL_MIPI_CAL_CTRL_0, MIPI_CAL_AUTOCAL_EN, 1)
REG(MIPI_CAL_MIPI_CAL_CTRL_0, MIPI_CAL_STARTCAL, 1)

t = ktime_get();
while( ktime_us_delta(ktime_get(), t) < 1000000 ) {
	cal_status = mipi_cal_read(MIPI_CAL_CIL_MIPI_CAL_STATUS2_0);
	printk("Status clk: %08x\n", cal_status);
	cal_status = mipi_cal_read(MIPI_CAL_CIL_MIPI_CAL_STATUS_0);
	printk("Status: %08x\n", cal_status);
	if ( cal_status & (1 << 16) ) {
		printk("Calibrated in %lld us\n", ktime_us_delta(ktime_get(), t));
		break;
	}
	usleep_range(200, 200);
}

if ( !(cal_status & (1 << 16)) )
	printk(KERN_ERR "Failed to calibrate\n");

//printk("-------------------\nDumping status.\n-------------------\n");
//PREG(MIPI_CAL_CIL_MIPI_CAL_STATUS_0);


	if ( cam->clk_72mhz )
		clk_disable_unprepare(cam->clk_72mhz);
	if ( cam->clk_mipi_cal )
		clk_disable_unprepare(cam->clk_mipi_cal);

}

static int vi2_clks_init(struct tegra_camera_dev *cam)
{
	struct platform_device *pdev = cam->ndev;
	struct tegra_camera_clk *clks;
	int i;

	clk_enabled = 0;

	switch (pdev->id) {
	case 0:
		cam->num_clks = ARRAY_SIZE(vi2_clks0);
		cam->clks = vi2_clks0;
		break;
	case 1:
		cam->num_clks = ARRAY_SIZE(vi2_clks1);
		cam->clks = vi2_clks1;
		break;
	default:
		dev_err(&pdev->dev, "Wrong device ID %d\n", pdev->id);
		return -ENODEV;
	}

	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];

		if (clks->use_devname) {
			char devname[MAX_DEVID_LENGTH];
			snprintf(devname, MAX_DEVID_LENGTH,
				 "tegra_%s", dev_name(&pdev->dev));
			clks->clk = clk_get_sys(devname, clks->name);
		} else
			clks->clk = clk_get(&pdev->dev, clks->name);
		if (IS_ERR_OR_NULL(clks->clk)) {
			dev_err(&pdev->dev, "Failed to get clock %s.\n",
				clks->name);
			return PTR_ERR(clks->clk);
		}

		if (clks->freq > 0)
			clk_set_rate(clks->clk, clks->freq);
	}
	
	cam->clk_mipi_cal = clk_get_sys("mipi-cal", NULL);
	if ( IS_ERR_OR_NULL(cam->clk_mipi_cal) ) {
                pr_warn("%s: cannot get mipi-cal clk: %d\n", __func__, cam->clk_mipi_cal);
                cam->clk_mipi_cal = NULL;
	}
	cam->clk_72mhz = clk_get_sys("clk72mhz", NULL);
	if ( IS_ERR_OR_NULL(cam->clk_72mhz) ) {
                pr_warn("%s: cannot get 72Mhz clk: %d\n", __func__, cam->clk_72mhz);
                cam->clk_72mhz = NULL;
	}

	//tegra_vi_mipical_calibrate(cam, 0);
	return 0;
}

static void vi2_clks_deinit(struct tegra_camera_dev *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = 0; i < cam->num_clks; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
			clk_put(clks->clk);
	}
	if ( cam->clk_mipi_cal )
		clk_put(cam->clk_mipi_cal);
	if ( cam->clk_72mhz )
		clk_put(cam->clk_72mhz);
}

static void vi2_clks_enable(struct tegra_camera_dev *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = 0; i < cam->num_clks - 1; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
			clk_prepare_enable(clks->clk);
	}

	if (cam->tpg_mode) {
		clks = &cam->clks[i];
		if (clks->clk) {
			clk_prepare_enable(clks->clk);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_PLLD_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_PLLD_DSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_MIPI_CSI_OUT_ENB, 0);
		}
	}
}

static void vi2_clks_disable(struct tegra_camera_dev *cam)
{
	struct tegra_camera_clk *clks;
	int i;

	for (i = 0; i < cam->num_clks - 1; i++) {
		clks = &cam->clks[i];
		if (clks->clk)
			clk_disable_unprepare(clks->clk);
	}

	if (cam->tpg_mode) {
		clks = &cam->clks[i];
		if (clks->clk) {
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_MIPI_CSI_OUT_ENB, 1);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_PLLD_CSI_OUT_ENB, 0);
			tegra_clk_cfg_ex(clks->clk,
					 TEGRA_CLK_PLLD_DSI_OUT_ENB, 0);
			clk_disable_unprepare(clks->clk);
		}
	}
}

static void vi2_init_syncpts(struct tegra_camera_dev *cam)
{
	cam->syncpt_id_csi_a = TEGRA_VI_SYNCPT_CSI_A; //nvhost_get_syncpt_client_managed("vi_csi_A");

	cam->syncpt_id_csi_b = TEGRA_VI_SYNCPT_CSI_B; //nvhost_get_syncpt_client_managed("vi_csi_B");
}

static void vi2_free_syncpts(struct tegra_camera_dev *cam)
{
//	nvhost_free_syncpt(cam->syncpt_id_csi_a);

//	nvhost_free_syncpt(cam->syncpt_id_csi_b);
}

static void vi2_incr_syncpts(struct tegra_camera_dev *cam)
{
	u32 val;

//	if (!nvhost_syncpt_read_ext_check(cam->ndev,
//			cam->syncpt_id_csi_a, &val))
		val = nvhost_syncpt_read_ext(cam->ndev, cam->syncpt_id_csi_a);
		cam->syncpt_csi_a = nvhost_syncpt_incr_max_ext(cam->ndev,
						cam->syncpt_id_csi_a, 1);

//	if (!nvhost_syncpt_read_ext_check(cam->ndev,
//			cam->syncpt_id_csi_b, &val))
		val = nvhost_syncpt_read_ext(cam->ndev, cam->syncpt_id_csi_b);
		cam->syncpt_csi_b = nvhost_syncpt_incr_max_ext(cam->ndev,
						cam->syncpt_id_csi_b, 1);
}

static void vi2_capture_clean(struct tegra_camera_dev *cam)
{
	/* Clean up status */
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_A_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_B_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_C_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_D_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_E_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILA_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILB_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILC_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CILD_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_B_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_ERROR_STATUS, 0xFFFFFFFF);
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_ERROR_STATUS, 0xFFFFFFFF);
	//TC_VI_REG_WT(cam, TEGRA_CSI_CSI_SW_SENSOR_A_RESET, 1);
	//TC_VI_REG_WT(cam, TEGRA_CSI_CSI_SW_SENSOR_B_RESET, 1);
}

static int vi2_capture_setup_csi_0(struct tegra_camera_dev *cam,
				    struct soc_camera_device *icd)
{
	struct soc_camera_subdev_desc *ssdesc = &icd->sdesc->subdev_desc;
	struct tegra_camera_platform_data *pdata = ssdesc->drv_priv;
	//icd->user_width = 1280;
	//icd->user_height = 720;

	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
											icd->current_fmt->host_fmt);
	const struct soc_camera_format_xlate *current_fmt = icd->current_fmt;
	u32 output_fourcc = current_fmt->host_fmt->fourcc;
	u32 output_format = 202;
#if 1
	switch (output_fourcc) {
		case V4L2_PIX_FMT_UYVY:
			output_format = 202;
			break;
		case V4L2_PIX_FMT_VYUY:
			output_format = 203;
			break;
		case V4L2_PIX_FMT_YUYV:
			output_format = 200;
			break;
		case V4L2_PIX_FMT_YVYU:
			output_format = 201;
			break;
		case V4L2_PIX_FMT_YUV420:
			output_format = 230;
			break;
		case V4L2_PIX_FMT_YVU420:
			output_format = 230; /* YUV420 planar */
			break;
		case V4L2_PIX_FMT_SBGGR8:
			output_format = 16;
			break;
		case V4L2_PIX_FMT_SBGGR10:
			output_format = 32;
			break;
		case V4L2_PIX_FMT_RGB32:
			output_format = 64;
			bytes_per_line = icd->user_width * 4;
			break;
		case V4L2_PIX_FMT_RGB24:
			output_format = 224;
			break;
	}
#endif
	printk(KERN_ERR "%s Using format %08x (%d) bytes_per_line: %d\n", __func__, output_fourcc, output_format, bytes_per_line);

	if ( icd->user_height == 480 )
		pdata->lanes = 4;
	else
		pdata->lanes = 4;
	
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0xf007);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_A_INTERRUPT_MASK, 0x0);
	//TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_CONTROL0, 0x080300f0);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_CONTROL0, 0x280301f0);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND, 0xf007);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_CONTROL1, 0x11);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_GAP, 0x140000);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_A_EXPECTED_FRAME, 0x300 << 4);

	TC_VI_REG_WT(cam, TEGRA_CSI_INPUT_STREAM_A_CONTROL,
			0x3f0000 | (pdata->lanes - 1));
	if (pdata->lanes == 4)
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND, 0x20000101);
	else
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND, 0x22020201);

	/* output format RAW10 T_R16_I, only support direct to mem */
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_IMAGE_DEF, (output_format << 16) | 0x1);
	/* input format is RAW10 */
	
	if ( output_format == 64 ) {
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_DT, 36);
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_SIZE_WC,
				icd->user_width * 3);
	}
	else {
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_DT, /*(1 << 12) |*/ 30);
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_SIZE_WC,
				icd->user_width * 2);
	}
		

	if (cam->tpg_mode) {
		TC_VI_REG_WT(cam, TEGRA_CSI_PATTERN_GENERATOR_CTRL_A,
				((cam->tpg_mode - 1) << 2) | 0x1);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_PHASE_A, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_RED_FREQ_A, 0x100010);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_RED_FREQ_RATE_A, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_GREEN_FREQ_A, 0x100010);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_GREEN_FREQ_RATE_A, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_BLUE_FREQ_A, 0x100010);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_BLUE_FREQ_RATE_A, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND, 0x22020202);

		/* output format A8B8G8R8, only support direct to mem */
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_IMAGE_DEF, (64 << 16) | 0x1);
		/* input format is RGB888 */
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_DT, 36);

		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_SIZE_WC,
				icd->user_width * 3);
	}

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_CSI_IMAGE_SIZE,
			(icd->user_height << 16) | icd->user_width);
#if 1
	if ( cam->sof )
		tegra_vi_mipical_calibrate(cam, 0);
#endif
	return 0;
}

static int vi2_capture_setup_csi_1(struct tegra_camera_dev *cam,
				     struct soc_camera_device *icd)
{
	struct soc_camera_subdev_desc *ssdesc = &icd->sdesc->subdev_desc;
	struct tegra_camera_platform_data *pdata = ssdesc->drv_priv;
	//icd->user_width = 1280;
	//icd->user_height = 720;

	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
											icd->current_fmt->host_fmt);
	const struct soc_camera_format_xlate *current_fmt = icd->current_fmt;
	u32 output_fourcc = current_fmt->host_fmt->fourcc;
	u32 output_format = 202;
#if 1
	switch (output_fourcc) {
		case V4L2_PIX_FMT_UYVY:
			output_format = 202;
			break;
		case V4L2_PIX_FMT_VYUY:
			output_format = 203;
			break;
		case V4L2_PIX_FMT_YUYV:
			output_format = 200;
			break;
		case V4L2_PIX_FMT_YVYU:
			output_format = 201;
			break;
		case V4L2_PIX_FMT_YUV420:
			output_format = 230;
			break;
		case V4L2_PIX_FMT_YVU420:
			output_format = 230; /* YUV420 planar */
			break;
		case V4L2_PIX_FMT_SBGGR8:
			output_format = 16;
			break;
		case V4L2_PIX_FMT_SBGGR10:
			output_format = 32;
			break;
		case V4L2_PIX_FMT_RGB32:
			output_format = 64;
			bytes_per_line = icd->user_width * 4;
			break;
		case V4L2_PIX_FMT_RGB24:
			output_format = 224;
			break;
	}
#endif
	printk(KERN_ERR "%s Using format %08x (%d) bytes_per_line: %d\n", __func__, output_fourcc, output_format, bytes_per_line);

	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0xf007);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_PIXEL_PARSER_B_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_B_CONTROL0, 0x280301f1);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND, 0xf007);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_B_CONTROL1, 0x11);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_B_GAP, 0x140000);
	TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_B_EXPECTED_FRAME, 0x300 << 4);

	TC_VI_REG_WT(cam, TEGRA_CSI_INPUT_STREAM_B_CONTROL,
			0x3f0000 | (pdata->lanes - 1));
	if (pdata->lanes == 4)
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND, 0x21010000);
	else if (pdata->lanes == 1 && pdata->port == TEGRA_CAMERA_PORT_CSI_E)
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND, 0x12020202);
	else
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND, 0x22010202);

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_IMAGE_DEF, (output_format << 16) | 0x1);

	if ( output_format == 64 ) {
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_DT, 36);
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_SIZE_WC,
				icd->user_width * 3);
	}
	else {
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_DT, /*(1 << 12) |*/ 30);
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_SIZE_WC,
				icd->user_width * 2);
	}

	if (cam->tpg_mode) {
		TC_VI_REG_WT(cam, TEGRA_CSI_PATTERN_GENERATOR_CTRL_B,
				((cam->tpg_mode - 1) << 2) | 0x1);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_PHASE_B, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_RED_FREQ_B, 0x100010);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_RED_FREQ_RATE_B, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_GREEN_FREQ_B, 0x100010);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_GREEN_FREQ_RATE_B, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_BLUE_FREQ_B, 0x100010);
		TC_VI_REG_WT(cam, TEGRA_CSI_PG_BLUE_FREQ_RATE_B, 0x0);
		TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND, 0x22020202);

		/* output format A8B8G8R8, only support direct to mem */
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_IMAGE_DEF, (64 << 16) | 0x1);
		/* input format is RGB888 */
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_DT, 36);
	}

	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_CSI_IMAGE_SIZE,
			(icd->user_height << 16) | icd->user_width);

#if 1
	if ( cam->sof )
		tegra_vi_mipical_calibrate(cam, 0);
#endif
	return 0;
}

static int vi2_capture_setup(struct tegra_camera_dev *cam)
{
	struct vb2_buffer *vb = cam->active;
	struct tegra_camera_buffer *buf = to_tegra_vb(vb);
	struct soc_camera_device *icd = buf->icd;
	struct soc_camera_subdev_desc *ssdesc = &icd->sdesc->subdev_desc;
	struct tegra_camera_platform_data *pdata = ssdesc->drv_priv;
	int port = pdata->port;
	/* TEST */
#if PORT_OVERRIDE
	port = TEGRA_CAMERA_PORT_CSI_B;
#endif
	/* Skip VI2/CSI2 setup for second and later frame capture */
	if (!cam->sof)
		return 0;

#ifdef DEBUG
	TC_VI_REG_WT(cam, TEGRA_CSI_DEBUG_CONTROL,
			0x3 | (0x1 << 5) | (0x40 << 8));
#endif

	/*
	 * PAD_CILA_PDVCLAMP 0, PAD_CILA_PDIO_CLK 0,
	 * PAD_CILA_PDIO 0, PAD_AB_BK_MODE 1
	 */
	TC_VI_REG_WT(cam, TEGRA_CSI_CILA_PAD_CONFIG0, (0x1 << 16) | (0x1 << 15));
	//TC_VI_REG_WT(cam, TEGRA_CSI_CILA_PAD_CONFIG0, (1 << 28) | (1 << 24) | (1 << 21 ) | (0x1 << 16) | (0x1 << 15));
	//TC_VI_REG_WT(cam, TEGRA_CSI_CILA_PAD_CONFIG0, (3 << 28) | (3 << 24) | (0x1 << 16) | (0x0 << 15));

	/* PAD_CILB_PDVCLAMP 0, PAD_CILB_PDIO_CLK 0, PAD_CILB_PDIO 0 */
	//TC_VI_REG_WT(cam, TEGRA_CSI_CILB_PAD_CONFIG0, (1 << 28) | (1 << 24) | (1 << 21 ) | (0x1 << 16) | (0x1 << 15));
	TC_VI_REG_WT(cam, TEGRA_CSI_CILB_PAD_CONFIG0, 0x0);

	/*
	 * PAD_CILC_PDVCLAMP 0, PAD_CILC_PDIO_CLK 0,
	 * PAD_CILC_PDIO 0, PAD_CD_BK_MODE 1
	 */
	TC_VI_REG_WT(cam, TEGRA_CSI_CILC_PAD_CONFIG0, (0x1 << 16) | (0x1 << 15));

	/* PAD_CILD_PDVCLAMP 0, PAD_CILD_PDIO_CLK 0, PAD_CILD_PDIO 0 */
	TC_VI_REG_WT(cam, TEGRA_CSI_CILD_PAD_CONFIG0, 0x0);

	/* PAD_CILE_PDVCLAMP 0, PAD_CILE_PDIO_CLK 0, PAD_CILE_PDIO 0 */
	TC_VI_REG_WT(cam, TEGRA_CSI_CILE_PAD_CONFIG0, 0x0);

	/* Common programming set for any config */
	TC_VI_REG_WT(cam, TEGRA_CSI_CLKEN_OVERRIDE, 0x0);

	//TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CIL_COMMAND, 0x22020202);

	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_A_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_B_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_C_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_D_INTERRUPT_MASK, 0x0);
	TC_VI_REG_WT(cam, TEGRA_CSI_CSI_CIL_E_INTERRUPT_MASK, 0x0);

	/*
	 * TODO: these values should be different with different
	 * sensor connected.
	 * Hardcode THS settle value just for TPG testing
	 */
	TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILA_CONTROL0, 0x42);
	TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILB_CONTROL0, 0x42);
	TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILC_CONTROL0, 0x42);
	TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILD_CONTROL0, 0x42);
	TC_VI_REG_WT(cam, TEGRA_CSI_PHY_CILE_CONTROL0, 0xa);
	
	/* Setup registers for CSI-A and CSI-B inputs */
	if (port == TEGRA_CAMERA_PORT_CSI_A)
		return vi2_capture_setup_csi_0(cam, icd);
	else if (port == TEGRA_CAMERA_PORT_CSI_B)
		return vi2_capture_setup_csi_1(cam, icd);
	else
		return -ENODEV;
}

static int vi2_capture_buffer_setup(struct tegra_camera_dev *cam,
			struct tegra_camera_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;
	struct soc_camera_subdev_desc *ssdesc = &icd->sdesc->subdev_desc;
	struct tegra_camera_platform_data *pdata = ssdesc->drv_priv;
	int port = pdata->port;
	/* TEST */
#if PORT_OVERRIDE
	port = TEGRA_CAMERA_PORT_CSI_B;
#endif

	//icd->user_width = 1280;
	//icd->user_height = 720;

	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
			icd->current_fmt->host_fmt);
	if ( port == TEGRA_CAMERA_PORT_CSI_A )
		switch (icd->current_fmt->host_fmt->fourcc) {
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YVU420:
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE0_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE0_OFFSET_LSB,
						 buf->buffer_addr);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE0_STRIDE,
						 bytes_per_line / 3 * 2 );

				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE1_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE1_OFFSET_LSB,
						 buf->buffer_addr_u);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE1_STRIDE,
						 bytes_per_line / 3);

				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE2_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE2_OFFSET_LSB,
						 buf->buffer_addr_v);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE2_STRIDE,
						 bytes_per_line / 3);
	//trace_printk("Y: %p, U: %p, V: %p\n", buf->buffer_addr, buf->buffer_addr_u, buf->buffer_addr_v);
			break;
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_VYUY:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		case V4L2_PIX_FMT_SBGGR8:
		case V4L2_PIX_FMT_SGBRG8:
		case V4L2_PIX_FMT_SBGGR10:
		case V4L2_PIX_FMT_SRGGB10:
		case V4L2_PIX_FMT_RGB32:
			//trace_printk("output channel: %d\n", buf->output_channel);
			switch (buf->output_channel) {
			case 0:
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE0_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE0_OFFSET_LSB,
						 buf->buffer_addr);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE0_STRIDE,
						 bytes_per_line);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE0_BF_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE0_BF_OFFSET_LSB,
						 buf->buffer_addr + bytes_per_line * icd->user_height / 2);
				break;
			case 1:
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE1_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE1_OFFSET_LSB,
						 buf->buffer_addr);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE1_STRIDE,
						 bytes_per_line);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE1_BF_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE1_BF_OFFSET_LSB,
						 buf->buffer_addr + bytes_per_line * icd->user_height / 2);
				break;
			case 2:
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE2_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE2_OFFSET_LSB,
						 buf->buffer_addr);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE2_STRIDE,
						 bytes_per_line);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE2_BF_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SURFACE2_BF_OFFSET_LSB,
						 buf->buffer_addr + bytes_per_line * icd->user_height / 2);
				break;
			}
			break;

		default:
			dev_err(&cam->ndev->dev, "Wrong host format %d\n",
				icd->current_fmt->host_fmt->fourcc);
			return -EINVAL;
		}
	else
		switch (icd->current_fmt->host_fmt->fourcc) {
		case V4L2_PIX_FMT_YUV420:
		case V4L2_PIX_FMT_YVU420:
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE0_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE0_OFFSET_LSB,
						 buf->buffer_addr);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE0_STRIDE,
						 bytes_per_line / 3 * 2 );

				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE1_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE1_OFFSET_LSB,
						 buf->buffer_addr_u);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE1_STRIDE,
						 bytes_per_line / 3);

				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE2_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE2_OFFSET_LSB,
						 buf->buffer_addr_v);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE2_STRIDE,
						 bytes_per_line / 3);
	//trace_printk("Y: %p, U: %p, V: %p\n", buf->buffer_addr, buf->buffer_addr_u, buf->buffer_addr_v);
			break;
		case V4L2_PIX_FMT_UYVY:
		case V4L2_PIX_FMT_VYUY:
		case V4L2_PIX_FMT_YUYV:
		case V4L2_PIX_FMT_YVYU:
		case V4L2_PIX_FMT_SBGGR8:
		case V4L2_PIX_FMT_SGBRG8:
		case V4L2_PIX_FMT_SBGGR10:
		case V4L2_PIX_FMT_SRGGB10:
		case V4L2_PIX_FMT_RGB32:
			//trace_printk("output channel: %d\n", buf->output_channel);
			switch (buf->output_channel) {
			case 0:
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE0_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE0_OFFSET_LSB,
						 buf->buffer_addr);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE0_STRIDE,
						 bytes_per_line);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE0_BF_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE0_BF_OFFSET_LSB,
						 buf->buffer_addr + bytes_per_line * icd->user_height / 2);
				break;
			case 1:
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE1_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE1_OFFSET_LSB,
						 buf->buffer_addr);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE1_STRIDE,
						 bytes_per_line);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE1_BF_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE1_BF_OFFSET_LSB,
						 buf->buffer_addr + bytes_per_line * icd->user_height / 2);
				break;
			case 2:
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE2_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE2_OFFSET_LSB,
						 buf->buffer_addr);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE2_STRIDE,
						 bytes_per_line);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE2_BF_OFFSET_MSB,
						 0x0);
				TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SURFACE2_BF_OFFSET_LSB,
						 buf->buffer_addr + bytes_per_line * icd->user_height / 2);
				break;
			}
			break;

		default:
			dev_err(&cam->ndev->dev, "Wrong host format %d\n",
				icd->current_fmt->host_fmt->fourcc);
			return -EINVAL;
		}

	return 0;
}

static int vi2_capture_error_status(struct tegra_camera_dev *cam)
{
	int err;

#ifdef DEBUG
	err = TC_VI_REG_RD(cam, TEGRA_CSI_DEBUG_COUNTER_0);
	if (err)
		pr_err("TEGRA_CSI_DEBUG_COUNTER_0 0x%08x\n", err);
#endif
	err = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_A_STATUS);
	if (err)
		pr_err("TEGRA_CSI_CSI_CIL_A_STATUS 0x%08x\n", err);
	err = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CILA_STATUS);
	if (err)
		pr_err("TEGRA_CSI_CSI_CILA_STATUS 0x%08x\n", err);
	err = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_B_STATUS);
	if (err)
		pr_err("TEGRA_CSI_CSI_CIL_B_STATUS 0x%08x\n", err);
	err = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CILB_STATUS);
	if (err)
		pr_err("TEGRA_CSI_CSI_CILB_STATUS 0x%08x\n", err);
	err = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_C_STATUS);
	if (err)
		pr_err("TEGRA_CSI_CSI_CIL_C_STATUS 0x%08x\n", err);
	err = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_D_STATUS);
	if (err)
		pr_err("TEGRA_CSI_CSI_CIL_D_STATUS 0x%08x\n", err);
	err = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_CIL_E_STATUS);
	if (err)
		pr_err("TEGRA_CSI_CSI_CIL_E_STATUS 0x%08x\n", err);
	err = TC_VI_REG_RD(cam, TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS);
	if (err)
		pr_err("TEGRA_CSI_CSI_PIXEL_PARSER_A_STATUS 0x%08x\n", err);
	err = TC_VI_REG_RD(cam, TEGRA_VI_CSI_0_ERROR_STATUS);
	if (err)
		pr_err("TEGRA_VI_CSI_0_ERROR_STATUS 0x%08x\n", err);

	return err;
}
#include <../drivers/video/tegra/host/host1x/host1x.h>


void nvhost_syncpt_set_min_eq_max(struct nvhost_syncpt *sp, u32 id)
{
        atomic_set(&sp->min_val[id], atomic_read(&sp->max_val[id]));
}

static struct nvhost_syncpt * syncpt_get_ext(struct platform_device *dev) {
        struct platform_device *pdev;
        struct nvhost_syncpt *sp;

        /* get the parent */
        pdev = to_platform_device(dev->dev.parent);
        sp = &(nvhost_get_host(pdev)->syncpt);
		return sp;
}
u32 nvhost_syncpt_read_min_ext(struct platform_device *dev, u32 id)
{
        struct platform_device *pdev;
        struct nvhost_syncpt *sp;

        /* get the parent */
        pdev = to_platform_device(dev->dev.parent);
        sp = &(nvhost_get_host(pdev)->syncpt);

        return nvhost_syncpt_read_min(sp, id);
}


u32 nvhost_syncpt_read_max_ext(struct platform_device *dev, u32 id)
{
        struct platform_device *pdev;
        struct nvhost_syncpt *sp;

        /* get the parent */
        pdev = to_platform_device(dev->dev.parent);
        sp = &(nvhost_get_host(pdev)->syncpt);

        return nvhost_syncpt_read_max(sp, id);
}
static void vi2_sw_reset(struct tegra_camera_dev *cam);

static int vi2_capture_start(struct tegra_camera_dev *cam,
				      struct tegra_camera_buffer *buf)
{
	struct soc_camera_device *icd = buf->icd;
	struct soc_camera_subdev_desc *ssdesc = &icd->sdesc->subdev_desc;
	struct tegra_camera_platform_data *pdata = ssdesc->drv_priv;
	int port = pdata->port;
	int err;
	/* TEST */
#if PORT_OVERRIDE
	port = TEGRA_CAMERA_PORT_CSI_B;
#endif

//trace_printk("%s +\n", __func__);
	err = vi2_capture_buffer_setup(cam, buf);
	if (err < 0)
		return err;
/*
	printk(KERN_ERR "syncpt min=%d, val=%d, max=%d\n", 
		nvhost_syncpt_read_min_ext(cam->ndev,
				TEGRA_VI_SYNCPT_CSI_A),
		nvhost_syncpt_read_ext(cam->ndev,
				TEGRA_VI_SYNCPT_CSI_A),
		nvhost_syncpt_read_max_ext(cam->ndev,
				TEGRA_VI_SYNCPT_CSI_A));
*/
//	printk(KERN_ERR "syncpt min=%d\n", 
//		nvhost_syncpt_read_min_ext(cam->ndev,
//				TEGRA_VI_SYNCPT_CSI_A));

	cam->syncpt_csi_a = nvhost_syncpt_read_min_ext(cam->ndev, TEGRA_VI_SYNCPT_CSI_A);

	TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT_CNTRL, (1 << 8));
	/* Only wait on CSI frame end syncpt if we're using CSI. */
	if (port == TEGRA_CAMERA_PORT_CSI_A) {
		TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT,
				(4 << 8) | TEGRA_VI_SYNCPT_CSI_A);
		TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND,
				0x0000f005);
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SINGLE_SHOT, 0x1);
//trace_printk("waiting for syncpt value %d\n", cam->syncpt_csi_a);
		err = nvhost_syncpt_wait_timeout_ext(cam->ndev,
				TEGRA_VI_SYNCPT_CSI_A,
				++cam->syncpt_csi_a,
				msecs_to_jiffies(TEGRA_SYNCPT_CSI_WAIT_TIMEOUT),
				NULL,
				NULL);
//trace_printk("wait_timeout(%d)\n", err);
		if ( err ) {
			printk(KERN_ERR "syncpt err %08x\n", TC_VI_REG_RD(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR));
			TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR, TC_VI_REG_RD(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR));
			//nvhost_syncpt_set_min_eq_max(syncpt_get_ext(cam->ndev), TEGRA_VI_SYNCPT_CSI_A);
			//nvhost_syncpt_reset_client(cam->ndev);
		}
	} else if (port == TEGRA_CAMERA_PORT_CSI_B) {
		TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT,
				(7 << 8) | TEGRA_VI_SYNCPT_CSI_B);
		TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND,
				0x0000f005);
		TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SINGLE_SHOT, 0x1);
//trace_printk("waiting for syncpt value %d\n", cam->syncpt_csi_b);
		err = nvhost_syncpt_wait_timeout_ext(cam->ndev,
				TEGRA_VI_SYNCPT_CSI_B,
				++cam->syncpt_csi_b,
				msecs_to_jiffies(TEGRA_SYNCPT_CSI_WAIT_TIMEOUT),
				NULL,
				NULL);
//trace_printk("wait_timeout(%d)\n", err);
		//if ( err ) {
			printk(KERN_ERR "syncpt err %08x\n", TC_VI_REG_RD(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR));
			TC_VI_REG_WT(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR, TC_VI_REG_RD(cam, TEGRA_VI_CFG_VI_INCR_SYNCPT_ERROR));
			//nvhost_syncpt_set_min_eq_max(syncpt_get_ext(cam->ndev), TEGRA_VI_SYNCPT_CSI_B);
			//nvhost_syncpt_reset_client(cam->ndev);
		//}
	}

	/* Mark SOF flag to Zero after we captured the FIRST frame */
	if (cam->sof)
		cam->sof = 0;

	/* Capture syncpt timeout err, then dump error status */
	//if (err) {
		err = vi2_capture_error_status(cam);
		//vi2_sw_reset(cam);
	//}

	return err;
}

static int vi2_capture_stop(struct tegra_camera_dev *cam, int port)
{
//	trace_printk("%s\n", __func__);
	if (port == TEGRA_CAMERA_PORT_CSI_A)
		TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPA_COMMAND,
			     0x0000f002);
	else if (port == TEGRA_CAMERA_PORT_CSI_B)
		TC_VI_REG_WT(cam, TEGRA_CSI_PIXEL_STREAM_PPB_COMMAND,
			     0x0000f002);

	return 0;
}

/* Reset VI2/CSI2 when activating, no sepecial ops for deactiving  */
static void vi2_sw_reset(struct tegra_camera_dev *cam)
{
	printk(KERN_ERR "%s\n", __func__);

#if 0
	/* T12_CG_2ND_LEVEL_EN */
	TC_VI_REG_WT(cam, TEGRA_VI_CFG_CG_CTRL, 1);
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_0_SW_RESET, 0x1F);
	TC_VI_REG_WT(cam, TEGRA_VI_CSI_1_SW_RESET, 0x1F);

	udelay(10);
#endif

	u32 reset_reg[4];

	if (1 /*pdev->id == 0*/) {
		reset_reg[0] = TEGRA_VI_CSI_0_SW_RESET;
		reset_reg[1] = TEGRA_CSI_CSI_SW_SENSOR_A_RESET;
		reset_reg[2] = TEGRA_CSI_CSICIL_SW_SENSOR_A_RESET;
		reset_reg[3] = TEGRA_VI_CSI_0_CSI_IMAGE_DT;
	} else {
		reset_reg[0] = TEGRA_VI_CSI_1_SW_RESET;
		reset_reg[1] = TEGRA_CSI_CSI_SW_SENSOR_B_RESET;
		reset_reg[2] = TEGRA_CSI_CSICIL_SW_SENSOR_B_RESET;
		reset_reg[3] = TEGRA_VI_CSI_1_CSI_IMAGE_DT;
	}
 
	TC_VI_REG_WT(cam, reset_reg[3], 0);
	TC_VI_REG_WT(cam, reset_reg[2], 0x1);
	TC_VI_REG_WT(cam, reset_reg[1], 0x1);
	TC_VI_REG_WT(cam, reset_reg[0], 0x1f);
 
 	udelay(10);
 
	TC_VI_REG_WT(cam, reset_reg[2], 0);
	TC_VI_REG_WT(cam, reset_reg[1], 0);
	TC_VI_REG_WT(cam, reset_reg[0], 0);

}

struct tegra_camera_ops vi2_ops = {
	.clks_init = vi2_clks_init,
	.clks_deinit = vi2_clks_deinit,
	.clks_enable = vi2_clks_enable,
	.clks_disable = vi2_clks_disable,

	.capture_clean = vi2_capture_clean,
	.capture_setup = vi2_capture_setup,
	.capture_start = vi2_capture_start,
	.capture_stop = vi2_capture_stop,

	.activate = vi2_sw_reset,

	.incr_syncpts = vi2_incr_syncpts,

	.port_is_valid = vi2_port_is_valid,
};

int vi2_register(struct tegra_camera_dev *cam)
{
	/* Init regulator */
	cam->regulator_name = "avdd_dsi_csi";

	/* Init VI2/CSI2 ops */
	cam->ops = &vi2_ops;

	return 0;
}
