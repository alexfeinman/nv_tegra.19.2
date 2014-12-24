/*
 * Copyright © 2011 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>

#include <mach/dc.h>
#include <mach/../../clock.h>

#include "dc_priv.h"
#include "dsi2lvds.h"
#include "dsi.h"

static struct tegra_dc_dsi2lvds_data *dsi2lvds;

enum i2c_transfer_type {
	I2C_WRITE,
	I2C_READ,
};
#ifndef TEGRA_GPIO_PU3
#define TEGRA_GPIO_PU3 163
#endif

#define DSI2LVDS_TEGRA_I2C_BUS	1
#define DSI2LVDS_REG_VAL(addr, val)	{(addr), (val)}

#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))


/* DSI D-PHY Layer Registers */
#define D0W_DPHYCONTTX		0x0004
#define CLW_DPHYCONTRX		0x0020
#define D0W_DPHYCONTRX		0x0024
#define D1W_DPHYCONTRX		0x0028
#define D2W_DPHYCONTRX		0x002C
#define D3W_DPHYCONTRX		0x0030
#define COM_DPHYCONTRX		0x0038
#define CLW_CNTRL		0x0040
#define D0W_CNTRL		0x0044
#define D1W_CNTRL		0x0048
#define D2W_CNTRL		0x004C
#define D3W_CNTRL		0x0050
#define DFTMODE_CNTRL		0x0054

/* DSI PPI Layer Registers */
#define PPI_STARTPPI		0x0104
#define PPI_BUSYPPI		0x0108
#define PPI_LINEINITCNT		0x0110
#define PPI_LPTXTIMECNT		0x0114
#define PPI_LANEENABLE		0x0134
#define PPI_TX_RX_TA		0x013C
#define PPI_CLS_ATMR		0x0140
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D2S_ATMR		0x014C
#define PPI_D3S_ATMR		0x0150
#define PPI_D0S_CLRSIPOCOUNT	0x0164
#define PPI_D1S_CLRSIPOCOUNT	0x0168
#define PPI_D2S_CLRSIPOCOUNT	0x016C
#define PPI_D3S_CLRSIPOCOUNT	0x0170
#define CLS_PRE			0x0180
#define D0S_PRE			0x0184
#define D1S_PRE			0x0188
#define D2S_PRE			0x018C
#define D3S_PRE			0x0190
#define CLS_PREP		0x01A0
#define D0S_PREP		0x01A4
#define D1S_PREP		0x01A8
#define D2S_PREP		0x01AC
#define D3S_PREP		0x01B0
#define CLS_ZERO		0x01C0
#define D0S_ZERO		0x01C4
#define D1S_ZERO		0x01C8
#define D2S_ZERO		0x01CC
#define D3S_ZERO		0x01D0
#define PPI_CLRFLG		0x01E0
#define PPI_CLRSIPO		0x01E4
#define HSTIMEOUT		0x01F0
#define HSTIMEOUTENABLE		0x01F4

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204
#define DSI_BUSYDSI		0x0208
#define DSI_LANEENABLE		0x0210
#define DSI_LANESTATUS0		0x0214
#define DSI_LANESTATUS1		0x0218
#define DSI_INTSTATUS		0x0220
#define DSI_INTMASK		0x0224
#define DSI_INTCLR		0x0228
#define DSI_LPTXTO		0x0230

/* DSI General Registers */
#define DSIERRCNT		0x0300

/* DSI Application Layer Registers */
#define APLCTRL			0x0400
#define RDPKTLN			0x0404

/* Video Path Registers */
#define VPCTRL			0x0450
#define HTIM1			0x0454
#define HTIM2			0x0458
#define VTIM1			0x045C
#define VTIM2			0x0460
#define VFUEN			0x0464

/* LVDS Registers */
#define LVMX0003		0x0480
#define LVMX0407		0x0484
#define LVMX0811		0x0488
#define LVMX1215		0x048C
#define LVMX1619		0x0490
#define LVMX2023		0x0494
#define LVMX2427		0x0498
#define LVCFG			0x049C
#define LVPHY0			0x04A0
#define LVPHY1			0x04A4

/* System Registers */
#define SYSSTAT			0x0500
#define SYSRST			0x0504

/* GPIO Registers */
/*#define GPIOC			0x0520*/
#define GPIOO			0x0524
#define GPIOI			0x0528

/* I2C Registers */
#define I2CTIMCTRL		0x0540
#define I2CMADDR		0x0544
#define WDATAQ			0x0548
#define RDATAQ			0x054C

/* Chip/Rev Registers */
#define IDREG			0x0580

/* Debug Registers */
#define DEBUG00			0x05A0
#define DEBUG01			0x05A4


/* Input muxing for registers LVMX0003...LVMX2427 */
enum {
	INPUT_R0,	/* 0 */
	INPUT_R1,
	INPUT_R2,
	INPUT_R3,
	INPUT_R4,
	INPUT_R5,
	INPUT_R6,
	INPUT_R7,
	INPUT_G0,	/* 8 */
	INPUT_G1,
	INPUT_G2,
	INPUT_G3,
	INPUT_G4,
	INPUT_G5,
	INPUT_G6,
	INPUT_G7,
	INPUT_B0,	/* 16 */
	INPUT_B1,
	INPUT_B2,
	INPUT_B3,
	INPUT_B4,
	INPUT_B5,
	INPUT_B6,
	INPUT_B7,
	INPUT_HSYNC,	/* 24 */
	INPUT_VSYNC,
	INPUT_DE,
	LOGIC_0,
	/* 28...31 undefined */
};

#define INPUT_MUX(lvmx03, lvmx02, lvmx01, lvmx00)		\
	(FLD_VAL(lvmx03, 29, 24) | FLD_VAL(lvmx02, 20, 16) |	\
	FLD_VAL(lvmx01, 12, 8) | FLD_VAL(lvmx00, 4, 0))

/**
 * tc35876x_regw - Write DSI-LVDS bridge register using I2C
 * @client: struct i2c_client to use
 * @reg: register address
 * @value: value to write
 *
 * Returns 0 on success, or a negative error value.
 */
static int tc35876x_regw(struct i2c_client *client, u16 reg, u32 value)
{
	int r;
	u8 tx_data[] = {
		/* NOTE: Register address big-endian, data little-endian. */
		(reg >> 8) & 0xff,
		reg & 0xff,
		value & 0xff,
		(value >> 8) & 0xff,
		(value >> 16) & 0xff,
		(value >> 24) & 0xff,
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d\n",
			__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
			__func__, reg, value, r);
		return -EAGAIN;
	}

	dev_dbg(&client->dev, "%s: reg 0x%04x val 0x%08x\n",
			__func__, reg, value);

	return 0;
}

/**
 * tc35876x_regr - Read DSI-LVDS bridge register using I2C
 * @client: struct i2c_client to use
 * @reg: register address
 * @value: pointer for storing the value
 *
 * Returns 0 on success, or a negative error value.
 */
static int tc35876x_regr(struct i2c_client *client, u16 reg, u32 *value)
{
	int r;
	u8 tx_data[] = {
		(reg >> 8) & 0xff,
		reg & 0xff,
	};
	u8 rx_data[4];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		 },
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
			reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
			reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0] << 24 | rx_data[1] << 16 |
		rx_data[2] << 8 | rx_data[3];

	dev_dbg(&client->dev, "%s: reg 0x%04x value 0x%08x\n", __func__,
		reg, *value);

	return 0;
}

#define TRY_WRITE(a) \
if ( (err = a) ) { \
	dev_err(&i2c->dev, "%s failed (%d)\n", #a, err); \
	goto fail; \
}

int tc35876x_configure_lvds_bridge(struct tegra_dc_dsi_data *dsi)
{
	u32 ppi_lptxtimecnt;
	u32 txtagocnt;
	u32 txtasurecnt;
	u32 id;
	int err = 0;
	struct tegra_dc_dsi2lvds_data *dsi2lvds = tegra_dsi_get_outdata(dsi);
	struct i2c_client *i2c = dsi2lvds->client_i2c;
	struct tegra_dc *dc = dsi->dc;

	if (WARN(!i2c, "%s called before probe", __func__))
		return -EINVAL;

	dev_dbg(&i2c->dev, "%s\n", __func__);

	if (!tc35876x_regr(i2c, IDREG, &id))
		dev_info(&i2c->dev, "tc35876x ID 0x%08x\n", id);
	else
		dev_err(&i2c->dev, "Cannot read ID\n");

	ppi_lptxtimecnt = 4;
	txtagocnt = (5 * ppi_lptxtimecnt - 3) / 4;
	txtasurecnt = 3 * ppi_lptxtimecnt / 2;
	TRY_WRITE((tc35876x_regw(i2c, PPI_TX_RX_TA, FLD_VAL(txtagocnt, 26, 16) | FLD_VAL(txtasurecnt, 10, 0))))
	TRY_WRITE((tc35876x_regw(i2c, PPI_LPTXTIMECNT, FLD_VAL(ppi_lptxtimecnt, 10, 0))))

	TRY_WRITE((tc35876x_regw(i2c, PPI_D0S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0))))
	TRY_WRITE((tc35876x_regw(i2c, PPI_D1S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0))))
	TRY_WRITE((tc35876x_regw(i2c, PPI_D2S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0))))
	TRY_WRITE((tc35876x_regw(i2c, PPI_D3S_CLRSIPOCOUNT, FLD_VAL(1, 5, 0))))

	/* Enabling MIPI & PPI lanes, Enable 4 lanes */
	//TRY_WRITE((tc35876x_regw(i2c, PPI_LANEENABLE, BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))))
	//TRY_WRITE((tc35876x_regw(i2c, DSI_LANEENABLE, BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))))
	dev_info(&i2c->dev, "Setting lane mask for %d lanes to %x\n", dsi->info.n_data_lanes, (1 << (dsi->info.n_data_lanes + 1)) - 1);
	TRY_WRITE((tc35876x_regw(i2c, PPI_LANEENABLE, (1 << (dsi->info.n_data_lanes + 1)) - 1)))
	TRY_WRITE((tc35876x_regw(i2c, DSI_LANEENABLE, (1 << (dsi->info.n_data_lanes + 1)) - 1)))
	TRY_WRITE((tc35876x_regw(i2c, PPI_STARTPPI, BIT(0))))
	TRY_WRITE((tc35876x_regw(i2c, DSI_STARTDSI, BIT(0))))

	/* Setting LVDS output frequency */
	TRY_WRITE((tc35876x_regw(i2c, LVPHY0, FLD_VAL(1, 20, 16) | FLD_VAL(2, 15, 14) | FLD_VAL(6, 4, 0)))) /* 0x00048006 */

	/* Setting video panel control register,0x00000120 VTGen=ON ?!?!? */
	TRY_WRITE((tc35876x_regw(i2c, VPCTRL, BIT(8) | BIT(5))))

	/* Horizontal back porch and horizontal pulse width. 0x00280028 */
	TRY_WRITE((tc35876x_regw(i2c, HTIM1, FLD_VAL(dc->mode.h_back_porch, 24, 16) | FLD_VAL(dc->mode.h_sync_width, 8, 0))))

	/* Horizontal front porch and horizontal active video size. 0x00500500*/
	TRY_WRITE((tc35876x_regw(i2c, HTIM2, FLD_VAL(dc->mode.h_front_porch, 24, 16) | FLD_VAL(dc->mode.h_active, 10, 0))))

	/* Vertical back porch and vertical sync pulse width. 0x000e000a */
	TRY_WRITE((tc35876x_regw(i2c, VTIM1, FLD_VAL(dc->mode.v_back_porch, 23, 16) | FLD_VAL(dc->mode.v_sync_width, 7, 0))))

	/* Vertical front porch and vertical display size. 0x000e0320 */
	TRY_WRITE((tc35876x_regw(i2c, VTIM2, FLD_VAL(dc->mode.v_front_porch, 23, 16) | FLD_VAL(dc->mode.v_active, 10, 0))))

	/* Set above HTIM1, HTIM2, VTIM1, and VTIM2 at next VSYNC. */
	TRY_WRITE((tc35876x_regw(i2c, VFUEN, BIT(0))))

	/* Soft reset LCD controller. */
	TRY_WRITE((tc35876x_regw(i2c, SYSRST, BIT(2))))

	/* LVDS-TX input muxing */
	TRY_WRITE((tc35876x_regw(i2c, LVMX0003, INPUT_MUX(INPUT_R5, INPUT_R4, INPUT_R3, INPUT_R2))))
	TRY_WRITE((tc35876x_regw(i2c, LVMX0407, INPUT_MUX(INPUT_G2, INPUT_R7, INPUT_R1, INPUT_R6))))
	TRY_WRITE((tc35876x_regw(i2c, LVMX0811, INPUT_MUX(INPUT_G1, INPUT_G0, INPUT_G4, INPUT_G3))))
	TRY_WRITE((tc35876x_regw(i2c, LVMX1215, INPUT_MUX(INPUT_B2, INPUT_G7, INPUT_G6, INPUT_G5))))
	TRY_WRITE((tc35876x_regw(i2c, LVMX1619, INPUT_MUX(INPUT_B4, INPUT_B3, INPUT_B1, INPUT_B0))))
	TRY_WRITE((tc35876x_regw(i2c, LVMX2023, INPUT_MUX(LOGIC_0,  INPUT_B7, INPUT_B6, INPUT_B5))))
	TRY_WRITE((tc35876x_regw(i2c, LVMX2427, INPUT_MUX(INPUT_R0, INPUT_DE, INPUT_VSYNC, INPUT_HSYNC))))

	/* Enable LVDS transmitter. */
	TRY_WRITE((tc35876x_regw(i2c, LVCFG, BIT(0))))

	/* Clear notifications. Don't write reserved bits. Was write 0xffffffff
	 * to 0x0288, must be in error?! */
	TRY_WRITE((tc35876x_regw(i2c, DSI_INTCLR, FLD_MASK(31, 30) | FLD_MASK(22, 0))))

fail:
	return err;
}


static struct i2c_driver tegra_dsi2lvds_i2c_slave_driver = {
    .driver = {
        .name = "tc35876x_bridge",
    },
};


static struct i2c_client *init_i2c_slave(struct tegra_dc_dsi_data *dsi)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info p_data = {
		.type = "tc35876x_bridge",
		.addr = 0x0f,
	};
	int bus = DSI2LVDS_TEGRA_I2C_BUS;
	int err = 0;

	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		dev_err(&dsi->dc->ndev->dev,
			"dsi2lvds: can't get adpater for bus %d\n", bus);
		err = -EBUSY;
		goto err;
	}

	client = i2c_new_device(adapter, &p_data);
	i2c_put_adapter(adapter);
	if (!client) {
		dev_err(&dsi->dc->ndev->dev,
			"dsi2lvds: can't add i2c slave device\n");
		err = -EBUSY;
		goto err;
	}

	err = i2c_add_driver(&tegra_dsi2lvds_i2c_slave_driver);
	if (err) {
		dev_err(&dsi->dc->ndev->dev,
			"dsi2lvds: can't add i2c slave driver\n");
		goto err_free;
	}
	
	dev_info(&dsi->dc->ndev->dev, "DSI2LVDS bridge TC35876X registered\n");

	return client;
err:
	return ERR_PTR(err);
err_free:
	i2c_unregister_device(client);
	return ERR_PTR(err);
}

static int tegra_dsi2lvds_init(struct tegra_dc_dsi_data *dsi)
{
	int err = 0;

pr_err("%s\n", __func__);
	if (dsi2lvds) {
		tegra_dsi_set_outdata(dsi, dsi2lvds);
		return err;
	}


	dsi2lvds = devm_kzalloc(&dsi->dc->ndev->dev, sizeof(*dsi2lvds), GFP_KERNEL);
	if (!dsi2lvds)
		return -ENOMEM;

	dsi2lvds->client_i2c = init_i2c_slave(dsi);
	if (IS_ERR_OR_NULL(dsi2lvds->client_i2c)) {
		dev_err(&dsi->dc->ndev->dev,
			"dsi2lvds: i2c slave setup failure\n");
	}

	dsi2lvds->dsi = dsi;

	tegra_dsi_set_outdata(dsi, dsi2lvds);

	mutex_init(&dsi2lvds->lock);

	dev_info(&dsi->dc->ndev->dev, "DSI2LVDS bridge TC35876X initialized\n");

	return err;
}

static void tegra_dsi2lvds_destroy(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2lvds_data *dsi2lvds = tegra_dsi_get_outdata(dsi);

pr_err("%s\n", __func__);
	if (!dsi2lvds)
		return;

	mutex_lock(&dsi2lvds->lock);
	i2c_del_driver(&tegra_dsi2lvds_i2c_slave_driver);
	i2c_unregister_device(dsi2lvds->client_i2c);
	mutex_unlock(&dsi2lvds->lock);
	mutex_destroy(&dsi2lvds->lock);
	kfree(dsi2lvds);
}

static void tegra_dsi2lvds_enable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_dsi2lvds_data *dsi2lvds = tegra_dsi_get_outdata(dsi);
	int ret = 0;
pr_err("%s\n", __func__);
	if (dsi2lvds && dsi2lvds->dsi2lvds_enabled)
		return;

	mutex_lock(&dsi2lvds->lock);
/*
	gpio_request(TEGRA_GPIO_PU3, "display_en");
    gpio_direction_output(TEGRA_GPIO_PU3, 0);
	usleep_range(1000, 2000);
    gpio_direction_output(TEGRA_GPIO_PU3, 1);
    gpio_export(TEGRA_GPIO_PU3, 0);
	usleep_range(1000, 2000);
*/
	{
		struct clk* clk = dsi->dsi_clk[0];
		if ( clk )
		pr_err("%s: %d\n", clk->name, clk->state);
		clk = dsi->dsi_clk[1];
		//if ( clk )
//		clk = (void*)-2;
//		pr_err("%s: %d\n", clk->name, clk->state);
//		pr_err("PU3: %d\n", gpio_get_value(TEGRA_GPIO_PU3));
	}
	
	if ( (ret = tc35876x_configure_lvds_bridge(dsi)) )
	{
		dev_err(&dsi->dc->ndev->dev, "DSI2LVDS bridge TC35876X failed to configure: %d\n", ret);
		goto fail;
	}
	dev_info(&dsi->dc->ndev->dev, "DSI2LVDS bridge TC35876X configured\n");
	
	dsi2lvds->dsi2lvds_enabled = true;
fail:
	mutex_unlock(&dsi2lvds->lock);
}

static void tegra_dsi2lvds_disable(struct tegra_dc_dsi_data *dsi)
{
	/* To be done */
}

#ifdef CONFIG_PM
static void tegra_dsi2lvds_suspend(struct tegra_dc_dsi_data *dsi)
{
pr_err("%s\n", __func__);
	dsi2lvds->dsi2lvds_enabled = false;
}

static void tegra_dsi2lvds_resume(struct tegra_dc_dsi_data *dsi)
{
	pr_err("%s\n", __func__);
	dsi2lvds->dsi2lvds_enabled = true;
	tc35876x_configure_lvds_bridge(dsi);
	
}
#endif

struct tegra_dsi_out_ops tegra_dsi2lvds_ops = {
	.init = tegra_dsi2lvds_init,
	.destroy = tegra_dsi2lvds_destroy,
	.enable = tegra_dsi2lvds_enable,
	.disable = tegra_dsi2lvds_disable,
#ifdef CONFIG_PM
	.suspend = tegra_dsi2lvds_suspend,
	.resume = tegra_dsi2lvds_resume,
#endif
};
