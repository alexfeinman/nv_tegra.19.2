/*
 * arch/arm/mach-tegra/board-ardbeg-panel.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/ioport.h>
#include <linux/fb.h>
#include <linux/nvmap.h>
#include <linux/nvhost.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/dma-contiguous.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/i2c/ds90uh925q_ser.h>

#include <mach/irqs.h>
#include <mach/dc.h>
#include <mach/io_dpd.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "board-ardbeg.h"
#include "board-panel.h"
#include "common.h"
#include "iomap.h"
#include "tegra12_host1x_devices.h"
#include "dvfs.h"

struct platform_device * __init ardbeg_host1x_init(void)
{
	struct platform_device *pdev = NULL;

#ifdef CONFIG_TEGRA_GRHOST
	if (!of_have_populated_dt())
		pdev = tegra12_register_host1x_devices();
	else
		pdev = to_platform_device(bus_find_device_by_name(
			&platform_bus_type, NULL, "host1x"));

	if (!pdev) {
		pr_err("host1x devices registration failed\n");
		return NULL;
	}
#endif
	return pdev;
}

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct resource ardbeg_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by ardbeg_panel_init() */
		.end	= 0, /* Filled in by ardbeg_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sor",
		.start  = TEGRA_SOR_BASE,
		.end    = TEGRA_SOR_BASE + TEGRA_SOR_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsia_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsib_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "mipi_cal",
		.start	= TEGRA_MIPI_CAL_BASE,
		.end	= TEGRA_MIPI_CAL_BASE + TEGRA_MIPI_CAL_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

#endif

static struct resource ardbeg_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by ardbeg_panel_init() */
		.end	= 0, /* Filled in by ardbeg_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "sor",
		.start  = TEGRA_SOR_BASE,
		.end    = TEGRA_SOR_BASE + TEGRA_SOR_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
    {
        .start  = 0, /* Filled in the panel file by init_resources() */
        .end    = 0, /* Filled in the panel file by init_resources() */
        .flags  = IORESOURCE_MEM,
    },
	{
        .name   = "ganged_dsib_regs",
        .start  = 0, /* Filled in the panel file by init_resources() */
        .end    = 0, /* Filled in the panel file by init_resources() */
		.flags  = IORESOURCE_MEM,
	},
	{
        .name   = "dsi_regs",
        .start  = 0, /* Filled in the panel file by init_resources() */
        .end    = 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
        .name   = "mipi_cal",
        .start  = TEGRA_MIPI_CAL_BASE,
        .end    = TEGRA_MIPI_CAL_BASE + TEGRA_MIPI_CAL_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};


#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct tegra_dc_sd_settings sd_settings;

static struct tegra_dc_out ardbeg_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,
//	.type		= TEGRA_DC_OUT_LVDS,
	.sd_settings	= &sd_settings,
	.parent_clk	= "pll_d_out0",
};
#endif

static struct tegra_dc_out ardbeg_disp2_out = {
	.type		= TEGRA_DC_OUT_LVDS,
//	.type		= TEGRA_DC_OUT_DSI,
	.parent_clk	= "pll_d2_out0",

	.sd_settings	= &sd_settings,

//	.align		= TEGRA_DC_ALIGN_MSB,
//	.order		= TEGRA_DC_ORDER_RED_BLUE,
};

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct tegra_fb_data ardbeg_disp1_fb_data = {
	.win		= 0,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data ardbeg_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &ardbeg_disp1_out,
	.fb		= &ardbeg_disp1_fb_data,
	.emc_clk_rate	= 204000000,
#ifdef CONFIG_TEGRA_DC_CMU
	.cmu_enable	= 1,
#endif
	.low_v_win	= 0x02,
};
#endif

static struct tegra_fb_data ardbeg_disp2_fb_data = {
	.win		= 0,
	.xres		= 1920,
	.yres		= 1080,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data ardbeg_disp2_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &ardbeg_disp2_out,
	.fb		= &ardbeg_disp2_fb_data,
	.emc_clk_rate	= 300000000,
};

static struct platform_device ardbeg_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= ardbeg_disp2_resources,
	.num_resources	= ARRAY_SIZE(ardbeg_disp2_resources),
	.dev = {
		.platform_data = &ardbeg_disp2_pdata,
	},
};

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
static struct platform_device ardbeg_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= ardbeg_disp1_resources,
	.num_resources	= ARRAY_SIZE(ardbeg_disp1_resources),
	.dev = {
		.platform_data = &ardbeg_disp1_pdata,
	},
};
#endif

static struct nvmap_platform_carveout ardbeg_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0, /* Filled in by ardbeg_panel_init() */
		.size		= 0, /* Filled in by ardbeg_panel_init() */
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0, /* Filled in by ardbeg_panel_init() */
		.size		= 0, /* Filled in by ardbeg_panel_init() */
	},
};

static struct nvmap_platform_data ardbeg_nvmap_data = {
	.carveouts	= ardbeg_carveouts,
	.nr_carveouts	= ARRAY_SIZE(ardbeg_carveouts),
};
static struct platform_device ardbeg_nvmap_device  = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &ardbeg_nvmap_data,
	},
};
static struct tegra_io_dpd dsic_io = {
	.name			= "DSIC",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 8,
};
static struct tegra_io_dpd dsid_io = {
	.name			= "DSID",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 9,
};

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
/* can be called multiple times */
static struct tegra_panel *ardbeg_panel_configure(struct tegra_dc_out* disp)
{
	struct tegra_panel *panel = NULL;
	if ( disp->type == TEGRA_DC_OUT_LVDS )
	{
		panel = &lvds_c_1366_14;
	}
	else if ( disp->type == TEGRA_DC_OUT_DSI )
	{
		panel = &dsi_lvds_1366_5;
		if (panel->init_dc_out)
			panel->init_dc_out(disp);
		disp->dsi->dsi_instance = DSI_INSTANCE_1;
		tegra_io_dpd_disable(&dsic_io);
		tegra_io_dpd_disable(&dsid_io);
	}
	
	return panel;
}

static void ardbeg_panel_select(struct tegra_dc_out* disp)
{
	struct tegra_panel *panel = NULL;
	int is_dev1 = (disp == &ardbeg_disp1_out);

	panel = ardbeg_panel_configure(disp);
pr_err("%s\n", __func__);

	if (panel) {
		if (panel->init_sd_settings)
			panel->init_sd_settings(&sd_settings);


		if (panel->init_fb_data)
			panel->init_fb_data(is_dev1? &ardbeg_disp1_fb_data: &ardbeg_disp2_fb_data);

		if (panel->init_cmu_data)
			panel->init_cmu_data(is_dev1? &ardbeg_disp1_pdata: &ardbeg_disp2_pdata);

		if (panel->set_disp_device)
			panel->set_disp_device(is_dev1? &ardbeg_disp1_device: &ardbeg_disp2_device);

		if (disp->type == TEGRA_DC_OUT_DSI) {
			tegra_dsi_resources_init(disp->dsi->dsi_instance,
				ardbeg_disp1_resources,
				ARRAY_SIZE(ardbeg_disp1_resources));
		}

		if (panel->register_bl_dev)
			panel->register_bl_dev();

		if (panel->register_i2c_bridge)
			panel->register_i2c_bridge();
	}

}
#endif

struct ds90uh925q_platform_data lvds_ser_platform_data = {
    .has_lvds_en_gpio = 1,  /* has GPIO to enable */
    .lvds_en_gpio = TEGRA_GPIO_PU3, /* GPIO */

    .is_fpdlinkII = false,
    .support_hdcp = false,
    .clk_rise_edge = false,
};

static struct i2c_board_info __initdata i2c_lvds_serializers[] ={
	{
    	I2C_BOARD_INFO("ds90uh925q", 0x14),
    	.platform_data  = &lvds_ser_platform_data,
	},
	{
    	I2C_BOARD_INFO("ds90uh925q", 0x18),
    	.platform_data  = &lvds_ser_platform_data,
	},
};

int __init ardbeg_panel_init(void)
{
	int err = 0;
	struct resource __maybe_unused *res;
	struct platform_device *phost1x = NULL;
	static struct board_info board_info;
	struct device_node *dc1_node = NULL;
	struct device_node *dc2_node = NULL;

pr_err("%s\n", __func__);

	find_dc_node(&dc1_node, &dc2_node);

	ardbeg_panel_select(&ardbeg_disp1_out);
	ardbeg_panel_select(&ardbeg_disp2_out);

#ifdef CONFIG_TEGRA_NVMAP
	ardbeg_carveouts[1].base = tegra_carveout_start;
	ardbeg_carveouts[1].size = tegra_carveout_size;

	ardbeg_carveouts[2].base = tegra_vpr_start;
	ardbeg_carveouts[2].size = tegra_vpr_size;
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	carveout_linear_set(&tegra_generic_cma_dev);
	ardbeg_carveouts[1].cma_dev = &tegra_generic_cma_dev;
	ardbeg_carveouts[1].resize = false;
	carveout_linear_set(&tegra_vpr_cma_dev);
	ardbeg_carveouts[2].cma_dev = &tegra_vpr_cma_dev;
	ardbeg_carveouts[2].resize = true;
	ardbeg_carveouts[2].cma_chunk_size = SZ_32M;
#endif

	err = platform_device_register(&ardbeg_nvmap_device);
	if (err) {
		pr_err("nvmap device registration failed\n");
		return err;
	}
#endif

	phost1x = ardbeg_host1x_init();
	if (!phost1x) {
		pr_err("host1x devices registration failed\n");
		return -EINVAL;
	}

	if (!of_have_populated_dt() || !dc1_node ||
		!of_device_is_available(dc1_node)) {
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
		res = platform_get_resource_byname(&ardbeg_disp1_device,
					 IORESOURCE_MEM, "fbmem");
#else
		res = platform_get_resource_byname(&ardbeg_disp2_device,
					 IORESOURCE_MEM, "fbmem");
#endif
		res->start = tegra_fb_start;
		res->end = tegra_fb_start + tegra_fb_size - 1;
	}

	/* Copy the bootloader fb to the fb. */
	if (tegra_bootloader_fb_size)
		__tegra_move_framebuffer(&ardbeg_nvmap_device,
				tegra_fb_start, tegra_bootloader_fb_start,
				min(tegra_fb_size, tegra_bootloader_fb_size));
	else
		__tegra_clear_framebuffer(&ardbeg_nvmap_device,
					  tegra_fb_start, tegra_fb_size);

	/* Copy the bootloader fb2 to the fb2. */
	if (tegra_bootloader_fb2_size)
		__tegra_move_framebuffer(&ardbeg_nvmap_device,
				tegra_fb2_start, tegra_bootloader_fb2_start,
				min(tegra_fb2_size, tegra_bootloader_fb2_size));
	else
		__tegra_clear_framebuffer(&ardbeg_nvmap_device,
					tegra_fb2_start, tegra_fb2_size);

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	if (!of_have_populated_dt() || !dc1_node ||
		!of_device_is_available(dc1_node)) {
		ardbeg_disp1_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&ardbeg_disp1_device);
		if (err) {
			pr_err("disp1 device registration failed\n");
			return err;
		}
	}
#endif
	i2c_register_board_info(1, i2c_lvds_serializers, 2);


	if (!of_have_populated_dt() || !dc2_node ||
		!of_device_is_available(dc2_node)) {
#ifndef CONFIG_TEGRA_HDMI_PRIMARY
		res = platform_get_resource_byname(&ardbeg_disp2_device,
					IORESOURCE_MEM, "fbmem");
		res->start = tegra_fb2_start;
		res->end = tegra_fb2_start + tegra_fb2_size - 1;
#endif
		ardbeg_disp2_device.dev.parent = &phost1x->dev;
		err = platform_device_register(&ardbeg_disp2_device);
		if (err) {
			pr_err("disp2 device registration failed\n");
			return err;
		}
	}

#ifdef CONFIG_TEGRA_NVAVP
	nvavp_device.dev.parent = &phost1x->dev;
	err = platform_device_register(&nvavp_device);
	if (err) {
		pr_err("nvavp device registration failed\n");
		return err;
	}
#endif
	return err;
}

int __init ardbeg_display_init(void)
{
	struct clk *disp1_clk = clk_get_sys("tegradc.0", NULL);
	struct clk *disp2_clk = clk_get_sys("tegradc.1", NULL);
	struct tegra_panel *panel;
	long disp1_rate = 0;
	long disp2_rate = 0;

pr_err("%s\n", __func__);
	gpio_export(TEGRA_GPIO_PU3, 0);
	gpio_direction_output(TEGRA_GPIO_PU3, 1);

	/*
	 * TODO
	 * Need to skip ardbeg_display_init
	 * when disp is registered by device_tree
	 */

	if (WARN_ON(IS_ERR(disp1_clk))) {
		if (disp2_clk && !IS_ERR(disp2_clk))
			clk_put(disp2_clk);
		return PTR_ERR(disp1_clk);
	}

	if (WARN_ON(IS_ERR(disp2_clk))) {
		clk_put(disp1_clk);
		return PTR_ERR(disp1_clk);
	}

#ifndef CONFIG_TEGRA_HDMI_PRIMARY
	panel = ardbeg_panel_configure(&ardbeg_disp1_out);

	if (panel && panel->init_dc_out) {
		panel->init_dc_out(&ardbeg_disp1_out);
		if (ardbeg_disp1_out.n_modes && ardbeg_disp1_out.modes) {

			int err;
			struct clk* parent = clk_get_sys(NULL, ardbeg_disp1_out.parent_clk);
			disp1_rate = ardbeg_disp1_out.modes[0].pclk;
			if ( !IS_ERR(parent) ) {
				if ( (err = clk_set_parent(disp1_clk, parent)) ) {
					pr_err("Failed to set clock parent to %s: %d\n", ardbeg_disp1_out.parent_clk, err);
				}
			}
			else
				clk_put(parent);

		}
	} else {
		disp1_rate = 0;
		if (!panel || !panel->init_dc_out)
			printk(KERN_ERR "disp1 panel output not specified!\n");
	}

	printk(KERN_ERR "disp1 pclk=%ld\n", disp1_rate);
	if (disp1_rate)
		tegra_dvfs_resolve_override(disp1_clk, disp1_rate);
#endif

	panel = ardbeg_panel_configure(&ardbeg_disp2_out);

	if (panel && panel->init_dc_out) {
		panel->init_dc_out(&ardbeg_disp2_out);
		if (ardbeg_disp2_out.n_modes && ardbeg_disp2_out.modes) {

			int err;
			struct clk* parent = clk_get_sys(NULL, ardbeg_disp2_out.parent_clk);
			disp1_rate = ardbeg_disp2_out.modes[0].pclk;
			if ( !IS_ERR(parent) ) {
				if ( (err = clk_set_parent(disp2_clk, parent)) ) {
					pr_err("Failed to set clock parent to %s: %d\n", ardbeg_disp2_out.parent_clk, err);
				}
			}
	else
				clk_put(parent);

		}
	} else {
		disp2_rate = 0;
		if (!panel || !panel->init_dc_out)
			printk(KERN_ERR "disp1 panel output not specified!\n");
	}

	printk(KERN_ERR "disp1 pclk=%ld\n", disp2_rate);
	if (disp2_rate)
		tegra_dvfs_resolve_override(disp2_clk, disp2_rate);



	clk_put(disp1_clk);
	clk_put(disp2_clk);
	return 0;
}
