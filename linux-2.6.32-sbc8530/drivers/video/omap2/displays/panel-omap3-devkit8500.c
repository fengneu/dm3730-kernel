/*
 * Generic panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>

#include <plat/display.h>

static struct omap_video_timings devkit8500_panel_timings = {
#ifdef CONFIG_LCD_43inch
        .x_res = 480,
        .y_res = 272,

        .pixel_clock    = 9600,

        .hsw            = 41,
        .hfp            = 2,
        .hbp            = 2,

        .vsw            = 10,
        .vfp            = 2,
        .vbp            = 2,

#elif defined(CONFIG_LCD_7inch)
        .x_res          = 800,
        .y_res          = 480,

        .hsw            = 48,   /* hsync_len (4) - 1 */
        .hfp            = 1,      /* right_margin (4) - 1 */
        .hbp            = 1,      /* left_margin (40) - 1 */
        .vsw            = 3,       /* vsync_len (2) - 1 */
        .vfp            = 12,     /* lower_margin */
        .vbp            = 25,     /* upper_margin (8) - 1 */

        .pixel_clock    = 36000,

#elif defined(CONFIG_VGA)
         .x_res          = 1024,
         .y_res          = 768,

         .hsw            = 53,     /* hsync_len (4) - 1 */
         .hfp            = 18,      /* right_margin (4) - 1 */
         .hbp            = 248,      /* left_margin (40) - 1 */
         .vsw            = 6,       /* vsync_len (2) - 1 */
         .vfp            = 3,     /* lower_margin */
         .vbp            = 29,     /* upper_margin (8) - 1 */

         .pixel_clock    = 72000,

#endif
};

static int devkit8500_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = devkit8500_panel_timings;

	return 0;
}

static void devkit8500_panel_remove(struct omap_dss_device *dssdev)
{

}

static int devkit8500_panel_enable(struct omap_dss_device *dssdev)
{

	int r = 0;

	if (dssdev->platform_enable){
		r = dssdev->platform_enable(dssdev);
	}

	return r;
}

static void devkit8500_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static int devkit8500_panel_suspend(struct omap_dss_device *dssdev)
{
	devkit8500_panel_disable(dssdev);
	return 0;
}

static int devkit8500_panel_resume(struct omap_dss_device *dssdev)
{
	return devkit8500_panel_enable(dssdev);
}

static struct omap_dss_driver devkit8500_driver = {
	.probe		= devkit8500_panel_probe,
	.remove		= devkit8500_panel_remove,

	.enable		= devkit8500_panel_enable,
	.disable	= devkit8500_panel_disable,
	.suspend	= devkit8500_panel_suspend,
	.resume		= devkit8500_panel_resume,

	.driver         = {
		.name   = "panel-devkit8500",
		.owner  = THIS_MODULE,
	},
};

static int __init devkit8500_panel_drv_init(void)
{
	return omap_dss_register_driver(&devkit8500_driver);
}

static void __exit devkit8500_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&devkit8500_driver);
}

module_init(devkit8500_panel_drv_init);
module_exit(devkit8500_panel_drv_exit);
MODULE_LICENSE("GPL");
