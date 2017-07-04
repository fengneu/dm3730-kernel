/*
 * linux/arch/arm/mach-omap2/board-sbc8530-camera.c
 *
 * Copyright (C) 2009 Texas Instruments Inc.
 * Sergio Aguirre <saaguirre@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <linux/i2c/twl.h>

#include <asm/io.h>

#include <mach/gpio.h>

static int cam_inited;
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#define LDPCAM_USE_XCLKA	0
#define LDPCAM_USE_XCLKB	1

#define VAUX_1_8_V		0x05
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00


#if defined(CONFIG_VIDEO_EV76C570) || defined(CONFIG_VIDEO_EV76C570_MODULE)
#define GPIO_CAM_TRIG		167
#define GPIO_CAM_RSTN		98
#include <media/ev76c570.h>

#define EV76C570_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(1600 * 1200 * 2)

static struct omap34xxcam_sensor_config ev76c570_hwc = {
	.sensor_isp = 1,
	.capture_mem = EV76C570_BIGGEST_FRAME_BYTE_SIZE * 2,
	.ival_default	= { 1, 15 },
};

static struct isp_interface_config ev76c570_if_config = {
	.ccdc_par_ser		= ISP_PARLL,
	.dataline_shift 	= 0x1,
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSFALL,
	.strobe 		= 0x0,
	.prestrobe 		= 0x0,
	.shutter 		= 0x0,
    	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs             = 2,
	.u.par.par_bridge       = 0x0,
	.u.par.par_clk_pol      = 0x0,
};

static int ev76c570_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.sensor_isp = ev76c570_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = ev76c570_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static int ev76c570_sensor_power_set(struct v4l2_int_device *s,
				   enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;

	if (!cam_inited) {
		printk(KERN_ERR "OV2656: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	switch (power) {
	case V4L2_POWER_ON:
		isp_configure_interface(vdev->cam->isp, &ev76c570_if_config);

		if (previous_power == V4L2_POWER_OFF) {
			/* turn on analog power */
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);

			gpio_direction_output(GPIO_CAM_TRIG, 0);
			gpio_direction_output(GPIO_CAM_RSTN, 0);
			mdelay(1);
			gpio_direction_output(GPIO_CAM_RSTN, 1);
		}
		break;
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
		break;
	case V4L2_POWER_STANDBY:
		break;
	}
	previous_power = power;

	return 0;
}

static u32 ev76c570_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	/* XCLKA-CLK_FIX, XCLKB-CLK_REF */
	return isp_set_xclk(vdev->cam->isp, xclkfreq, LDPCAM_USE_XCLKB);
}

struct ev76c570_platform_data autoget_ev76c570_platform_data = {
	.power_set	 = ev76c570_sensor_power_set,
	.priv_data_set	 = ev76c570_sensor_set_prv_data,
	.set_xclk	 = ev76c570_sensor_set_xclk,
};

#endif//CONFIG_VIDEO_EV76C570

void __init autoget_cam_init(void)
{
#if defined(CONFIG_VIDEO_EV76C570) || defined(CONFIG_VIDEO_EV76C570_MODULE)
	cam_inited = 0;

        if (gpio_request(GPIO_CAM_TRIG, "EV76C570 TRIG") < 0) {
                printk("Can't get GPIO %d\n", GPIO_CAM_TRIG);
                return;
        }

        if (gpio_request(GPIO_CAM_RSTN, "EV76C570 RSTN") < 0) {
                printk("Can't get GPIO %d\n", GPIO_CAM_RSTN);
                return;
        }

        gpio_direction_output(GPIO_CAM_TRIG, 0);
        gpio_direction_output(GPIO_CAM_RSTN, 0);
        mdelay(1);
        gpio_direction_output(GPIO_CAM_RSTN, 1);
        mdelay(1);
#endif
	cam_inited = 1;
}
