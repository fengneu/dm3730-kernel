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
	.sensor_isp = 0,
	.capture_mem = EV76C570_BIGGEST_FRAME_BYTE_SIZE * 3,
	.ival_default	= { 1, 25 },
};

static struct isp_interface_config ev76c570_if_config = {
	.ccdc_par_ser		= ISP_PARLL,
	.dataline_shift 	= 0x1,
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSRISE,
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
		printk(KERN_ERR "EV76C570: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	switch (power) {
	case V4L2_POWER_ON:
		isp_configure_interface(vdev->cam->isp, &ev76c570_if_config);

		if (previous_power == V4L2_POWER_OFF) {
			//printk("%s: power on.. \n", __func__);
			/* turn on analog power */
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);

			gpio_direction_output(GPIO_CAM_TRIG, 0);
			gpio_direction_output(GPIO_CAM_RSTN, 0);
			mdelay(10);
			gpio_direction_output(GPIO_CAM_RSTN, 1);
			mdelay(50);
		}
		break;
	case V4L2_POWER_OFF:
		//printk("%s: power off.. \n", __func__);
		/* Power Down Sequence */
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
		gpio_direction_output(GPIO_CAM_RSTN, 0);
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

#endif	/* CONFIG_VIDEO_EV76C570 */

#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)
#define GPIO_CAM_TRIG		167
#define GPIO_CAM_RSTN		98
#include <media/mt9p031.h>

#define MT9P031_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(2592 * 1944 * 2)

static struct omap34xxcam_sensor_config mt9p031_hwc = {
	.sensor_isp = 0,
	.capture_mem = MT9P031_BIGGEST_FRAME_BYTE_SIZE * 3,
	.ival_default	= { 1, 10 },
};

static struct isp_interface_config mt9p031_if_config = {
	.ccdc_par_ser		= ISP_PARLL,
	.dataline_shift 	= 0x1,
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe 		= 0x0,
	.prestrobe 		= 0x0,
	.shutter 		= 0x0,
	.prev_sph 		= 2,
	.prev_slv 		= 0,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs             = 2,
	.u.par.par_bridge       = 0x0,
	.u.par.par_clk_pol      = 0x0,
};

static int mt9p031_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.sensor_isp = mt9p031_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = mt9p031_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static int mt9p031_sensor_power_set(struct v4l2_int_device *s,
				   enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;

	if (!cam_inited) {
		printk(KERN_ERR "MT9P031: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	switch (power) {
	case V4L2_POWER_ON:
		isp_configure_interface(vdev->cam->isp, &mt9p031_if_config);

		if (previous_power == V4L2_POWER_OFF) {
			//printk("%s: power on.. \n", __func__);
			/* turn on analog power */
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);

			/* Set STANDBY_BAR to High */
			gpio_direction_output(GPIO_CAM_TRIG, 1);
			gpio_direction_output(GPIO_CAM_RSTN, 0);
			mdelay(10);
			gpio_direction_output(GPIO_CAM_RSTN, 1);
			mdelay(50);
		}
		break;
	case V4L2_POWER_OFF:
		//printk("%s: power off.. \n", __func__);
		/* Power Down Sequence */
		twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
		gpio_direction_output(GPIO_CAM_RSTN, 0);
		break;
	case V4L2_POWER_STANDBY:
		gpio_direction_output(GPIO_CAM_TRIG, 0);	/* Stamdby */
		break;
	}
	previous_power = power;

	return 0;
}

static u32 mt9p031_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	/* XCLKA-CLK_FIX, XCLKB-CLK_REF */
	return isp_set_xclk(vdev->cam->isp, xclkfreq, LDPCAM_USE_XCLKB);
}

static int mt9p031_sensor_reset(struct v4l2_int_device *s, int active)
{
	if (active) /* Ensure RESET_BAR is low */
		gpio_direction_output(GPIO_CAM_RSTN, 0);
	else	/* Now RESET_BAR must be high */
		gpio_direction_output(GPIO_CAM_RSTN, 1);

	return 0;
}


struct mt9p031_platform_data autoget_mt9p031_platform_data = {
	.power_set      = mt9p031_sensor_power_set,
	.priv_data_set  = mt9p031_sensor_set_prv_data,
	.set_xclk       = mt9p031_sensor_set_xclk,
	.reset          = mt9p031_sensor_reset,

	.ext_freq       = 21000000,
	.target_freq    = 48000000,
};


#endif	/* CONFIG_VIDEO_MT9P031 */

void __init autoget_cam_init(void)
{
	cam_inited = 0;
#if defined(CONFIG_VIDEO_EV76C570) || defined(CONFIG_VIDEO_EV76C570_MODULE)
	if (gpio_request(GPIO_CAM_TRIG, "EV76C570 TRIG") < 0) {
		printk("Can't get GPIO %d\n", GPIO_CAM_TRIG);
		return;
	}

	if (gpio_request(GPIO_CAM_RSTN, "EV76C570 RSTN") < 0) {
		printk("Can't get GPIO %d\n", GPIO_CAM_RSTN);
		return;
	}

	/* Hold RSTN low until camera power on */
	gpio_direction_output(GPIO_CAM_TRIG, 0);
	gpio_direction_output(GPIO_CAM_RSTN, 0);
#endif	/* CONFIG_VIDEO_EV76C570 */

#if defined(CONFIG_VIDEO_MT9P031) || defined(CONFIG_VIDEO_MT9P031_MODULE)
	if (gpio_request(GPIO_CAM_TRIG, "MT9P031 TRIG") < 0) {
		printk("Can't get GPIO %d\n", GPIO_CAM_TRIG);
		return;
	}

	if (gpio_request(GPIO_CAM_RSTN, "MT9P031 RSTN") < 0) {
		printk("Can't get GPIO %d\n", GPIO_CAM_RSTN);
		return;
	}

	/* Hold RSTN low until camera power on */
	gpio_direction_output(GPIO_CAM_TRIG, 0);
	gpio_direction_output(GPIO_CAM_RSTN, 0);
#endif	/* CONFIG_VIDEO_MT9P031 */

	cam_inited = 1;
}
