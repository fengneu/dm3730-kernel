/*
 * linux/arch/arm/mach-omap2/board-sbc8510-camera.c
 *
 * Copyright (C) 2009 Texas Instruments Inc.
 * Sergio Aguirre <saaguirre@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_TWL4030_CORE

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

#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
#include <media/tvp514x-int.h>
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
static struct omap34xxcam_hw_config decoder_hwc = {
        .dev_index              = 0,
        .dev_minor              = 0,
        .dev_type               = OMAP34XXCAM_SLAVE_SENSOR,
        .u.sensor.sensor_isp    = 1,
        .u.sensor.capture_mem   = PAGE_ALIGN(720*525*2*4),
};

static struct isp_interface_config tvp5146_if_config = {
        .ccdc_par_ser           = ISP_PARLL_YUV_BT,
        .dataline_shift         = 0x1,
        .hsvs_syncdetect        = ISPCTRL_SYNC_DETECT_VSRISE,
        .strobe                 = 0x0,
        .prestrobe              = 0x0,
        .shutter                = 0x0,
        .wait_hs_vs             = 2,
        .u.par.par_bridge       = 0x0,
        .u.par.par_clk_pol      = 0x0,
};
#endif

static struct v4l2_ifparm ifparm = {
        .if_type = V4L2_IF_TYPE_BT656,
        .u       = {
                .bt656 = {
                        .frame_start_on_rising_vs = 1,
                        .bt_sync_correct = 0,
                        .swap           = 0,
                        .latch_clk_inv  = 0,
                        .nobt_hs_inv    = 0,    /* active high */
                        .nobt_vs_inv    = 0,    /* active high */
                        .mode           = V4L2_IF_TYPE_BT656_MODE_BT_8BIT,
                        .clock_min      = TVP514X_XCLK_BT656,
                        .clock_max      = TVP514X_XCLK_BT656,
                },
        },
};

/**
 * @brief tvp5146_ifparm - Returns the TVP5146 decoder interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_ifparm(struct v4l2_ifparm *p)
{
        if (p == NULL)
                return -EINVAL;

        *p = ifparm;
        return 0;
}

/**
 * @brief tvp5146_set_prv_data - Returns tvp5146 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_set_prv_data(struct v4l2_int_device *s, void *priv)
{
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
        struct omap34xxcam_hw_config *hwc = priv;

        if (priv == NULL)
                return -EINVAL;

        hwc->u.sensor.sensor_isp = decoder_hwc.u.sensor.sensor_isp;
        hwc->u.sensor.capture_mem = decoder_hwc.u.sensor.capture_mem;
        hwc->dev_index = decoder_hwc.dev_index;
        hwc->dev_minor = decoder_hwc.dev_minor;
        hwc->dev_type = decoder_hwc.dev_type;
        return 0;
#else
        return -EINVAL;
#endif
}

/**
 * @brief tvp5146_power_set - Power-on or power-off TVP5146 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
        struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

        switch (power) {
        case V4L2_POWER_OFF:
                break;

        case V4L2_POWER_STANDBY:
                break;

        case V4L2_POWER_ON:
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
                isp_configure_interface(vdev->cam->isp, &tvp5146_if_config);
#endif
                break;

        default:
                return -ENODEV;
                break;
        }
        return 0;
}

struct tvp514x_platform_data tvp5146_pdata = {
        .master         = "omap34xxcam",
        .power_set      = tvp5146_power_set,
        .priv_data_set  = tvp5146_set_prv_data,
        .ifparm         = tvp5146_ifparm,
        /* Some interface dependent params */
        .clk_polarity   = 0, /* data clocked out on falling edge */
        .hs_polarity    = 1, /* 0 - Active low, 1- Active high */
        .vs_polarity    = 1, /* 0 - Active low, 1- Active high */
};
#endif                          /* #ifdef CONFIG_VIDEO_TVP514X */

#if defined(CONFIG_VIDEO_OV2656) || defined(CONFIG_VIDEO_OV2656_MODULE)
#define GPIO_CAM_PDN            167
#define GPIO_CAM_RST            126
#include <media/ov2656.h>

#define OV2656_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(2048 * 1536 * 2)

static struct omap34xxcam_sensor_config ov2656_hwc = {
	.sensor_isp = 1,
	.capture_mem = OV2656_BIGGEST_FRAME_BYTE_SIZE * 2,
	.ival_default	= { 1, 15 },
};

static struct isp_interface_config ov2656_if_config = {
	.ccdc_par_ser		= ISP_PARLL,
	.dataline_shift 	= 0x1,
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe 		= 0x0,
	.prestrobe 		= 0x0,
	.shutter 		= 0x0,
    	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs             = 2,
	.u.par.par_bridge       = 0x3,
	.u.par.par_clk_pol      = 0x0,
};

static int ov2656_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.sensor_isp = ov2656_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = ov2656_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static int ov2656_sensor_power_set(struct v4l2_int_device *s,
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
		isp_configure_interface(vdev->cam->isp, &ov2656_if_config);

		if (previous_power == V4L2_POWER_OFF) {
			/* turn on analog power */
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);

			gpio_direction_output(GPIO_CAM_PDN, 0);
			gpio_direction_output(GPIO_CAM_RST, 0);
			mdelay(1);
			gpio_direction_output(GPIO_CAM_RST, 1);
			mdelay(1);
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

static u32 ov2656_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	return isp_set_xclk(vdev->cam->isp, xclkfreq, LDPCAM_USE_XCLKA);
}

struct ov2656_platform_data sbc8510_ov2656_platform_data = {
	.power_set	 = ov2656_sensor_power_set,
	.priv_data_set	 = ov2656_sensor_set_prv_data,
	.set_xclk	 = ov2656_sensor_set_xclk,
};

#endif//CONFIG_VIDEO_OV2656

void __init sbc8510_cam_init(void)
{
#if defined(CONFIG_VIDEO_OV2656) || defined(CONFIG_VIDEO_OV2656_MODULE)
	cam_inited = 0;

        if (gpio_request(GPIO_CAM_PDN, "OV2656 PND") < 0) {
                printk("Can't get GPIO %d\n", GPIO_CAM_PDN);
                return;
        }

        if (gpio_request(GPIO_CAM_RST, "OV2656 RST") < 0) {
                printk("Can't get GPIO %d\n", GPIO_CAM_RST);
                return;
        }

        gpio_direction_output(GPIO_CAM_PDN, 0);
        gpio_direction_output(GPIO_CAM_RST, 0);
        mdelay(1);
        gpio_direction_output(GPIO_CAM_RST, 1);
        mdelay(1);
#endif
	cam_inited = 1;
}
#else//CONFIG_TWL4030_CORE
void __init sbc8510_cam_init(void)
{
}
#endif
