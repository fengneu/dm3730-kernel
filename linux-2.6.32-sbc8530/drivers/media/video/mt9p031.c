/*
 * Driver for MT9P031 CMOS Image Sensor from Aptina
 *
 * Copyright (C) 2017, ZhangYan <niat_zhy@163.com>
 *
 * Based on the MT9P012 driver and latest MT9P031 driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>

#include <media/mt9p031.h>

#define DRIVER_NAME  "mt9p031"

#define MT9P031_PIXEL_ARRAY_WIDTH			2752
#define MT9P031_PIXEL_ARRAY_HEIGHT			2004

#define MT9P031_CHIP_VERSION				0x00
#define		MT9P031_CHIP_VERSION_VALUE		0x1801
#define MT9P031_ROW_START				0x01
#define		MT9P031_ROW_START_MIN			0
#define		MT9P031_ROW_START_MAX			2004
#define		MT9P031_ROW_START_DEF			54
#define MT9P031_COLUMN_START				0x02
#define		MT9P031_COLUMN_START_MIN		0
#define		MT9P031_COLUMN_START_MAX		2750
#define		MT9P031_COLUMN_START_DEF		16
#define MT9P031_WINDOW_HEIGHT				0x03
#define		MT9P031_WINDOW_HEIGHT_MIN		2
#define		MT9P031_WINDOW_HEIGHT_MAX		2006
#define		MT9P031_WINDOW_HEIGHT_DEF		1944
#define MT9P031_WINDOW_WIDTH				0x04
#define		MT9P031_WINDOW_WIDTH_MIN		2
#define		MT9P031_WINDOW_WIDTH_MAX		2752
#define		MT9P031_WINDOW_WIDTH_DEF		2592
#define MT9P031_HORIZONTAL_BLANK			0x05
#define		MT9P031_HORIZONTAL_BLANK_MIN		0
#define		MT9P031_HORIZONTAL_BLANK_MAX		4095
#define MT9P031_VERTICAL_BLANK				0x06
#define		MT9P031_VERTICAL_BLANK_MIN		0
#define		MT9P031_VERTICAL_BLANK_MAX		4095
#define		MT9P031_VERTICAL_BLANK_DEF		25
#define MT9P031_OUTPUT_CONTROL				0x07
#define		MT9P031_OUTPUT_CONTROL_CEN		2
#define		MT9P031_OUTPUT_CONTROL_SYN		1
#define		MT9P031_OUTPUT_CONTROL_DEF		0x1f82
#define MT9P031_SHUTTER_WIDTH_UPPER			0x08
#define MT9P031_SHUTTER_WIDTH_LOWER			0x09
#define		MT9P031_SHUTTER_WIDTH_MIN		1
#define		MT9P031_SHUTTER_WIDTH_MAX		1048575
#define		MT9P031_SHUTTER_WIDTH_DEF		1943
#define	MT9P031_PLL_CONTROL				0x10
#define		MT9P031_PLL_CONTROL_PWROFF		0x0050
#define		MT9P031_PLL_CONTROL_PWRON		0x0051
#define		MT9P031_PLL_CONTROL_USEPLL		0x0052
#define	MT9P031_PLL_CONFIG_1				0x11
#define	MT9P031_PLL_CONFIG_2				0x12
#define MT9P031_PIXEL_CLOCK_CONTROL			0x0a
#define MT9P031_FRAME_RESTART				0x0b
#define MT9P031_SHUTTER_DELAY				0x0c
#define MT9P031_RST					0x0d
#define		MT9P031_RST_ENABLE			1
#define		MT9P031_RST_DISABLE			0
#define MT9P031_READ_MODE_1				0x1e
#define MT9P031_READ_MODE_2				0x20
#define		MT9P031_READ_MODE_2_ROW_MIR		(1 << 15)
#define		MT9P031_READ_MODE_2_COL_MIR		(1 << 14)
#define		MT9P031_READ_MODE_2_ROW_BLC		(1 << 6)
#define MT9P031_ROW_ADDRESS_MODE			0x22
#define MT9P031_COLUMN_ADDRESS_MODE			0x23
#define MT9P031_GLOBAL_GAIN				0x35
#define		MT9P031_GLOBAL_GAIN_MIN			8
#define		MT9P031_GLOBAL_GAIN_MAX			1024
#define		MT9P031_GLOBAL_GAIN_DEF			8
#define		MT9P031_GLOBAL_GAIN_MULT		(1 << 6)
#define MT9P031_ROW_BLACK_DEF_OFFSET			0x4b
#define MT9P031_TEST_PATTERN				0xa0
#define		MT9P031_TEST_PATTERN_SHIFT		3
#define		MT9P031_TEST_PATTERN_ENABLE		(1 << 0)
#define		MT9P031_TEST_PATTERN_DISABLE		(0 << 0)
#define MT9P031_TEST_PATTERN_GREEN			0xa1
#define MT9P031_TEST_PATTERN_RED			0xa2
#define MT9P031_TEST_PATTERN_BLUE			0xa3

struct mt9p031_pll_divs {
	u32 ext_freq;
	u32 target_freq;
	u8 m;
	u8 n;
	u8 p1;
};


/**
 * struct mt9p031_sensor - main structure for storage of sensor information
 * @dev:
 * @pdata: access functions and data for platform level information
 * @v4l2_int_device: V4L2 device structure structure
 * @pix: V4L2 pixel format information structure
 * @timeperframe: time per frame expressed as V4L fraction
 * @scaler:
 * @ver: mt9p031 chip version
 * @fps: frames per second value
 */

struct mt9p031_sensor {
	struct device *dev;
	struct mt9p031_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int detected;
	int ver;

    struct v4l2_rect crop;  /* Sensor window */
	struct mutex power_lock; /* lock to protect power_count */
	int power_count;
	u16 xskip;
	u16 yskip;

	const struct mt9p031_pll_divs *pll;

	/* Registers cache */
	u16 output_control;
	u16 mode2;

};

/**
 * struct capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct mt9p031_capture_size {
	unsigned long width;
	unsigned long height;
};

/*
 * Array of image sizes supported by MT9P031.  These must be ordered from
 * smallest image size to largest.
 */
const static struct mt9p031_capture_size mt9p031_sizes[] = {
	{ 1296, 972 },	/* 2X BINNING */
	{ 2048, 1536},	/* 3 MP */
	{ 2592, 1944},	/* 5 MP */
};

/* Debug functions */
static int debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug Enabled (0-1)");


/* list of image formats supported by mt9p031 sensor */
const static struct v4l2_fmtdesc mt9p031_formats[] = {
	{
		.description    = "Bayer12 (GrR/BGb)",
		.pixelformat    = V4L2_PIX_FMT_SGRBG12,
	}
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(mt9p031_formats)


/**
 * struct vcontrol - Video controls
 * @v4l2_queryctrl: V4L2 VIDIOC_QUERYCTRL ioctl structure
 * @current_value: current value of this control
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = MT9P031_SHUTTER_WIDTH_MIN,
			.maximum = MT9P031_SHUTTER_WIDTH_MAX,
			.step = 1,
			.default_value = MT9P031_SHUTTER_WIDTH_DEF,
		},
		.current_value = MT9P031_SHUTTER_WIDTH_DEF,
	},
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Gain",
			.minimum = MT9P031_GLOBAL_GAIN_MIN,
			.maximum = MT9P031_GLOBAL_GAIN_MAX,
			.step = 1,
			.default_value = MT9P031_GLOBAL_GAIN_DEF,
		},
		.current_value = MT9P031_GLOBAL_GAIN_DEF,
	}
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

static int mt9p031_read(struct i2c_client *client, u8 reg)
{
	s32 data = i2c_smbus_read_word_data(client, reg);
	return data < 0 ? data : be16_to_cpu(data);
}

static int mt9p031_write(struct i2c_client *client, u8 reg, u16 data)
{
	return i2c_smbus_write_word_data(client, reg, cpu_to_be16(data));
}

static int mt9p031_set_output_control(struct mt9p031_sensor *mt9p031, u16 clear,
				      u16 set)
{
	struct i2c_client *client = to_i2c_client(mt9p031->dev);
	u16 value = (mt9p031->output_control & ~clear) | set;
	int ret;

	ret = mt9p031_write(client, MT9P031_OUTPUT_CONTROL, value);
	if (ret < 0)
		return ret;

	mt9p031->output_control = value;
	return 0;
}


static int mt9p031_set_mode2(struct mt9p031_sensor *mt9p031, u16 clear, u16 set)
{
	struct i2c_client *client = to_i2c_client(mt9p031->dev);
	u16 value = (mt9p031->mode2 & ~clear) | set;
	int ret;

	ret = mt9p031_write(client, MT9P031_READ_MODE_2, value);
	if (ret < 0)
		return ret;

	mt9p031->mode2 = value;
	return 0;
}

static int mt9p031_reset(struct mt9p031_sensor *mt9p031)
{
	struct i2c_client *client = to_i2c_client(mt9p031->dev);
	int ret;

	/* Disable chip output, synchronous option update */
	ret = mt9p031_write(client, MT9P031_RST, MT9P031_RST_ENABLE);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_RST, MT9P031_RST_DISABLE);
	if (ret < 0)
		return ret;

	return mt9p031_set_output_control(mt9p031, MT9P031_OUTPUT_CONTROL_CEN,
					  0);
}



/*
 * This static table uses ext_freq and vdd_io values to select suitable
 * PLL dividers m, n and p1 which have been calculated as specifiec in p36
 * of Aptina's mt9p031 datasheet. New values should be added here.
 */
static const struct mt9p031_pll_divs mt9p031_divs[] = {
	/* ext_freq	target_freq	m	n	p1 */
	{21000000,	48000000,	26,	2,	6}
};

static int mt9p031_pll_get_divs(struct mt9p031_sensor *mt9p031)
{
	struct i2c_client *client = to_i2c_client(mt9p031->dev);
	int i;

	for (i = 0; i < ARRAY_SIZE(mt9p031_divs); i++) {
		if (mt9p031_divs[i].ext_freq == mt9p031->pdata->ext_freq &&
		  mt9p031_divs[i].target_freq == mt9p031->pdata->target_freq) {
			mt9p031->pll = &mt9p031_divs[i];
			return 0;
		}
	}

	dev_err(&client->dev, "Couldn't find PLL dividers for ext_freq = %d, "
		"target_freq = %d\n", mt9p031->pdata->ext_freq,
		mt9p031->pdata->target_freq);
	return -EINVAL;
}

static int mt9p031_pll_enable(struct mt9p031_sensor *mt9p031)
{
	struct i2c_client *client = to_i2c_client(mt9p031->dev);
	int ret;

	ret = mt9p031_write(client, MT9P031_PLL_CONTROL,
			    MT9P031_PLL_CONTROL_PWRON);
	if (ret < 0)
		return ret;

	ret = mt9p031_write(client, MT9P031_PLL_CONFIG_1,
			    (mt9p031->pll->m << 8) | (mt9p031->pll->n - 1));
	if (ret < 0)
		return ret;

	ret = mt9p031_write(client, MT9P031_PLL_CONFIG_2, mt9p031->pll->p1 - 1);
	if (ret < 0)
		return ret;

	mdelay(10);
	ret = mt9p031_write(client, MT9P031_PLL_CONTROL,
			    MT9P031_PLL_CONTROL_PWRON |
			    MT9P031_PLL_CONTROL_USEPLL);
	return ret;
}

static inline int mt9p031_pll_disable(struct mt9p031_sensor *mt9p031)
{
	struct i2c_client *client = to_i2c_client(mt9p031->dev);

	return mt9p031_write(client, MT9P031_PLL_CONTROL,
			     MT9P031_PLL_CONTROL_PWROFF);
}

static int mt9p031_power_on(struct mt9p031_sensor *mt9p031)
{
	/* Ensure RESET_BAR is low */
	if (mt9p031->pdata->reset) {
		mt9p031->pdata->reset(mt9p031->v4l2_int_device, 1);
		mdelay(10);
	}

	/* Emable clock */
	if (mt9p031->pdata->set_xclk)
		mt9p031->pdata->set_xclk(mt9p031->v4l2_int_device,
					 mt9p031->pdata->ext_freq);

	/* Now RESET_BAR must be high */
	if (mt9p031->pdata->reset) {
		mt9p031->pdata->reset(mt9p031->v4l2_int_device, 0);
		mdelay(10);
	}

	return 0;
}

static void mt9p031_power_off(struct mt9p031_sensor *mt9p031)
{
	if (mt9p031->pdata->reset) {
		mt9p031->pdata->reset(mt9p031->v4l2_int_device, 1);
		mdelay(10);
	}

	if (mt9p031->pdata->set_xclk)
		mt9p031->pdata->set_xclk(mt9p031->v4l2_int_device, 0);
}



static int mt9p031_set_params(struct mt9p031_sensor *mt9p031)
{
	struct i2c_client *client = to_i2c_client(mt9p031->dev);
	const struct v4l2_pix_format *format = &mt9p031->pix;
	const struct v4l2_rect *crop = &mt9p031->crop;
	unsigned int hblank;
	unsigned int vblank;
	unsigned int xskip;
	unsigned int yskip;
	unsigned int xbin;
	unsigned int ybin;
	int ret;

	/* Windows position and size.
	 *
	 * TODO: Make sure the start coordinates and window size match the
	 * skipping, binning and mirroring (see description of registers 2 and 4
	 * in table 13, and Binning section on page 41).
	 */
	ret = mt9p031_write(client, MT9P031_COLUMN_START, crop->left);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_ROW_START, crop->top);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_WINDOW_WIDTH, crop->width - 1);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_WINDOW_HEIGHT, crop->height - 1);
	if (ret < 0)
		return ret;

	/* Row and column binning and skipping. Use the maximum binning value
	 * compatible with the skipping settings.
	 */
	xskip = DIV_ROUND_CLOSEST(crop->width, format->width);
	yskip = DIV_ROUND_CLOSEST(crop->height, format->height);
	xbin = 1 << (ffs(xskip) - 1);
	ybin = 1 << (ffs(yskip) - 1);

	ret = mt9p031_write(client, MT9P031_COLUMN_ADDRESS_MODE,
			    ((xbin - 1) << 4) | (xskip - 1));
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_ROW_ADDRESS_MODE,
			    ((ybin - 1) << 4) | (yskip - 1));
	if (ret < 0)
		return ret;

	/* Blanking - use minimum value for horizontal blanking and default
	 * value for vertical blanking.
	 */
	hblank = 346 * ybin + 64 + (80 >> max_t(unsigned int, xbin, 3));
	vblank = MT9P031_VERTICAL_BLANK_DEF;

	ret = mt9p031_write(client, MT9P031_HORIZONTAL_BLANK, hblank);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_VERTICAL_BLANK, vblank);
	if (ret < 0)
		return ret;

	return ret;
}


/**
 * mt9p031_configure - Configure the mt9p031 for the specified image mode
 * @s: pointer to standard V4L2 device structure
 *
 * Configure the mt9p031 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the mt9p031.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int mt9p031_configure(struct v4l2_int_device *s)
{
	struct mt9p031_sensor *mt9p031 = s->priv;
	int ret;

	/* Set HFLIP & VFLIP */
	mt9p031_set_mode2(mt9p031, MT9P031_READ_MODE_2_COL_MIR, 0);
	mt9p031_set_mode2(mt9p031, MT9P031_READ_MODE_2_ROW_MIR, 0);

	ret = mt9p031_set_params(mt9p031);
	if (ret < 0)
		return ret;

	/* Switch to master "normal" mode */
	ret = mt9p031_set_output_control(mt9p031, 0,
					 MT9P031_OUTPUT_CONTROL_CEN);
	if (ret < 0)
		return ret;

	ret = mt9p031_pll_enable(mt9p031);

	return ret;
}

/**
 * mt9p031_detect - Detect if an mt9p031 is present, and if so which revision
 * @client: pointer to the i2c client driver structure
 *
 * Detect if an mt9p031 is present, and if so which revision.
 * A device is considered to be detected if the manufacturer ID (MIDH and MIDL)
 * and the product ID (PID) registers match the expected values.
 * Any value of the version ID (VER) register is accepted.
 * Here are the version numbers we know about:
 * Returns a negative error number if no device is detected, or the
 * non-negative value of the version ID register if a device is detected.
 */
static int mt9p031_detect(struct i2c_client *client)
{
	s32 data;

	/* Read out the chip version register */
	data = mt9p031_read(client, MT9P031_CHIP_VERSION);
	if (data != MT9P031_CHIP_VERSION_VALUE) {
		dev_err(&client->dev, "MT9P031 not detected, wrong version "
			"0x%04x\n", data);
		return -ENODEV;
	}

	dev_info(&client->dev, "MT9P031 detected at address 0x%02x\n",
		 client->addr);

	return 0;

}


static int __mt9p031_set_power(struct mt9p031_sensor *mt9p031, bool on)
{
	struct i2c_client *client = to_i2c_client(mt9p031->dev);
	int ret;

	if (!on) {
		mt9p031_power_off(mt9p031);
		return 0;
	}

	ret = mt9p031_power_on(mt9p031);
	if (ret < 0)
		return ret;

	ret = mt9p031_reset(mt9p031);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to reset the camera\n");
		return ret;
	}

	return 0;
}

/**
 * mt9p031_set_exposure_time - sets exposure time per input value
 * @exp_time: exposure time to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 exposure entry in video_controls array
 *
 * If the requested exposure time is within the allowed limits, the HW
 * is configured to use the new exposure time, and the video_controls
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
static int mt9p031_set_exposure_time(u32 exp_time, struct v4l2_int_device *s,
				    struct vcontrol *lvc)
{
	struct mt9p031_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int err;

	if ((exp_time < lvc->qc.minimum) ||
			(exp_time > lvc->qc.maximum)) {
		dev_err(&client->dev, "Exposure time not within the "
			"legal range.\n");
		dev_err(&client->dev, "Min time %d us Max time %d us",
			lvc->qc.minimum, lvc->qc.maximum);
		return -EINVAL;
	}

	err = mt9p031_write(client, MT9P031_SHUTTER_WIDTH_UPPER,
				    (exp_time >> 16) & 0xffff);
	if (err < 0)
		return err;

	err =  mt9p031_write(client, MT9P031_SHUTTER_WIDTH_LOWER,
				     exp_time & 0xffff);

	if (err)
		dev_err(&client->dev, "Error setting exposure time %d\n",
									err);
	else
		lvc->current_value = exp_time;

	return err;
}

/**
 * mt9p031_set_gain - sets sensor analog gain per input value
 * @gain: analog gain value to be set on device
 * @s: pointer to standard V4L2 device structure
 * @lvc: pointer to V4L2 analog gain entry in video_controls array
 *
 * If the requested analog gain is within the allowed limits, the HW
 * is configured to use the new gain value, and the video_controls
 * array is updated with the new current value.
 * The function returns 0 upon success.  Otherwise an error code is
 * returned.
 */
static int mt9p031_set_gain(u16 gain, struct v4l2_int_device *s,
			   struct vcontrol *lvc)
{
	struct mt9p031_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	u16 data;
	int err;

	if ((gain < MT9P031_GLOBAL_GAIN_MIN) || (gain > MT9P031_GLOBAL_GAIN_MAX)) {
		dev_err(&client->dev, "Gain not within the legal range");
		return -EINVAL;
	}

	/* Gain is controlled by 2 analog stages and a digital stage.
	 * Valid values for the 3 stages are
	 *
	 * Stage                Min     Max     Step
	 * ------------------------------------------
	 * First analog stage   x1      x2      1
	 * Second analog stage  x1      x4      0.125
	 * Digital stage        x1      x16     0.125
	 *
	 * To minimize noise, the gain stages should be used in the
	 * second analog stage, first analog stage, digital stage order.
	 * Gain from a previous stage should be pushed to its maximum
	 * value before the next stage is used.
	 */
	if (gain <= 32) {
		data = gain;
	} else if (gain <= 64) {
		gain &= ~1;
		data = (1 << 6) | (gain >> 1);
	} else {
		gain &= ~7;
		data = ((gain - 64) << 5) | (1 << 6) | 32;
	}

	err = mt9p031_write(client, MT9P031_GLOBAL_GAIN, data);


	if (err) {
		dev_err(&client->dev, "Error setting gain.%d", err);
		return err;
	} else
		lvc->current_value = gain;

	return err;
}

/**
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
	return 0;
}

/**
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct vcontrol *lvc;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case  V4L2_CID_EXPOSURE:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	}

	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	int retval = -EINVAL;
	int i;
	struct vcontrol *lvc;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;
	lvc = &video_control[i];

	switch (vc->id) {
	case V4L2_CID_EXPOSURE:
		retval = mt9p031_set_exposure_time(vc->value, s, lvc);
		break;
	case V4L2_CID_GAIN:
		retval = mt9p031_set_gain(vc->value, s, lvc);
		break;
	}

	return retval;
}

/**
 * ioctl_enum_fmt_cap - Implement the CAPTURE buffer VIDIOC_ENUM_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @fmt: standard V4L2 VIDIOC_ENUM_FMT ioctl structure
 *
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = mt9p031_formats[index].flags;
	strlcpy(fmt->description, mt9p031_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = mt9p031_formats[index].pixelformat;

	return 0;
}

/**
 * ioctl_try_fmt_cap - Implement the CAPTURE buffer VIDIOC_TRY_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_TRY_FMT ioctl structure
 *
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	int isize = 2;  /* 0=1296x972; 1=2048x1536; 2=2592x1944 */
	int ifmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct mt9p031_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix2 = &sensor->pix;

	pix->width = mt9p031_sizes[isize].width;
	pix->height = mt9p031_sizes[isize].height;
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == mt9p031_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = mt9p031_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_SRGB;

	*pix2 = *pix;
	return 0;
}

/**
 * ioctl_s_fmt_cap - V4L2 sensor interface handler for VIDIOC_S_FMT ioctl
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 VIDIOC_S_FMT ioctl structure
 *
 * If the requested format is supported, configures the HW to use that
 * format, returns error code if format not supported or HW can't be
 * correctly configured.
 */
static int ioctl_s_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct mt9p031_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (!rval)
		sensor->pix = *pix;

	return rval;
}

/**
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct mt9p031_sensor *sensor = s->priv;
	f->fmt.pix = sensor->pix;

	return 0;
}

/**
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct mt9p031_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

/**
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct mt9p031_sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

	sensor->timeperframe = *timeperframe;
	*timeperframe = sensor->timeperframe;

	return 0;
}

/**
 * ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold sensor's private data address
 *
 * Returns device's (sensor's) private data area address in p parameter
 */
static int ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct mt9p031_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(s, p);
}

/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call mt9p031_configure())
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the dev. at slave detach.  The complement of ioctl_dev_init.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

/**
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.  Returns 0 if
 * mt9p031 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct mt9p031_sensor *sensor = s->priv;
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int err;

	err = mt9p031_detect(client);
	if (err < 0) {
		dev_err(&client->dev, "Unable to detect sensor\n");
		sensor->detected = 0;
		return err;
	}
	sensor->detected = 1;
	sensor->ver = err;
	dev_dbg(&client->dev, "Chip version 0x%02x detected\n", sensor->ver);

	return 0;
}
/**
 * ioctl_enum_framesizes - V4L2 sensor if handler for vidioc_int_enum_framesizes
 * @s: pointer to standard V4L2 device structure
 * @frms: pointer to standard V4L2 framesizes enumeration structure
 *
 * Returns possible framesizes depending on choosen pixel format
 **/
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *frms)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frms->pixel_format == mt9p031_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= 3)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = mt9p031_sizes[frms->index].width;
	frms->discrete.height = mt9p031_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract mt9p031_frameintervals[] = {
	{  .numerator = 1, .denominator = 15 },
	{  .numerator = 1, .denominator = 25 },
	{  .numerator = 1, .denominator = 30 },
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
				     struct v4l2_frmivalenum *frmi)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == mt9p031_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frmi->index != 0)
		return -EINVAL;

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
				mt9p031_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				mt9p031_frameintervals[frmi->index].denominator;

	return 0;
}


static int mt9p031_set_power(struct v4l2_int_device *s, int on)
{
	struct mt9p031_sensor *mt9p031 = s->priv;
	int ret = 0;

	mutex_lock(&mt9p031->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (mt9p031->power_count == !on) {
		ret = __mt9p031_set_power(mt9p031, !!on);
		if (ret < 0)
			goto out;
	}

	/* Update the power count. */
	mt9p031->power_count += on ? 1 : -1;
	WARN_ON(mt9p031->power_count < 0);

out:
	mutex_unlock(&mt9p031->power_lock);
	return ret;
}


/**
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 */
static int ioctl_s_power(struct v4l2_int_device *s, enum v4l2_power new_power)
{
	struct mt9p031_sensor *sensor = s->priv;
	int rval = 0;

	switch (new_power) {
	case V4L2_POWER_ON:
		rval = sensor->pdata->power_set(s, V4L2_POWER_ON);
		if (rval)
			break;

		rval = mt9p031_set_power(s, 1);
		if (rval == -EINVAL)
			break;

		if (sensor->detected)
			mt9p031_configure(s);
		else {
			rval = ioctl_dev_init(s);
			if (rval)
				goto err_on;
		}
		break;
	case V4L2_POWER_OFF:
err_on:
		rval = mt9p031_set_power(s, 0);
		rval = sensor->pdata->power_set(s, V4L2_POWER_OFF);
		if (rval)
			break;

		break;

	case V4L2_POWER_STANDBY:
		rval = mt9p031_set_power(s, 0);
		rval = sensor->pdata->power_set(s, V4L2_POWER_STANDBY);
		if (rval)
			break;

	default:
		return -EINVAL;
	}

	return rval;
}

static struct v4l2_int_ioctl_desc mt9p031_ioctl_desc[] = {
	{ .num = vidioc_int_enum_framesizes_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ .num = vidioc_int_enum_frameintervals_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ .num = vidioc_int_dev_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ .num = vidioc_int_dev_exit_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_dev_exit },
	{ .num = vidioc_int_s_power_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_power },
	{ .num = vidioc_int_g_priv_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_priv },
	{ .num = vidioc_int_init_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_init },
	{ .num = vidioc_int_enum_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ .num = vidioc_int_try_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
	{ .num = vidioc_int_g_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ .num = vidioc_int_s_fmt_cap_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ .num = vidioc_int_g_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ .num = vidioc_int_s_parm_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ .num = vidioc_int_queryctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ .num = vidioc_int_g_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ .num = vidioc_int_s_ctrl_num,
	  .func = (v4l2_int_ioctl_func *)ioctl_s_ctrl },
};

static struct v4l2_int_slave mt9p031_slave = {
	.ioctls = mt9p031_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9p031_ioctl_desc),
};

static struct v4l2_int_device mt9p031_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9p031_slave,
	},
};

/**
 * mt9p031_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int mt9p031_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct mt9p031_sensor *mt9p031;
	struct mt9p031_platform_data *pdata;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "no platform data?\n");
		return -EINVAL;
	}

	mt9p031 = kzalloc(sizeof(*mt9p031), GFP_KERNEL);
	if (!mt9p031)
		return -ENOMEM;

	/* Don't keep pointer to platform data, copy elements instead */
	mt9p031->pdata = kzalloc(sizeof(*mt9p031->pdata), GFP_KERNEL);
	if (!mt9p031->pdata) {
		err = -ENOMEM;
		goto on_err1;
	}

	mt9p031->pdata->power_set = pdata->power_set;
	mt9p031->pdata->set_xclk = pdata->set_xclk;
	mt9p031->pdata->priv_data_set = pdata->priv_data_set;
	mt9p031->pdata->reset = pdata->reset;
	mt9p031->pdata->ext_freq = pdata->ext_freq;
	mt9p031->pdata->target_freq = pdata->target_freq;

	mt9p031->output_control	= MT9P031_OUTPUT_CONTROL_DEF;
	mt9p031->mode2 = MT9P031_READ_MODE_2_ROW_BLC;

	/* Set mt9p031 default values */
	mt9p031->timeperframe.numerator = 1;
	mt9p031->timeperframe.denominator = 15;

	mt9p031->pix.width = MT9P031_WINDOW_WIDTH_DEF;
	mt9p031->pix.height = MT9P031_WINDOW_HEIGHT_DEF;
	mt9p031->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;

	mt9p031->crop.left = MT9P031_COLUMN_START_DEF;
	mt9p031->crop.top = MT9P031_ROW_START_DEF;
	mt9p031->crop.width = MT9P031_WINDOW_WIDTH_DEF;
	mt9p031->crop.height = MT9P031_WINDOW_HEIGHT_DEF;

	mt9p031->xskip = 1;
	mt9p031->yskip = 1;

	mt9p031->v4l2_int_device = &mt9p031_int_device;
	mt9p031->v4l2_int_device->priv = mt9p031;
	mt9p031->dev = &client->dev;

	i2c_set_clientdata(client, mt9p031);

	err = v4l2_int_device_register(mt9p031->v4l2_int_device);
	if (err) {
		goto on_err2;
	}
	mt9p031_pll_get_divs(mt9p031);

	return 0;
on_err2:
	i2c_set_clientdata(client, NULL);
	kfree(mt9p031->pdata);
on_err1:
	kfree(mt9p031);
	return err;
}

/**
 * mt9p031_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of mt9p031_probe().
 */
static int mt9p031_remove(struct i2c_client *client)
{
	struct mt9p031_sensor *sensor = i2c_get_clientdata(client);

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);
	kfree(sensor->pdata);
	kfree(sensor);

	return 0;
}

static const struct i2c_device_id mt9p031_id[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, mt9p031_id);

static struct i2c_driver mt9p031_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9p031_probe,
	.remove = mt9p031_remove,
	.id_table = mt9p031_id,
};

/**
 * mt9p031_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init mt9p031_init(void)
{
	return i2c_add_driver(&mt9p031_i2c_driver);
}
module_init(mt9p031_init);

/**
 * mt9p031_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of mt9p031_init.
 */
static void __exit mt9p031_cleanup(void)
{
	i2c_del_driver(&mt9p031_i2c_driver);
}
module_exit(mt9p031_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mt9p031 camera sensor driver");
