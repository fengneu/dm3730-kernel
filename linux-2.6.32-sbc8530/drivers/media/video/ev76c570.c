/*
 * EV76C570 based spi CMOS sensor driver
 *
 * Copyright(C) 2017 Jasper Zhang
 *
 */
#define DEBUG 	1

#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include <media/v4l2-int-device.h>
#include <media/ev76c570.h>
#include <asm/irq.h>

#define DRIVER_NAME  "ev76c570"

/* Analog gain values */
#define EV76C570_MIN_GAIN	0x00
#define EV76C570_MAX_GAIN	0x07
#define EV76C570_DEF_GAIN	0x00
#define EV76C570_GAIN_STEP   	0x1

/* Exposure time values, for 57MHz CLK_CTRL & 0xC9 line_length,
 * step is 21.89us
 */
#define EV76C570_DEF_MIN_EXPOSURE	0x000F	/* 15ms */
#define EV76C570_DEF_MAX_EXPOSURE	0x03E8	/* 1000ms */
#define EV76C570_DEF_EXPOSURE	    0x0028
#define EV76C570_EXPOSURE_STEP      1


/* capture 2 MP */
#define EV76C570_IMAGE_WIDTH_MAX	1600
#define EV76C570_IMAGE_HEIGHT_MAX	1200
#define EV76C570_IMAGE_WIDTH_DEF	1600	//1280
#define EV76C570_IMAGE_HEIGHT_DEF	1200	//1024

/* EV76C570 has 8/16 registers */
#define EV76C570_8BIT			1
#define EV76C570_16BIT			2

/* terminating token for reg list */
#define EV76C570_TOK_TERM 		0xFF

/* delay token for reg list */
#define EV76C570_TOK_DELAY		100

/* The ID values we are looking for */
#define EV76C570_CHIP_ID			0x0900
/**
 * struct ev76c570_reg - ev76c570 register format
 * @length: length of the register
 * @reg: 8-bit offset to register
 * @val: 8/16bit register value
 *
 * Define a structure for ev76c570 register initialization values
 */
struct ev76c570_reg {
	u8 length;
	u8 reg;
	u16 val;
};


/**
 * struct capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct ev76c570_capture_size {
	unsigned long width;
	unsigned long height;
};

/*
 * Array of image sizes supported by EV76C570.  These must be ordered from
 * smallest image size to largest.
 */
const static struct ev76c570_capture_size ev76c570_sizes[] = {
	{ 1600, 1200 },	/* Full-Size */
	{ 1280, 1024 },	/* ROI1-Size */
};

#define CAPTURE_WIDTH	1600
#define CAPTURE_HEIGHT	1200


struct ev76c570_sensor {
	struct spi_device	*spi;
	struct ev76c570_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int color_en;
	int scaler;
	int chipid;
	int fps;
	int detected;
	unsigned long xclk_current;
};


/* list of image formats supported by ev76c570 sensor */
const static struct v4l2_fmtdesc ev76c570_formats[] = {
	{
		.description    = "Bayer10 (GrR/BGb)",
		.pixelformat    = V4L2_PIX_FMT_SGRBG10,
	}
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(ev76c570_formats)



/* Structure to set analog gain */
static struct ev76c570_reg set_analog_gain[] = {
	{EV76C570_16BIT, 0x11, 0x0000},		/* ROI1 */
	/* updating, other ROIs */
	
	{EV76C570_TOK_TERM, 0, 0},	/* End of List */
};

static struct ev76c570_reg set_exposure_time[] = {
	{EV76C570_16BIT, 0x0E, 0x0000},		/* ROI1 */
	/* updating, other ROIs */

	{EV76C570_TOK_TERM, 0, 0},	/* End of List */
};



static struct ev76c570_reg initial_common_regs[] = {
	{EV76C570_16BIT, 0x07, 0x0A06},

	{EV76C570_16BIT, 0x3A, 0x80AF},
	{EV76C570_16BIT, 0x3F, 0x1E1E},

	{EV76C570_16BIT, 0x47, 0x036F},
	{EV76C570_16BIT, 0x48, 0xFFFD},
	{EV76C570_16BIT, 0x49, 0x8D6F},
	{EV76C570_16BIT, 0x4A, 0xBAC8},
	{EV76C570_16BIT, 0x4B, 0x0127},
	{EV76C570_16BIT, 0x4C, 0x0B1E},
	{EV76C570_16BIT, 0x4D, 0x161C},
	{EV76C570_16BIT, 0x4E, 0xA0B9},
	{EV76C570_16BIT, 0x4F, 0x0124},

	{EV76C570_16BIT, 0x51, 0x308C},
	{EV76C570_16BIT, 0x52, 0x8383},
	{EV76C570_16BIT, 0x53, 0x053D},
	{EV76C570_16BIT, 0x54, 0x6F8C},
	{EV76C570_16BIT, 0x55, 0x053C},
	{EV76C570_16BIT, 0x56, 0x3F44},
	{EV76C570_16BIT, 0x57, 0x3F4E},
	{EV76C570_16BIT, 0x58, 0x053C},
	{EV76C570_16BIT, 0x59, 0x708B},
	{EV76C570_16BIT, 0x5A, 0x0760},
	{EV76C570_16BIT, 0x5B, 0x053D},
	{EV76C570_16BIT, 0x5C, 0x6F8C},
	{EV76C570_16BIT, 0x5E, 0x5257},

	{EV76C570_16BIT, 0x67, 0x4452},
	{EV76C570_16BIT, 0x68, 0x0541},
	{EV76C570_16BIT, 0x69, 0x5471},
	{EV76C570_16BIT, 0x6A, 0x0941},
	{EV76C570_16BIT, 0x6B, 0x5470},
	{EV76C570_16BIT, 0x6C, 0x0871},
	{EV76C570_16BIT, 0x6E, 0x0941},
	{EV76C570_16BIT, 0x6F, 0x0745},

	{EV76C570_16BIT, 0x70, 0x0541},
	{EV76C570_16BIT, 0x71, 0x5471},
	{EV76C570_16BIT, 0x73, 0x5053},
	{EV76C570_16BIT, 0x79, 0x2000},
	{EV76C570_16BIT, 0x7A, 0x308A},
	{EV76C570_16BIT, 0x7B, 0x0101},

	{EV76C570_TOK_TERM, 0, 0},	/* End of List */
};

static struct ev76c570_reg initial_setup_regs[] = {
	{EV76C570_16BIT, 0x04, 0x8866},
	{EV76C570_16BIT, 0x05, 0x0000},
	{EV76C570_16BIT, 0x06, 0x345A},
	{EV76C570_16BIT, 0x07, 0x0A01},
	{EV76C570_16BIT, 0x08, 0xDF21},
	{EV76C570_16BIT, 0x09, 0x6335},
	//{EV76C570_16BIT, 0x0A, 0x02C1},
	{EV76C570_16BIT, 0x0A, 0x02C0},
	{EV76C570_16BIT, 0x0B, 0x0006},
	{EV76C570_16BIT, 0x0C, 0x0000},
	{EV76C570_16BIT, 0x0D, 0x0000},
	//{EV76C570_16BIT, 0x0E, 0x186A},
	{EV76C570_16BIT, 0x0E, 0x0720},
	{EV76C570_16BIT, 0x0F, 0x0000},

	{EV76C570_16BIT, 0x10, 0x0000},
	{EV76C570_16BIT, 0x11, 0x0000},
	{EV76C570_16BIT, 0x12, 0x0006},
	//{EV76C570_16BIT, 0x13, 0x0400},
	{EV76C570_16BIT, 0x13, 0x04B0},
	{EV76C570_16BIT, 0x14, 0x0006},
	//{EV76C570_16BIT, 0x15, 0x0500},
	{EV76C570_16BIT, 0x15, 0x0640},
	{EV76C570_16BIT, 0x16, 0x0406},
	{EV76C570_16BIT, 0x17, 0x0000},
	//{EV76C570_16BIT, 0x18, 0x0506},
	{EV76C570_16BIT, 0x18, 0x0000},
	{EV76C570_16BIT, 0x19, 0x0000},
	{EV76C570_16BIT, 0x1A, 0x0000},
	//{EV76C570_16BIT, 0x1B, 0x0391},
	{EV76C570_16BIT, 0x1B, 0x016F},
	{EV76C570_16BIT, 0x1C, 0x0000},
	{EV76C570_16BIT, 0x1D, 0x0000},
	//{EV76C570_16BIT, 0x1E, 0x0000},
	{EV76C570_16BIT, 0x1E, 0x0300},
	{EV76C570_16BIT, 0x1F, 0x0006},

	{EV76C570_16BIT, 0x20, 0x0400},
	{EV76C570_16BIT, 0x21, 0x0006},
	{EV76C570_16BIT, 0x22, 0x0500},
	{EV76C570_16BIT, 0x23, 0x0000},
	//{EV76C570_16BIT, 0x24, 0x0391},
	{EV76C570_16BIT, 0x24, 0x016F},
	{EV76C570_16BIT, 0x25, 0x0000},
	{EV76C570_16BIT, 0x26, 0x0000},
	//{EV76C570_16BIT, 0x27, 0x0000},
	{EV76C570_16BIT, 0x27, 0x0300},
	{EV76C570_16BIT, 0x28, 0x0006},
	{EV76C570_16BIT, 0x29, 0x0400},
	{EV76C570_16BIT, 0x2A, 0x0006},
	{EV76C570_16BIT, 0x2B, 0x0500},
	{EV76C570_16BIT, 0x2C, 0x0000},
	//{EV76C570_16BIT, 0x2D, 0x0391},
	{EV76C570_16BIT, 0x2D, 0x016F},
	{EV76C570_16BIT, 0x2E, 0x0000},
	{EV76C570_16BIT, 0x2F, 0x0000},

	//{EV76C570_16BIT, 0x30, 0x0000},
	{EV76C570_16BIT, 0x30, 0x0300},
	{EV76C570_16BIT, 0x31, 0x0006},
	{EV76C570_16BIT, 0x32, 0x0400},
	{EV76C570_16BIT, 0x33, 0x0006},
	{EV76C570_16BIT, 0x34, 0x0500},
	{EV76C570_16BIT, 0x35, 0x0000},
	{EV76C570_16BIT, 0x36, 0x8080},
	{EV76C570_16BIT, 0x37, 0x8080},
	{EV76C570_16BIT, 0x38, 0x0080},
	{EV76C570_16BIT, 0x39, 0x3880},
	{EV76C570_16BIT, 0x3A, 0x80AF},
	//{EV76C570_16BIT, 0x3B, 0x0000},
	//{EV76C570_16BIT, 0x3C, 0xA520},
	//{EV76C570_16BIT, 0x3D, 0x1DC0},
	//{EV76C570_16BIT, 0x3E, 0x00C0},
	{EV76C570_16BIT, 0x3F, 0x1E1E},

	{EV76C570_16BIT, 0x40, 0x0000},
	//{EV76C570_16BIT, 0x41, 0xE93F},
	{EV76C570_16BIT, 0x41, 0x9922},
	{EV76C570_16BIT, 0x42, 0x001D},
	{EV76C570_16BIT, 0x43, 0x0104},
	{EV76C570_16BIT, 0x44, 0x7C00},
	{EV76C570_16BIT, 0x45, 0xA6A5},
	{EV76C570_16BIT, 0x46, 0x4300},
	//{EV76C570_16BIT, 0x47, 0x036F},
	{EV76C570_16BIT, 0x47, 0x034F},
	{EV76C570_16BIT, 0x48, 0xFFFD},
	{EV76C570_16BIT, 0x49, 0x7A6F},
	{EV76C570_16BIT, 0x4A, 0xBAC8},
	{EV76C570_16BIT, 0x4B, 0x0127},
	{EV76C570_16BIT, 0x4C, 0x0B1E},
	{EV76C570_16BIT, 0x4D, 0x161C},
	{EV76C570_16BIT, 0x4E, 0xA0B9},
	{EV76C570_16BIT, 0x4F, 0x0124},

	{EV76C570_16BIT, 0x50, 0x0000},
	{EV76C570_16BIT, 0x51, 0x308C},
	{EV76C570_16BIT, 0x52, 0x8383},
	{EV76C570_16BIT, 0x53, 0x053D},
	{EV76C570_16BIT, 0x54, 0x6F8C},
	{EV76C570_16BIT, 0x55, 0x053C},
	{EV76C570_16BIT, 0x56, 0x3F44},
	{EV76C570_16BIT, 0x57, 0x3F4E},
	{EV76C570_16BIT, 0x58, 0x053C},
	{EV76C570_16BIT, 0x59, 0x708B},
	{EV76C570_16BIT, 0x5A, 0x0760},
	{EV76C570_16BIT, 0x5B, 0x053D},
	{EV76C570_16BIT, 0x5C, 0x6F8C},
	{EV76C570_16BIT, 0x5D, 0x0104},
	{EV76C570_16BIT, 0x5E, 0x5257},
	{EV76C570_16BIT, 0x5F, 0x1427},

	{EV76C570_16BIT, 0x60, 0x0000},
	{EV76C570_16BIT, 0x61, 0x0105},
	{EV76C570_16BIT, 0x62, 0x0D22},
	{EV76C570_16BIT, 0x63, 0x0F1E},
	{EV76C570_16BIT, 0x64, 0x0101},
	{EV76C570_16BIT, 0x65, 0x0513},
	{EV76C570_16BIT, 0x66, 0x0426},
	{EV76C570_16BIT, 0x67, 0x4452},
	{EV76C570_16BIT, 0x68, 0x0541},
	{EV76C570_16BIT, 0x69, 0x5471},
	{EV76C570_16BIT, 0x6A, 0x0941},
	{EV76C570_16BIT, 0x6B, 0x5470},
	{EV76C570_16BIT, 0x6C, 0x0871},
	{EV76C570_16BIT, 0x6D, 0x516E},
	{EV76C570_16BIT, 0x6E, 0x0941},
	{EV76C570_16BIT, 0x6F, 0x0745},

	{EV76C570_16BIT, 0x70, 0x0541},
	{EV76C570_16BIT, 0x71, 0x5471},
	{EV76C570_16BIT, 0x72, 0x0104},
	{EV76C570_16BIT, 0x73, 0x5053},
	{EV76C570_16BIT, 0x74, 0x0000},
	{EV76C570_16BIT, 0x75, 0x0000},
	{EV76C570_16BIT, 0x76, 0x0000},
	{EV76C570_16BIT, 0x77, 0x0000},
	{EV76C570_16BIT, 0x78, 0x0000},
	//{EV76C570_16BIT, 0x79, 0x0000},
	{EV76C570_16BIT, 0x79, 0x2000},
	{EV76C570_16BIT, 0x7A, 0x308A},
	{EV76C570_16BIT, 0x7B, 0x0101},
	{EV76C570_16BIT, 0x7C, 0x0000},
	{EV76C570_16BIT, 0x7D, 0xFFFF},
	//{EV76C570_16BIT, 0x7E, 0x0000},
	{EV76C570_16BIT, 0x7E, 0x0056},
	//{EV76C570_16BIT, 0x7F, 0x0900},

	{EV76C570_TOK_TERM, 0, 0},	/* End of List */
};




static int ev76c570_write_reg(struct spi_device *spi, u8 data_length,
			u8 reg, u16 val)
{
	struct spi_message msg;
	struct spi_transfer wreg_xfer = {
		.len		= 3,
		//.delay_usecs	= 50,
	};
	u8	buffer[4];

	spi_message_init(&msg);

	/* register write */
	if (data_length == EV76C570_8BIT) {
		buffer[0] = reg | 0x80;	/* first bit = 1 */
		buffer[1] =  (u8) (val & 0xff);
	} else {	/* 16Bit */
		buffer[0] = reg | 0x80;	/* first bit = 1 */
		buffer[1] = (u8) (val >> 8);
		buffer[2] = (u8) (val & 0xff);
	}
	wreg_xfer.len = 1 + data_length;
	wreg_xfer.tx_buf = buffer;
	spi_message_add_tail(&wreg_xfer, &msg);

	return spi_sync(spi, &msg);
}

#if 1
static int ev76c570_read_reg16(struct spi_device *spi, u8 reg, u16 *val)
{
	struct spi_message msg;
	struct spi_transfer addr_xfer = {
		.len		= 1,
	};
	struct spi_transfer val_xfer = {
		.len		= 2,
	};
	u8	buffer[8];
	int ret = 0;

	spi_message_init(&msg);

	/* register read */
	buffer[0] = reg & 0x7f;	/* first bit = 0 */
	addr_xfer.tx_buf = buffer;
	addr_xfer.rx_buf = NULL;
	spi_message_add_tail(&addr_xfer, &msg);

	val_xfer.tx_buf = NULL;
	val_xfer.rx_buf = buffer + 4;
	spi_message_add_tail(&val_xfer, &msg);

	ret = spi_sync(spi, &msg);
	printk("%s: read buff %02x %02x  \n", __func__, buffer[4], buffer[5]);
	*val = (buffer[4] << 8) | buffer[5];

	//return spi_sync(spi, &msg);
	return ret;
}
#else
static int ev76c570_read_reg16(struct spi_device *spi, u8 reg, u16 *val)
{
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 1,
	};
	u8	buffer[8];

	spi_message_init(&msg);

	/* register read */
	buffer[0] = reg & 0x7f;	/* first bit = 0 */
	xfer.tx_buf = buffer;
	xfer.rx_buf = val;
	spi_message_add_tail(&xfer, &msg);
	
	return spi_sync(spi, &msg);
}

#endif
/**
 * ev76c570_write_regs - Initializes a list of EV76C570 registers
 * @spidev: spi driver client structure
 * @reglist: list of registers to be written
 *
 * Initializes a list of EV76C570 registers. The list of registers is
 * terminated by EV76C570_TOK_TERM.
 */
static int ev76c570_write_regs(struct spi_device *spidev,
			      const struct ev76c570_reg reglist[])
{
	int err;
	const struct ev76c570_reg *next = reglist;

	for (; next->length != EV76C570_TOK_TERM; next++) {
		if (next->length == EV76C570_TOK_DELAY) {
			mdelay(next->val);
			continue;
		}

		err = ev76c570_write_reg(spidev, next->length,
						next->reg, next->val);
		if (err)
			return err;
	}
	return 0;
}


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
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Analog Gain",
			.minimum = EV76C570_MIN_GAIN,
			.maximum = EV76C570_MAX_GAIN,
			.step = EV76C570_GAIN_STEP,
			.default_value = EV76C570_DEF_GAIN,
		},
		.current_value = EV76C570_DEF_GAIN,
	},
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = EV76C570_DEF_MIN_EXPOSURE,
			.maximum = EV76C570_DEF_MAX_EXPOSURE,
			.step = EV76C570_EXPOSURE_STEP,
			.default_value = EV76C570_DEF_EXPOSURE,
		},
		.current_value = EV76C570_DEF_EXPOSURE,
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


/**
 * ev76c570_set_gain - sets sensor analog gain per input value
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
static int ev76c570_set_gain(u16 gain, struct v4l2_int_device *s,
			   struct vcontrol *lvc)
{
	int err;
	struct ev76c570_sensor *sensor = s->priv;
	struct spi_device *spidev = sensor->spi;

	if ((gain < EV76C570_MIN_GAIN) || (gain > EV76C570_MAX_GAIN)) {
		dev_err(&spidev->dev, "Gain not within the legal range");
		return -EINVAL;
	}
	set_analog_gain[0].val = (gain & 0x07) << 8;	/* Analog Gain */
	err = ev76c570_write_regs(spidev, set_analog_gain);
	if (err) {
		dev_err(&spidev->dev, "Error setting gain.%d", err);
		return err;
	} else
		lvc->current_value = gain;

	return err;
}

/**
 * ev76c570_set_exposure_time - sets exposure time per input value
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
static int ev76c570_set_exposure_time(u32 exp_time, struct v4l2_int_device *s,
				    struct vcontrol *lvc)
{
	int err;
	struct ev76c570_sensor *sensor = s->priv;
	struct spi_device *spidev = sensor->spi;
	u32 coarse_int_time = 0;

	printk("%s: exp_time %d.. \n", __func__, exp_time);
	if ((exp_time < EV76C570_DEF_MIN_EXPOSURE) ||
			(exp_time > EV76C570_DEF_MAX_EXPOSURE)) {
		dev_err(&spidev->dev, "Exposure time not within the "
			"legal range.\n");
		dev_err(&spidev->dev, "Min time %d us Max time %d us",
			EV76C570_DEF_MIN_EXPOSURE, EV76C570_DEF_MAX_EXPOSURE);
		return -EINVAL;
	}

	/* for line_length & clk_ctrl, 21.89us per step */
	coarse_int_time = (exp_time * 1000) / 22;

	set_exposure_time[0].val = coarse_int_time;	/* Analog Gain */
	err = ev76c570_write_regs(spidev, set_exposure_time);
	if (err) {
		dev_err(&spidev->dev, "Error setting gain.%d", err);
		return err;
	} else
		lvc->current_value = exp_time;

	printk("%s: exp_time %d with 0x0E reg 0x%04x.. \n", __func__, exp_time, coarse_int_time);
	return err;
}



/**
 * ev76c570_configure - Configure the ev76c570 for the specified image mode
 * @s: pointer to standard V4L2 device structure
 *
 * Configure the ev76c570 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the ev76c570.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int ev76c570_configure(struct v4l2_int_device *s)
{
	struct ev76c570_sensor *sensor = s->priv;
	struct spi_device *spidev = sensor->spi;
	struct vcontrol *lvc;
	int err = 0, i = 0;

	printk("Configure ev76c570 strean on .... \n");

	/* configure setup register and stream ON */
	err = ev76c570_write_regs(spidev, initial_setup_regs);
	printk("Configure ev76c570 strean on with err %d.... \n", err);

	/* update exposure time */
	i = find_vctrl(V4L2_CID_EXPOSURE);
	if (i >= 0)
		lvc = &video_control[i];

	err = ev76c570_set_exposure_time(lvc->current_value, s, lvc);

	return err;
}


/**
 * ev76c570_detect - Detect if an mt9p012 is present, and if so which revision
 * @client: pointer to the i2c client driver structure
 *
 * Detect if an mt9p012 is present, and if so which revision.
 */
static int ev76c570_detect(struct spi_device *spidev)
{
	u16 chipid = 0;
	int err;

	if (!spidev)
		return -ENODEV;

	if (ev76c570_read_reg16(spidev, 0x7F, &chipid))
		return -ENODEV;

	dev_info(&spidev->dev, "Chip id detected 0x%x \n", chipid);
	if (chipid != EV76C570_CHIP_ID) {
		/* We didn't read the values we expected, so
		 * this must not be an EV76C570?
		 */
		dev_warn(&spidev->dev, "Chip id mismatch 0x%x \n", chipid);

//		return -ENODEV;
	}

#if 1
	/* common register initialization */
	err = ev76c570_write_regs(spidev, initial_common_regs);
	if (err) {
		printk("## error write initial common regs \n");
		return err;
	}
#endif
	return 0;

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
	case V4L2_CID_GAIN:
		vc->value = lvc->current_value;
		break;
	case V4L2_CID_EXPOSURE:
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
	case V4L2_CID_GAIN:
		retval = ev76c570_set_gain(vc->value, s, lvc);
		break;
	case V4L2_CID_EXPOSURE:
		retval = ev76c570_set_exposure_time(vc->value, s, lvc);
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

	fmt->flags = ev76c570_formats[index].flags;
	strlcpy(fmt->description, ev76c570_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = ev76c570_formats[index].pixelformat;

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
	int isize = 0;	/* 1=1280x1024, 0=1600x1200 */
	int ifmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct ev76c570_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix2 = &sensor->pix;

	pix->width = ev76c570_sizes[isize].width;
	pix->height = ev76c570_sizes[isize].height;
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ev76c570_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = ev76c570_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_RGB555X:
	default:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
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
	struct ev76c570_sensor *sensor = s->priv;
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
	struct ev76c570_sensor *sensor = s->priv;
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
	struct ev76c570_sensor *sensor = s->priv;
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
	struct ev76c570_sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;

	sensor->timeperframe = *timeperframe;
	//sensor->xclk_current = mt9p012_calc_xclk(client);
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
	struct ev76c570_sensor *sensor = s->priv;

	return sensor->pdata->priv_data_set(s, p);
}

/**
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 *
 * Initialize the sensor device (call mt9p012_configure())
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
 * ev76c570 device could be found, otherwise returns appropriate error.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct ev76c570_sensor *sensor = s->priv;
	struct spi_device *spidev = sensor->spi;
	int err;

#if 1
	err = ev76c570_detect(spidev);
	if (err < 0) {
		dev_err(&spidev->dev, "Unable to detect sensor\n");
		sensor->detected = 0;
		return err;
	}
#endif
	sensor->detected = 1;
	sensor->chipid = err;
	dev_dbg(&spidev->dev, "Chip ID 0x%02x detected\n", sensor->chipid);

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
		if (frms->pixel_format == ev76c570_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frms->index >= 1)
		return -EINVAL;

	frms->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frms->discrete.width = ev76c570_sizes[frms->index].width;
	frms->discrete.height = ev76c570_sizes[frms->index].height;

	return 0;
}

const struct v4l2_fract ev76c570_frameintervals[] = {
	{  .numerator = 1, .denominator = 25 },	/* TODO: ? */
};

static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
				     struct v4l2_frmivalenum *frmi)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (frmi->pixel_format == ev76c570_formats[ifmt].pixelformat)
			break;
	}
	/* Is requested pixelformat not found on sensor? */
	if (ifmt == NUM_CAPTURE_FORMATS)
		return -EINVAL;

	/* Do we already reached all discrete framesizes? */
	if (frmi->index >= 1)
		return -EINVAL;

	frmi->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	frmi->discrete.numerator =
				ev76c570_frameintervals[frmi->index].numerator;
	frmi->discrete.denominator =
				ev76c570_frameintervals[frmi->index].denominator;

	return 0;
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
	struct ev76c570_sensor *sensor = s->priv;
	int rval = 0;

	switch (new_power) {
	case V4L2_POWER_ON:
		printk("%s-%d: power on.. \n", __func__, __LINE__);
		sensor->xclk_current = 24000000;	/* TODO: fix clk_ref */
		rval = sensor->pdata->set_xclk(s, sensor->xclk_current);
		if (rval == -EINVAL)
			break;
		rval = sensor->pdata->power_set(s, V4L2_POWER_ON);
		if (rval)
			break;

		if (sensor->detected) {
			ev76c570_configure(s);
			printk("%s-%d: stream on.. \n", __func__, __LINE__);
		} else {
			rval = ioctl_dev_init(s);
			if (rval)
				goto err_on;
			printk("%s-%d: .. \n", __func__, __LINE__);
		}
		break;
	case V4L2_POWER_OFF:
err_on:
		printk("%s: power off.. \n", __func__);
		rval = sensor->pdata->power_set(s, V4L2_POWER_OFF);
		sensor->pdata->set_xclk(s, 0);
		break;
	case V4L2_POWER_STANDBY:
		rval = sensor->pdata->power_set(s, V4L2_POWER_STANDBY);
		sensor->pdata->set_xclk(s, 0);
		break;
	default:
		return -EINVAL;
	}

	return rval;
}





static struct v4l2_int_ioctl_desc ev76c570_ioctl_desc[] = {
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

static struct v4l2_int_slave ev76c570_slave = {
	.ioctls = ev76c570_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ev76c570_ioctl_desc),
};

static struct v4l2_int_device ev76c570_int_device = {
	.module = THIS_MODULE,
	.name = DRIVER_NAME,
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ev76c570_slave,
	},
};


static int __devinit ev76c570_probe(struct spi_device *spi)
{
	struct ev76c570_sensor *sensor = NULL;
	struct ev76c570_platform_data *pdata = spi->dev.platform_data;
	int err;

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	sensor = kzalloc(sizeof (*sensor), GFP_KERNEL);
	if (!sensor) {
		dev_err(&spi->dev, "could not allocate ev76c570_sensor\n");
		return -ENOMEM;
	}
	
	/* Don't keep pointer to platform data, copy elements instead */
	sensor->pdata = kzalloc(sizeof(*sensor->pdata), GFP_KERNEL);
	if (!sensor->pdata) {
		err = -ENOMEM;
		goto on_err1;
	}
	sensor->spi = spi;
	spi_set_drvdata(spi, sensor);

	sensor->pdata->power_set = pdata->power_set;
	sensor->pdata->set_xclk = pdata->set_xclk;
	sensor->pdata->priv_data_set = pdata->priv_data_set;

	/* Set sensor default values, default 1280x1024 */
	sensor->pix.width = EV76C570_IMAGE_WIDTH_DEF;
	sensor->pix.height = EV76C570_IMAGE_HEIGHT_DEF;
	sensor->pix.pixelformat = V4L2_PIX_FMT_SGRBG10;

	sensor->v4l2_int_device = &ev76c570_int_device;
	sensor->v4l2_int_device->priv = sensor;

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err) {
		goto on_err2;
	}
	return 0;
on_err2:
	spi_set_drvdata(spi, NULL);
	kfree(sensor->pdata);
on_err1:
	kfree(sensor);
	return 0;
}


static int __devexit ev76c570_remove(struct spi_device *spi)
{
	struct ev76c570_sensor *sensor = spi_get_drvdata(spi);

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	spi_set_drvdata(spi, NULL);
	kfree(sensor->pdata);
	kfree(sensor);

	return 0;
}

static struct spi_driver ev76c570_driver = {
	.driver = {
		.name	= "ev76c570",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ev76c570_probe,
	.remove		= __devexit_p(ev76c570_remove),
};


static int __init ev76c570_init(void)
{
	return spi_register_driver(&ev76c570_driver);
}
module_init(ev76c570_init);

static void __exit ev76c570_exit(void)
{
	spi_unregister_driver(&ev76c570_driver);
}
module_exit(ev76c570_exit);

MODULE_DESCRIPTION("EV76C570 CMOS Sensor Driver");
MODULE_LICENSE("GPL");
