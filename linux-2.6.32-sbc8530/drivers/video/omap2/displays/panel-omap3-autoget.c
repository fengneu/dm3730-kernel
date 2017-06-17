/*
 * LCD panel driver for LG4573A
 *
 * Author: ZhangYan <niat_zhy@163.com>
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
#include <linux/spi/spi.h>
#include <linux/gpio.h>

#include <plat/display.h>

#define USE_GPIO	1
#ifdef USE_GPIO
#define PANEL_PIN_CS		175	
#define PANEL_PIN_SCL	171
#define PANEL_PIN_SDA	172
#endif	/* USE_GPIO */

static struct spi_device	*spidev;

static struct omap_video_timings lg4573_timings = {
#ifdef CONFIG_LCD_32inch
        .x_res          = 480,
        .y_res          = 800,

        .hsw            = 16,   /* 0xB5:SDT=0x10?  */
        .hfp            = 10,      /* right_margin (4) - 1 */
        .hbp            = 30,      /* 0xB1: HBP=0x1E */
        .vsw            = 16,       /* vsync_len (2) - 1 */
        .vfp            = 10,     /* lower_margin */
        .vbp            = 12,     /* 0xB1: VBP=0xC */

        .pixel_clock    = 36000,
#endif
};

static int lg4573_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = lg4573_timings;

	return 0;
}

static void lg4573_panel_remove(struct omap_dss_device *dssdev)
{
}
#ifdef USE_GPIO
#define S_DELAY_US	20

static int lg4573_write_reg(u8 data, int is_cmd)
{
	int i = 0;

	gpio_direction_output(PANEL_PIN_CS, 1);
	gpio_direction_output(PANEL_PIN_SCL, 1);
	gpio_direction_output(PANEL_PIN_CS, 0);

	if (is_cmd)
		gpio_direction_output(PANEL_PIN_SDA, 0);	/* Command */
	else
		gpio_direction_output(PANEL_PIN_SDA, 1);	/* Data */

	udelay(S_DELAY_US);

	gpio_direction_output(PANEL_PIN_SCL, 0);
	udelay(S_DELAY_US);
	gpio_direction_output(PANEL_PIN_SCL, 1);

	for (i = 0; i < 8; i++) {
		if(data & 0x80)
			gpio_direction_output(PANEL_PIN_SDA, 1);
		else
			gpio_direction_output(PANEL_PIN_SDA, 0);

		gpio_direction_output(PANEL_PIN_SCL, 0);
		udelay(S_DELAY_US);
		gpio_direction_output(PANEL_PIN_SCL, 1);
		udelay(S_DELAY_US);
		data <<= 1;
	}
	gpio_direction_output(PANEL_PIN_SDA, 1);
	gpio_direction_output(PANEL_PIN_SCL, 1);
	gpio_direction_output(PANEL_PIN_CS, 1);

	return 0;
}
#else	/* USE_GPIO */
#if 0
static int lg4573_write_reg(u8 reg, u8 *regval, int size)
{
	struct spi_message msg;
	struct spi_transfer index_xfer = {
		.len		= 2,
		.cs_change	= 1,
	};
	struct spi_transfer value_xfer = {
		//.len		= 3,
	};
	u8	ixbuffer[4];
	u8	regbuf[16];
	int i = 0;

	spi_message_init(&msg);

	/* register index */
	ixbuffer[0] = 0x70;
	ixbuffer[1] = reg;
	index_xfer.tx_buf = ixbuffer;
	spi_message_add_tail(&index_xfer, &msg);

	/* register value */
	if (regval != NULL) {
		regbuf[0] = 0x72;
		for (i=1; i<size+1; i++) {
			regbuf[i] = *(regval++);
			//printk("\t 0x%02x ", regbuf[i]);
		}

		value_xfer.tx_buf = regbuf;
		value_xfer.len = size+1;
		spi_message_add_tail(&value_xfer, &msg);
	}
	return spi_sync(spidev, &msg);
}
#else
static int lg4573_write_reg(u8 val, int is_command)
{
	struct spi_message msg;
	struct spi_transfer xfer = {
		.len		= 1,
		//.cs_change = 1,
		//.delay_usecs = 10,	/* delay 10ns */
		.speed_hz = 500000,	/* 500KHz, MUSTBE! just for reconfig chnl */
	};
	u8	buffer[4];

	spi_message_init(&msg);

	/* register index */
	buffer[0] = val;
	xfer.tx_buf = buffer;
	if (is_command) {
		/* REVISIT: 	notify omap2 mcspi dual for SBPOL 0 */
		xfer.bits_per_word = 0;
	} else {
		/* sbpol =1, notify omap2 mcspi dual for SBPOL 1 */
		xfer.bits_per_word = 8;
	}
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(spidev, &msg);
}

#endif
#endif	/* USE_GPIO */


static int lg4573_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	pr_info("lg4573: panel_enable: 0x%px\n", spidev);
	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable)
		r = dssdev->platform_enable(dssdev);

	/* Panel init */
	lg4573_write_reg(0x20, 1);	// 
	lg4573_write_reg(0x11, 1);	// Sleep Out Powers for the display are On

	lg4573_write_reg(0x3A, 1);	//Interface Pixel format
	lg4573_write_reg(0x66, 0);

	lg4573_write_reg(0xB1, 1);	//RGB Interface Setting
	lg4573_write_reg(0x06, 0);	/* HSPL | VSPL */
	lg4573_write_reg(0x1E, 0);
	lg4573_write_reg(0x0C, 0);

	lg4573_write_reg(0xB2, 1);	//Panel Characteristics Setting
	lg4573_write_reg(0x10, 0);
	lg4573_write_reg(0xC8, 0);

	lg4573_write_reg(0xB3, 1);	//Panel Drive Setting (Column Inversion)[ 1-Dot Inversion : Set 0001h ]
	lg4573_write_reg(0x00, 0);

	lg4573_write_reg(0xB4, 1);	 //Display Mode Control
	lg4573_write_reg(0x04, 0);

	lg4573_write_reg(0xB5, 1);	//Display Control (1)
	lg4573_write_reg(0x10, 0);
	lg4573_write_reg(0x30, 0);
	lg4573_write_reg(0x30, 0);
	lg4573_write_reg(0x00, 0);
	lg4573_write_reg(0x00, 0);

	lg4573_write_reg(0xB6, 1);	//Display Control (2)
	lg4573_write_reg(0x01, 0);
	lg4573_write_reg(0x18, 0);
	lg4573_write_reg(0x02, 0);
	lg4573_write_reg(0x40, 0);
	lg4573_write_reg(0x10, 0);
	lg4573_write_reg(0x00, 0);

	lg4573_write_reg(0xC0, 1);	//Oscillator Control OSC control----
	lg4573_write_reg(0x01, 0);
	lg4573_write_reg(0x1F, 0);

	lg4573_write_reg(0xC3, 1);	 //Power Control (3)
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x04, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x04, 0);

	lg4573_write_reg(0xC4, 1);	//Power Control (4)
	lg4573_write_reg(0x12, 0);
	lg4573_write_reg(0x23, 0);
	lg4573_write_reg(0x10, 0);
	lg4573_write_reg(0x10, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x6C, 0);

	lg4573_write_reg(0xC5, 1);	//Power Control (5)
	lg4573_write_reg(0x6B, 0);

	lg4573_write_reg(0xC6, 1);	//Power Control (6)
	lg4573_write_reg(0x24, 0);
	lg4573_write_reg(0x50, 0);
	lg4573_write_reg(0x00, 0);

	lg4573_write_reg(0xD0, 1);	//Positive Gamma for Red
	lg4573_write_reg(0x00, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x15, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x61, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x03, 0);

	lg4573_write_reg(0xD1, 1);	//Negative Gamma for Red
	lg4573_write_reg(0x00, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x15, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x61, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x03, 0);

	lg4573_write_reg(0xD2, 1);	//Positive Gamma for Green
	lg4573_write_reg(0x00, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x15, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x61, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x03, 0);

	lg4573_write_reg(0xD3, 1);	//Positive Gamma for Green
	lg4573_write_reg(0x00, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x15, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x61, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x03, 0);

	lg4573_write_reg(0xD4, 1);	//Positive Gamma for Blue
	lg4573_write_reg(0x00, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x15, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x61, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x03, 0);

	lg4573_write_reg(0xD5, 1);	//Positive Gamma for Blue
	lg4573_write_reg(0x00, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x44, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x15, 0);
	lg4573_write_reg(0x03, 0);
	lg4573_write_reg(0x61, 0);
	lg4573_write_reg(0x16, 0);
	lg4573_write_reg(0x03, 0);

	lg4573_write_reg(0x29, 1);	//Display on

	msleep(20);

	return r;
}

static void lg4573_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);
}

static int lg4573_panel_suspend(struct omap_dss_device *dssdev)
{
	pr_info("lg4573: panel_suspend\n");
	lg4573_panel_disable(dssdev);
	return 0;
}

static int lg4573_panel_resume(struct omap_dss_device *dssdev)
{
	pr_info("lg4573: panel_resume\n");
	return lg4573_panel_enable(dssdev);
}

static struct omap_dss_driver lg4573_driver = {
	.probe		= lg4573_panel_probe,
	.remove		= lg4573_panel_remove,

	.enable		= lg4573_panel_enable,
	.disable	= lg4573_panel_disable,
	.suspend	= lg4573_panel_suspend,
	.resume		= lg4573_panel_resume,

	.driver         = {
		.name   = "lg4573_panel",
		.owner  = THIS_MODULE,
	},
};
#ifdef USE_GPIO

static int lg4573_spi_gpio_init(void)
{
	if (gpio_request(PANEL_PIN_CS, "panel cs") < 0)
		printk(KERN_ERR "can't get panel cs GPIO\n");
	
	if (gpio_request(PANEL_PIN_SCL, "panel scl") < 0)
		printk(KERN_ERR "can't get panel scl GPIO\n");

	if (gpio_request(PANEL_PIN_SDA, "panel sda") < 0)
		printk(KERN_ERR "can't get panel sda GPIO\n");
	
	gpio_direction_output(PANEL_PIN_CS, 1);
	gpio_direction_output(PANEL_PIN_SCL, 1);
	gpio_direction_output(PANEL_PIN_SDA, 0);

	return 0;
}

static void lg4573_spi_gpio_free(void)
{
	gpio_free(PANEL_PIN_CS);
	gpio_free(PANEL_PIN_SCL);
	gpio_free(PANEL_PIN_SDA);
}
static int __init lg4573_panel_drv_init(void)
{
	int ret;

	lg4573_spi_gpio_init();

	ret = omap_dss_register_driver(&lg4573_driver);
	if (ret != 0)
		pr_err("lg4573: Unable to register panel driver: %d\n", ret);

	return ret;
}

static void __exit lg4573_panel_drv_exit(void)
{
	lg4573_spi_gpio_free();
	omap_dss_unregister_driver(&lg4573_driver);
}

#else
static int __devinit lg4573_panel_spi_probe(struct spi_device *spi)
{
	spidev = spi;
	return 0;
}

static int __devexit lg4573_panel_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver lg4573_spi_driver = {
	.driver		= {
		.name	= "lg4573_panel-spi",
		.owner	= THIS_MODULE,
	},
	.probe		= lg4573_panel_spi_probe,
	.remove		= __devexit_p (lg4573_panel_spi_remove),
};

static int __init lg4573_panel_drv_init(void)
{
	int ret;
	ret = spi_register_driver(&lg4573_spi_driver);
	if (ret != 0)
		pr_err("lg4573: Unable to register SPI driver: %d\n", ret);

	ret = omap_dss_register_driver(&lg4573_driver);
	if (ret != 0)
		pr_err("lg4573: Unable to register panel driver: %d\n", ret);

	return ret;
}

static void __exit lg4573_panel_drv_exit(void)
{
	spi_unregister_driver(&lg4573_spi_driver);
	omap_dss_unregister_driver(&lg4573_driver);
}
#endif	/* USE_GPIO */

module_init(lg4573_panel_drv_init);
module_exit(lg4573_panel_drv_exit);
MODULE_LICENSE("GPL");
