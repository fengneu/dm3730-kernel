/*
 * linux/arch/arm/mach-omap2/board-omap3autoget.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio_keys.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#include <linux/backlight.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/timer-gp.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/mcspi.h>
#include <plat/control.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "prm-regbits-34xx.h"
#include "omap3-opp.h"
#include "board-omap3evm-camera.h"

#include <linux/interrupt.h>

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE		SZ_128K

extern struct regulator_consumer_supply twl4030_vmmc1_supply;
extern struct regulator_consumer_supply twl4030_vmmc2_supply;
extern struct regulator_consumer_supply twl4030_vsim_supply;

extern struct regulator_init_data vmmc1_data;
extern struct regulator_init_data vmmc2_data;
extern struct regulator_init_data vsim_data;


static struct mtd_partition omap3autoget_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data omap3autoget_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= omap3autoget_nand_partitions,
	.nr_parts	= ARRAY_SIZE(omap3autoget_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource omap3autoget_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device omap3autoget_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap3autoget_nand_data,
	},
	.num_resources	= 1,
	.resource	= &omap3autoget_nand_resource,
};


/* DSS */
static int lcd_enabled;
static int dvi_enabled;
#define DVI_UP_GPIO         129

static void __init omap3_autoget_display_init(void)
{
#if 0
        int ret;


        ret = gpio_request(DVI_UP_GPIO, "dvi up");
        if (ret < 0) {
                printk(KERN_ERR "Failed to request GPIO %d for dvi up\n",
                                DVI_UP_GPIO);
        }
#endif
        return;
}


static int omap3_autoget_enable_lcd(struct omap_dss_device *dssdev)
{
        char value;

	twl_i2c_read_u8(TWL4030_MODULE_LED, &value, 0x0);
        twl_i2c_write_u8(TWL4030_MODULE_LED, value & ~(0x1), 0x0);

        lcd_enabled = 1;
        return 0;
}

static void omap3_autoget_disable_lcd(struct omap_dss_device *dssdev)
{
        char value;

        twl_i2c_read_u8(TWL4030_MODULE_LED, &value, 0x0);
        twl_i2c_write_u8(TWL4030_MODULE_LED, value | (0x1), 0x0);

        lcd_enabled = 0;
}

static struct omap_dss_device autoget_lcd_device = {
	.name                   = "lcd",
	.driver_name            = "lg4573_panel",
	.type                   = OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines     = 24,
	//.panel.recommended_bpp  = 16,
	.platform_enable        = omap3_autoget_enable_lcd,
	.platform_disable       = omap3_autoget_disable_lcd,
};

/*
 * PWMA/B register offsets (TWL4030_MODULE_PWMA)
 */
#define TWL_LED_EN      0x0
#define TWL_LED_PWMON   0x0
#define TWL_LED_PWMOFF  0x1

static void omap3autoget_set_bl_intensity(int intensity)
{
        unsigned char c;

        if (!lcd_enabled || intensity > 100)
                return;
        /*
         * Enable LEDA for backlight
         */
        if(0 == intensity){
                twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x90, 0xc);
                twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x00, 0xd);
		twl_i2c_write_u8(TWL4030_MODULE_GPIO, 0x40, 0x03);//GPIO6 set to output
		twl_i2c_write_u8(TWL4030_MODULE_GPIO, 0x00, 0x06);//GPIO6 output '0'
        }else{
                twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x95, 0xc);
                twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x04, 0xd);
                c = ((125 * (intensity)) / 100) + 1;
                twl_i2c_write_u8(TWL4030_MODULE_PWM0, c, TWL_LED_PWMOFF);
                twl_i2c_write_u8(TWL4030_MODULE_PWM0, 0x1, TWL_LED_PWMON);
        }
}

static struct generic_bl_info omap3autoget_bl_platform_data = {
        .name                   = "omap-backlight",
        .max_intensity          = 100,
        .default_intensity      = 70,
        .limit_mask             = 0,
        .set_bl_intensity       = omap3autoget_set_bl_intensity,
        .kick_battery           = NULL,
};

static struct platform_device omap3autoget_bklight_device = {
        .name           = "generic-bl",
        .id             = -1,
        .dev            = {
                .platform_data  = &omap3autoget_bl_platform_data,
        },
};

#if 0
static int sbc8530_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(DVI_UP_GPIO, 1);

	dvi_enabled = 1;
	return 0;
}

static void sbc8530_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(DVI_UP_GPIO, 0);

	dvi_enabled = 0;
}

static struct omap_dss_device sbc8530_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_panel",
	.phy.dpi.data_lines = 24,
	.platform_enable = sbc8530_enable_dvi,
	.platform_disable = sbc8530_disable_dvi,
};

static int sbc8530_panel_enable_tv(struct omap_dss_device *dssdev)
{
#define ENABLE_VDAC_DEDICATED           0x03
#define ENABLE_VDAC_DEV_GRP             0x20

	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,
			TWL4030_VDAC_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);

	return 0;
}

static void sbc8530_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}

static struct omap_dss_device sbc8530_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable = sbc8530_panel_enable_tv,
	.platform_disable = sbc8530_panel_disable_tv,
};
#endif

static struct omap_dss_device *autoget_dss_devices[] = {
	&autoget_lcd_device,
#if 0
	&sbc8530_dvi_device,
	&sbc8530_tv_device,
#endif
};

static struct omap_dss_board_info autoget_dss_data = {
	.num_devices = ARRAY_SIZE(autoget_dss_devices),
	.devices = autoget_dss_devices,
	.default_device = &autoget_lcd_device,
};

static struct platform_device autoget_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &autoget_dss_data,
	},
};

static struct regulator_consumer_supply autoget_vdac_supply = {
	.supply		= "vdda_dac",
	.dev		= &autoget_dss_device.dev,
};

static struct regulator_consumer_supply autoget_vdvi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &autoget_dss_device.dev,
};


#include "sdram-micron-mt46h32m32lf-6.h"

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 8,
		.gpio_wp	= -1,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply autoget_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply autoget_vmmc2_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply autoget_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct regulator_consumer_supply autoget_vaux2_supplies = {
	.supply		= "hsusb1",
};

static struct gpio_led gpio_leds[];

static int autoget_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	autoget_vmmc1_supply.dev = mmc[0].dev;
        autoget_vmmc2_supply.dev = mmc[1].dev;
	autoget_vsim_supply.dev = mmc[0].dev;

        /* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */

	gpio_leds[0].gpio = gpio + TWL4030_GPIO_MAX;
	gpio_leds[1].gpio = gpio + TWL4030_GPIO_MAX + 1;

	return 0;
}

static struct twl4030_gpio_platform_data autoget_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= autoget_twl_gpio_setup,
};

/* VAUX2 for USB */
static struct regulator_init_data autoget_vaux2 = {
	.constraints = {
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= 1,
	.consumer_supplies		= &autoget_vaux2_supplies,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data autoget_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &autoget_vmmc1_supply,
};

/* VMMC2 for MMC2 pins CMD, CLK, DAT0..DAT3 (max 100 mA) */
static struct regulator_init_data autoget_vmmc2 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &autoget_vmmc2_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data autoget_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &autoget_vsim_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data autoget_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &autoget_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data autoget_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &autoget_vdvi_supply,
};

static struct twl4030_usb_data autoget_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_codec_audio_data autoget_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data autoget_codec_data = {
	.audio_mclk = 26000000,
	.audio = &autoget_audio_data,
};

static struct twl4030_madc_platform_data autoget_madc_data = {
	.irq_line	= 1,
};

#if 1	/* Jasper 20170601 */
#ifdef CONFIG_PM
/*
 * Save the state of keypad
 *
 * TODO: This definition should ideally be in a header file, but
 *       matrix_keypad.h is not the right one. Also, plat/keypad.h
 *       is no longer used.
 */
struct omap_keypad_pm_state {
      void __iomem *wk_st;
      void __iomem *wk_en;
      u32 wk_mask;
      u32 padconf;
};
/*
 * Board specific hook for keypad suspend
 */
void omap3_evm_kp_suspend(void *ptr)
{
      struct omap_keypad_pm_state *pstate = (struct omap_keypad_pm_state *)ptr;

      if (pstate) {
            /*
             * Set wake-enable bit
             */
            if (pstate->wk_en && pstate->wk_mask) {
                  u32 v = __raw_readl(pstate->wk_en);
                  v |= pstate->wk_mask;
                  __raw_writel(v, pstate->wk_en);
            }
            /*
             * Set corresponding IOPAD wakeup-enable
             */
            if (cpu_is_omap34xx() && pstate->padconf) {
                  u16 v = omap_ctrl_readw(pstate->padconf);
                  v |= OMAP3_PADCONF_WAKEUPENABLE0;
                  omap_ctrl_writew(v, pstate->padconf);
            }
      }
}

/*
 * Board specific hook for keypad resume
 */
void omap3_evm_kp_resume(void *ptr)
{
      struct omap_keypad_pm_state *pstate = (struct omap_keypad_pm_state *)ptr;

      if (pstate) {
            /*
             * Clear wake-enable bit
             */
            if (pstate->wk_en && pstate->wk_mask) {
                  u32 v = __raw_readl(pstate->wk_en);
                  v &= ~pstate->wk_mask;
                  __raw_writel(v, pstate->wk_en);
            }
            /*
             * Clear corresponding IOPAD wakeup-enable
             */
            if (cpu_is_omap34xx() && pstate->padconf) {
                  u16 v = omap_ctrl_readw(pstate->padconf);
                  v &= ~OMAP3_PADCONF_WAKEUPENABLE0;
                  omap_ctrl_writew(v, pstate->padconf);
            }
      }
}

static struct omap_keypad_pm_state omap3evm_kp_pm_state = {
      .wk_st            = OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKST1),
      .wk_en            = OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKEN1),
      .wk_mask    = OMAP3430_EN_GPIO1,
      .padconf    = 0x1e0,
};

static struct omap_opp * _omap35x_mpu_rate_table      = omap35x_mpu_rate_table;
static struct omap_opp * _omap37x_mpu_rate_table      = omap37x_mpu_rate_table;
static struct omap_opp * _omap35x_dsp_rate_table      = omap35x_dsp_rate_table;
static struct omap_opp * _omap37x_dsp_rate_table      = omap37x_dsp_rate_table;
static struct omap_opp * _omap35x_l3_rate_table       = omap35x_l3_rate_table;
static struct omap_opp * _omap37x_l3_rate_table       = omap37x_l3_rate_table;
#else /* CONFIG_PM */
static struct omap_opp * _omap35x_mpu_rate_table      = NULL;
static struct omap_opp * _omap37x_mpu_rate_table      = NULL;
static struct omap_opp * _omap35x_dsp_rate_table      = NULL;
static struct omap_opp * _omap37x_dsp_rate_table      = NULL;
static struct omap_opp * _omap35x_l3_rate_table       = NULL;
static struct omap_opp * _omap37x_l3_rate_table       = NULL;
#endif      /* CONFIG_PM */



static int board_keymap[] = {
      KEY(0, 0, KEY_A),
      KEY(0, 1, KEY_B),
      KEY(0, 2, KEY_C),
      KEY(0, 3, KEY_D),
      KEY(0, 4, KEY_E),
      KEY(0, 5, KEY_F),

      KEY(1, 0, KEY_G),
      KEY(1, 1, KEY_H),
      KEY(1, 2, KEY_I),
      KEY(1, 3, KEY_J),
      KEY(1, 4, KEY_K),
      KEY(1, 5, KEY_L),

      KEY(2, 0, KEY_M),
      KEY(2, 1, KEY_N),
      KEY(2, 2, KEY_O),
      KEY(2, 3, KEY_P),
      KEY(2, 4, KEY_Q),
      KEY(2, 5, KEY_R),

      KEY(3, 0, KEY_S),
      KEY(3, 1, KEY_U),
      KEY(3, 2, KEY_V),
      KEY(3, 3, KEY_W),
      KEY(3, 4, KEY_X),
      KEY(3, 5, KEY_Y),

      KEY(4, 0, KEY_Z),
      KEY(4, 1, KEY_1),
      KEY(4, 2, KEY_2),
      KEY(4, 3, KEY_3),
      KEY(4, 4, KEY_4),
      KEY(4, 5, KEY_5),

      KEY(5, 0, KEY_6),
      KEY(5, 1, KEY_7),
      KEY(5, 2, KEY_8),
      KEY(5, 3, KEY_9),
      KEY(5, 4, KEY_0),
      KEY(5, 5, KEY_MINUS)
};

static struct matrix_keymap_data board_map_data = {
      .keymap		= board_keymap,
      .keymap_size	= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data sbc8530_kp_data = {
      .keymap_data      = &board_map_data,
      .rows       	= 6,
      .cols       	= 6,
      .rep        	= 1,
#ifdef CONFIG_PM
      .pm_state   	= (void *)&omap3evm_kp_pm_state,
      .on_suspend 	= omap3_evm_kp_suspend,
      .on_resume  	= omap3_evm_kp_resume,
#endif
};
#endif	/* Jasper 20170601 */

static struct twl4030_platform_data autoget_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	//.keypad		= &sbc8530_kp_data,
	.usb		= &autoget_usb_data,
	.gpio		= &autoget_gpio_data,
	//.codec		= &autoget_codec_data,
	.madc		= &autoget_madc_data,
	.vmmc1		= &autoget_vmmc1,
	.vmmc2		= &autoget_vmmc2,
	.vsim		= &autoget_vsim,
	.vdac		= &autoget_vdac,
	.vpll2		= &autoget_vpll2,
	.vaux2		= &autoget_vaux2,
};

static struct i2c_board_info __initdata autoget_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &autoget_twldata,
	},
};

#include <media/v4l2-int-device.h>

extern void autoget_cam_init(void);

#define TMD2772X_I2C_ADDR	0x39
#define BQ27541_I2C_ADDR		0x55

static struct i2c_board_info __initdata autoget_i2c2_boardinfo[] = {
	/* light sensor: tmd2772x */
	{
		I2C_BOARD_INFO("tmd2772x", TMD2772X_I2C_ADDR),
		//.platform_data = &sbc8530_ov2656_platform_data,
       },
       /* battary: bq27541 */
       {
		I2C_BOARD_INFO("bq27541", BQ27541_I2C_ADDR),
		//.platform_data = &sbc8530_ov2656_platform_data,
       },
};

static int __init omap3_autoget_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, autoget_i2c1_boardinfo,
			ARRAY_SIZE(autoget_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, autoget_i2c2_boardinfo,
                                ARRAY_SIZE(autoget_i2c2_boardinfo));
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

#define GPIO_LEDR	139
#define GPIO_LEDG	138
#define GPIO_LEDB	137

static struct gpio_led gpio_leds[] = {
	{
		.name                 = "sys_led",
		.default_trigger        = "heartbeat",
		.gpio                   = -EINVAL,
		.active_low		= true,
	},
	{
		.name                = "user_led",
		.gpio                  = -EINVAL,
		.active_low	= true,
	},
	{
		.name                = "red_led",
		.gpio                  = GPIO_LEDR,
		.active_low	= true,
	},
	{
		.name		= "green_led",
		.gpio			= GPIO_LEDG,
		.active_low	= true,
	},
	{
		.name		= "blue_led",
		.gpio			= GPIO_LEDB,
		.active_low	= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
        {
                .code                   = BTN_FORWARD,
                .gpio                   = 73,
                .desc                   = "forward",
                .active_low             = true,
        },
        {
                .code                   = BTN_BACK,
                .gpio                   = 74,
                .desc                   = "back",
                .active_low             = true,
        },
        {
                .code                   = BTN_TASK,
                .gpio                   = 71,
                .desc                   = "measure",
                .active_low             = true,
        },
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

#define OMAP3_AUTOGET_TS_GPIO       19

static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_AUTOGET_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(OMAP3_AUTOGET_TS_GPIO);

	omap_set_gpio_debounce(OMAP3_AUTOGET_TS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP3_AUTOGET_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_AUTOGET_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
//	.x_plate_ohms		= 180,
//	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 5,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 150,
	.wakeup			= true,
        .swap_xy                = 1,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

#include <media/v4l2-int-device.h>
#if defined(CONFIG_VIDEO_EV76C570) || defined(CONFIG_VIDEO_EV76C570_MODULE)
#include <media/ev76c570.h>
extern struct ev76c570_platform_data autoget_ev76c570_platform_data;
#endif

struct spi_board_info omap3autoget_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_AUTOGET_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
#if 0	/* Mask it for temporary */
	[1] = {
		.modalias		= "lg4573_panel-spi",
		.bus_num		= 1,
		.chip_select		= 1,
		.max_speed_hz	= 500000,
		.mode			= SPI_MODE_3,
	},
#endif
#if defined(CONFIG_VIDEO_EV76C570) || defined(CONFIG_VIDEO_EV76C570_MODULE)
	[2] = {
		.modalias		= "ev76c570",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz	= 800000,
		.mode			= SPI_MODE_0,
		.platform_data 		= &autoget_ev76c570_platform_data,
	},
#endif
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	[3] = {
		.modalias = "spidev",
		.max_speed_hz = 500000,     /* max spi clock (SCK) speed in HZ */
		.bus_num = 3,
		.chip_select = 0,
	},
#endif

};

static void __init omap3_autoget_init_irq(void)
{
        if (cpu_is_omap3630())
        {
                omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
                                        NULL,
                                        _omap37x_mpu_rate_table,
                                        _omap37x_dsp_rate_table,
                                        _omap37x_l3_rate_table);
        }
        else
        {
                omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
                                        NULL,
                                        _omap35x_mpu_rate_table,
                                        _omap35x_dsp_rate_table,
                                        _omap35x_l3_rate_table);
        }
	omap_init_irq();
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(12);
#endif
	omap_gpio_init();
	ads7846_dev_init();
}

static struct platform_device *omap3_autoget_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
	&autoget_dss_device,
	&omap3autoget_bklight_device,
};

static void __init omap3autoget_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		omap3autoget_nand_data.cs = nandcs;
		omap3autoget_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		omap3autoget_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&omap3autoget_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,	
	.reset_gpio_port[1]  = 17,
	.reset_gpio_port[2]  = -EINVAL
};

#define USB_HUB_RESET	13
static void __init usb_hub_init(void)
{
	if (gpio_request(USB_HUB_RESET, "HUB_RESET") < 0)
		printk(KERN_ERR "can't get HUB_RESET GPIO\n");

	msleep(200);	/* JasperZhang: Must be later, later, and later!! */

	gpio_direction_output(USB_HUB_RESET, 1);      
	udelay(10);
	gpio_direction_output(USB_HUB_RESET, 0);	
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static void __init omap3_autoget_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	omap3_autoget_i2c_init();
	platform_add_devices(omap3_autoget_devices,
			ARRAY_SIZE(omap3_autoget_devices));

        spi_register_board_info(omap3autoget_spi_board_info,
                                ARRAY_SIZE(omap3autoget_spi_board_info));

	omap_serial_init();

	usb_musb_init();
	usb_ehci_init(&ehci_pdata);
	omap3autoget_flash_init();

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	omap3_autoget_display_init();
	//omap3_sbc8530_init_dm9000();
	autoget_cam_init();
}
static void __init omap3_autoget_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3_AUTOGET, "OMAP3 AutoGet Board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_autoget_map_io,
	.init_irq	= omap3_autoget_init_irq,
	.init_machine	= omap3_autoget_init,
	.timer		= &omap_timer,
MACHINE_END

late_initcall(usb_hub_init);
