/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/init.h>
#include <linux/pwm_backlight.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <drm/imx-ipu-v3.h>
#include <drm/i2c/sii9022.h>
#include <linux/mfd/da9052/pdata.h>

#include <linux/regulator/machine.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/iomux-mx53.h>
#include <mach/ipu-v3.h>
#include <drm/drmP.h>
#include "drm/sdrm_encon.h"

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/imx-ldb.h>

#include "crm-regs-imx5.h"

#include "devices-imx53.h"

#define MX53_LOCO_POWER			IMX_GPIO_NR(1, 8)
#define MX53_LOCO_UI1			IMX_GPIO_NR(2, 14)
#define MX53_LOCO_UI2			IMX_GPIO_NR(2, 15)
#define LOCO_FEC_PHY_RST		IMX_GPIO_NR(7, 6)
#define LOCO_LED			IMX_GPIO_NR(7, 7)
#define LOCO_SD3_CD			IMX_GPIO_NR(3, 11)
#define LOCO_SD3_WP			IMX_GPIO_NR(3, 12)
#define LOCO_SD1_CD			IMX_GPIO_NR(3, 13)
#define LOCO_ACCEL_EN			IMX_GPIO_NR(6, 14)
#define MX53_LOCO_USB_PWREN		IMX_GPIO_NR(7, 8)
#define MX53_LOCO_LCD_BL		IMX_GPIO_NR(7, 2)
#define DA9052_INT			IMX_GPIO_NR(7, 11)

static iomux_v3_cfg_t mx53_loco_pads[] = {
	/* FEC */
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
	/* FEC_nRST */
	MX53_PAD_PATA_DA_0__GPIO7_6,
	/* FEC_nINT */
	MX53_PAD_PATA_DATA4__GPIO2_4,
	/* AUDMUX5 */
	MX53_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX53_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX53_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX53_PAD_KEY_ROW1__AUDMUX_AUD5_RXD,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	MX53_PAD_NANDF_CS1__GPIO6_14,	/* Accelerometer Enable */
	/* I2C2 */
	MX53_PAD_KEY_COL3__I2C2_SCL,
	MX53_PAD_KEY_ROW3__I2C2_SDA,
	/* SD1 */
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
	/* SD1_CD */
	MX53_PAD_EIM_DA13__GPIO3_13,
	/* SD3 */
	MX53_PAD_PATA_DATA8__ESDHC3_DAT0,
	MX53_PAD_PATA_DATA9__ESDHC3_DAT1,
	MX53_PAD_PATA_DATA10__ESDHC3_DAT2,
	MX53_PAD_PATA_DATA11__ESDHC3_DAT3,
	MX53_PAD_PATA_DATA0__ESDHC3_DAT4,
	MX53_PAD_PATA_DATA1__ESDHC3_DAT5,
	MX53_PAD_PATA_DATA2__ESDHC3_DAT6,
	MX53_PAD_PATA_DATA3__ESDHC3_DAT7,
	MX53_PAD_PATA_IORDY__ESDHC3_CLK,
	MX53_PAD_PATA_RESET_B__ESDHC3_CMD,
	/* SD3_CD */
	MX53_PAD_EIM_DA11__GPIO3_11,
	/* SD3_WP */
	MX53_PAD_EIM_DA12__GPIO3_12,
	/* VGA */
	MX53_PAD_EIM_OE__IPU_DI1_PIN7,
	MX53_PAD_EIM_RW__IPU_DI1_PIN8,
	/* DISPLB */
	MX53_PAD_EIM_D20__IPU_SER_DISP0_CS,
	MX53_PAD_EIM_D21__IPU_DISPB0_SER_CLK,
	MX53_PAD_EIM_D22__IPU_DISPB0_SER_DIN,
	MX53_PAD_EIM_D23__IPU_DI0_D0_CS,
	/* DISP0_POWER_EN */
	MX53_PAD_EIM_D24__GPIO3_24,
	/* DISP0 DET INT */
	MX53_PAD_EIM_D31__GPIO3_31,
	/* LVDS */
	MX53_PAD_LVDS0_TX3_P__LDB_LVDS0_TX3,
	MX53_PAD_LVDS0_CLK_P__LDB_LVDS0_CLK,
	MX53_PAD_LVDS0_TX2_P__LDB_LVDS0_TX2,
	MX53_PAD_LVDS0_TX1_P__LDB_LVDS0_TX1,
	MX53_PAD_LVDS0_TX0_P__LDB_LVDS0_TX0,
	MX53_PAD_LVDS1_TX3_P__LDB_LVDS1_TX3,
	MX53_PAD_LVDS1_TX2_P__LDB_LVDS1_TX2,
	MX53_PAD_LVDS1_CLK_P__LDB_LVDS1_CLK,
	MX53_PAD_LVDS1_TX1_P__LDB_LVDS1_TX1,
	MX53_PAD_LVDS1_TX0_P__LDB_LVDS1_TX0,
	/* I2C1 */
	MX53_PAD_CSI0_DAT8__I2C1_SDA,
	MX53_PAD_CSI0_DAT9__I2C1_SCL,
	/* UART1 */
	MX53_PAD_CSI0_DAT10__UART1_TXD_MUX,
	MX53_PAD_CSI0_DAT11__UART1_RXD_MUX,
	/* CSI0 */
	MX53_PAD_CSI0_DAT12__IPU_CSI0_D_12,
	MX53_PAD_CSI0_DAT13__IPU_CSI0_D_13,
	MX53_PAD_CSI0_DAT14__IPU_CSI0_D_14,
	MX53_PAD_CSI0_DAT15__IPU_CSI0_D_15,
	MX53_PAD_CSI0_DAT16__IPU_CSI0_D_16,
	MX53_PAD_CSI0_DAT17__IPU_CSI0_D_17,
	MX53_PAD_CSI0_DAT18__IPU_CSI0_D_18,
	MX53_PAD_CSI0_DAT19__IPU_CSI0_D_19,
	MX53_PAD_CSI0_VSYNC__IPU_CSI0_VSYNC,
	MX53_PAD_CSI0_MCLK__IPU_CSI0_HSYNC,
	MX53_PAD_CSI0_PIXCLK__IPU_CSI0_PIXCLK,
	/* DISPLAY */
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	/* Audio CLK*/
	MX53_PAD_GPIO_0__CCM_SSI_EXT1_CLK,
	/* PWM */
	MX53_PAD_GPIO_1__PWM2_PWMO,
	/* SPDIF */
	MX53_PAD_GPIO_7__SPDIF_PLOCK,
	MX53_PAD_GPIO_17__SPDIF_OUT1,
	/* GPIO */
	MX53_PAD_PATA_DA_1__GPIO7_7,		/* LED */
	MX53_PAD_PATA_DA_2__GPIO7_8,
	MX53_PAD_PATA_DATA5__GPIO2_5,
	MX53_PAD_PATA_DATA6__GPIO2_6,
	MX53_PAD_PATA_DATA14__GPIO2_14,
	MX53_PAD_PATA_DATA15__GPIO2_15,
	MX53_PAD_PATA_INTRQ__GPIO7_2,
	MX53_PAD_EIM_WAIT__GPIO5_0,
	MX53_PAD_NANDF_WP_B__GPIO6_9,
	MX53_PAD_NANDF_RB0__GPIO6_10,
	MX53_PAD_NANDF_CS1__GPIO6_14,
	MX53_PAD_NANDF_CS2__GPIO6_15,
	MX53_PAD_NANDF_CS3__GPIO6_16,
	MX53_PAD_GPIO_5__GPIO1_5,
	MX53_PAD_GPIO_16__GPIO7_11,
	MX53_PAD_GPIO_8__GPIO1_8,
};

#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}

static struct gpio_keys_button loco_buttons[] = {
	GPIO_BUTTON(MX53_LOCO_POWER, KEY_POWER, 1, "power", 0),
	GPIO_BUTTON(MX53_LOCO_UI1, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(MX53_LOCO_UI2, KEY_VOLUMEDOWN, 1, "volume-down", 0),
};

static const struct gpio_keys_platform_data loco_button_data __initconst = {
	.buttons        = loco_buttons,
	.nbuttons       = ARRAY_SIZE(loco_buttons),
};

static const struct esdhc_platform_data mx53_loco_sd1_data __initconst = {
	.cd_gpio = LOCO_SD1_CD,
	.cd_type = ESDHC_CD_GPIO,
	.wp_type = ESDHC_WP_NONE,
};

static const struct esdhc_platform_data mx53_loco_sd3_data __initconst = {
	.cd_gpio = LOCO_SD3_CD,
	.wp_gpio = LOCO_SD3_WP,
	.cd_type = ESDHC_CD_GPIO,
	.wp_type = ESDHC_WP_GPIO,
};

static inline void mx53_loco_fec_reset(void)
{
	int ret;

	/* reset FEC PHY */
	ret = gpio_request(LOCO_FEC_PHY_RST, "fec-phy-reset");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO_FEC_PHY_RESET: %d\n", ret);
		return;
	}
	gpio_direction_output(LOCO_FEC_PHY_RST, 0);
	msleep(1);
	gpio_set_value(LOCO_FEC_PHY_RST, 1);
}

static const struct fec_platform_data mx53_loco_fec_data __initconst = {
	.phy = PHY_INTERFACE_MODE_RMII,
};

static const struct imxi2c_platform_data mx53_loco_i2c_data __initconst = {
	.bitrate = 100000,
};

static const struct gpio_led mx53loco_leds[] __initconst = {
	{
		.name			= "green",
		.default_trigger	= "heartbeat",
		.gpio			= LOCO_LED,
	},
};

#define DA9052_VIR_INT_BASE	MX53_INT_GPIO7_HIGH + 1

static struct da9052_pdata da9052_pdata = {
		.irq_base = DA9052_VIR_INT_BASE,
};

static struct i2c_board_info mx53loco_i2c_devices[] = {
	{
		I2C_BOARD_INFO("mma8450", 0x1C),
	}, {
		I2C_BOARD_INFO("da9052", 0x48),
		.platform_data = &da9052_pdata,
	},
};

static const struct gpio_led_platform_data mx53loco_leds_data __initconst = {
	.leds		= mx53loco_leds,
	.num_leds	= ARRAY_SIZE(mx53loco_leds),
};

static struct imx_ipuv3_platform_data ipu_data = {
	.di[0] = {
		.clk_ext = 0,
	},
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 1,
	.max_brightness = 255,
	.dft_brightness = 255,
	.pwm_period_ns = 50000,
};

#define DISP0_POWER_EN		IMX_GPIO_NR(3, 24)
#define DISP0_DET_INT		IMX_GPIO_NR(3, 31)
#define DISP0_RESET		IMX_GPIO_NR(5, 0)

#define DA9052_INT		IMX_GPIO_NR(7, 11)	//(6*32 + 11)	/* GPIO7_11 */
#define DA9052_VIR_INT_BASE	MX53_INT_GPIO7_HIGH + 1


static struct sii9022_platform_data sii9022_pdata = {
	.drm_name = "imx-drm.0",
	.encoder_id = 0,
};

static struct i2c_board_info loco_i2c1_info[] __initdata = {
	{
		.type = "sii9022",
		.addr = 0x39,
		.platform_data = &sii9022_pdata,
	},
};

static int mx53_loco_hdmi_adapter = 1;

void __init imx53_qsb_common_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(mx53_loco_pads,
					 ARRAY_SIZE(mx53_loco_pads));
}

static void __init mx53_loco_board_init(void)
{
	int ret;
	imx53_soc_init();
	imx53_qsb_common_init();

	/* Sii9022 HDMI controller */
	gpio_request(DISP0_RESET, "disp0-reset");
	gpio_direction_output(DISP0_RESET, 0);
	msleep(10);
	gpio_set_value(DISP0_RESET, 1);

	gpio_request(DISP0_DET_INT, "disp0-detect");
	gpio_direction_input(DISP0_DET_INT);
	gpio_free(DISP0_DET_INT);

	/* LCD panel power enable */
	gpio_request(DISP0_POWER_EN, "disp0-power-en");
	gpio_direction_output(DISP0_POWER_EN, 1);

	imx53_add_imx_uart(0, NULL);
	mx53_loco_fec_reset();
	imx53_add_fec(&mx53_loco_fec_data);
	imx53_add_imx2_wdt(0, NULL);

	ret = gpio_request_one(LOCO_ACCEL_EN, GPIOF_OUT_INIT_HIGH, "accel_en");
	if (ret)
		pr_err("Cannot request ACCEL_EN pin: %d\n", ret);

	mx53loco_i2c_devices[1].irq = gpio_to_irq(DA9052_INT);
	i2c_register_board_info(0, mx53loco_i2c_devices,
				ARRAY_SIZE(mx53loco_i2c_devices));
	imx53_add_imx_i2c(0, &mx53_loco_i2c_data);
	imx53_add_imx_i2c(1, &mx53_loco_i2c_data);
	imx53_add_sdhci_esdhc_imx(0, &mx53_loco_sd1_data);
	imx53_add_sdhci_esdhc_imx(2, &mx53_loco_sd3_data);

	loco_i2c1_info[0].irq  = gpio_to_irq(DISP0_DET_INT);
	i2c_register_board_info(1, loco_i2c1_info, ARRAY_SIZE(loco_i2c1_info));

	if (mx53_loco_hdmi_adapter) {
		i2c_register_board_info(1, loco_i2c1_info,
				ARRAY_SIZE(loco_i2c1_info));
	}

	imx_add_gpio_keys(&loco_button_data);
	gpio_led_register_device(-1, &mx53loco_leds_data);
	imx53_add_ahci_imx();
}

static void __init mx53_loco_timer_init(void)
{
	mx53_clocks_init(32768, 24000000, 0, 0);
}

static struct sys_timer mx53_loco_timer = {
	.init	= mx53_loco_timer_init,
};

static void loco_add_bl_pwm(void)
{
	printk("FIXME: LOCO PWM\n");
#if 0
	imx53_add_mxc_pwm(1);

	mxc_register_device(&mxc_pwm1_backlight_device,
			&mxc_pwm_backlight_data);
#endif
}

#define DISPLAY_HITACHI_800_480_NAME "hitachi-800x480"

static void (*loco_display_init)(void);

static struct drm_display_mode hitachi_800x480_mode[] = {
	{
		.name = DISPLAY_HITACHI_800_480_NAME,
		.vrefresh = 60,
		.clock = 33500,
		.hdisplay = 800,
		.hsync_start = 964,
		.hsync_end = 974,
		.htotal = 1063,
		.vdisplay = 480,
		.vsync_start = 490,
		.vsync_end = 500,
		.vtotal = 523,
		.type = 0x0,
		.flags = 0x0,
	},
	{}
};

static struct sdrm_encon_dummy_pdata hitachi_800x480_encon_data = {
	.drm_name = "imx-drm.0",
	.possible_crtcs = 0x1,
	.possible_clones = 0x1,
	.modes = hitachi_800x480_mode,
	.num_modes = ARRAY_SIZE(hitachi_800x480_mode),
//	.gpio_backlight = 133,
//	.flags = DRM_ENCON_DUMMY_USE_BL_GPIO,
};

static void mx53_loco_init_hitachi_800x480(void)
{
	platform_device_register_data(NULL, "drm-encon-dummy", 0,
                        &hitachi_800x480_encon_data,
			sizeof(hitachi_800x480_encon_data));

	ipu_data.di[0].clk_ext = 0;

	loco_add_bl_pwm();
}

#define DISPLAY_HITACHI_800_480_LVDS_NAME "hitachi-800x480-lvds"

static void (*loco_display_init)(void);

static struct drm_display_mode hitachi_800x480_lvds_mode[] = {
	{
		.name = DISPLAY_HITACHI_800_480_LVDS_NAME,
		.vrefresh = 60,
		.clock = 33333,
		.hdisplay = 800,
		.hsync_start = 800,
		.hsync_end = 1055,
		.htotal = 1055,
		.vdisplay = 480,
		.vsync_start = 480,
		.vsync_end = 525,
		.vtotal = 525,
		.type = 0x48,
		.flags = 0x9,
	},
	{}
};

static struct sdrm_encon_dummy_pdata hitachi_800x480_lvds_encon_data = {
	.drm_name = "imx-drm.0",
	.possible_crtcs = 0x1,
	.possible_clones = 0x1,
	.modes = hitachi_800x480_lvds_mode,
	.num_modes = ARRAY_SIZE(hitachi_800x480_lvds_mode),
//	.gpio_backlight = 133,
//	.flags = DRM_ENCON_DUMMY_USE_BL_GPIO,
};

#define LOCO_GPIO_DISP_BL	IMX_GPIO_NR(1, 1)

static void mx53_loco_init_hitachi_800x480_lvds(void)
{
	iomux_v3_cfg_t gpio1_1 = MX53_PAD_GPIO_1__GPIO1_1;

	mx53_clock_ldb_pll4(33333333 * 7);

	mx53_setup_ldb(LDB_DI0_VS_POL_ACT_LOW | LDB_BIT_MAP_CH0_JEIDA |
			LDB_CH0_MODE_EN_TO_DI0);

	ipu_data.di[0].clk_ext = 1;
	ipu_data.di[0].clk_sync = 1;

	platform_device_register_data(NULL, "drm-encon-dummy", 0,
                        &hitachi_800x480_lvds_encon_data,
			sizeof(hitachi_800x480_lvds_encon_data));

	mxc_iomux_v3_setup_pad(gpio1_1);

	gpio_request(LOCO_GPIO_DISP_BL, "disp0-bl");
	gpio_direction_output(LOCO_GPIO_DISP_BL, 0);
}

#define DISPLAY_CHIMEI_1280_800_NAME "chimei-1280x800"

static struct drm_display_mode chimei_1280x800_mode[] = {
	{
		.name = DISPLAY_CHIMEI_1280_800_NAME,
		.vrefresh = 60,
		.clock = 71000,
		.hdisplay = 1280,
		.hsync_start = 1280,
		.hsync_end = 1440,
		.htotal = 1440,
		.vdisplay = 800,
		.vsync_start = 800,
		.vsync_end = 823,
		.vtotal = 823,
		.type = 0x48,
		.flags = 0x9,
	},
	{}
};

static struct sdrm_encon_dummy_pdata chimei_1280x800_encon_data = {
	.drm_name = "imx-drm.0",
	.possible_crtcs = 0x1,
	.possible_clones = 0x1,
	.modes = chimei_1280x800_mode,
	.num_modes = ARRAY_SIZE(chimei_1280x800_mode),
//	.gpio_backlight = 133,
//	.flags = DRM_ENCON_DUMMY_USE_BL_GPIO,
};

static void mx53_loco_init_chimei_1280x800(void)
{
	/* LCD panel power enable */
	gpio_request(MX53_LOCO_LCD_BL, "disp0-bl");
	gpio_direction_output(MX53_LOCO_LCD_BL, 1);

	mx53_clock_ldb_pll4(71000000 * 7);

	mx53_setup_ldb(LDB_DI0_VS_POL_ACT_LOW | LDB_BIT_MAP_CH0_JEIDA |
			LDB_CH0_MODE_EN_TO_DI0);

	ipu_data.di[0].clk_ext = 1;
	ipu_data.di[0].clk_sync = 1;

	platform_device_register_data(NULL, "drm-encon-dummy", 0,
                        &chimei_1280x800_encon_data,
			sizeof(chimei_1280x800_encon_data));

	loco_add_bl_pwm();
}

static int __init mx53_loco_fb_init(void)
{
	if (!machine_is_mx53_loco())
		return 0;
#if 0
	if (loco_display_init) {
		loco_display_init();
	} else {
		pr_info("No display specified. Pass video= followed by one of the "
				"following display types: %s %s %s\n",
				DISPLAY_HITACHI_800_480_NAME,
				DISPLAY_HITACHI_800_480_LVDS_NAME,
				DISPLAY_CHIMEI_1280_800_NAME);
		return 0;
	}
#endif
	imx53_add_ipuv3(&ipu_data);

	return 0;
}
late_initcall(mx53_loco_fb_init);

static int __init mx53_loco_display(char *options)
{
	if (!strcmp(options, DISPLAY_HITACHI_800_480_NAME)) {
		loco_display_init = mx53_loco_init_hitachi_800x480;
		mx53_loco_hdmi_adapter = 0;
		return 0;
	}

	if (!strcmp(options, DISPLAY_HITACHI_800_480_LVDS_NAME)) {
		loco_display_init = mx53_loco_init_hitachi_800x480_lvds;
		mx53_loco_hdmi_adapter = 0;
		return 0;
	}

	if (!strcmp(options, DISPLAY_CHIMEI_1280_800_NAME)) {
		loco_display_init = mx53_loco_init_chimei_1280x800;
		mx53_loco_hdmi_adapter = 0;
		return 0;
	}

	pr_err("unknown display option %s\n", options);

	return 0;
}
__setup("video=", mx53_loco_display);

static void __init mx53_loco_map_io(void)
{
	mx53_map_io();
	init_consistent_dma_size(SZ_8M + SZ_4M);
}

MACHINE_START(MX53_LOCO, "Freescale MX53 LOCO Board")
	.map_io = mx53_loco_map_io,
	.init_early = imx53_init_early,
	.init_irq = mx53_init_irq,
	.handle_irq = imx53_handle_irq,
	.timer = &mx53_loco_timer,
	.init_machine = mx53_loco_board_init,
	.restart	= mxc_restart,
MACHINE_END
