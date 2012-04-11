/*
 * Copyright 2007 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */

#include <linux/io.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <drm/drmP.h>
#include <drm/drms_encon.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/imx-uart.h>
#include <mach/mxc_nand.h>
#include <mach/spi.h>
#include <mach/imxfb.h>
#include <mach/iomux-mx27.h>
#include <mach/ulpi.h>

#include "devices-imx27.h"

static int vht_pins[] = {
	/* UART1 */
	PE12_PF_UART1_TXD,
	PE13_PF_UART1_RXD,
	PE14_PF_UART1_CTS,
	PE15_PF_UART1_RTS,
	/* FEC */
	PD0_AIN_FEC_TXD0,
	PD1_AIN_FEC_TXD1,
	PD2_AIN_FEC_TXD2,
	PD3_AIN_FEC_TXD3,
	PD4_AOUT_FEC_RX_ER,
	PD5_AOUT_FEC_RXD1,
	PD6_AOUT_FEC_RXD2,
	PD7_AOUT_FEC_RXD3,
	PD8_AF_FEC_MDIO,
	PD9_AIN_FEC_MDC,
	PD10_AOUT_FEC_CRS,
	PD11_AOUT_FEC_TX_CLK,
	PD12_AOUT_FEC_RXD0,
	PD13_AOUT_FEC_RX_DV,
	PD14_AOUT_FEC_RX_CLK,
	PD15_AOUT_FEC_COL,
	PD16_AIN_FEC_TX_ER,
	PF23_AIN_FEC_TX_EN,
	/* display */
	PA5_PF_LSCLK,
	PA6_PF_LD0,
	PA7_PF_LD1,
	PA8_PF_LD2,
	PA9_PF_LD3,
	PA10_PF_LD4,
	PA11_PF_LD5,
	PA12_PF_LD6,
	PA13_PF_LD7,
	PA14_PF_LD8,
	PA15_PF_LD9,
	PA16_PF_LD10,
	PA17_PF_LD11,
	PA18_PF_LD12,
	PA19_PF_LD13,
	PA20_PF_LD14,
	PA21_PF_LD15,
	PA22_PF_LD16,
	PA23_PF_LD17,
	PA24_PF_REV,
	PA25_PF_CLS,
	PA26_PF_PS,
	PA27_PF_SPL_SPR,
	PA28_PF_HSYNC,
	PA29_PF_VSYNC,
	PA30_PF_CONTRAST,
	PA31_PF_OE_ACD,
	PC7_PF_USBOTG_DATA5,
	PC8_PF_USBOTG_DATA6,
	PC9_PF_USBOTG_DATA0,
	PC10_PF_USBOTG_DATA2,
	PC11_PF_USBOTG_DATA1,
	PC12_PF_USBOTG_DATA4,
	PC13_PF_USBOTG_DATA3,
	PE0_PF_USBOTG_NXT,
	PE1_PF_USBOTG_STP,
	PE2_PF_USBOTG_DIR,
	PE24_PF_USBOTG_CLK,
	PE25_PF_USBOTG_DATA7,
	/* USBH2 */
	PA0_PF_USBH2_CLK,
	PA1_PF_USBH2_DIR,
	PA2_PF_USBH2_DATA7,
	PA3_PF_USBH2_NXT,
	PA4_PF_USBH2_STP,
	PD19_AF_USBH2_DATA4,
	PD20_AF_USBH2_DATA3,
	PD21_AF_USBH2_DATA6,
	PD22_AF_USBH2_DATA0,
	PD23_AF_USBH2_DATA2,
	PD24_AF_USBH2_DATA1,
	PD26_AF_USBH2_DATA5,
};

static struct imxuart_platform_data uart_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

#include <linux/workqueue.h>
#include <linux/clk.h>

static void __iomem *vht_wd_base;

struct delayed_work vht_work;

static void vht_wd_work(struct work_struct *work)
{
	writew(0x5555, vht_wd_base + 0x2);
	writew(0xaaaa, vht_wd_base + 0x2);

	schedule_delayed_work(&vht_work, 200);
}

static void vht_wd_init(void)
{
	struct clk *clk;

	clk = clk_get_sys("imx2-wdt.0", NULL);
	clk_enable(clk);

	vht_wd_base = ioremap(0x10002000, 0x1000);

	INIT_DELAYED_WORK(&vht_work, vht_wd_work);

	schedule_delayed_work(&vht_work, 200);
}

static void __init vht_init(void)
{
	imx27_soc_init();

	mxc_gpio_setup_multiple_pins(vht_pins, ARRAY_SIZE(vht_pins),
			"PCM038");

	vht_wd_init();

	imx27_add_fec(NULL);
	imx27_add_imx_uart0(&uart_pdata);

	mxc_gpio_mode(GPIO_PORTF | 20 | GPIO_GPIO | GPIO_OUT);
	gpio_request(GPIO_PORTF | 20, "BL");
	gpio_direction_output(GPIO_PORTF | 20, 1);
}

static void __init vht_timer_init(void)
{
	mx27_clocks_init(26000000);
}

static struct sys_timer vht_timer = {
	.init = vht_timer_init,
};

MACHINE_START(CPS_EUROPE_VHT, "CPS Europe VHT")
	.atag_offset = 0x100,
	.map_io = mx27_map_io,
	.init_early = imx27_init_early,
	.init_irq = mx27_init_irq,
	.handle_irq = imx27_handle_irq,
	.init_machine = vht_init,
	.timer = &vht_timer,
MACHINE_END
#if 1

static struct drm_display_mode vht_mode = {
	.name = "some display",
	.vrefresh = 60,
	.clock = 33334,
	.hdisplay = 800,
	.hsync_start = 800 + 40,
	.hsync_end = 800 + 40 + 32,
	.htotal = 800 + 40 + 32 + 88,
	.vdisplay = 600,
	.vsync_start = 600 + 1,
	.vsync_end = 600 + 1 + 4,
	.vtotal = 600 + 1 + 4 + 23,
	.type = 0x0,
	.flags = 0x0,
};

static struct drms_encon_dummy_pdata pcm038_encon_data = {
	.drm_name = "imx-lcdc-crtc.0",
	.possible_crtcs = 0x1,
	.possible_clones = 0x1,
	.modes = &vht_mode,
	.num_modes = 1,
	.gpio_backlight = 133,
	.flags = DRM_ENCON_DUMMY_USE_BL_GPIO,
};

static struct platform_device *__init imx_add_imx_drm(
		struct imx_drm_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = MX27_LCDC_BASE_ADDR,
			.end = MX27_LCDC_BASE_ADDR + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = MX27_INT_LCDC,
			.end = MX27_INT_LCDC,
			.flags = IORESOURCE_IRQ,
		},
	};
	return imx_add_platform_device_dmamask("imx-lcdc-crtc", 0,
			res, ARRAY_SIZE(res),
			pdata, sizeof(*pdata), DMA_BIT_MASK(32));
}

static struct imx_drm_platform_data drm_pdata = {
	/*
	 * - HSYNC active high
	 * - VSYNC active high
	 * - clk notenabled while idle
	 * - clock not inverted
	 * - data not inverted
	 * - data enable low active
	 * - enable sharp mode
	 */
	.pcr		= 0xf0c08080,
};

static int add_drm(void)
{
	if (!machine_is_vht())
		return 0;

	platform_device_register_data(NULL, "drm-encon-dummy", 0,
			&pcm038_encon_data, sizeof(pcm038_encon_data));
	imx_add_imx_drm(&drm_pdata);
	return 0;
}
device_initcall(add_drm);
#endif
