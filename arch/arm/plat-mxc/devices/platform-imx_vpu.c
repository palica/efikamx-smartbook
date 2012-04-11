/*
 * Copyright (C) 2010 Pengutronix
 * Uwe Kleine-Koenig <u.kleine-koenig@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation.
 */
#include <mach/hardware.h>
#include <mach/devices-common.h>

#ifdef CONFIG_SOC_IMX27
struct platform_device *__init mx27_add_mxc_vpu(void)
{
	struct resource res[] = {
		{
			.start = MX27_VPU_BASE_ADDR,
			.end = MX27_VPU_BASE_ADDR + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = MX27_INT_VPU,
			.end = MX27_INT_VPU,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("imx27-vpu", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}
#endif /* CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX51
struct platform_device *__init mx51_add_mxc_vpu(void)
{
	struct resource res[] = {
		{
			.start = MX51_VPU_BASE_ADDR,
			.end = MX51_VPU_BASE_ADDR + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = MX51_INT_VPU,
			.end = MX51_INT_VPU,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("imx51-vpu", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}
#endif /* CONFIG_SOC_IMX51 */

#ifdef CONFIG_SOC_IMX53
struct platform_device *__init mx53_add_mxc_vpu(void)
{
	struct resource res[] = {
		{
			.start = MX53_VPU_BASE_ADDR,
			.end = MX53_VPU_BASE_ADDR + SZ_4K - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = MX53_INT_VPU,
			.end = MX53_INT_VPU,
			.flags = IORESOURCE_IRQ,
		},
	};

	return imx_add_platform_device("imx53-vpu", 0,
			res, ARRAY_SIZE(res), NULL, 0);
}
#endif /* CONFIG_SOC_IMX53 */
