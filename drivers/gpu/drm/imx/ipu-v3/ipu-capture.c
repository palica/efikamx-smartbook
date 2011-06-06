/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#define DEBUG
#include <linux/export.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <mach/ipu-v3.h>
#include <drm/imx-ipu-v3.h>

#include "ipu-prv.h"

static u32 *ipu_csi_reg[2];
static u32 *ipu_smfc_reg;
extern spinlock_t ipu_lock;

static struct device *ipu_dev;

/*SMFC Registers */
#define SMFC_MAP	(ipu_smfc_reg)
#define SMFC_WMC	(ipu_smfc_reg + 0x0004/4)
#define SMFC_BS		(ipu_smfc_reg + 0x0008/4)

/* CMOS Sensor Interface Registers */
#define CSI_SENS_CONF(csi)	(ipu_csi_reg[csi])
#define CSI_SENS_FRM_SIZE(csi)	(ipu_csi_reg[csi] + 0x0004/4)
#define CSI_ACT_FRM_SIZE(csi)	(ipu_csi_reg[csi] + 0x0008/4)
#define CSI_OUT_FRM_CTRL(csi)	(ipu_csi_reg[csi] + 0x000C/4)
#define CSI_TST_CTRL(csi)	(ipu_csi_reg[csi] + 0x0010/4)
#define CSI_CCIR_CODE_1(csi)	(ipu_csi_reg[csi] + 0x0014/4)
#define CSI_CCIR_CODE_2(csi)	(ipu_csi_reg[csi] + 0x0018/4)
#define CSI_CCIR_CODE_3(csi)	(ipu_csi_reg[csi] + 0x001C/4)
#define CSI_MIPI_DI(csi)	(ipu_csi_reg[csi] + 0x0020/4)
#define CSI_SKIP(csi)		(ipu_csi_reg[csi] + 0x0024/4)
#define CSI_CPD_CTRL(csi)	(ipu_csi_reg[csi] + 0x0028/4)
#define CSI_CPD_RC(csi, n)	(ipu_csi_reg[csi] + 0x002C/4 + n)
#define CSI_CPD_RS(csi, n)	(ipu_csi_reg[csi] + 0x004C/4 + n)
#define CSI_CPD_GRC(csi, n)	(ipu_csi_reg[csi] + 0x005C/4 + n)
#define CSI_CPD_GRS(csi, n)	(ipu_csi_reg[csi] + 0x007C/4 + n)
#define CSI_CPD_GBC(csi, n)	(ipu_csi_reg[csi] + 0x008C/4 + n)
#define CSI_CPD_GBS(csi, n)	(ipu_csi_reg[csi] + 0x00AC/4 + n)
#define CSI_CPD_BC(csi, n)	(ipu_csi_reg[csi] + 0x00BC/4 + n)
#define CSI_CPD_BS(csi, n)	(ipu_csi_reg[csi] + 0x00DC/4 + n)
#define CSI_CPD_OFFSET1(csi)	(ipu_csi_reg[csi] + 0x00EC/4)
#define CSI_CPD_OFFSET2(csi)	(ipu_csi_reg[csi] + 0x00F0/4)

enum {
	CSI_SENS_CONF_DATA_FMT_SHIFT = 8,
	CSI_SENS_CONF_DATA_FMT_MASK = 0x00000700,
	CSI_SENS_CONF_DATA_FMT_RGB_YUV444 = 0L,
	CSI_SENS_CONF_DATA_FMT_YUV422_YUYV = 1L,
	CSI_SENS_CONF_DATA_FMT_YUV422_UYVY = 2L,
	CSI_SENS_CONF_DATA_FMT_BAYER = 3L,
	CSI_SENS_CONF_DATA_FMT_RGB565 = 4L,
	CSI_SENS_CONF_DATA_FMT_RGB555 = 5L,
	CSI_SENS_CONF_DATA_FMT_RGB444 = 6L,
	CSI_SENS_CONF_DATA_FMT_JPEG = 7L,

	CSI_SENS_CONF_VSYNC_POL_SHIFT = 0,
	CSI_SENS_CONF_HSYNC_POL_SHIFT = 1,
	CSI_SENS_CONF_DATA_POL_SHIFT = 2,
	CSI_SENS_CONF_PIX_CLK_POL_SHIFT = 3,
	CSI_SENS_CONF_SENS_PRTCL_MASK = 0x00000070L,
	CSI_SENS_CONF_SENS_PRTCL_SHIFT = 4,
	CSI_SENS_CONF_PACK_TIGHT_SHIFT = 7,
	CSI_SENS_CONF_DATA_WIDTH_SHIFT = 11,
	CSI_SENS_CONF_EXT_VSYNC_SHIFT = 15,
	CSI_SENS_CONF_DIVRATIO_SHIFT = 16,

	CSI_SENS_CONF_DIVRATIO_MASK = 0x00FF0000L,
	CSI_SENS_CONF_DATA_DEST_SHIFT = 24,
	CSI_SENS_CONF_DATA_DEST_MASK = 0x07000000L,
	CSI_SENS_CONF_JPEG8_EN_SHIFT = 27,
	CSI_SENS_CONF_JPEG_EN_SHIFT = 28,
	CSI_SENS_CONF_FORCE_EOF_SHIFT = 29,
	CSI_SENS_CONF_DATA_EN_POL_SHIFT = 31,

	CSI_DATA_DEST_ISP = 1L,
	CSI_DATA_DEST_IC = 2L,
	CSI_DATA_DEST_IDMAC = 4L,

	CSI_CCIR_ERR_DET_EN = 0x01000000L,
	CSI_HORI_DOWNSIZE_EN = 0x80000000L,
	CSI_VERT_DOWNSIZE_EN = 0x40000000L,
	CSI_TEST_GEN_MODE_EN = 0x01000000L,

	CSI_HSC_MASK = 0x1FFF0000,
	CSI_HSC_SHIFT = 16,
	CSI_VSC_MASK = 0x00000FFF,
	CSI_VSC_SHIFT = 0,

	CSI_TEST_GEN_R_MASK = 0x000000FFL,
	CSI_TEST_GEN_R_SHIFT = 0,
	CSI_TEST_GEN_G_MASK = 0x0000FF00L,
	CSI_TEST_GEN_G_SHIFT = 8,
	CSI_TEST_GEN_B_MASK = 0x00FF0000L,
	CSI_TEST_GEN_B_SHIFT = 16,

	CSI_MIPI_DI0_MASK = 0x000000FFL,
	CSI_MIPI_DI0_SHIFT = 0,
	CSI_MIPI_DI1_MASK = 0x0000FF00L,
	CSI_MIPI_DI1_SHIFT = 8,
	CSI_MIPI_DI2_MASK = 0x00FF0000L,
	CSI_MIPI_DI2_SHIFT = 16,
	CSI_MIPI_DI3_MASK = 0xFF000000L,
	CSI_MIPI_DI3_SHIFT = 24,

	CSI_MAX_RATIO_SKIP_ISP_MASK = 0x00070000L,
	CSI_MAX_RATIO_SKIP_ISP_SHIFT = 16,
	CSI_SKIP_ISP_MASK = 0x00F80000L,
	CSI_SKIP_ISP_SHIFT = 19,
	CSI_MAX_RATIO_SKIP_SMFC_MASK = 0x00000007L,
	CSI_MAX_RATIO_SKIP_SMFC_SHIFT = 0,
	CSI_SKIP_SMFC_MASK = 0x000000F8L,
	CSI_SKIP_SMFC_SHIFT = 3,
	CSI_ID_2_SKIP_MASK = 0x00000300L,
	CSI_ID_2_SKIP_SHIFT = 8,

	CSI_COLOR_FIRST_ROW_MASK = 0x00000002L,
	CSI_COLOR_FIRST_COMP_MASK = 0x00000001L,

	SMFC_MAP_CH0_MASK = 0x00000007L,
	SMFC_MAP_CH0_SHIFT = 0,
	SMFC_MAP_CH1_MASK = 0x00000038L,
	SMFC_MAP_CH1_SHIFT = 3,
	SMFC_MAP_CH2_MASK = 0x000001C0L,
	SMFC_MAP_CH2_SHIFT = 6,
	SMFC_MAP_CH3_MASK = 0x00000E00L,
	SMFC_MAP_CH3_SHIFT = 9,

	SMFC_WM0_SET_MASK = 0x00000007L,
	SMFC_WM0_SET_SHIFT = 0,
	SMFC_WM1_SET_MASK = 0x000001C0L,
	SMFC_WM1_SET_SHIFT = 6,
	SMFC_WM2_SET_MASK = 0x00070000L,
	SMFC_WM2_SET_SHIFT = 16,
	SMFC_WM3_SET_MASK = 0x01C00000L,
	SMFC_WM3_SET_SHIFT = 22,

	SMFC_WM0_CLR_MASK = 0x00000038L,
	SMFC_WM0_CLR_SHIFT = 3,
	SMFC_WM1_CLR_MASK = 0x00000E00L,
	SMFC_WM1_CLR_SHIFT = 9,
	SMFC_WM2_CLR_MASK = 0x00380000L,
	SMFC_WM2_CLR_SHIFT = 19,
	SMFC_WM3_CLR_MASK = 0x0E000000L,
	SMFC_WM3_CLR_SHIFT = 25,

	SMFC_BS0_MASK = 0x0000000FL,
	SMFC_BS0_SHIFT = 0,
	SMFC_BS1_MASK = 0x000000F0L,
	SMFC_BS1_SHIFT = 4,
	SMFC_BS2_MASK = 0x00000F00L,
	SMFC_BS2_SHIFT = 8,
	SMFC_BS3_MASK = 0x0000F000L,
	SMFC_BS3_SHIFT = 12,
};

int ipu_csi_init_interface(uint16_t width, uint16_t height, uint32_t pixel_fmt,
	u32 cfg_param)
{
	uint32_t data_fmt;
	unsigned long lock_flags;
	int csi = 0;
	uint32_t clk_mode = IPU_CSI_CLK_MODE_NONGATED_CLK;

	cfg_param &= ~CSI_SENS_CONF_DATA_FMT_MASK;

	/* Set SENS_DATA_FORMAT bits (8, 9 and 10)
	   RGB or YUV444 is 0 which is current value in data so not set
	   explicitly
	   This is also the default value if attempts are made to set it to
	   something invalid. */
	switch (pixel_fmt) {
	case V4L2_PIX_FMT_YUYV:
		data_fmt = CSI_SENS_CONF_DATA_FMT_YUV422_YUYV;
		break;
	case V4L2_PIX_FMT_UYVY:
		data_fmt = CSI_SENS_CONF_DATA_FMT_YUV422_UYVY;
		break;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		data_fmt = CSI_SENS_CONF_DATA_FMT_RGB_YUV444;
		break;
	case IPUV3_PIX_FMT_GENERIC:
		data_fmt = CSI_SENS_CONF_DATA_FMT_BAYER;
		break;
	case V4L2_PIX_FMT_RGB565:
		data_fmt = CSI_SENS_CONF_DATA_FMT_RGB565;
		break;
	case V4L2_PIX_FMT_RGB555:
		data_fmt = CSI_SENS_CONF_DATA_FMT_RGB555;
		break;
	default:
		return -EINVAL;
	}

	cfg_param |= data_fmt << CSI_SENS_CONF_DATA_FMT_SHIFT;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	printk("%s data: 0x%08x\n", __func__, cfg_param);

	writel(cfg_param, CSI_SENS_CONF(csi));

	/* Setup sensor frame size */
	writel((width - 1) | (height - 1) << 16, CSI_SENS_FRM_SIZE(csi));

	/* Set CCIR registers */
	switch (clk_mode) {
	case IPU_CSI_CLK_MODE_CCIR656_PROGRESSIVE:
		writel(0x40030, CSI_CCIR_CODE_1(csi));
		writel(0xFF0000, CSI_CCIR_CODE_3(csi));
		break;
	case IPU_CSI_CLK_MODE_CCIR656_INTERLACED:
		if (width == 720 && height == 625) {
			/* PAL case */
			/*
			 * Field0BlankEnd = 0x6, Field0BlankStart = 0x2,
			 * Field0ActiveEnd = 0x4, Field0ActiveStart = 0
			 */
			writel(CSI_CCIR_ERR_DET_EN | 0x40596, CSI_CCIR_CODE_1(csi));
			/*
			 * Field1BlankEnd = 0x7, Field1BlankStart = 0x3,
			 * Field1ActiveEnd = 0x5, Field1ActiveStart = 0x1
			 */
			writel(0xD07DF, CSI_CCIR_CODE_2(csi));
			writel(0xFF0000, CSI_CCIR_CODE_3(csi));
		} else if (width == 720 && height == 525) {
			/* NTSC case */
			/*
			 * Field0BlankEnd = 0x7, Field0BlankStart = 0x3,
			 * Field0ActiveEnd = 0x5, Field0ActiveStart = 0x1
			 */
			writel(CSI_CCIR_ERR_DET_EN | 0xD07DF, CSI_CCIR_CODE_1(csi));
			/*
			 * Field1BlankEnd = 0x6, Field1BlankStart = 0x2,
			 * Field1ActiveEnd = 0x4, Field1ActiveStart = 0
			 */
			writel(0x40596, CSI_CCIR_CODE_2(csi));
			writel(0xFF0000, CSI_CCIR_CODE_3(csi));
		} else {
			spin_unlock_irqrestore(&ipu_lock, lock_flags);
			dev_err(ipu_dev, "Unsupported CCIR656 interlaced "
					"video mode\n");
			return -EINVAL;
		}
		break;
	case IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_DDR:
	case IPU_CSI_CLK_MODE_CCIR1120_PROGRESSIVE_SDR:
	case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_DDR:
	case IPU_CSI_CLK_MODE_CCIR1120_INTERLACED_SDR:
		writel(CSI_CCIR_ERR_DET_EN | 0x40030, CSI_CCIR_CODE_1(csi));
		writel(0xFF0000, CSI_CCIR_CODE_3(csi));
		break;
	case IPU_CSI_CLK_MODE_GATED_CLK:
	case IPU_CSI_CLK_MODE_NONGATED_CLK:
		break;
	}

	dev_dbg(ipu_dev, "CSI_SENS_CONF = 0x%08X\n",
		readl(CSI_SENS_CONF(csi)));
	dev_dbg(ipu_dev, "CSI_ACT_FRM_SIZE = 0x%08X\n",
		readl(CSI_ACT_FRM_SIZE(csi)));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);

	return 0;
}
EXPORT_SYMBOL(ipu_csi_init_interface);

/*!
 * ipu_csi_get_window_size
 *
 * @param	width	pointer to window width
 * @param	height	pointer to window height
 * @param	csi	csi 0 or csi 1
 */
void ipu_csi_get_window_size(uint32_t *width, uint32_t *height, uint32_t csi)
{
	uint32_t reg;
	unsigned long lock_flags;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	reg = readl(CSI_ACT_FRM_SIZE(csi));
	*width = (reg & 0xFFFF) + 1;
	*height = (reg >> 16 & 0xFFFF) + 1;

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_csi_get_window_size);

/*!
 * ipu_csi_set_window_size
 *
 * @param	width	window width
 * @param       height	window height
 * @param       csi	csi 0 or csi 1
 */
void ipu_csi_set_window_size(uint32_t width, uint32_t height, uint32_t csi)
{
	unsigned long lock_flags;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	writel((width - 1) | (height - 1) << 16, CSI_ACT_FRM_SIZE(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_csi_set_window_size);

/*!
 * ipu_csi_set_window_pos
 *
 * @param       left	uint32 window x start
 * @param       top	uint32 window y start
 * @param       csi	csi 0 or csi 1
 */
void ipu_csi_set_window_pos(uint32_t left, uint32_t top, uint32_t csi)
{
	uint32_t temp;
	unsigned long lock_flags;

	spin_lock_irqsave(&ipu_lock, lock_flags);

	temp = readl(CSI_OUT_FRM_CTRL(csi));
	temp &= ~(CSI_HSC_MASK | CSI_VSC_MASK);
	temp |= ((top << CSI_VSC_SHIFT) | (left << CSI_HSC_SHIFT));
	writel(temp, CSI_OUT_FRM_CTRL(csi));

	spin_unlock_irqrestore(&ipu_lock, lock_flags);
}
EXPORT_SYMBOL(ipu_csi_set_window_pos);

/*!
 * _ipu_csi_init
 *
 * @param	channel      IDMAC channel
 * @param	csi	     csi 0 or csi 1
 *
 * @return	Returns 0 on success or negative error code on fail
 */
int _ipu_csi_init(int channel, int csi, int burstsize, int mipi_id)
{
	unsigned long flags;
	u32 val, shift;

	spin_lock_irqsave(&ipu_lock, flags);

	val = readl(CSI_SENS_CONF(csi));
	val &= ~CSI_SENS_CONF_DATA_DEST_MASK;
	val |= CSI_DATA_DEST_IDMAC << CSI_SENS_CONF_DATA_DEST_SHIFT;
	writel(val, CSI_SENS_CONF(csi));

	shift = channel * 4;
	val = readl(SMFC_BS);
	val &= ~(0xf << shift);
	val |= burstsize << shift;
	writel(val, SMFC_BS);

	shift = channel * 3;
	val = readl(SMFC_MAP);
	val &= ~(0x7 << shift);
	val |= ((csi << 2) | mipi_id) << shift;
	writel(val, SMFC_MAP);

	spin_unlock_irqrestore(&ipu_lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(_ipu_csi_init);

int ipu_capture_init(struct ipu_soc *ipu, struct device *dev, unsigned long csi1_base,
		unsigned long csi2_base, unsigned long smfc_base)
{
	printk("%s: csi1: 0x%08lx csi2: 0x%08lx smfc: 0x%08lx\n", __func__,
			csi1_base, csi2_base, smfc_base);
	ipu_csi_reg[0] = ioremap(csi1_base, PAGE_SIZE);
	if (!ipu_csi_reg[0])
		return -ENOMEM;
	ipu_csi_reg[1] = ioremap(csi2_base, PAGE_SIZE);
	if (!ipu_csi_reg[1])
		return -ENOMEM;
	ipu_smfc_reg = ioremap(smfc_base, PAGE_SIZE);
	if (!ipu_smfc_reg)
		return -ENOMEM;

	ipu_dev = dev;

	return 0;
}

void ipu_capture_exit(struct ipu_soc *ipu)
{
	 iounmap(ipu_csi_reg[0]);
	 iounmap(ipu_csi_reg[1]);
	 iounmap(ipu_smfc_reg);
}
