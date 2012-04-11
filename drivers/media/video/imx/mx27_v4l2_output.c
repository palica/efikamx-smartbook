/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gcd.h>
#include <media/v4l2-dev.h>
#include <asm/poll.h>
#include <asm/io.h>
#include <media/v4l2-ioctl.h>
#include <linux/clk.h>
#include <media/videobuf-dma-contig.h>

#include <media/v4l2-dev.h>

#define PP_TBL_MAX	40

/* PP register definitions */
#define PP_REG(ofs)    (IO_ADDRESS(EMMA_BASE_ADDR) - 0x400 + ofs)

/* Register offsets */
#define PP_CNTL			PP_REG(0x00)
#define PP_INTRCNTL	 	PP_REG(0x04)
#define PP_INTRSTATUS		PP_REG(0x08)
#define PP_SOURCE_Y_PTR		PP_REG(0x0C)
#define PP_SOURCE_CB_PTR	PP_REG(0x10)
#define PP_SOURCE_CR_PTR	PP_REG(0x14)
#define PP_DEST_RGB_PTR 	PP_REG(0x18)
#define PP_QUANTIZER_PTR	PP_REG(0x1C)
#define PP_PROCESS_FRAME_PARA	PP_REG(0x20)
#define PP_SOURCE_FRAME_WIDTH	PP_REG(0x24)
#define PP_DEST_DISPLAY_WIDTH	PP_REG(0x28)
#define PP_DEST_IMAGE_SIZE	PP_REG(0x2C)
#define PP_DEST_FRAME_FMT_CNTL	PP_REG(0x30)
#define PP_RESIZE_INDEX		PP_REG(0x34)
#define	PP_CSC_COEF_0123	PP_REG(0x38)
#define	PP_CSC_COEF_4		PP_REG(0x3C)
#define PP_RESIZE_COEF_TBL	PP_REG(0x100)

/* resize table dimensions 
    dest pixel index    left/32    right/32    #src pixels to read
    0                   [BC_COEF]  [BC_COEF]   [BC_NXT]
    :
    pp_tbl_max-1
*/
#define BC_NXT		2
#define BC_COEF		5
#define SZ_COEF		(1 << BC_COEF)
#define SZ_NXT		(1 << BC_NXT)

/* PP operations */
#define EN_DEBLOCK 	0x02
#define EN_DERING	0x04
#define EN_CSC		0x10
#define EN_MACROBLOCK	0x20
#define EN_DEF		0x16
#define EN_MASK		0x36
#define EN_BIGDATA	0x1000
#define EN_BIGQP	0x2000

/* PP CSC tables */
#define CSC_TBL_NONE	0x80
#define CSC_TBL_REUSE	0x81
#define CSC_TBL_A1	0x00
#define CSC_TBL_A0	0x20
#define CSC_TBL_B1	0x40
#define CSC_TBL_B0	0x60
/* converts from 4 decimal fixed point to hw setting & vice versa */
#define PP_CSC_FP4_2_HW(coeff)	((((coeff) << 7) + 5000) / 10000)
#define PP_CSC_HW_2_FP4(coeff)	((((coeff) * 10000) + 64) >> 7)

#define PP_PIX_YUYV	0
#define PP_PIX_YVYU	8
#define PP_PIX_UYVY	16
#define PP_PIX_VYUY	24

/* PP size & width calculation macros */
#define PP_CALC_QP_WIDTH(cfg)	\
	(!((cfg)->operation & (EN_DEBLOCK | EN_DERING)) ? 0 : \
		(((((cfg)->dim.in.width + 15) >> 4) + 3) & ~3))

#define PP_SKIP		1

#define EMMA_BASE_ADDR          (AIPI_BASE_ADDR + 0x26400)
#define INT_EMMAPP              52

struct vout_buffer {
	struct videobuf_buffer		vb;
	struct vout_data		*vout;
};

struct emma_pp_size {
	int width;
	int height;
};

/*
 * if num.width != 0
 * 	resize ratio = num.width : den.width
 * else
 * 	resize ratio = in.width : out.width
 * same for height
 */
struct emma_pp_scale {
	struct emma_pp_size num;
	struct emma_pp_size den;
	struct emma_pp_size in;		/* clip */
	struct emma_pp_size out;	/* 0 -> same as in */
};

/*
 * physical addresses for bus mastering 
 * v=0 -> yuv packed 
 * v=0 & qp=0 -> yuv packed with qp appended
 */
struct emma_pp_ptr {
	unsigned int y;		/* Y data (line align8) */
	unsigned int u;		/* U data (line align4) */
	unsigned int v;		/* V data (line align4) */
	unsigned int qp;	/* Quantization (line align4) */
};

struct emma_pp_cfg {
	unsigned char operation;	/* OR of EN_xx defines */

	/* if resolution is 0, the minimum for the sum of widths is chosen */
	short rgb_resolution;	/* 8,16,24 bpp only */

	struct emma_pp_ptr ptr;	/* dma buffer pointers */
	unsigned int outptr;	/* RGB/YUV output */
	struct emma_pp_scale dim;	/* in/out dimensions */

	/* pixels between two adjacent input Y rows */
	unsigned short in_y_stride;	/* 0 = in_width */
	/* PIXELS between two adjacent output rows */
	unsigned short out_stride;	/* 0 = out_width */
};

struct vout_data {
	struct video_device	*video_dev;

	int			width, height; /* Input picture size */
	struct v4l2_rect	overlay;       /* where to put the image */

	struct emma_pp_cfg	cfg;
	int			irq;
	struct clk		*clk;
	int			open;

	unsigned short		scale_tbl[PP_TBL_MAX];
	int			g_hlen, g_vlen;

	u32			pp_dest_frame_fmt_cntl;
	u32			pp_cntl;

	struct list_head	vout_list;
	struct fb_info		*fbinfo;
	struct videobuf_queue	vidq;

	int			needs_config;
};

struct vout_device {
	spinlock_t		irqlock;
	struct platform_device	*pdev;
	struct clk		*clk;
	int			irq;
	struct list_head	queued;
	struct videobuf_buffer	*active;
};

static struct vout_device vout_dev;

static const unsigned char pp_coeftab[] = {
	 2,  1, 19, 10, 17,  9, 15,  8, 13,  7, 11,  6,
	20, 11,	 9,  5, 16,  9,  7,  4, 19, 11, 12,  7,
	17, 10,  5,  3,	18, 11,	13,  8,  8,  5,	19, 12,
	11,  7, 14,  9,	17, 11,	20, 13,  3,  2, 19, 13,
	16, 11, 13,  9, 10,  7, 17, 12,  7,  5, 18, 13,
	11,  8, 15, 11, 19, 14,  4,  3, 17, 13, 13, 10,
	 9,  7, 14, 11, 19, 15,  5,  4, 16, 13, 11,  9,
	17, 14,  6,  5, 19, 16, 13, 11, 20, 17,  7,  6,
	15, 13,  8,  7, 17, 15,  9,  8, 19, 17, 10,  9,
	11, 10, 12, 11, 13, 12, 14, 13, 15, 14, 16, 15,
	17, 16, 18, 17, 19, 18, 20, 19,  1,  1, 19, 20,
	18, 19, 17, 18, 16, 17, 15, 16, 14, 15, 13, 14,
	12, 13, 11, 12, 10, 11,  9, 10, 17, 19,  8,  9,
	15, 17,  7,  8, 13, 15,  6,  7, 17, 20, 11, 13,
	16, 19,  5,  6, 14, 17,  9, 11, 13, 16,  4,  5,
	15, 19, 11, 14,  7,  9, 10, 13, 13, 17,  3,  4,
	14, 19, 11, 15,  8, 11, 13, 18,  5,  7, 12, 17,
	 7, 10,  9, 13, 11, 16, 13, 19,  2,  3, 13, 20,
	11, 17,  9, 14,  7, 11, 12, 19,  5,  8,  8, 13,
	11, 18,  3,  5, 10, 17,  7, 12, 11, 19,  4,  7,
	 9, 16,  5,  9, 11, 20,  6, 11,  7, 13,  8, 15,
	 9, 17, 10, 19,  1,  2,  9, 19,  8, 17,  7, 15,
	 6, 13,  5, 11,  9, 20,  4,  9,  7, 16,  3,  7,
	 8, 19,  5, 12,  7, 17,  2,  5,  7, 18,  5, 13,
	 3,  8,  7, 19,  4, 11,  5, 14,  6, 17,  7, 20,
	 1,  3,  6, 19,  5, 16,  4, 13,  3, 10,  5, 17,
	 2,  7,  5, 18,  3, 11,  4, 15,  5, 19,  1, 4,
};

/*!
 * @brief Get ratio.
 * @param x	First input value
 * @param y	Second input value
 * @param den	Denominator of the ratio (corresponding to y)
 * @return	Numerator of the ratio (corresponding to x)
 */
static int ratio(int x, int y, int *den)
{
	int g;

	if (!x || !y)
		return 0;

	g = gcd(x, y);
	*den = y / g;

	return x / g;
}

/*!
 * @brief Build PP coefficient entry
 * Build one or more coefficient entries for PP coefficient table based
 * on given coefficient.
 *
 * @param k	The index of the coefficient in coefficient table
 * @param coeff	The weighting coefficient
 * @param base	The base of the coefficient
 * @param nxt	Number of pixels to be read
 *
 * @return	The index of the next coefficient entry on success
 *		-1 on failure
 */
static int scale_0d(struct vout_data *vout, int k, int coeff, int base, int nxt)
{
	if (k >= PP_TBL_MAX) {
		/* no more space in table */
		pr_info("no space in scale table, k = %d\n", k);
		return -EINVAL;
	}

	coeff = ((coeff << BC_COEF) + (base >> 1)) / base;

	/*
	 * Valid values for weighting coefficient are 0, 2 to 30, and 31.
	 * A value of 31 is treated as 32 and therefore 31 is an
	 * invalid co-efficient.
	 */
	if (coeff >= SZ_COEF - 1)
		coeff--;
	else if (coeff == 1)
		coeff++;
	coeff = coeff << BC_NXT;

	if (nxt < SZ_NXT) {
		coeff |= nxt;
		coeff <<= 1;
		coeff |= 1;
	} else {
		/*
		 * src inc field is 2 bit wide, for 4+, use special
		 * code 0:0:1 to prevent dest inc
		 */
		coeff |= PP_SKIP;
		coeff <<= 1;
		coeff |= 1;
		nxt -= PP_SKIP;
		do {
			pr_debug("tbl = %03X\n", coeff);
			vout->scale_tbl[k++] = coeff;
			coeff = (nxt > PP_SKIP) ? PP_SKIP : nxt;
			coeff <<= 1;
		} while ((nxt -= PP_SKIP) > 0);
	}
	pr_debug("tbl = %03X\n", coeff);
	vout->scale_tbl[k++] = coeff;

	return k;
}

/*!
 * @brief Get approximate ratio
 *
 * @param pscale	The pointer to scale_t structure which holdes
 * 			coefficient tables
 * @param mt		Scale ratio numerator
 * @param nt		Scale ratio denominator
 * @param *n		denominator of approximate ratio
 * @return		numerator of approximate ratio
 */
static int approx_ratio(int mt, int nt, int *n)
{
	int index = sizeof(pp_coeftab) / sizeof(pp_coeftab[0]) / 2;
	int left = 0;
	int right = index - 1;
	int nom = 0, den = 0;
	while (index > 0) {
		nom = pp_coeftab[(((right + left) >> 1) << 1)];
		den = pp_coeftab[(((right + left) >> 1) << 1) + 1];
		if ((nom * nt - mt * den) > 0) {
			left = (right + left) >> 1;
		} else {
			right = (right + left) >> 1;
		}
		index = index >> 1;
	}
	*n = pp_coeftab[right * 2 + 1];
	nom = pp_coeftab[right * 2];
	return nom;
}

/*
 * @brief Build PP coefficient table
 * Build PP coefficient table for one dimension (width or height)
 * based on given input and output resolution
 *
 * @param inv	input resolution
 * @param outv	output resolution
 * @param k	index of free table entry
 *
 * @return	The index of the next free coefficient entry on success
 *		-1 on failure
 */
static int scale_1d(struct vout_data *vout, int inv, int outv, int k)
{
	int v;			/* overflow counter */
	int coeff, nxt;		/* table output */

	if (inv == outv)
		return scale_0d(vout, k, 1, 1, 1);	/* force scaling */

	v = 0;
	if (inv < outv) {
		/* upscale: mix <= 2 input pixels per output pixel */
		do {
			coeff = outv - v;
			v += inv;
			if (v >= outv) {
				v -= outv;
				nxt = 1;
			} else
				nxt = 0;
			pr_debug("upscale: coeff = %d/%d nxt = %d\n", coeff,
				 outv, nxt);
			k = scale_0d(vout, k, coeff, outv, nxt);
			if (k < 0)
				return k;
		} while (v);
	} else if (inv >= 2 * outv) {
		/* PP doesn't support resize ratio > 2:1 except 4:1. */
		if ((inv != 2 * outv) && (inv != 4 * outv))
			return -EINVAL;
		/* downscale: >=2:1 bilinear approximation */
		coeff = inv - 2 * outv;
		v = 0;
		nxt = 0;
		do {
			v += coeff;
			nxt = 2;
			while (v >= outv) {
				v -= outv;
				nxt++;
			}
			pr_debug("downscale: coeff = 1/2 nxt = %d\n", nxt);
			k = scale_0d(vout, k, 1, 2, nxt);
			if (k < 0)
				return k;
		} while (v);
	} else {
		/* downscale: bilinear */
		int in_pos_inc = 2 * outv;
		int out_pos = inv;
		int out_pos_inc = 2 * inv;
		int init_carry = inv - outv;
		int carry = init_carry;

		v = outv + in_pos_inc;
		do {
			coeff = v - out_pos;
			out_pos += out_pos_inc;
			carry += out_pos_inc;
			for (nxt = 0; v < out_pos; nxt++) {
				v += in_pos_inc;
				carry -= in_pos_inc;
			}
			pr_debug("downscale: coeff = %d/%d nxt = %d\n", coeff,
				 in_pos_inc, nxt);
			k = scale_0d(vout, k, coeff, in_pos_inc, nxt);
			if (k < 0)
				return k;
		} while (carry != init_carry);
	}
	return k;
}

/*
 * @brief Build PP coefficient table
 * Build PP coefficient table for one dimension (width or height)
 * based on given input and output resolution. The given input
 * and output resolution might be not supported due to hardware
 * limits. In this case this function rounds the input and output
 * to closest possible values and return them to caller.
 *
 * @param inv	input resolution, might be modified after the call
 * @param outv	output resolution, might be modified after the call
 * @param k	index of free table entry
 *
 * @return	The index of the next free coefficient entry on success
 *		-1 on failure
 */
static int scale_1d_smart(struct vout_data *vout, int inv, int *outv, int index)
{
	int len, num, den = 0, approx_num, approx_den;
	static int num1, den1;

	/* Both should be non-zero */
	if (!inv || !(*outv))
		return -EINVAL;

	/*
	 * We can do arbitrary upscale ratios between 1:1 and 1:2, arbitrary
	 * downscale ratios between 1:1 and 2:1 and a fixed downscaling of 4:1
	 */
	if (*outv > 2 * inv)
		*outv = 2 * inv;
	if (*outv * 4 < inv)
		*outv = inv / 4;
	if (*outv * 2 < inv && *outv * 4 != inv)
		*outv = inv / 2;

	num = ratio(inv, *outv, &den);

	if (index == 0) {
		if ((num > 20) || (den > 20)) {
			approx_num = approx_ratio(num, den, &approx_den);
			num = approx_num;
			den = approx_den;
		}
	} else {
		if ((num > (40 - index)) || (den > (40 - index))) {
			approx_num = approx_ratio(num, den, &approx_den);
			num = approx_num;
			den = approx_den;
		}
		if ((num == num1) && (den == den1))
			return index;
	}

	len = scale_1d(vout, num, den, index);
	if (len < 0)
		return len;

	if (index == 0) {
		num1 = num;
		den1 = den;
	}

	return len;
}

#define ROUND_UP_2(num)	(((num) + 1) & ~1)
#define ROUND_UP_4(num)	(((num) + 3) & ~3)
#define ROUND_UP_8(num)	(((num) + 7) & ~7)

static int pp_cfg(struct vout_data *vout)
{
	int ix, iy, ox, oy, top, left;
	struct emma_pp_cfg *cfg = &vout->cfg;
	int bytes_per_pixel;

	cfg->rgb_resolution = vout->fbinfo->var.bits_per_pixel;

	ix = vout->width & ~7;
	iy = vout->height & ~7;

	ox = vout->overlay.width;
	vout->g_hlen = scale_1d_smart(vout, ix, &ox, 0);
	if (vout->g_hlen < 0)
		return vout->g_hlen;

	oy = (ox * iy) / ix;
	if (oy > vout->overlay.height) {
		oy = vout->overlay.height;
		ox = (ix * vout->overlay.height) / iy;
		vout->g_hlen = scale_1d_smart(vout, ix, &ox, 0);
		if (vout->g_hlen < 0)
			return vout->g_hlen;
	}

	vout->g_vlen = scale_1d_smart(vout, iy, &oy, vout->g_hlen);
	if (vout->g_vlen < 0)
		return vout->g_vlen;

	cfg->dim.out.width = ox;
	cfg->dim.out.height = oy;
	cfg->dim.in.width = vout->width & ~7;
	cfg->dim.in.height = vout->height & ~7;

	top = abs(vout->overlay.height - oy) / 2;
	left = abs(vout->overlay.width - ox) / 2;

	pr_info("in: %dx%d out: %dx%d\n", vout->width, vout->height,
			cfg->dim.out.width,
			cfg->dim.out.height);

	cfg->dim.in.width = vout->width;
	cfg->dim.in.height = vout->height;

	cfg->in_y_stride = ROUND_UP_4(vout->width);

	bytes_per_pixel = cfg->rgb_resolution >> 3;
	cfg->outptr = vout->fbinfo->fix.smem_start;
	cfg->outptr += (top + vout->overlay.top) * vout->fbinfo->var.xres * bytes_per_pixel
		    + (vout->overlay.left + left) * bytes_per_pixel;
	cfg->out_stride = vout->fbinfo->var.xres * bytes_per_pixel;

	vout->pp_cntl = __raw_readl(PP_CNTL) & ~EN_MASK;
	vout->pp_cntl |= EN_CSC;
	vout->pp_cntl &= ~0xC00;
	if (cfg->rgb_resolution < 32)
		vout->pp_cntl |= cfg->rgb_resolution << 7;

	vout->pp_dest_frame_fmt_cntl =
			(vout->fbinfo->var.red.offset   << 26) |
			(vout->fbinfo->var.green.offset << 21) |
			(vout->fbinfo->var.blue.offset  << 16) |
			(vout->fbinfo->var.red.length   <<  8) |
			(vout->fbinfo->var.green.length <<  4) |
			(vout->fbinfo->var.blue.length  <<  0);

	return 0;
}

static void pphw_cfg(struct vout_data *vout)
{
	struct emma_pp_cfg *cfg = &vout->cfg;
	int i;

	/* Frame mode */
	__raw_writel(0x05, PP_INTRCNTL);

	__raw_writel((cfg->dim.out.width << 16) | cfg->dim.out.height,
		     PP_DEST_IMAGE_SIZE);
	__raw_writel(((vout->g_hlen - 1) << 16) |
			(vout->g_vlen == vout->g_hlen ? 0 : (vout->g_hlen << 8)) |
			(vout->g_vlen - 1), PP_RESIZE_INDEX);

	for (i = 0; i < vout->g_vlen; i++)
		__raw_writel(vout->scale_tbl[i], PP_RESIZE_COEF_TBL + i * 4);

	__raw_writel(vout->pp_dest_frame_fmt_cntl, PP_DEST_FRAME_FMT_CNTL);

	/* add csc formatting */
	__raw_writel(0x80b42c5b, PP_CSC_COEF_0123);
	__raw_writel(0xe4, PP_CSC_COEF_4);

	__raw_writel(vout->pp_cntl, PP_CNTL);

	__raw_writel(cfg->outptr, PP_DEST_RGB_PTR);

	/*
	 * #MB in a row = input_width / 16pix
	 * 1 byte per QP per MB
	 * QP must be formatted to be 4-byte aligned
	 * YUV lines are to be 4-byte aligned as well
	 * So Y is 8 byte aligned, as U = V = Y/2 for 420
	 * MPEG MBs are 16x16 anyway
	 */
	__raw_writel((cfg->dim.in.width << 16) | cfg->dim.in.height,
		     PP_PROCESS_FRAME_PARA);
	__raw_writel(cfg->in_y_stride | (PP_CALC_QP_WIDTH(cfg) << 16),
		     PP_SOURCE_FRAME_WIDTH);
	__raw_writel(cfg->out_stride, PP_DEST_DISPLAY_WIDTH);
}

static int frame_calc_size(int width, int height)
{
	int ystride, ustride, vstride, size;

	ystride = ROUND_UP_4(width);
	ustride = ROUND_UP_8(width) / 2;
	vstride = ROUND_UP_8(ystride) / 2;

	size = ystride * ROUND_UP_2(height);
	size += ustride * ROUND_UP_2(height) / 2;
	size += vstride * ROUND_UP_2(height) / 2;

	return size;
}

static void vout_start_frame(struct vout_data *vout, struct videobuf_buffer *vb)
{
	struct emma_pp_cfg *cfg = &vout->cfg;
	struct vout_device *vdev = &vout_dev;
	dma_addr_t dma;
	int ustride, vstride;

	if (vout->needs_config) {
		pp_cfg(vout);
		vout->needs_config = 0;
	}

	pphw_cfg(vout);

	dma = videobuf_to_dma_contig(vb);

	cfg->ptr.y = dma;

	/* yuv - packed */
	cfg->ptr.u = cfg->ptr.y + cfg->in_y_stride * ROUND_UP_2(cfg->dim.in.height);
	ustride = ROUND_UP_8(cfg->dim.in.width) / 2;
	cfg->ptr.v = cfg->ptr.u + ustride * ROUND_UP_2 (cfg->dim.in.height) / 2;
	vstride = ROUND_UP_8(cfg->dim.in.width) / 2;
	/* yuv packed with qp appended */
	cfg->ptr.qp = cfg->ptr.v + ustride * ROUND_UP_2(cfg->dim.in.height) / 2;

	__raw_writel(cfg->ptr.y, PP_SOURCE_Y_PTR);
	__raw_writel(cfg->ptr.u, PP_SOURCE_CB_PTR);
	__raw_writel(cfg->ptr.v, PP_SOURCE_CR_PTR);
	__raw_writel(cfg->ptr.qp, PP_QUANTIZER_PTR);

	vdev->active = vb;

	vb->state = VIDEOBUF_ACTIVE;

	__raw_writel(__raw_readl(PP_CNTL) | 1, PP_CNTL);
}

static struct vout_data *vout_active;

static irqreturn_t pp_isr(int irq, void *dev_id)
{
	int status;
	struct vout_data *vout;
	struct vout_device *vdev = &vout_dev;
	struct videobuf_buffer *vb = vdev->active;
	struct vout_buffer *v;
	unsigned long flags;

	pr_debug("pp: in isr.\n");
	status = __raw_readl(PP_INTRSTATUS) & 7;

	if (status & 4)
		pr_info("pp: isr state error.\n");

	/* clear interrupt status */
	__raw_writel(status, PP_INTRSTATUS);
	if ((status & 0x1) == 0) {	/* Not frame complete interrupt */
		pr_info("not pp frame complete interrupt\n");
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&vdev->irqlock, flags);

	list_del_init(&vb->queue);
	vb->state = VIDEOBUF_DONE;
	vb->field_count++;
	wake_up(&vb->done);

	if (list_empty(&vdev->queued)) {
                vdev->active = NULL;
                goto out;
        }

	v = list_entry(vdev->queued.next, struct vout_buffer, vb.queue);
	vout = v->vout;
	vdev->active = &v->vb;

	vout_start_frame(vout, vdev->active);
out:
	spin_unlock_irqrestore(&vdev->irqlock, flags);

	return IRQ_HANDLED;
}

static int pphw_reset(struct vout_device *vout)
{
	int i;

	__raw_writel(0x100, PP_CNTL);

	/* timeout */
	for (i = 0; i < 1000; i++) {
		if (!(__raw_readl(PP_CNTL) & 0x100)) {
			pr_info("pp reset over\n");
			break;
		}
	}

	/* check reset value */
	if (__raw_readl(PP_CNTL) != 0x876) {
		pr_info("pp reset value err = 0x%08X\n", __raw_readl(PP_CNTL));
		return -1;
	}

	return 0;
}

static int mxc_v4l2out_open(struct file *file)
{
	struct video_device *dev = video_devdata(file);
	struct vout_data *vout = video_get_drvdata(dev);

	if (vout->open)
		return -EBUSY;

	vout->open = 1;

	file->private_data = dev;

	return 0;
}

static int mxc_v4l2out_close(struct file *file)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);

	vout->open = 0;

	file->private_data = NULL;

	return 0;
}

static int vidioc_querycap(struct file *file, void  *priv,
		struct v4l2_capability *cap)
{
	strcpy(cap->driver, "mxc_v4l2_output");
	cap->version = 0;
	cap->capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OVERLAY;
	cap->card[0] = '\0';
	cap->bus_info[0] = '\0';
	return 0;
}

static void dump_rects(struct vout_data *vout, const char *prefix)
{
	printk("%s:\n", prefix);
	printk("input image:  %dx%d\n", vout->width, vout->height);
	printk("framebuffer:  %dx%d\n", vout->fbinfo->var.xres, vout->fbinfo->var.yres);
	printk("start image:  %d,%d\n", vout->overlay.left, vout->overlay.top);
	printk("output image: %dx%d\n", vout->overlay.width, vout->overlay.height);
}

static int vidioc_g_fmt_vid_out(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		printk("%s falied\n", __func__);
		return -EINVAL;
	}

	f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
	f->fmt.pix.bytesperline = vout->width;
	f->fmt.pix.width = vout->width;
	f->fmt.pix.height = vout->height;
	f->fmt.pix.sizeimage = (vout->width * vout->height * 3) / 2;

	printk("%s in: %dx%d\n", __func__, f->fmt.pix.width, f->fmt.pix.height);

	return 0;
}

static int vidioc_s_fmt_vid_out(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		printk("%s failed\n", __func__);
		return -EINVAL;
	}

	printk("%s in: %dx%d\n", __func__, f->fmt.pix.width, f->fmt.pix.height);

	f->fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
	f->fmt.pix.bytesperline = f->fmt.pix.width;
	f->fmt.pix.sizeimage = frame_calc_size(f->fmt.pix.width, f->fmt.pix.height);

	vout->width = f->fmt.pix.width;
	vout->height = f->fmt.pix.height;
	vout->overlay.left = 0;
	vout->overlay.top = 0;
	vout->overlay.width = vout->fbinfo->var.xres;
	vout->overlay.height = vout->fbinfo->var.yres;

	dump_rects(vout, __func__);
	vout->needs_config = 1;

	return 0;
}

static int vidioc_g_fmt_vid_overlay(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		printk("%s falied\n", __func__);
		return -EINVAL;
	}

	f->fmt.win.w.left = vout->overlay.left;
	f->fmt.win.w.top = vout->overlay.top;
	f->fmt.win.w.width = vout->overlay.width;
	f->fmt.win.w.height = vout->overlay.height;

	return 0;
}

static int vidioc_s_fmt_vid_overlay(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		printk("%s falied\n", __func__);
		return -EINVAL;
	}

	if (f->fmt.win.w.width > vout->fbinfo->var.xres)
		f->fmt.win.w.width = vout->fbinfo->var.xres;
	if (f->fmt.win.w.height > vout->fbinfo->var.yres)
		f->fmt.win.w.height = vout->fbinfo->var.yres;
	if (f->fmt.win.w.left + f->fmt.win.w.width > vout->fbinfo->var.xres)
		f->fmt.win.w.left = vout->fbinfo->var.xres - f->fmt.win.w.width;
	if (f->fmt.win.w.top + f->fmt.win.w.height > vout->fbinfo->var.yres)
		f->fmt.win.w.top = vout->fbinfo->var.yres - f->fmt.win.w.height;

	vout->overlay.left = f->fmt.win.w.left;
	vout->overlay.top = f->fmt.win.w.top;
	vout->overlay.width = f->fmt.win.w.width;
	vout->overlay.height = f->fmt.win.w.height;

	dump_rects(vout, __func__);

	return 0;
}

static int vout_videobuf_setup(struct videobuf_queue *q,
		unsigned int *count, unsigned int *size)
{
	struct vout_data *vout = q->priv_data;

	*size = frame_calc_size(vout->width, vout->height);

	return 0;
}

static int vout_videobuf_prepare(struct videobuf_queue *q,
		struct videobuf_buffer *vb,
		enum v4l2_field field)
{
	struct vout_data *vout = q->priv_data;
	struct vout_buffer *vbuf = container_of(vb, struct vout_buffer, vb);
	int ret = 0;

	vbuf->vout = vout;
	vb->width = vout->width;
	vb->height = vout->height;
	vb->size = frame_calc_size(vb->width, vb->height);

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(&vout->vidq, vb, NULL);
		if (ret)
			goto fail;

		vb->state = VIDEOBUF_PREPARED;
	}

fail:
	return ret;
}

static void vout_videobuf_queue(struct videobuf_queue *q,
		struct videobuf_buffer *vb)
{
	struct vout_data *vout = q->priv_data;
	struct vout_device *vdev = &vout_dev;

	list_add_tail(&vb->queue, &vdev->queued);

	vb->state = VIDEOBUF_QUEUED;

	if (!vdev->active) {
		vout_active = vout;
		vout_start_frame(vout, vb);
	}
}

static void vout_videobuf_release(struct videobuf_queue *q,
		struct videobuf_buffer *vb)
{
	struct vout_device *vdev = &vout_dev;

	spin_lock_irq(&vdev->irqlock);

	if (vb->state == VIDEOBUF_ACTIVE) {
		printk("Clean the active one\n");
	}

	spin_unlock_irq(&vdev->irqlock);

	videobuf_dma_contig_free(q, vb);

	vb->state = VIDEOBUF_NEEDS_INIT;
}

static struct videobuf_queue_ops vout_videobuf_ops = {
	.buf_setup	= vout_videobuf_setup,
	.buf_prepare	= vout_videobuf_prepare,
	.buf_queue	= vout_videobuf_queue,
	.buf_release	= vout_videobuf_release,
};

static int vidioc_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *reqbuf)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);
	struct vout_device *vdev = &vout_dev;
	int ret;

	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	videobuf_queue_dma_contig_init(&vout->vidq,
					    &vout_videobuf_ops, &dev->dev,
					    &vdev->irqlock,
					    reqbuf->type,
					    V4L2_FIELD_NONE,
					    sizeof(struct vout_buffer),
					    vout);

	/* Allocate buffers */
	ret = videobuf_reqbufs(&vout->vidq, reqbuf);

	return ret;
}

static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);

	return videobuf_querybuf(&vout->vidq, buf);
}

static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);
	int ret;

	ret = videobuf_qbuf(&vout->vidq, buf);
	if(ret)
		printk("%s failed with %d\n", __func__, ret);
	return ret;
}

static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *buf)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);
	int ret;

	ret = videobuf_dqbuf(&vout->vidq, buf, file->f_flags & O_NONBLOCK);
	if(ret)
		printk("%s failed with %d\n", __func__, ret);

	return ret;
}

static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);
	int ret;

	if (pp_cfg(vout)) {
		printk("%s failed\n", __func__);
		return -EINVAL;
	}

	ret = videobuf_streamon(&vout->vidq);
	if(ret)
		printk("%s failed with %d\n", __func__, ret);
	return ret;
}

static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);

	return videobuf_streamoff(&vout->vidq);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *fh,
		struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_YUV420;

	return 0;
}

static int mxc_v4l2out_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct video_device *dev = file->private_data;
	struct vout_data *vout = video_get_drvdata(dev);

	return videobuf_mmap_mapper(&vout->vidq, vma);
}

static const struct v4l2_ioctl_ops mxc_ioctl_ops = {
	.vidioc_querycap		= vidioc_querycap,
	.vidioc_g_fmt_vid_out		= vidioc_g_fmt_vid_out,
	.vidioc_s_fmt_vid_out		= vidioc_s_fmt_vid_out,
	.vidioc_g_fmt_vid_overlay	= vidioc_g_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay	= vidioc_s_fmt_vid_overlay,
	.vidioc_enum_fmt_vid_out	= vidioc_enum_fmt_vid_out,
	.vidioc_reqbufs			= vidioc_reqbufs,
	.vidioc_querybuf		= vidioc_querybuf,
	.vidioc_qbuf			= vidioc_qbuf,
	.vidioc_dqbuf			= vidioc_dqbuf,
	.vidioc_streamon		= vidioc_streamon,
	.vidioc_streamoff		= vidioc_streamoff,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
	.vidiocgmbuf			= vidiocgmbuf,
#endif
};

static struct v4l2_file_operations mxc_v4l2out_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc_v4l2out_open,
	.release	= mxc_v4l2out_close,
	.ioctl		= video_ioctl2,
	.mmap		= mxc_v4l2out_mmap,
};

static struct vout_data *vout_register_new(struct fb_info *info)
{
	struct vout_data *vout;
	int ret;

	vout = kzalloc(sizeof(struct vout_data), GFP_KERNEL);
	if (!vout)
		return NULL;

	vout->video_dev = video_device_alloc();
	if (!vout->video_dev) {
		ret = -ENOMEM;
		goto failed_vdev_alloc;
	}

	vout->video_dev->minor = -1;

	strcpy(vout->video_dev->name, "vout");
	vout->video_dev->fops = &mxc_v4l2out_fops;
	vout->video_dev->ioctl_ops = &mxc_ioctl_ops;
	vout->video_dev->release = video_device_release;

	ret = video_register_device(vout->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(&vout_dev.pdev->dev, "register failed with %d\n", ret);
		goto failed_register;
	}

	video_set_drvdata(vout->video_dev, vout);

	vout->fbinfo = info;

	return vout;

failed_register:
	kfree(vout->video_dev);
failed_vdev_alloc:
	kfree(vout);
	return NULL;
}

static LIST_HEAD(vouts);

static int fb_event_notify(struct notifier_block *self,
	unsigned long action, void *data)
{
	struct fb_event *event = data;
	struct fb_info *info = event->info;
	struct vout_data *vout;

	switch (action) {
	case FB_EVENT_FB_REGISTERED:

		vout = vout_register_new(info);
		if (!vout)
			printk("register failed\n");
		list_add(&vout->vout_list, &vouts);
		break;
	case FB_EVENT_FB_UNREGISTERED:
		break;
	}
	return 0;
}

static struct notifier_block fb_event_notifier = {
	.notifier_call = fb_event_notify,
};

static int mxc_v4l2out_probe(struct platform_device *pdev)
{
	int ret;
	struct vout_device *vdev = &vout_dev;

	vdev->clk = clk_get(NULL, "emma");
	if (IS_ERR(vdev->clk)) {
		ret = PTR_ERR(vdev->clk);
		goto failed_clk;
	}

	clk_enable(vdev->clk);

	spin_lock_init(&vdev->irqlock);
	INIT_LIST_HEAD(&vdev->queued);

	vdev->irq = INT_EMMAPP;
	ret = request_irq(vdev->irq, pp_isr, 0, dev_name(&pdev->dev), vdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		goto failed_irq;
	}

	pphw_reset(vdev);

	platform_set_drvdata(pdev, vdev);

	ret = fb_register_client(&fb_event_notifier);
	if (ret)
		goto failed_register;

	return 0;

failed_register:
	free_irq(vdev->irq, vdev);
failed_irq:
	clk_disable(vdev->clk);
	clk_put(vdev->clk);
failed_clk:
	kfree(vdev);

	return ret;
}

static int mxc_v4l2out_remove(struct platform_device *pdev)
{
	struct vout_data *vout = platform_get_drvdata(pdev);

	free_irq(vout->irq, vout);

	video_unregister_device(vout->video_dev);

	fb_unregister_client(&fb_event_notifier);

	clk_disable(vout->clk);
	clk_put(vout->clk);

	kfree(vout);

	return 0;
}

static struct platform_driver mxc_v4l2out_driver = {
	.driver = {
		   .name = "MXC Video Output",
		   .owner = THIS_MODULE,
		   .bus = &platform_bus_type,
	},
	.probe = mxc_v4l2out_probe,
	.remove = mxc_v4l2out_remove,
};

static void camera_platform_release(struct device *device)
{
}

static struct platform_device mxc_v4l2out_device = {
	.name = "MXC Video Output",
	.dev = {
		.release = camera_platform_release,
		},
	.id = 0,
};

static int mxc_v4l2out_init(void)
{
	u8 err = 0;

	err = platform_driver_register(&mxc_v4l2out_driver);
	if (err == 0) {
		platform_device_register(&mxc_v4l2out_device);
	}
	return err;
}

static void mxc_v4l2out_clean(void)
{
	platform_driver_unregister(&mxc_v4l2out_driver);
	platform_device_unregister(&mxc_v4l2out_device);
}

module_init(mxc_v4l2out_init);
module_exit(mxc_v4l2out_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("V4L2-driver for MXC video output");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("video");
