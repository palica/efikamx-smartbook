/*
 * Copyright (C) 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/rtc.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/sched.h>

#define DRIVER_NAME "imx-srtc"

#define SRTC_LPSCLR_LLPSC_LSH	17	 /* start bit for LSB time value */

#define SRTC_LPPDR_INIT       0x41736166	/* init for glitch detect */

#define SRTC_LPCR_SWR_LP      (1 << 0)	/* lp software reset */
#define SRTC_LPCR_EN_LP       (1 << 3)	/* lp enable */
#define SRTC_LPCR_WAE         (1 << 4)	/* lp wakeup alarm enable */
#define SRTC_LPCR_SAE         (1 << 5)	/* lp security alarm enable */
#define SRTC_LPCR_SI          (1 << 6)	/* lp security interrupt enable */
#define SRTC_LPCR_ALP         (1 << 7)	/* lp alarm flag */
#define SRTC_LPCR_LTC         (1 << 8)	/* lp lock time counter */
#define SRTC_LPCR_LMC         (1 << 9)	/* lp lock monotonic counter */
#define SRTC_LPCR_SV          (1 << 10)	/* lp security violation */
#define SRTC_LPCR_NSA         (1 << 11)	/* lp non secure access */
#define SRTC_LPCR_NVEIE       (1 << 12)	/* lp non valid state exit int en */
#define SRTC_LPCR_IEIE        (1 << 13)	/* lp init state exit int enable */
#define SRTC_LPCR_NVE         (1 << 14)	/* lp non valid state exit bit */
#define SRTC_LPCR_IE          (1 << 15)	/* lp init state exit bit */

#define SRTC_LPCR_ALL_INT_EN (SRTC_LPCR_WAE | SRTC_LPCR_SAE | \
			      SRTC_LPCR_SI | SRTC_LPCR_ALP | \
			      SRTC_LPCR_NVEIE | SRTC_LPCR_IEIE)

#define SRTC_LPSR_TRI         (1 << 0)	/* lp time read invalidate */
#define SRTC_LPSR_PGD         (1 << 1)	/* lp power supply glitc detected */
#define SRTC_LPSR_CTD         (1 << 2)	/* lp clock tampering detected */
#define SRTC_LPSR_ALP         (1 << 3)	/* lp alarm flag */
#define SRTC_LPSR_MR          (1 << 4)	/* lp monotonic counter rollover */
#define SRTC_LPSR_TR          (1 << 5)	/* lp time rollover */
#define SRTC_LPSR_EAD         (1 << 6)	/* lp external alarm detected */
#define SRTC_LPSR_IT0         (1 << 7)	/* lp IIM throttle */
#define SRTC_LPSR_IT1         (1 << 8)
#define SRTC_LPSR_IT2         (1 << 9)
#define SRTC_LPSR_SM0         (1 << 10)	/* lp security mode */
#define SRTC_LPSR_SM1         (1 << 11)
#define SRTC_LPSR_STATE_LP0   (1 << 12)	/* lp state */
#define SRTC_LPSR_STATE_LP1   (1 << 13)
#define SRTC_LPSR_NVES        (1 << 14)	/* lp non-valid state exit status */
#define SRTC_LPSR_IES         (1 << 15)	/* lp init state exit status */

#define MAX_PIE_NUM     15
#define MAX_PIE_FREQ    32768
#define MIN_PIE_FREQ	1

#define SRTC_PI0         (1 << 0)
#define SRTC_PI1         (1 << 1)
#define SRTC_PI2         (1 << 2)
#define SRTC_PI3         (1 << 3)
#define SRTC_PI4         (1 << 4)
#define SRTC_PI5         (1 << 5)
#define SRTC_PI6         (1 << 6)
#define SRTC_PI7         (1 << 7)
#define SRTC_PI8         (1 << 8)
#define SRTC_PI9         (1 << 9)
#define SRTC_PI10        (1 << 10)
#define SRTC_PI11        (1 << 11)
#define SRTC_PI12        (1 << 12)
#define SRTC_PI13        (1 << 13)
#define SRTC_PI14        (1 << 14)
#define SRTC_PI15        (1 << 15)

#define PIT_ALL_ON      (SRTC_PI1 | SRTC_PI2 | SRTC_PI3 | \
			SRTC_PI4 | SRTC_PI5 | SRTC_PI6 | SRTC_PI7 | \
			SRTC_PI8 | SRTC_PI9 | SRTC_PI10 | SRTC_PI11 | \
			SRTC_PI12 | SRTC_PI13 | SRTC_PI14 | SRTC_PI15)

#define SRTC_SWR_HP      (1 << 0)	/* hp software reset */
#define SRTC_EN_HP       (1 << 3)	/* hp enable */
#define SRTC_TS          (1 << 4)	/* time syncronize hp with lp */

#define SRTC_IE_AHP      (1 << 16)	/* Alarm HP Interrupt Enable bit */
#define SRTC_IE_WDHP     (1 << 18)	/* Write Done HP Interrupt Enable bit */
#define SRTC_IE_WDLP     (1 << 19)	/* Write Done LP Interrupt Enable bit */

#define SRTC_ISR_AHP     (1 << 16)	/* interrupt status: alarm hp */
#define SRTC_ISR_WDHP    (1 << 18)	/* interrupt status: write done hp */
#define SRTC_ISR_WDLP    (1 << 19)	/* interrupt status: write done lp */
#define SRTC_ISR_WPHP    (1 << 20)	/* interrupt status: write pending hp */
#define SRTC_ISR_WPLP    (1 << 21)	/* interrupt status: write pending lp */

#define SRTC_LPSCMR	0x00	/* LP Secure Counter MSB Reg */
#define SRTC_LPSCLR	0x04	/* LP Secure Counter LSB Reg */
#define SRTC_LPSAR	0x08	/* LP Secure Alarm Reg */
#define SRTC_LPSMCR	0x0C	/* LP Secure Monotonic Counter Reg */
#define SRTC_LPCR	0x10	/* LP Control Reg */
#define SRTC_LPSR	0x14	/* LP Status Reg */
#define SRTC_LPPDR	0x18	/* LP Power Supply Glitch Detector Reg */
#define SRTC_LPGR	0x1C	/* LP General Purpose Reg */
#define SRTC_HPCMR	0x20	/* HP Counter MSB Reg */
#define SRTC_HPCLR	0x24	/* HP Counter LSB Reg */
#define SRTC_HPAMR	0x28	/* HP Alarm MSB Reg */
#define SRTC_HPALR	0x2C	/* HP Alarm LSB Reg */
#define SRTC_HPCR	0x30	/* HP Control Reg */
#define SRTC_HPISR	0x34	/* HP Interrupt Status Reg */
#define SRTC_HPIENR	0x38	/* HP Interrupt Enable Reg */

#define SRTC_SECMODE_MASK	0x3	/* the mask of SRTC security mode */
#define SRTC_SECMODE_LOW	0x0	/* Low Security */
#define SRTC_SECMODE_MED	0x1	/* Medium Security */
#define SRTC_SECMODE_HIGH	0x2	/* High Security */
#define SRTC_SECMODE_RESERVED	0x3	/* Reserved */

struct rtc_drv_data {
	spinlock_t lock;
	struct rtc_device *rtc;
	void __iomem *base;
	int irq;
	struct clk *clk;
};

/*
 * This function does write synchronization for writes to the lp srtc block.
 * To take care of the asynchronous CKIL clock, all writes from the IP domain
 * will be synchronized to the CKIL domain.
 */
static inline void rtc_write_sync_lp(void __iomem *base)
{
	unsigned int i, count;

	/* Wait for 3 CKIL cycles */
	for (i = 0; i < 3; i++) {
		count = __raw_readl(base + SRTC_LPSCLR);
		while
			((__raw_readl(base + SRTC_LPSCLR)) == count);
	}
}

/*
 * This function updates the RTC alarm registers and then clears all the
 * interrupt status bits.
 *
 * @param  alrm         the new alarm value to be updated in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int rtc_update_alarm(struct device *dev, struct rtc_time *alrm)
{
	struct rtc_drv_data *imxrtc = dev_get_drvdata(dev);
	void __iomem *base = imxrtc->base;
	struct rtc_time alarm_tm, now_tm;
	unsigned long now, time;
	int ret;

	now = __raw_readl(base + SRTC_LPSCMR);
	rtc_time_to_tm(now, &now_tm);

	alarm_tm.tm_year = now_tm.tm_year;
	alarm_tm.tm_mon = now_tm.tm_mon;
	alarm_tm.tm_mday = now_tm.tm_mday;
	alarm_tm.tm_hour = alrm->tm_hour;
	alarm_tm.tm_min = alrm->tm_min;
	alarm_tm.tm_sec = alrm->tm_sec;

	ret = rtc_tm_to_time(&alarm_tm, &time);

	__raw_writel(time, base + SRTC_LPSAR);

	/* clear alarm interrupt status bit */
	__raw_writel(SRTC_LPSR_ALP, base + SRTC_LPSR);

	return ret;
}

/*
 * This function is the RTC interrupt service routine.
 *
 * @param  irq          RTC IRQ number
 * @param  dev_id       device ID which is not used
 *
 * @return IRQ_HANDLED as defined in the include/linux/interrupt.h file.
 */
static irqreturn_t imx_srtc_interrupt(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct rtc_drv_data *imxrtc = platform_get_drvdata(pdev);
	void __iomem *base = imxrtc->base;
	u32 lp_status, lp_cr;
	u32 events = 0;

	lp_status = __raw_readl(base + SRTC_LPSR);
	lp_cr = __raw_readl(base + SRTC_LPCR);

	/* update irq data & counter */
	if (lp_status & SRTC_LPSR_ALP) {
		if (lp_cr & SRTC_LPCR_ALP)
			events |= (RTC_AF | RTC_IRQF);

		/* disable further lp alarm interrupts */
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);
	}

	/* Update interrupt enables */
	__raw_writel(lp_cr, base + SRTC_LPCR);

	/* clear interrupt status */
	__raw_writel(lp_status, base + SRTC_LPSR);

	rtc_write_sync_lp(base);
	rtc_update_irq(imxrtc->rtc, 1, events);
	return IRQ_HANDLED;
}

/*
 * This function reads the current RTC time into tm in Gregorian date.
 *
 * @param  tm           contains the RTC time value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int imx_srtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *imxrtc = dev_get_drvdata(dev);
	void __iomem *base = imxrtc->base;

	rtc_time_to_tm(__raw_readl(base + SRTC_LPSCMR), tm);
	return 0;
}

/*
 * This function sets the internal RTC time based on tm in Gregorian date.
 *
 * @param  tm           the time value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int imx_srtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_drv_data *imxrtc = dev_get_drvdata(dev);
	void __iomem *base = imxrtc->base;
	unsigned long time;
	int ret;

	ret = rtc_tm_to_time(tm, &time);
	if (ret)
		return ret;

	__raw_writel(time, base + SRTC_LPSCMR);
	rtc_write_sync_lp(base);

	return 0;
}

/*
 * This function reads the current alarm value into the passed in \b alrm
 * argument. It updates the \b alrm's pending field value based on the whether
 * an alarm interrupt occurs or not.
 *
 * @param  alrm         contains the RTC alarm value upon return
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int imx_srtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *imxrtc = dev_get_drvdata(dev);
	void __iomem *base = imxrtc->base;

	rtc_time_to_tm(__raw_readl(base + SRTC_LPSAR), &alrm->time);
	alrm->pending = (__raw_readl(base + SRTC_LPSR) & SRTC_LPSR_ALP) ? 1 : 0;

	return 0;
}

/*
 * This function sets the RTC alarm based on passed in alrm.
 *
 * @param  alrm         the alarm value to be set in the RTC
 *
 * @return  0 if successful; non-zero otherwise.
 */
static int imx_srtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_drv_data *imxrtc = dev_get_drvdata(dev);
	void __iomem *base = imxrtc->base;
	unsigned long lock_flags = 0;
	u32 lp_cr;
	int ret;

	spin_lock_irqsave(&imxrtc->lock, lock_flags);
	lp_cr = __raw_readl(base + SRTC_LPCR);

	ret = rtc_update_alarm(dev, &alrm->time);
	if (ret)
		goto out;

	if (alrm->enabled)
		lp_cr |= (SRTC_LPCR_ALP | SRTC_LPCR_WAE);
	else
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);

	__raw_writel(lp_cr, base + SRTC_LPCR);

out:
	rtc_write_sync_lp(base);
	spin_unlock_irqrestore(&imxrtc->lock, lock_flags);
	return ret;
}

static int imx_srtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	struct rtc_drv_data *imxrtc = dev_get_drvdata(dev);
	void __iomem *base = imxrtc->base;
	u32 lp_cr;
	unsigned long lock_flags = 0;

	spin_lock_irqsave(&imxrtc->lock, lock_flags);

	lp_cr = __raw_readl(base + SRTC_LPCR);

	if (enable)
		lp_cr |= SRTC_LPCR_ALP | SRTC_LPCR_WAE;
	else
		lp_cr &= ~(SRTC_LPCR_ALP | SRTC_LPCR_WAE);

	__raw_writel(lp_cr, base + SRTC_LPCR);

	rtc_write_sync_lp(base);
	spin_unlock_irqrestore(&imxrtc->lock, lock_flags);
	return 0;
}

static struct rtc_class_ops imx_srtc_ops = {
	.read_time = imx_srtc_read_time,
	.set_time = imx_srtc_set_time,
	.read_alarm = imx_srtc_read_alarm,
	.set_alarm = imx_srtc_set_alarm,
	.alarm_irq_enable = imx_srtc_alarm_irq_enable,
};

static int imx_srtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct rtc_device *rtc;
	struct rtc_drv_data *imxrtc;
	int irq;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	imxrtc = devm_kzalloc(&pdev->dev, sizeof(*imxrtc), GFP_KERNEL);
	if (!imxrtc)
		return -ENOMEM;

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, irq, imx_srtc_interrupt, 0,
			pdev->name, pdev);
	if (ret)
		return ret;

	imxrtc->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(imxrtc->clk))
		return PTR_ERR(imxrtc->clk);

	clk_prepare_enable(imxrtc->clk);

	res = devm_request_mem_region(&pdev->dev, res->start,
			resource_size(res), DRIVER_NAME);
	if (!res)
		return -ENOENT;

	imxrtc->base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!imxrtc->base)
		return -EINVAL;

	spin_lock_init(&imxrtc->lock);

	/* initialize glitch detect */
	__raw_writel(SRTC_LPPDR_INIT, imxrtc->base + SRTC_LPPDR);
	udelay(100);

	/* clear lp interrupt status */
	__raw_writel(0xFFFFFFFF, imxrtc->base + SRTC_LPSR);
	udelay(100);

	/* move out of init state */
	__raw_writel(SRTC_LPCR_IE | SRTC_LPCR_NSA, imxrtc->base + SRTC_LPCR);

	udelay(100);

	while ((__raw_readl(imxrtc->base + SRTC_LPSR) & SRTC_LPSR_IES) == 0)
		;

	/* move out of non-valid state */
	__raw_writel((SRTC_LPCR_IE | SRTC_LPCR_NVE | SRTC_LPCR_NSA |
		      SRTC_LPCR_EN_LP), imxrtc->base + SRTC_LPCR);

	udelay(100);

	while ((__raw_readl(imxrtc->base + SRTC_LPSR) & SRTC_LPSR_NVES) == 0)
		;

	__raw_writel(0xFFFFFFFF, imxrtc->base + SRTC_LPSR);
	udelay(100);

	platform_set_drvdata(pdev, imxrtc);
	rtc = rtc_device_register(pdev->name, &pdev->dev,
				  &imx_srtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		goto err_out;
	}

	imxrtc->rtc = rtc;

	/* By default, devices should wakeup if they can */
	/* So srtc is set as "should wakeup" as it can */
	device_init_wakeup(&pdev->dev, 1);

	return 0;

err_out:
	clk_disable_unprepare(imxrtc->clk);
	clk_put(imxrtc->clk);

	return ret;
}

static int __exit imx_srtc_remove(struct platform_device *pdev)
{
	struct rtc_drv_data *imxrtc = platform_get_drvdata(pdev);

	rtc_device_unregister(imxrtc->rtc);

	clk_disable_unprepare(imxrtc->clk);
	clk_put(imxrtc->clk);

	return 0;
}

/*
 * This function is called to save the system time delta relative to
 * the IMX SRTC when entering a low power state. This time delta is
 * then used on resume to adjust the system time to account for time
 * loss while suspended.
 */
static int imx_srtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtc_drv_data *imxrtc = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(imxrtc->irq);

	return 0;
}

/*
 * This function is called to correct the system time based on the
 * current IMX SRTC time relative to the time delta saved during
 * suspend.
 */
static int imx_srtc_resume(struct platform_device *pdev)
{
	struct rtc_drv_data *imxrtc = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(imxrtc->irq);

	return 0;
}

static struct platform_driver imx_srtc_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = imx_srtc_probe,
	.remove = __exit_p(imx_srtc_remove),
	.suspend = imx_srtc_suspend,
	.resume = imx_srtc_resume,
};

module_platform_driver(imx_srtc_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Secure Realtime Clock Driver (SRTC)");
MODULE_LICENSE("GPL");
