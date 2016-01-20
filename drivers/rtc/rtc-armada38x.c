/*
 * RTC driver for the Armada 38x Marvell SoCs
 *
 * Copyright (C) 2015 Marvell
 *
 * Gregory Clement <gregory.clement@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

#define RTC_STATUS	    0x0
#define RTC_STATUS_ALARM1	    BIT(0)
#define RTC_STATUS_ALARM2	    BIT(1)
#define RTC_IRQ1_CONF	    0x4
#define RTC_IRQ1_AL_EN		    BIT(0)
#define RTC_IRQ1_FREQ_EN	    BIT(1)
#define RTC_IRQ1_FREQ_1HZ	    BIT(2)
#define RTC_IRQ2_CONF	    0x8
#define RTC_TIME	    0xC
#define RTC_ALARM1	    0x10
#define RTC_ALARM2	    0x14
#define RTC_CLOCK_CORR	    0x18
#define RTC_TEST_CONFIG	    0x1C

#define SOC_RTC_INTERRUPT   0x8
#define SOC_RTC_ALARM1		BIT(0)
#define SOC_RTC_ALARM2		BIT(1)
#define SOC_RTC_ALARM1_MASK	BIT(2)
#define SOC_RTC_ALARM2_MASK	BIT(3)

#define RTC_NOMINAL_TIMING 0x0
#define RTC_SZ_STATUS_ALARM1_MASK	0x1
#define RTC_SZ_STATUS_ALARM2_MASK	0x2

struct armada38x_rtc {
	struct rtc_device   *rtc_dev;
	void __iomem	    *regs;
	void __iomem	    *regs_soc;
	spinlock_t	    lock;
	int		    irq;
};

/*
 * According to the datasheet, the OS should wait 5us after every
 * register write to the RTC hard macro so that the required update
 * can occur without holding off the system bus
 */
static void rtc_delayed_write(u32 val, struct armada38x_rtc *rtc, int offset)
{
	writel(val, rtc->regs + offset);
	udelay(5);
}

static int armada38x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct armada38x_rtc *rtc = dev_get_drvdata(dev);
	unsigned long time, time_check, flags;

	spin_lock_irqsave(&rtc->lock, flags);
	time = readl(rtc->regs + RTC_TIME);
	/*
	 * WA for failing time set attempts. As stated in HW ERRATA if
	 * more than one second between two time reads is detected
	 * then read once again.
	 */
	time_check = readl(rtc->regs + RTC_TIME);
	if ((time_check - time) > 1)
		time_check = readl(rtc->regs + RTC_TIME);

	spin_unlock_irqrestore(&rtc->lock, flags);

	//printk("armada38x_rtc_read_time 0x%x\n", time_check);

	rtc_time_to_tm(time_check, tm);

	return 0;
}

static int armada38x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct armada38x_rtc *rtc = dev_get_drvdata(dev);
	int ret = 0;
	unsigned long time, flags;

	ret = rtc_tm_to_time(tm, &time);

	if (ret)
		goto out;

	//printk("armada38x_rtc_set_time 0x%x\n", time);
	/*
	 * According to errata FE-3124064, Write to RTC TIME register
	 * may fail. As a workaround, after writing to RTC TIME
	 * register, issue a dummy write of 0x0 twice to RTC Status
	 * register.
	 */
	spin_lock_irqsave(&rtc->lock, flags);
	rtc_delayed_write(time, rtc, RTC_TIME);
	rtc_delayed_write(0, rtc, RTC_STATUS);
	rtc_delayed_write(0, rtc, RTC_STATUS);
	spin_unlock_irqrestore(&rtc->lock, flags);

out:
	return ret;
}

static bool rtc_init_state(struct armada38x_rtc *rtc)
{
	uint32_t stat, alrm1, alrm2, int1, int2, tstcfg;

	/* Update RTC-MBUS bridge timing parameters */
	writel(0xFD4D4CFA, rtc->regs_soc);

	/* Make sure we are not in any test mode */
	writel(0, rtc->regs + RTC_TEST_CONFIG);
	msleep_interruptible(500);

	/* Setup nominal register access timing */
	writel(RTC_NOMINAL_TIMING, rtc->regs + RTC_CLOCK_CORR);

	/* Turn off Int1 sources & clear the Alarm count */
	writel(0, rtc->regs + RTC_IRQ1_CONF);
	writel(0, rtc->regs + RTC_ALARM1);

	/* Turn off Int2 sources & clear the Periodic count */
	writel(0, rtc->regs + RTC_IRQ2_CONF);
	writel(0, rtc->regs + RTC_ALARM2);

	/* Clear any pending Status bits */
	writel((RTC_SZ_STATUS_ALARM1_MASK | RTC_SZ_STATUS_ALARM2_MASK), rtc->regs + RTC_STATUS);

	stat   = readl(rtc->regs + RTC_STATUS) & 0xFF;
	alrm1  = readl(rtc->regs + RTC_ALARM1);
	int1   = readl(rtc->regs + RTC_IRQ1_CONF) & 0xFF;
	alrm2  = readl(rtc->regs + RTC_ALARM2);
	int2   = readl(rtc->regs + RTC_IRQ2_CONF) & 0xFF;
	tstcfg = readl(rtc->regs + RTC_TEST_CONFIG) & 0xFF;

	if ((0xFC == stat) && (0 == alrm1) && (0xC0 == int1) &&
	    (0 == alrm2) && (0xC0 == int2) && (0 == tstcfg)) {
		return true;
	} else {
		return false;
	}
}

static int armada38x_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct armada38x_rtc *rtc = dev_get_drvdata(dev);
	unsigned long time, flags;
	u32 val;

	spin_lock_irqsave(&rtc->lock, flags);

	time = readl(rtc->regs + RTC_ALARM1);
	val = readl(rtc->regs + RTC_IRQ1_CONF) & RTC_IRQ1_AL_EN;

	spin_unlock_irqrestore(&rtc->lock, flags);

	alrm->enabled = val ? 1 : 0;
	rtc_time_to_tm(time,  &alrm->time);

	return 0;
}

static int armada38x_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct armada38x_rtc *rtc = dev_get_drvdata(dev);
	unsigned long time, flags;
	int ret = 0;
	u32 val;

	ret = rtc_tm_to_time(&alrm->time, &time);

	if (ret)
		goto out;

	spin_lock_irqsave(&rtc->lock, flags);

	rtc_delayed_write(time, rtc, RTC_ALARM1);

	if (alrm->enabled) {
			rtc_delayed_write(RTC_IRQ1_AL_EN, rtc, RTC_IRQ1_CONF);
			val = readl(rtc->regs_soc + SOC_RTC_INTERRUPT);
			writel(val | SOC_RTC_ALARM1_MASK,
			       rtc->regs_soc + SOC_RTC_INTERRUPT);
	}

	spin_unlock_irqrestore(&rtc->lock, flags);

out:
	return ret;
}

static int armada38x_rtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct armada38x_rtc *rtc = dev_get_drvdata(dev);
	unsigned long flags;

	spin_lock_irqsave(&rtc->lock, flags);

	if (enabled)
		rtc_delayed_write(RTC_IRQ1_AL_EN, rtc, RTC_IRQ1_CONF);
	else
		rtc_delayed_write(0, rtc, RTC_IRQ1_CONF);

	spin_unlock_irqrestore(&rtc->lock, flags);

	return 0;
}

static irqreturn_t armada38x_rtc_alarm_irq(int irq, void *data)
{
	struct armada38x_rtc *rtc = data;
	u32 val;
	int event = RTC_IRQF | RTC_AF;

	dev_dbg(&rtc->rtc_dev->dev, "%s:irq(%d)\n", __func__, irq);

	spin_lock(&rtc->lock);

	val = readl(rtc->regs_soc + SOC_RTC_INTERRUPT);

	writel(val & ~SOC_RTC_ALARM1, rtc->regs_soc + SOC_RTC_INTERRUPT);
	val = readl(rtc->regs + RTC_IRQ1_CONF);
	/* disable all the interrupts for alarm 1 */
	rtc_delayed_write(0, rtc, RTC_IRQ1_CONF);
	/* Ack the event */
	rtc_delayed_write(RTC_STATUS_ALARM1, rtc, RTC_STATUS);

	spin_unlock(&rtc->lock);

	if (val & RTC_IRQ1_FREQ_EN) {
		if (val & RTC_IRQ1_FREQ_1HZ)
			event |= RTC_UF;
		else
			event |= RTC_PF;
	}

	rtc_update_irq(rtc->rtc_dev, 1, event);

	return IRQ_HANDLED;
}

static struct rtc_class_ops armada38x_rtc_ops = {
	.read_time = armada38x_rtc_read_time,
	.set_time = armada38x_rtc_set_time,
	.read_alarm = armada38x_rtc_read_alarm,
	.set_alarm = armada38x_rtc_set_alarm,
	.alarm_irq_enable = armada38x_rtc_alarm_irq_enable,
};

static __init int armada38x_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct armada38x_rtc *rtc;
	int ret;
	uint32_t init_count = 3;

	rtc = devm_kzalloc(&pdev->dev, sizeof(struct armada38x_rtc),
			    GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	spin_lock_init(&rtc->lock);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rtc");
	rtc->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtc->regs))
		return PTR_ERR(rtc->regs);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rtc-soc");
	rtc->regs_soc = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtc->regs_soc))
		return PTR_ERR(rtc->regs_soc);

	while (!rtc_init_state(rtc) && --init_count);

	if (init_count == 0) {
		dev_err(&pdev->dev, "%probe: Error in rtc_init_state\n", __func__);
		return -ENODEV;
	}

	rtc->irq = platform_get_irq(pdev, 0);

	if (rtc->irq < 0) {
		dev_err(&pdev->dev, "no irq\n");
		return rtc->irq;
	}
	if (devm_request_irq(&pdev->dev, rtc->irq, armada38x_rtc_alarm_irq,
				0, pdev->name, rtc) < 0) {
		dev_warn(&pdev->dev, "Interrupt not available.\n");
		rtc->irq = -1;
		/*
		 * If there is no interrupt available then we can't
		 * use the alarm
		 */
		armada38x_rtc_ops.set_alarm = NULL;
		armada38x_rtc_ops.alarm_irq_enable = NULL;
	}
	platform_set_drvdata(pdev, rtc);
	if (rtc->irq != -1)
		device_init_wakeup(&pdev->dev, 1);

	rtc->rtc_dev = devm_rtc_device_register(&pdev->dev, pdev->name,
					&armada38x_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc_dev)) {
		ret = PTR_ERR(rtc->rtc_dev);
		dev_err(&pdev->dev, "Failed to register RTC device: %d\n", ret);
		return ret;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int armada38x_rtc_suspend(struct device *dev)
{
	if (device_may_wakeup(dev)) {
		struct armada38x_rtc *rtc = dev_get_drvdata(dev);

		return enable_irq_wake(rtc->irq);
	}

	return 0;
}

static int armada38x_rtc_resume(struct device *dev)
{
	if (device_may_wakeup(dev)) {
		struct armada38x_rtc *rtc = dev_get_drvdata(dev);

		return disable_irq_wake(rtc->irq);
	}

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(armada38x_rtc_pm_ops,
			 armada38x_rtc_suspend, armada38x_rtc_resume);

#ifdef CONFIG_OF
static const struct of_device_id armada38x_rtc_of_match_table[] = {
	{ .compatible = "marvell,armada-380-rtc", },
	{}
};
MODULE_DEVICE_TABLE(of, armada38x_rtc_of_match_table);
#endif

static struct platform_driver armada38x_rtc_driver = {
	.driver		= {
		.name	= "armada38x-rtc",
		.pm	= &armada38x_rtc_pm_ops,
		.of_match_table = of_match_ptr(armada38x_rtc_of_match_table),
	},
};

module_platform_driver_probe(armada38x_rtc_driver, armada38x_rtc_probe);

MODULE_DESCRIPTION("Marvell Armada 38x RTC driver");
MODULE_AUTHOR("Gregory CLEMENT <gregory.clement@free-electrons.com>");
MODULE_LICENSE("GPL");
