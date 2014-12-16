/*
 * Copyright (C) 2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/bitops.h>

/***********************************************************************
 * Definitions
 ***********************************************************************/
#define IREG_BBL_RTC_PER		0x00000000
#define IREG_BBL_RTC_MATCH		0x00000004
#define IREG_BBL_RTC_DIV		0x00000008
#define IREG_BBL_RTC_SECOND		0x0000000C
#define IREG_BBL_INTERRUPT_EN		0x00000010
#define IREG_BBL_INTERRUPT_STAT		0x00000014
#define IREG_BBL_INTERRUPT_CLR		0x00000018
#define IREG_BBL_CONTROL		0x0000001C
#define IREG_BBL_ADDR_MASK		0x3FF

#define BBL_PER_1s	0x00000008

#define CRMU_AUTH_CODE_PWD	0x12345678
#define CRMU_AUTH_CODE_PWD_RST	0x99999999
#define CRMU_AUTH_CODE_PWD_CLR	0x0

#define RTC_REG_ACC_DONE	BIT(0)
#define RTC_REG_RTC_STOP	BIT(0)
#define RTC_REG_PERIO_INTR	BIT(0)
#define RTC_REG_ALARM_INTR	BIT(1)
#define RTC_IND_SOFT_RST_N	BIT(10)
#define RTC_REG_WR_CMD		BIT(11)
#define RTC_REG_RD_CMD		BIT(12)
#define CRMU_ISO_PDBBL		BIT(16)
#define CRMU_ISO_PDBBL_TAMPER	BIT(24)

/* Timeout when waiting on register
reads or writes */
#define REG_TIMEOUT_MICROSECONDS 250

/*  SPRU Source Select status
	   0 - SPRU is powered by AON power
	   1 - SPRU is powerd by battery */
#define CRMU_SPRU_SOURCE_SEL_AON 0

struct rtc_regs_t {
	u32 SPRU_BBL_WDATA;
	u32 SPRU_BBL_CMD;
	u32 SPRU_BBL_STATUS;
	u32 SPRU_BBL_RDATA;
};

struct crmu_regs_t {
	u32 CRMU_PWR_GOOD_STATUS;
	u32 CRMU_POWER_REQ_CFG;
	u32 CRMU_POWER_POLL;
	u32 CRMU_ISO_CELL_CONTROL;
	u32 rsvd;
	u32 CRMU_SPRU_SOURCE_SEL_STAT;
};

struct bbl_auth_t {
	u32 CRMU_BBL_AUTH_CODE;
	u32 CRMU_BBL_AUTH_CHECK;
};

struct iproc_rtc_t {
	struct rtc_device *rtc;
	struct rtc_regs_t *regs;
	struct crmu_regs_t *crmu_regs;
	struct bbl_auth_t *auth_regs;
	int periodic_irq;
	int alarm_irq;
	spinlock_t lock;
};
/***********************************************************************
 *  End Definitions
 ***********************************************************************/

static inline int wait_acc_done(struct iproc_rtc_t *iproc_rtc)
{
	u32 reg_val;
	int timeout = REG_TIMEOUT_MICROSECONDS;

	reg_val = readl(&iproc_rtc->regs->SPRU_BBL_STATUS);
	while (!(reg_val & RTC_REG_ACC_DONE)) {
		if (--timeout == 0)
			return -EIO;
		udelay(1);
		reg_val = readl(&iproc_rtc->regs->SPRU_BBL_STATUS);
	}

	return 0;
}

static inline int rtc_reg_write(u32 reg_addr, u32 reg_val,
				struct iproc_rtc_t *iproc_rtc)
{
	u32 cmd;
	int ret;

	writel(reg_val, &iproc_rtc->regs->SPRU_BBL_WDATA);
	/* Write command */
	cmd = (reg_addr & IREG_BBL_ADDR_MASK) |
		RTC_REG_WR_CMD | RTC_IND_SOFT_RST_N;
	writel(cmd, &iproc_rtc->regs->SPRU_BBL_CMD);
	ret = wait_acc_done(iproc_rtc);
	if (ret < 0)
		pr_err("RTC: reg write to 0x%x failed!", reg_addr);

	return ret;
}

static inline int rtc_reg_read(u32 reg_addr, u32 *data,
				struct iproc_rtc_t *iproc_rtc)
{
	u32 cmd;
	int ret;

	/* Read command */
	cmd = (reg_addr & IREG_BBL_ADDR_MASK) |
		RTC_REG_RD_CMD | RTC_IND_SOFT_RST_N;
	writel(cmd, &iproc_rtc->regs->SPRU_BBL_CMD);
	ret = wait_acc_done(iproc_rtc);
	if (ret < 0)
		pr_err("RTC: reg read to 0x%x failed", reg_addr);
	else
		*data = readl(&iproc_rtc->regs->SPRU_BBL_RDATA);

	return ret;
}

static int bbl_init(struct iproc_rtc_t *iproc_rtc)
{
	u32 reg_val;
	int timeout = REG_TIMEOUT_MICROSECONDS;

	/* Check SPRU Source Select status
	   0 - SPRU is powered by AON power
	   1 - SPRU is powerd by battery */
	reg_val = readl(&iproc_rtc->crmu_regs->CRMU_SPRU_SOURCE_SEL_STAT);
	while (reg_val != CRMU_SPRU_SOURCE_SEL_AON) {
		if (--timeout == 0) {
			pr_info("RTC: BBL AON power not available\n");
			return -ENODEV;
		}
		udelay(1);
		reg_val = readl(
			&iproc_rtc->crmu_regs->CRMU_SPRU_SOURCE_SEL_STAT);
	}

	/* Wait for reset cycle */
	writel(0, &iproc_rtc->regs->SPRU_BBL_CMD);
	udelay(200);
	writel(RTC_IND_SOFT_RST_N, &iproc_rtc->regs->SPRU_BBL_CMD);

	/* Remove BBL related isolation from CRMU */
	reg_val = readl(&iproc_rtc->crmu_regs->CRMU_ISO_CELL_CONTROL);
	reg_val &= ~(CRMU_ISO_PDBBL | CRMU_ISO_PDBBL_TAMPER);
	writel(reg_val, &iproc_rtc->crmu_regs->CRMU_ISO_CELL_CONTROL);

	/* program CRMU auth_code resister */
	writel(CRMU_AUTH_CODE_PWD, &iproc_rtc->auth_regs->CRMU_BBL_AUTH_CODE);
	/* program CRMU auth_code_check register */
	/* auth_code must equal to auth_code_check */
	writel(CRMU_AUTH_CODE_PWD, &iproc_rtc->auth_regs->CRMU_BBL_AUTH_CHECK);

	return 0;
}

static void bbl_exit(struct iproc_rtc_t *iproc_rtc)
{
	u32 reg_val;

	/*- Set BBL related isolation from CRMU */
	reg_val = readl(&iproc_rtc->crmu_regs->CRMU_ISO_CELL_CONTROL);
	reg_val |= (CRMU_ISO_PDBBL | CRMU_ISO_PDBBL_TAMPER);
	writel(reg_val, &iproc_rtc->crmu_regs->CRMU_ISO_CELL_CONTROL);

	/* Change the AUTH CODE register so it does not match
	 * the AUTH_CHECK register */
	writel(CRMU_AUTH_CODE_PWD_CLR,
	       &iproc_rtc->auth_regs->CRMU_BBL_AUTH_CODE);
	writel(CRMU_AUTH_CODE_PWD_RST,
	       &iproc_rtc->auth_regs->CRMU_BBL_AUTH_CHECK);
}
static int iproc_rtc_enable(struct iproc_rtc_t *iproc_rtc)
{
	u32 reg_val;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&iproc_rtc->lock, flags);

	/* Set periodic timer as 1 second */
	ret = rtc_reg_write(IREG_BBL_RTC_PER, BBL_PER_1s, iproc_rtc);
	if (ret < 0)
		goto err;

	ret = rtc_reg_read(IREG_BBL_INTERRUPT_EN, &reg_val, iproc_rtc);
	if (ret < 0)
		goto err;

	/* Disable alarm&periodic interrupt */
	reg_val &= ~(RTC_REG_PERIO_INTR | RTC_REG_ALARM_INTR);

	ret = rtc_reg_write(IREG_BBL_INTERRUPT_EN, reg_val, iproc_rtc);
	if (ret < 0)
		goto err;

	ret = rtc_reg_write(IREG_BBL_INTERRUPT_CLR,
		RTC_REG_PERIO_INTR | RTC_REG_ALARM_INTR, iproc_rtc);
	if (ret < 0)
		goto err;

	reg_val |= RTC_REG_PERIO_INTR | RTC_REG_ALARM_INTR;
	/* enable RTC periodic interrupt */
	ret = rtc_reg_write(IREG_BBL_INTERRUPT_EN, reg_val, iproc_rtc);

err:
	spin_unlock_irqrestore(&iproc_rtc->lock, flags);
	return ret;
}

static int iproc_rtc_disable(struct iproc_rtc_t *iproc_rtc)
{
	u32 reg_val;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&iproc_rtc->lock, flags);

	ret = rtc_reg_read(IREG_BBL_INTERRUPT_EN, &reg_val, iproc_rtc);
	if (ret < 0)
		goto err;

	reg_val &= ~(RTC_REG_PERIO_INTR | RTC_REG_ALARM_INTR);
	/* To disable RTC periodic interrupt */
	ret = rtc_reg_write(IREG_BBL_INTERRUPT_EN, reg_val, iproc_rtc);

err:
	spin_unlock_irqrestore(&iproc_rtc->lock, flags);
	return ret;
}

static irqreturn_t iproc_rtc_interrupt(int irq, void *class_dev)
{
	unsigned long flags, events = 0;
	u32 reg_val, irq_flg;
	int ret = 0;
	struct iproc_rtc_t *iproc_rtc =  class_dev;

	spin_lock_irqsave(&iproc_rtc->lock, flags);
	ret = rtc_reg_read(IREG_BBL_INTERRUPT_STAT, &irq_flg, iproc_rtc);
	if (ret < 0) {
		pr_err("RTC: Interrupt Routine failed");
		goto err;
	}

	irq_flg &= RTC_REG_PERIO_INTR | RTC_REG_ALARM_INTR;
	if (!irq_flg)
		goto err;

	if (irq_flg & RTC_REG_PERIO_INTR) {
		/* Clear periodic interrupt status */
		ret = rtc_reg_write(IREG_BBL_INTERRUPT_CLR, RTC_REG_PERIO_INTR,
					iproc_rtc);
		if (ret < 0)
			pr_err("RTC: Failed to clear RTC periodic interrupt event");
		events |= RTC_IRQF | RTC_PF;
	}

	if (irq_flg & RTC_REG_ALARM_INTR) {
		/* Clear alarm interrupt status */
		ret = rtc_reg_write(IREG_BBL_INTERRUPT_CLR, RTC_REG_ALARM_INTR,
					iproc_rtc);
		if (ret < 0)
			pr_err("RTC: Failed to clear RTC Alarm interrupt event");
		events |= RTC_IRQF | RTC_AF;

		ret = rtc_reg_read(IREG_BBL_INTERRUPT_EN, &reg_val, iproc_rtc);
		if (ret < 0) {
			pr_err("RTC: Failed to disable RTC Alarm interrupt after clear");
			goto err;
		}
		reg_val &= ~RTC_REG_ALARM_INTR;
		 /* Disable Alarm interrupt */
		ret = rtc_reg_write(IREG_BBL_INTERRUPT_EN, reg_val, iproc_rtc);
		if (ret < 0) {
			pr_err("RTC: Failed to disable RTC Alarm interrupt after clear");
			goto err;
		}
	}

err:
	if (events)
		rtc_update_irq(iproc_rtc->rtc, 1, events);
	spin_unlock_irqrestore(&iproc_rtc->lock, flags);
	return IRQ_HANDLED;
}

static int iproc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags;
	u32 seconds;
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct iproc_rtc_t *iproc_rtc = platform_get_drvdata(pdev);

	spin_lock_irqsave(&iproc_rtc->lock, flags);

	ret = rtc_reg_read(IREG_BBL_RTC_SECOND, &seconds, iproc_rtc);
	if (ret < 0) {
		pr_err("RTC: iproc_rtc_read_time failed");
		goto err;
	}
	rtc_time_to_tm(seconds, tm);

	ret = rtc_valid_tm(tm);

err:
	spin_unlock_irqrestore(&iproc_rtc->lock, flags);
	return ret;
}

static int iproc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long flags, seconds;
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct iproc_rtc_t *iproc_rtc = platform_get_drvdata(pdev);

	rtc_tm_to_time(tm, &seconds);
	spin_lock_irqsave(&iproc_rtc->lock, flags);

	/* bbl_rtc_stop = 1, RTC hal */
	ret = rtc_reg_write(IREG_BBL_CONTROL, RTC_REG_RTC_STOP, iproc_rtc);
	if (ret < 0)
		goto err;
	/* Update DIV */
	ret = rtc_reg_write(IREG_BBL_RTC_DIV, 0, iproc_rtc);
	if (ret < 0)
		goto err;
	/* Update second */
	ret = rtc_reg_write(IREG_BBL_RTC_SECOND, seconds, iproc_rtc);
	if (ret < 0)
		goto err;
	/* bl_rtc_stop = 0, RTC release */
	ret = rtc_reg_write(IREG_BBL_CONTROL, ~RTC_REG_RTC_STOP, iproc_rtc);

err:
	spin_unlock_irqrestore(&iproc_rtc->lock, flags);
	if (ret < 0)
		pr_err("RTC: iproc_rtc_set_time failed");
	return ret;
}

static int iproc_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	unsigned long flags;
	u32 reg_val;
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct iproc_rtc_t *iproc_rtc = platform_get_drvdata(pdev);

	spin_lock_irqsave(&iproc_rtc->lock, flags);
	ret = rtc_reg_read(IREG_BBL_INTERRUPT_EN, &reg_val, iproc_rtc);
	if (ret < 0)
		goto err;

	if (enabled)
		reg_val |= RTC_REG_ALARM_INTR;
	else
		reg_val &= ~RTC_REG_ALARM_INTR;

	ret = rtc_reg_write(IREG_BBL_INTERRUPT_EN, reg_val, iproc_rtc);

err:
	spin_unlock_irqrestore(&iproc_rtc->lock, flags);
	if (ret < 0)
		pr_err("RTC: iproc_rtc_alarm_irq_enable failed");
	return ret;
}

static int iproc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long flags;
	u32 reg_val, seconds;
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct iproc_rtc_t *iproc_rtc = platform_get_drvdata(pdev);

	spin_lock_irqsave(&iproc_rtc->lock, flags);
	ret = rtc_reg_read(IREG_BBL_RTC_MATCH, &seconds, iproc_rtc);
	if (ret < 0)
		goto err;
	seconds = (seconds << 7);
	rtc_time_to_tm(seconds, &alm->time);
	ret = rtc_reg_read(IREG_BBL_INTERRUPT_EN, &reg_val, iproc_rtc);
	if (ret < 0)
		goto err;
	reg_val &= RTC_REG_ALARM_INTR;
	alm->pending = !reg_val;
	alm->enabled = alm->pending && device_may_wakeup(dev);

err:
	spin_unlock_irqrestore(&iproc_rtc->lock, flags);
	if (ret < 0)
		pr_err("RTC: iproc_rtc_read_alarm failed");
	return ret;
}

static int iproc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned long flags;
	unsigned long seconds;
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct iproc_rtc_t *iproc_rtc = platform_get_drvdata(pdev);

	spin_lock_irqsave(&iproc_rtc->lock, flags);
	rtc_tm_to_time(&alm->time, &seconds);
	seconds = ((seconds & (0xFFFF << 7)) >> 7) & 0xFFFF;
	ret = rtc_reg_write(IREG_BBL_RTC_MATCH, seconds, iproc_rtc);
	spin_unlock_irqrestore(&iproc_rtc->lock, flags);
	if (ret < 0)
		pr_err("RTC: iproc_rtc_set_alarm failed");

	return ret;
}

static struct rtc_class_ops iproc_rtc_ops = {
	.read_time		= iproc_rtc_read_time,
	.set_time		= iproc_rtc_set_time,
	.alarm_irq_enable	= iproc_rtc_alarm_irq_enable,
	.read_alarm		= iproc_rtc_read_alarm,
	.set_alarm		= iproc_rtc_set_alarm,
};

static const struct of_device_id iproc_rtc_of_match[] = {
	{.compatible = "brcm,iproc-rtc",},
	{ }
};

static int iproc_rtc_probe(struct platform_device *pdev)
{
	struct device_node *dev_of = pdev->dev.of_node;
	struct resource *res;
	int ret = 0;
	struct iproc_rtc_t *iproc_rtc;

	iproc_rtc = devm_kzalloc(&pdev->dev, sizeof(struct iproc_rtc_t),
				 GFP_KERNEL);
	spin_lock_init(&iproc_rtc->lock);

	iproc_rtc->periodic_irq = irq_of_parse_and_map(dev_of, 0);
	if (iproc_rtc->periodic_irq < 0) {
		dev_err(&pdev->dev, "no RTC periodic irq\n");
		ret = -ENODEV;
		goto fail;
	}

	iproc_rtc->alarm_irq = irq_of_parse_and_map(dev_of, 1);
	if (iproc_rtc->alarm_irq < 0) {
		dev_err(&pdev->dev, "no RTC alarm irq\n");
		ret = -ENODEV;
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "spru_bbl MEM resource missing\n");
		ret = -ENOMEM;
		goto fail;
	}
	iproc_rtc->regs = (struct rtc_regs_t *)
		devm_ioremap_resource(&pdev->dev, res);
	if (!iproc_rtc->regs) {
		dev_err(&pdev->dev, "unable to ioremap spru_bbl MEM resource\n");
		ret = -ENOMEM;
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res == NULL) {
		dev_err(&pdev->dev, "crmu_pwr_good MEM resource missing\n");
		ret = -ENOMEM;
		goto fail;
	}
	iproc_rtc->crmu_regs = (struct crmu_regs_t *)
		devm_ioremap_resource(&pdev->dev, res);
	if (!iproc_rtc->crmu_regs) {
		dev_err(&pdev->dev, "unable to ioremap crmu_pwr_good MEM resource\n");
		ret = -ENOMEM;
		goto fail;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res == NULL) {
		dev_err(&pdev->dev, "crmu_bbl_auth MEM resource missing\n");
		ret = -ENOMEM;
		goto fail;
	}
	iproc_rtc->auth_regs = (struct bbl_auth_t *)
		devm_ioremap_resource(&pdev->dev, res);
	if (!iproc_rtc->auth_regs) {
		dev_err(&pdev->dev, "unable to ioremap crmu_bbl_auth MEM resource\n");
		ret = -ENOMEM;
		goto fail;
	}

	platform_set_drvdata(pdev, iproc_rtc);

	ret = bbl_init(iproc_rtc);
	if (ret < 0)
		goto fail;

	ret = iproc_rtc_enable(iproc_rtc);
	if (ret < 0) {
		pr_err("RTC: Enable failed");
		goto fail_bbl;
	}

	ret = devm_request_irq
		(&pdev->dev, iproc_rtc->periodic_irq, iproc_rtc_interrupt,
		 IRQF_SHARED, "iproc_rtc_peri", iproc_rtc);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register iproc RTC periodic interrupt\n");
		goto fail_rtc;
	}

	ret = devm_request_irq
		(&pdev->dev, iproc_rtc->alarm_irq, iproc_rtc_interrupt, 0,
		"iproc_rtc_alarm", iproc_rtc);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register iproc RTC alarm interrupt\n");
		goto fail_rtc;
	}

	iproc_rtc->rtc =  devm_rtc_device_register(&pdev->dev, "iproc-rtc",
			&iproc_rtc_ops, THIS_MODULE);
	if (IS_ERR(iproc_rtc->rtc)) {
		dev_err(&pdev->dev, "unable to register RTC device, err %ld\n",
				PTR_ERR(iproc_rtc->rtc));
		ret = PTR_ERR(iproc_rtc->rtc);
		goto fail_rtc;
	}

	device_init_wakeup(&pdev->dev, 1);
	return 0;

fail_rtc:
	iproc_rtc_disable(iproc_rtc);
fail_bbl:
	bbl_exit(iproc_rtc);
fail:
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int iproc_rtc_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct iproc_rtc_t *iproc_rtc = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);
	rtc_device_unregister(iproc_rtc->rtc);
	ret = iproc_rtc_disable(iproc_rtc);
	if (ret < 0)
		pr_err("RTC: Disable failed");
	bbl_exit(iproc_rtc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver iproc_rtc_driver = {
	.probe		= iproc_rtc_probe,
	.remove		= iproc_rtc_remove,
	.driver		= {
		.name = "iproc-rtc",
		.of_match_table = iproc_rtc_of_match
	},
};

module_platform_driver(iproc_rtc_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom IPROC RTC Driver");
MODULE_LICENSE("GPL");
