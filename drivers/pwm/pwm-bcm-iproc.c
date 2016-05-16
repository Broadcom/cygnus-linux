/*
 * Copyright (C) 2016 Broadcom
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define IPROC_PWM_CTRL_OFFSET			(0x00)
#define IPROC_PWM_CTRL_TYPE_SHIFT(chan)		(15 + (chan))
#define IPROC_PWM_CTRL_POLARITY_SHIFT(chan)	(8 + (chan))
#define IPROC_PWM_CTRL_EN_SHIFT(chan)		(chan)

#define IPROC_PWM_PERIOD_OFFSET(chan)		(0x04 + ((chan) << 3))
#define IPROC_PWM_PERIOD_MIN			(0x02)
#define IPROC_PWM_PERIOD_MAX			(0xffff)

#define IPROC_PWM_DUTY_CYCLE_OFFSET(chan)	(0x08 + ((chan) << 3))
#define IPROC_PWM_DUTY_CYCLE_MIN		(0x00)
#define IPROC_PWM_DUTY_CYCLE_MAX		(0xffff)

#define IPROC_PWM_PRESCALE_OFFSET		(0x24)
#define IPROC_PWM_PRESCALE_BITS			(0x06)
#define IPROC_PWM_PRESCALE_SHIFT(chan)		((3 - (chan)) *	\
						IPROC_PWM_PRESCALE_BITS)
#define IPROC_PWM_PRESCALE_MASK(chan)		(IPROC_PWM_PRESCALE_MAX << \
						IPROC_PWM_PRESCALE_SHIFT(chan))
#define IPROC_PWM_PRESCALE_MIN			(0x00)
#define IPROC_PWM_PRESCALE_MAX			(0x3f)

#define IPROC_PWM_CHANNEL_COUNT			(0x04)

struct iproc_pwmc {
	struct pwm_chip chip;
	void __iomem *base;
	struct clk *clk;
};

static inline struct iproc_pwmc *to_iproc_pwmc(struct pwm_chip *_chip)
{
	return container_of(_chip, struct iproc_pwmc, chip);
}

static void iproc_pwmc_unset_enable_bit(struct iproc_pwmc *ip, u32 chan)
{
	unsigned int value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);

	value &= ~(1 << IPROC_PWM_CTRL_EN_SHIFT(chan));
	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);

	/*
	 * There must be a min 400ns delay between clearing trigger and setting
	 * it. Failing to do this may result in no PWM signal.
	 */
	ndelay(400);
}

static void iproc_pwmc_set_enable_bit(struct iproc_pwmc *ip, unsigned int chan)
{
	unsigned int value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);

	/* Set trigger bit to apply new settings */
	value |= 1 << IPROC_PWM_CTRL_EN_SHIFT(chan);
	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);

	/* Trigger bit must be held high for at least 400 ns. */
	ndelay(400);
}

static int iproc_pwmc_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	unsigned long prescale = IPROC_PWM_PRESCALE_MIN;
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	unsigned int value, chan = pwm->hwpwm;
	unsigned long period_cnt, duty_cnt;
	u64 val, div, rate;

	rate = clk_get_rate(ip->clk);
	if (!rate) {
		dev_err(pwm->chip->dev, "pwm clock has no frequency\n");
		return -EINVAL;
	}

	/*
	 * Find period count, duty count and prescale to suit duty_ns and
	 * period_ns. This is done according to formulas described below:
	 *
	 * period_ns = 10^9 * (PRESCALE + 1) * PC / PWM_CLK_RATE
	 * duty_ns = 10^9 * (PRESCALE + 1) * DC / PWM_CLK_RATE
	 *
	 * PC = (PWM_CLK_RATE * period_ns) / (10^9 * (PRESCALE + 1))
	 * DC = (PWM_CLK_RATE * duty_ns) / (10^9 * (PRESCALE + 1))
	 */
	while (1) {
		div = 1000000000;
		div *= 1 + prescale;
		val = rate * period_ns;
		period_cnt = div64_u64(val, div);
		val = rate * duty_ns;
		duty_cnt = div64_u64(val, div);

		if (period_cnt < IPROC_PWM_PERIOD_MIN ||
				duty_cnt < IPROC_PWM_DUTY_CYCLE_MIN)
			return -EINVAL;

		if (period_cnt <= IPROC_PWM_PERIOD_MAX &&
				 duty_cnt <= IPROC_PWM_DUTY_CYCLE_MAX)
			break;

		/* Otherwise, increase prescale and recalculate counts */
		if (++prescale > IPROC_PWM_PRESCALE_MAX)
			return -EINVAL;
	}

	/*
	 * Don't apply settings if disabled. The period and duty cycle are
	 * always calculated above to ensure the new values are
	 * validated immediately instead of on enable.
	 */
	if (pwm_is_enabled(pwm)) {
		iproc_pwmc_unset_enable_bit(ip, chan);

		value = readl(ip->base + IPROC_PWM_PRESCALE_OFFSET);
		value &= ~IPROC_PWM_PRESCALE_MASK(chan);
		value |= prescale << IPROC_PWM_PRESCALE_SHIFT(chan);

		writel(value, ip->base + IPROC_PWM_PRESCALE_OFFSET);
		writel(period_cnt, ip->base + IPROC_PWM_PERIOD_OFFSET(chan));
		writel(duty_cnt, ip->base + IPROC_PWM_DUTY_CYCLE_OFFSET(chan));

		iproc_pwmc_set_enable_bit(ip, chan);
	}

	return 0;
}

static int iproc_pwmc_set_polarity(struct pwm_chip *chip,
				   struct pwm_device *pwm,
				   enum pwm_polarity polarity)
{
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	unsigned int chan = pwm->hwpwm;
	unsigned int value;
	int ret;

	ret = clk_prepare_enable(ip->clk);
	if (ret < 0) {
		dev_err(chip->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	iproc_pwmc_unset_enable_bit(ip, chan);

	value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);
	if (polarity == PWM_POLARITY_NORMAL)
		value |= 1 << IPROC_PWM_CTRL_POLARITY_SHIFT(chan);
	else
		value &= ~(1 << IPROC_PWM_CTRL_POLARITY_SHIFT(chan));
	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);

	iproc_pwmc_set_enable_bit(ip, chan);

	clk_disable_unprepare(ip->clk);

	return 0;
}

static int iproc_pwmc_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	int ret;

	ret = clk_prepare_enable(ip->clk);
	if (ret < 0) {
		dev_err(chip->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	ret = iproc_pwmc_config(chip, pwm, pwm_get_duty_cycle(pwm),
						pwm_get_period(pwm));
	if (ret < 0) {
		clk_disable_unprepare(ip->clk);
		return ret;
	}

	return 0;
}

static void iproc_pwmc_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	unsigned int chan = pwm->hwpwm;
	unsigned int value;

	iproc_pwmc_unset_enable_bit(ip, chan);

	writel(0, ip->base + IPROC_PWM_DUTY_CYCLE_OFFSET(chan));
	writel(0, ip->base + IPROC_PWM_PERIOD_OFFSET(chan));

	/* Set prescale to 0 for this channel */
	value = readl(ip->base + IPROC_PWM_PRESCALE_OFFSET);
	value &= ~IPROC_PWM_PRESCALE_MASK(chan);
	writel(value, ip->base + IPROC_PWM_PRESCALE_OFFSET);

	iproc_pwmc_set_enable_bit(ip, chan);

	clk_disable_unprepare(ip->clk);
}

static const struct pwm_ops iproc_pwm_ops = {
	.config = iproc_pwmc_config,
	.set_polarity = iproc_pwmc_set_polarity,
	.enable = iproc_pwmc_enable,
	.disable = iproc_pwmc_disable,
	.owner = THIS_MODULE,
};

static int iproc_pwmc_probe(struct platform_device *pdev)
{
	struct iproc_pwmc *ip;
	struct resource *res;
	unsigned int value;
	unsigned int chan;
	int ret;

	ip = devm_kzalloc(&pdev->dev, sizeof(*ip), GFP_KERNEL);
	if (ip == NULL)
		return -ENOMEM;

	platform_set_drvdata(pdev, ip);

	ip->chip.dev = &pdev->dev;
	ip->chip.ops = &iproc_pwm_ops;
	ip->chip.base = -1;
	ip->chip.npwm = IPROC_PWM_CHANNEL_COUNT;
	ip->chip.of_xlate = of_pwm_xlate_with_flags;
	ip->chip.of_pwm_n_cells = 3;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ip->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ip->base))
		return PTR_ERR(ip->base);

	ip->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ip->clk)) {
		dev_err(&pdev->dev, "failed to get clock: %ld\n",
			PTR_ERR(ip->clk));
		return PTR_ERR(ip->clk);
	}

	ret = clk_prepare_enable(ip->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	/* Set full drive and normal polarity for all channels */
	value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);
	for (chan = 0; chan < ip->chip.npwm; chan++) {
		value &= ~(1 << IPROC_PWM_CTRL_TYPE_SHIFT(chan));
		value |= 1 << IPROC_PWM_CTRL_POLARITY_SHIFT(chan);
	}
	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);

	clk_disable_unprepare(ip->clk);

	ret = pwmchip_add(&ip->chip);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);

	return ret;
}

static int iproc_pwmc_remove(struct platform_device *pdev)
{
	struct iproc_pwmc *ip = platform_get_drvdata(pdev);
	unsigned int chan;

	for (chan = 0; chan < ip->chip.npwm; chan++)
		if (pwm_is_enabled(&ip->chip.pwms[chan]))
			clk_disable_unprepare(ip->clk);

	return pwmchip_remove(&ip->chip);
}

static const struct of_device_id bcm_iproc_pwmc_dt[] = {
	{ .compatible = "brcm,iproc-pwm" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_iproc_pwmc_dt);

static struct platform_driver iproc_pwmc_driver = {
	.driver = {
		.name = "bcm-iproc-pwm",
		.of_match_table = bcm_iproc_pwmc_dt,
	},
	.probe = iproc_pwmc_probe,
	.remove = iproc_pwmc_remove,
};
module_platform_driver(iproc_pwmc_driver);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Broadcom iProc PWM driver");
MODULE_LICENSE("GPL v2");
