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

static void iproc_pwmc_enable(struct iproc_pwmc *ip, u32 chan, bool set)
{
	unsigned int value = readl(ip->base + IPROC_PWM_CTRL_OFFSET);

	if (set)
		value |= 1 << IPROC_PWM_CTRL_EN_SHIFT(chan);
	else
		value &= ~(1 << IPROC_PWM_CTRL_EN_SHIFT(chan));
	writel(value, ip->base + IPROC_PWM_CTRL_OFFSET);

	/* must be a min 400ns delay between clearing and setting enable bit. */
	ndelay(400);
}

void iproc_pwmc_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
			  struct pwm_state *state)
{
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	u32 chan = pwm->hwpwm;
	u64 val, multi, rate;
	u32 reg_data;
	u32 prescale;

	rate = clk_get_rate(ip->clk);

	reg_data = readl(ip->base + IPROC_PWM_CTRL_OFFSET);
	if (reg_data & BIT(IPROC_PWM_CTRL_EN_SHIFT(chan)))
		state->enabled = true;
	else
		state->enabled = false;

	if (reg_data & BIT(IPROC_PWM_CTRL_POLARITY_SHIFT(chan)))
		state->polarity = PWM_POLARITY_NORMAL;
	else
		state->polarity = PWM_POLARITY_INVERSED;

	reg_data = readl(ip->base + IPROC_PWM_PRESCALE_OFFSET);
	prescale = reg_data >> IPROC_PWM_PRESCALE_SHIFT(chan);
	prescale &= IPROC_PWM_PRESCALE_MAX;

	multi = 1000000000;

	reg_data = readl(ip->base + IPROC_PWM_PERIOD_OFFSET(chan));
	val = (reg_data & IPROC_PWM_PERIOD_MAX) * (prescale + 1) * multi;
	val = div64_u64(val, rate);
	state->period = val;

	reg_data = readl(ip->base + IPROC_PWM_DUTY_CYCLE_OFFSET(chan));
	val = (reg_data & IPROC_PWM_PERIOD_MAX) * (prescale + 1) * multi;
	val = div64_u64(val, rate);
	state->duty_cycle = val;
}

static int iproc_pwmc_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    struct pwm_state *state)
{
	unsigned long prescale = IPROC_PWM_PRESCALE_MIN;
	struct iproc_pwmc *ip = to_iproc_pwmc(chip);
	unsigned int reg_data, chan = pwm->hwpwm;
	unsigned long period_cnt, duty_cnt;
	u64 val, div, rate;

	rate = clk_get_rate(ip->clk);

	/*
	 * Find period count, duty count and prescale to suit duty_cycle and
	 * period. This is done according to formulas described below:
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
		val = rate * state->period;
		period_cnt = div64_u64(val, div);
		val = rate * state->duty_cycle;
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

	iproc_pwmc_enable(ip, chan, false);

	/* Set prescale */
	reg_data = readl(ip->base + IPROC_PWM_PRESCALE_OFFSET);
	reg_data &= ~IPROC_PWM_PRESCALE_MASK(chan);
	reg_data |= prescale << IPROC_PWM_PRESCALE_SHIFT(chan);
	writel(reg_data, ip->base + IPROC_PWM_PRESCALE_OFFSET);

	/* set period and duty cycle */
	writel(period_cnt, ip->base + IPROC_PWM_PERIOD_OFFSET(chan));
	writel(duty_cnt, ip->base + IPROC_PWM_DUTY_CYCLE_OFFSET(chan));

	/* set polarity */
	reg_data = readl(ip->base + IPROC_PWM_CTRL_OFFSET);
	if (state->polarity == PWM_POLARITY_NORMAL)
		reg_data |= 1 << IPROC_PWM_CTRL_POLARITY_SHIFT(chan);
	else
		reg_data &= ~(1 << IPROC_PWM_CTRL_POLARITY_SHIFT(chan));
	writel(reg_data, ip->base + IPROC_PWM_CTRL_OFFSET);

	if (state->enabled)
		iproc_pwmc_enable(ip, chan, true);

	return 0;
}

static const struct pwm_ops iproc_pwm_ops = {
	.apply = iproc_pwmc_apply,
	.get_state = iproc_pwmc_get_state,
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

	ret = pwmchip_add(&ip->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		clk_disable_unprepare(ip->clk);
	}

	return ret;
}

static int iproc_pwmc_remove(struct platform_device *pdev)
{
	struct iproc_pwmc *ip = platform_get_drvdata(pdev);

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

MODULE_AUTHOR("Yendapally Reddy Dhananjaya Reddy <yendapally.reddy@broadcom.com>");
MODULE_DESCRIPTION("Broadcom iProc PWM driver");
MODULE_LICENSE("GPL v2");
