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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/types.h>

/*
 * The Kona PWM has some unusual characteristics.  Here are the main points.
 *
 * 1) There is no disable bit and the hardware docs advise programming a zero
 *    duty to achieve output equivalent to that of a normal disable operation.
 *
 * 2) Changes to prescale, duty, period, and polarity do not take effect until
 *    a subsequent rising edge of the trigger bit.
 *
 * 3) If the smooth bit and trigger bit are both low, the output is a constant
 *    high signal.  Otherwise, the earlier waveform continues to be output.
 *
 * 4) If the smooth bit is set on the rising edge of the trigger bit, output
 *    will transition to the new settings on a period boundary (which could be
 *    seconds away).  If the smooth bit is clear, new settings will be applied
 *    as soon as possible (the hardware always has a 400ns delay).
 *
 * 5) When the external clock that feeds the PWM is disabled, output is pegged
 *    high or low depending on its state at that exact instant.
 */

#define PWM_CONTROL_OFFSET			(0x00000000)
#define PWM_CONTROL_SMOOTH_SHIFT(chan)		(24 + (chan))
#define PWM_CONTROL_TYPE_SHIFT(shift, chan)	(shift + chan)
#define PWM_CONTROL_POLARITY_SHIFT(chan)	(8 + (chan))
#define PWM_CONTROL_TRIGGER_SHIFT(chan)		(chan)

#define PRESCALE_OFFSET				(0x00000004)
#define PRESCALE_SHIFT				(0x00000004)
#define PRESCALE_MASK				(0x00000007)
#define PRESCALE_MIN				(0x00000000)
#define PRESCALE_MAX				(0x00000007)

#define PERIOD_COUNT_OFFSET(offset, chan)	(offset + (chan << 3))
#define PERIOD_COUNT_MIN			(0x00000002)
#define PERIOD_COUNT_MAX			(0x00ffffff)
#define KONA_PERIOD_COUNT_OFFSET		(0x00000008)

#define DUTY_CYCLE_HIGH_OFFSET(offset, chan)	(offset + (chan << 3))
#define DUTY_CYCLE_HIGH_MIN			(0x00000000)
#define DUTY_CYCLE_HIGH_MAX			(0x00ffffff)
#define KONA_DUTY_CYCLE_HIGH_OFFSET		(0x0000000c)

#define PWM_CHANNEL_CNT				(0x00000006)
#define SIGNAL_PUSH_PULL			(0x00000001)
#define PWMOUT_TYPE_SHIFT			(0x00000010)

#define IPROC_PRESCALE_OFFSET			(0x00000024)
#define IPROC_PRESCALE_SHIFT			(0x00000006)
#define IPROC_PRESCALE_MAX			(0x0000003f)

#define IPROC_PERIOD_COUNT_OFFSET		(0x00000004)
#define IPROC_PERIOD_COUNT_MIN			(0x00000002)
#define IPROC_PERIOD_COUNT_MAX			(0x0000ffff)

#define IPROC_DUTY_CYCLE_HIGH_OFFSET		(0x00000008)
#define IPROC_DUTY_CYCLE_HIGH_MIN		(0x00000000)
#define IPROC_DUTY_CYCLE_HIGH_MAX		(0x0000ffff)

#define IPROC_PWM_CHANNEL_CNT			(0x00000004)
#define IPROC_SIGNAL_PUSH_PULL			(0x00000000)
#define IPROC_PWMOUT_TYPE_SHIFT			(0x0000000f)

/*
 * pwm controller reg structure
 *
 * @prescale_offset: prescale register offset
 * @period_offset: period register offset
 * @duty_offset: duty register offset
 * @no_of_channels: number of channels
 * @out_type_shift: out type shift in the register
 * @signal_type: push-pull or open drain
 * @prescale_max: prescale max
 * @prescale_shift: prescale shift in register
 * @prescale_ch_ascending: prescale ch order in prescale register
 * @duty_cycle_max: value of max duty cycle
 * @duty_cycle_min: value of min duty cycle
 * @period_count_max: max period count val
 * @period_count_min: min period count val
 * @smooth_output_support: pwm smooth output support
 */
struct kona_pwmc_reg {
	u32 prescale_offset;
	u32 period_offset;
	u32 duty_offset;
	u32 no_of_channels;
	u32 out_type_shift;
	u32 signal_type;
	u32 prescale_max;
	u32 prescale_shift;
	bool prescale_ch_ascending;
	u32 duty_cycle_max;
	u32 duty_cycle_min;
	u32 period_count_max;
	u32 period_count_min;
	bool smooth_output_support;
};

struct kona_pwmc {
	struct pwm_chip chip;
	void __iomem *base;
	struct clk *clk;
	const struct kona_pwmc_reg *reg;
};

static const struct of_device_id bcm_kona_pwmc_dt[];

static inline struct kona_pwmc *to_kona_pwmc(struct pwm_chip *_chip)
{
	return container_of(_chip, struct kona_pwmc, chip);
}

/*
 * Clear trigger bit but set smooth bit to maintain old output.
 */
static void kona_pwmc_prepare_for_settings(struct kona_pwmc *kp,
	unsigned int chan)
{
	unsigned int value = readl(kp->base + PWM_CONTROL_OFFSET);

	if (kp->reg->smooth_output_support)
		value |= 1 << PWM_CONTROL_SMOOTH_SHIFT(chan);

	value &= ~(1 << PWM_CONTROL_TRIGGER_SHIFT(chan));
	writel(value, kp->base + PWM_CONTROL_OFFSET);

	/*
	 * There must be a min 400ns delay between clearing trigger and setting
	 * it. Failing to do this may result in no PWM signal.
	 */
	ndelay(400);
}

static void kona_pwmc_apply_settings(struct kona_pwmc *kp, unsigned int chan)
{
	unsigned int value = readl(kp->base + PWM_CONTROL_OFFSET);

	/* Set trigger bit and clear smooth bit to apply new settings */
	if (kp->reg->smooth_output_support)
		value &= ~(1 << PWM_CONTROL_SMOOTH_SHIFT(chan));

	value |= 1 << PWM_CONTROL_TRIGGER_SHIFT(chan);
	writel(value, kp->base + PWM_CONTROL_OFFSET);

	/* Trigger bit must be held high for at least 400 ns. */
	ndelay(400);
}

static int kona_pwmc_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	struct kona_pwmc *kp = to_kona_pwmc(chip);
	u64 val, div, rate;
	unsigned long prescale = PRESCALE_MIN, pc, dc;
	unsigned int value, chan = pwm->hwpwm;

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

	rate = clk_get_rate(kp->clk);

	while (1) {
		div = 1000000000;
		div *= 1 + prescale;
		val = rate * period_ns;
		pc = div64_u64(val, div);
		val = rate * duty_ns;
		dc = div64_u64(val, div);

		/* If duty_ns or period_ns are not achievable then return */
		if (pc < kp->reg->period_count_min ||
						dc < kp->reg->duty_cycle_min)
			return -EINVAL;

		/* If pc and dc are in bounds, the calculation is done */
		if (pc <= kp->reg->period_count_max &&
						dc <= kp->reg->duty_cycle_max)
			break;

		/* Otherwise, increase prescale and recalculate pc and dc */
		if (++prescale > kp->reg->prescale_max)
			return -EINVAL;
	}

	/*
	 * Don't apply settings if disabled. The period and duty cycle are
	 * always calculated above to ensure the new values are
	 * validated immediately instead of on enable.
	 */
	if (pwm_is_enabled(pwm)) {
		u32 ch_pre_shift = kp->reg->prescale_shift;

		kona_pwmc_prepare_for_settings(kp, chan);

		if (kp->reg->prescale_ch_ascending)
			/*
			 * The prescale bits mask is in ascending
			 * order in the register
			 * "ch(n-2)bits..ch(n-1)bits..ch(n)bits".
			 */
			ch_pre_shift *= ((kp->reg->no_of_channels - 1) - chan);
		else
			ch_pre_shift *= chan;

		value = readl(kp->base + kp->reg->prescale_offset);
		value &= ~(kp->reg->prescale_max << ch_pre_shift);
		value |= prescale << ch_pre_shift;
		writel(value, kp->base + kp->reg->prescale_offset);

		writel(pc, kp->base +
			PERIOD_COUNT_OFFSET(kp->reg->period_offset, chan));

		writel(dc, kp->base +
			DUTY_CYCLE_HIGH_OFFSET(kp->reg->duty_offset, chan));

		kona_pwmc_apply_settings(kp, chan);
	}

	return 0;
}

static int kona_pwmc_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				  enum pwm_polarity polarity)
{
	struct kona_pwmc *kp = to_kona_pwmc(chip);
	unsigned int chan = pwm->hwpwm;
	unsigned int value;
	int ret;

	ret = clk_prepare_enable(kp->clk);
	if (ret < 0) {
		dev_err(chip->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	kona_pwmc_prepare_for_settings(kp, chan);

	value = readl(kp->base + PWM_CONTROL_OFFSET);

	if (polarity == PWM_POLARITY_NORMAL)
		value |= 1 << PWM_CONTROL_POLARITY_SHIFT(chan);
	else
		value &= ~(1 << PWM_CONTROL_POLARITY_SHIFT(chan));

	writel(value, kp->base + PWM_CONTROL_OFFSET);

	kona_pwmc_apply_settings(kp, chan);

	clk_disable_unprepare(kp->clk);

	return 0;
}

static int kona_pwmc_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct kona_pwmc *kp = to_kona_pwmc(chip);
	int ret;

	ret = clk_prepare_enable(kp->clk);
	if (ret < 0) {
		dev_err(chip->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	ret = kona_pwmc_config(chip, pwm, pwm_get_duty_cycle(pwm),
			       pwm_get_period(pwm));
	if (ret < 0) {
		clk_disable_unprepare(kp->clk);
		return ret;
	}

	return 0;
}

static void kona_pwmc_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct kona_pwmc *kp = to_kona_pwmc(chip);
	unsigned int chan = pwm->hwpwm;
	unsigned int value;
	u32 ch_pre_shift = kp->reg->prescale_shift;

	if (kp->reg->prescale_ch_ascending)
		/*
		 * The prescale bits mask is in ascending order
		 * in the register "ch(n-2)bits..ch(n-1)bits..ch(n)bits".
		 */
		ch_pre_shift *= ((kp->reg->no_of_channels - 1) - chan);
	else
		ch_pre_shift *= chan;

	kona_pwmc_prepare_for_settings(kp, chan);

	/* Simulate a disable by configuring for zero duty */
	writel(0, kp->base +
			DUTY_CYCLE_HIGH_OFFSET(kp->reg->duty_offset, chan));
	writel(0, kp->base +
			PERIOD_COUNT_OFFSET(kp->reg->period_offset, chan));

	/* Set prescale to 0 for this channel */
	value = readl(kp->base + kp->reg->prescale_offset);
	value &= ~(kp->reg->prescale_max << ch_pre_shift);
	writel(value, kp->base + kp->reg->prescale_offset);

	kona_pwmc_apply_settings(kp, chan);

	clk_disable_unprepare(kp->clk);
}

static const struct pwm_ops kona_pwm_ops = {
	.config = kona_pwmc_config,
	.set_polarity = kona_pwmc_set_polarity,
	.enable = kona_pwmc_enable,
	.disable = kona_pwmc_disable,
	.owner = THIS_MODULE,
};

static int kona_pwmc_probe(struct platform_device *pdev)
{
	struct kona_pwmc *kp;
	struct resource *res;
	unsigned int chan;
	unsigned int value = 0;
	int ret = 0;
	const struct of_device_id *match;

	kp = devm_kzalloc(&pdev->dev, sizeof(*kp), GFP_KERNEL);
	if (kp == NULL)
		return -ENOMEM;

	match = of_match_device(bcm_kona_pwmc_dt, &pdev->dev);
	if (!match)
		return -EINVAL;

	kp->reg = (struct kona_pwmc_reg *)match->data;
	platform_set_drvdata(pdev, kp);

	kp->chip.dev = &pdev->dev;
	kp->chip.ops = &kona_pwm_ops;
	kp->chip.base = -1;
	kp->chip.npwm = kp->reg->no_of_channels;
	kp->chip.of_xlate = of_pwm_xlate_with_flags;
	kp->chip.of_pwm_n_cells = 3;
	kp->chip.can_sleep = true;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	kp->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(kp->base))
		return PTR_ERR(kp->base);

	kp->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(kp->clk)) {
		dev_err(&pdev->dev, "failed to get clock: %ld\n",
			PTR_ERR(kp->clk));
		return PTR_ERR(kp->clk);
	}

	ret = clk_prepare_enable(kp->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	/* Set push/pull for all channels */
	for (chan = 0; chan < kp->chip.npwm; chan++)
		value |= (kp->reg->signal_type <<
			PWM_CONTROL_TYPE_SHIFT(kp->reg->out_type_shift, chan));

	writel(value, kp->base + PWM_CONTROL_OFFSET);

	clk_disable_unprepare(kp->clk);

	ret = pwmchip_add_with_polarity(&kp->chip, PWM_POLARITY_INVERSED);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);

	return ret;
}

static int kona_pwmc_remove(struct platform_device *pdev)
{
	struct kona_pwmc *kp = platform_get_drvdata(pdev);
	unsigned int chan;

	for (chan = 0; chan < kp->chip.npwm; chan++)
		if (pwm_is_enabled(&kp->chip.pwms[chan]))
			clk_disable_unprepare(kp->clk);

	return pwmchip_remove(&kp->chip);
}

static const struct kona_pwmc_reg kona_pwmc_reg_data = {
	.prescale_offset = PRESCALE_OFFSET,
	.period_offset = KONA_PERIOD_COUNT_OFFSET,
	.duty_offset = KONA_DUTY_CYCLE_HIGH_OFFSET,
	.no_of_channels = PWM_CHANNEL_CNT,
	.out_type_shift = PWMOUT_TYPE_SHIFT,
	.signal_type = SIGNAL_PUSH_PULL,
	.prescale_max = PRESCALE_MAX,
	.prescale_shift = PRESCALE_SHIFT,
	.prescale_ch_ascending = false,
	.duty_cycle_max = DUTY_CYCLE_HIGH_MAX,
	.duty_cycle_min = DUTY_CYCLE_HIGH_MIN,
	.period_count_max = PERIOD_COUNT_MAX,
	.period_count_min = PERIOD_COUNT_MIN,
	.smooth_output_support = true,
};

static const struct kona_pwmc_reg iproc_pwmc_reg_data = {
	.prescale_offset = IPROC_PRESCALE_OFFSET,
	.period_offset = IPROC_PERIOD_COUNT_OFFSET,
	.duty_offset = IPROC_DUTY_CYCLE_HIGH_OFFSET,
	.no_of_channels = IPROC_PWM_CHANNEL_CNT,
	.out_type_shift = IPROC_PWMOUT_TYPE_SHIFT,
	.signal_type = IPROC_SIGNAL_PUSH_PULL,
	.prescale_max = IPROC_PRESCALE_MAX,
	.prescale_shift = IPROC_PRESCALE_SHIFT,
	.prescale_ch_ascending = true,
	.duty_cycle_max = IPROC_DUTY_CYCLE_HIGH_MAX,
	.duty_cycle_min = IPROC_DUTY_CYCLE_HIGH_MIN,
	.period_count_max = IPROC_PERIOD_COUNT_MAX,
	.period_count_min = IPROC_PERIOD_COUNT_MIN,
	.smooth_output_support = false,
};

static const struct of_device_id bcm_kona_pwmc_dt[] = {
	{ .compatible = "brcm,kona-pwm", .data = &kona_pwmc_reg_data},
	{ .compatible = "brcm,iproc-pwm", .data = &iproc_pwmc_reg_data},
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_kona_pwmc_dt);

static struct platform_driver kona_pwmc_driver = {
	.driver = {
		.name = "bcm-kona-pwm",
		.of_match_table = bcm_kona_pwmc_dt,
	},
	.probe = kona_pwmc_probe,
	.remove = kona_pwmc_remove,
};
module_platform_driver(kona_pwmc_driver);

MODULE_AUTHOR("Broadcom Corporation <bcm-kernel-feedback-list@broadcom.com>");
MODULE_AUTHOR("Tim Kryger <tkryger@broadcom.com>");
MODULE_DESCRIPTION("Broadcom Kona PWM driver");
MODULE_LICENSE("GPL v2");
