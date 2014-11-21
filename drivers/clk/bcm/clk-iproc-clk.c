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
#include <linux/err.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clkdev.h>
#include <linux/of_address.h>
#include <linux/delay.h>

#include "clk-iproc.h"

struct iproc_pll;

struct iproc_clk {
	struct clk_hw hw;
	const char *name;
	struct iproc_pll *pll;
	unsigned long rate;
	const struct iproc_clk_ctrl *ctrl;
};

struct iproc_pll {
	void __iomem *base;
	struct clk_onecell_data clk_data;
	struct iproc_clk *clks;
};

#define to_iproc_clk(hw) container_of(hw, struct iproc_clk, hw)

static int iproc_clk_enable(struct clk_hw *hw)
{
	struct iproc_clk *clk = to_iproc_clk(hw);
	const struct iproc_clk_ctrl *ctrl = clk->ctrl;
	struct iproc_pll *pll = clk->pll;
	u32 val;

	/* channel enable is active low */
	val = readl(pll->base + ctrl->enable.offset);
	val &= ~(1 << ctrl->enable.enable_shift);
	writel(val, pll->base + ctrl->enable.offset);

	/* also make sure channel is not held */
	val = readl(pll->base + ctrl->enable.offset);
	val &= ~(1 << ctrl->enable.hold_shift);
	writel(val, pll->base + ctrl->enable.offset);

	return 0;
}

static void iproc_clk_disable(struct clk_hw *hw)
{
	struct iproc_clk *clk = to_iproc_clk(hw);
	const struct iproc_clk_ctrl *ctrl = clk->ctrl;
	struct iproc_pll *pll = clk->pll;
	u32 val;

	if (ctrl->flags & IPROC_CLK_AON)
		return;

	val = readl(pll->base + ctrl->enable.offset);
	val |= 1 << ctrl->enable.enable_shift;
	writel(val, pll->base + ctrl->enable.offset);
}

static unsigned long iproc_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct iproc_clk *clk = to_iproc_clk(hw);
	const struct iproc_clk_ctrl *ctrl = clk->ctrl;
	struct iproc_pll *pll = clk->pll;
	u32 val;
	unsigned int mdiv;

	if (parent_rate == 0)
		return 0;

	val = readl(pll->base + ctrl->mdiv.offset);
	mdiv = (val >> ctrl->mdiv.shift) & bit_mask(ctrl->mdiv.width);
	if (mdiv == 0)
		mdiv = 256;

	clk->rate = parent_rate / mdiv;

	return clk->rate;
}

static long iproc_clk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	unsigned int div;

	if (rate == 0 || *parent_rate == 0)
		return -EINVAL;

	if (rate == *parent_rate)
		return *parent_rate;

	div = DIV_ROUND_UP(*parent_rate, rate);
	if (div < 2)
		return *parent_rate;

	if (div > 256)
		div = 256;

	return *parent_rate / div;
}

static int iproc_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct iproc_clk *clk = to_iproc_clk(hw);
	const struct iproc_clk_ctrl *ctrl = clk->ctrl;
	struct iproc_pll *pll = clk->pll;
	u32 val;
	unsigned int div;

	if (rate == 0 || parent_rate == 0)
		return -EINVAL;

	div = DIV_ROUND_UP(parent_rate, rate);
	if (div > 256)
		return -EINVAL;

	val = readl(pll->base + ctrl->mdiv.offset);
	if (div == 256) {
		val &= ~(bit_mask(ctrl->mdiv.width) << ctrl->mdiv.shift);
	} else {
		val &= ~(bit_mask(ctrl->mdiv.width) << ctrl->mdiv.shift);
		val |= div << ctrl->mdiv.shift;
	}
	writel(val, pll->base + ctrl->mdiv.offset);
	clk->rate = parent_rate / div;

	return 0;
}

static const struct clk_ops iproc_clk_ops = {
	.enable = iproc_clk_enable,
	.disable = iproc_clk_disable,
	.recalc_rate = iproc_clk_recalc_rate,
	.round_rate = iproc_clk_round_rate,
	.set_rate = iproc_clk_set_rate,
};

void __init iproc_clk_setup(struct device_node *node,
		const struct iproc_clk_ctrl *ctrl, unsigned int num_clks)
{
	int i, ret;
	struct iproc_pll *pll;

	if (WARN_ON(!ctrl))
		return;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (WARN_ON(!pll))
		return;

	pll->clk_data.clk_num = num_clks;
	pll->clk_data.clks = kcalloc(num_clks, sizeof(*pll->clk_data.clks),
			GFP_KERNEL);
	if (WARN_ON(!pll->clk_data.clks))
		goto err_clks;

	pll->clks = kcalloc(num_clks, sizeof(*pll->clks), GFP_KERNEL);
	if (WARN_ON(!pll->clks))
		goto err_pll_clks;

	pll->base = of_iomap(node, 0);
	if (WARN_ON(!pll->base))
		goto err_iomap;

	for (i = 0; i < num_clks; i++) {
		struct clk_init_data init;
		struct clk *clk;
		const char *parent_name;
		struct iproc_clk *iclk;
		const char *clk_name;

		clk_name = kzalloc(IPROC_CLK_NAME_LEN, GFP_KERNEL);
		if (WARN_ON(!clk_name))
			goto err_clk_register;

		ret = of_property_read_string_index(node, "clock-output-names",
				i, &clk_name);
		if (WARN_ON(ret))
			goto err_clk_register;

		iclk = &pll->clks[i];
		iclk->name = clk_name;
		iclk->pll = pll;
		iclk->ctrl = &ctrl[i];
		init.name = clk_name;
		init.ops = &iproc_clk_ops;
		init.flags = 0;
		parent_name = of_clk_get_parent_name(node, 0);
		init.parent_names = (parent_name ? &parent_name : NULL);
		init.num_parents = (parent_name ? 1 : 0);
		iclk->hw.init = &init;

		clk = clk_register(NULL, &iclk->hw);
		if (WARN_ON(IS_ERR(clk)))
			goto err_clk_register;
		pll->clk_data.clks[i] = clk;
	}

	ret = of_clk_add_provider(node, of_clk_src_onecell_get, &pll->clk_data);
	if (WARN_ON(ret))
		goto err_clk_register;

	return;

err_clk_register:
	for (i = 0; i < num_clks; i++)
		kfree(pll->clks[i].name);
	iounmap(pll->base);

err_iomap:
	kfree(pll->clks);

err_pll_clks:
	kfree(pll->clk_data.clks);

err_clks:
	kfree(pll);
}
