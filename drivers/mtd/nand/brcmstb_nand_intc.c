/*
 * Copyright © 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "brcmnand.h"

#define DRV_NAME	"brcmnand-intc"

struct brcmnand_intc_ofdata {
	int (*init)(struct brcmnand_intc *intc);
	bool (*ctlrdy_ack)(struct brcmnand_intc *intc);
	void (*ctlrdy_set_enabled)(struct brcmnand_intc *intc, bool en);
};

struct bcm63138_nand_intc_priv {
	void __iomem *base;
};

#define BCM63138_NAND_INT_STATUS		0x00
#define BCM63138_NAND_INT_EN			0x04

enum {
	BCM63138_CTLRDY		= BIT(4),
};

static bool bcm63138_nand_intc_ack(struct brcmnand_intc *intc)
{
	struct bcm63138_nand_intc_priv *priv = intc->priv;
	void __iomem *mmio = priv->base + BCM63138_NAND_INT_STATUS;
	u32 val = __raw_readl(mmio);

	if (val & BCM63138_CTLRDY) {
		__raw_writel(val & ~BCM63138_CTLRDY, mmio);
		return true;
	}

	return false;
}

static void bcm63138_nand_intc_set(struct brcmnand_intc *intc, bool en)
{
	struct bcm63138_nand_intc_priv *priv = intc->priv;
	void __iomem *mmio = priv->base + BCM63138_NAND_INT_EN;
	u32 val = __raw_readl(mmio);

	if (en)
		val |= BCM63138_CTLRDY;
	else
		val &= ~BCM63138_CTLRDY;

	__raw_writel(val, mmio);
}

static int bcm63138_nand_intc_init(struct brcmnand_intc *intc)
{
	struct bcm63138_nand_intc_priv *priv;

	priv = devm_kzalloc(intc->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = of_io_request_and_map(intc->dn, 0, DRV_NAME);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	intc->priv = priv;

	/* Clear the interrupt */
	bcm63138_nand_intc_ack(intc);

	return 0;
}

static const struct brcmnand_intc_ofdata bcm63138_nand_intc = {
	.init			= bcm63138_nand_intc_init,
	.ctlrdy_ack		= bcm63138_nand_intc_ack,
	.ctlrdy_set_enabled	= bcm63138_nand_intc_set,
};

static const struct of_device_id brcmnand_intc_ofmatch[] = {
	{
		.compatible	= "brcm,nand-intc-bcm63138",
		.data		= &bcm63138_nand_intc,
	},
	{},
};

struct brcmnand_intc *devm_brcmnand_probe_intc(struct device *dev,
					       struct device_node *dn)
{
	const struct brcmnand_intc_ofdata *intc_data;
	const struct of_device_id *match;
	struct brcmnand_intc *intc;
	int ret;

	match = of_match_node(brcmnand_intc_ofmatch, dn);
	if (!match)
		return NULL;

	intc_data = match->data;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return NULL;

	intc->dev = dev;
	intc->dn = dn;
	intc->ctlrdy_ack = intc_data->ctlrdy_ack;
	intc->ctlrdy_set_enabled = intc_data->ctlrdy_set_enabled;
	if (intc_data->init) {
		ret = intc_data->init(intc);
		if (ret)
			return NULL;
	}

	return intc;
}
