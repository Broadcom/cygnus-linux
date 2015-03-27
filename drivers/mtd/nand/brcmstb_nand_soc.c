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

#define DRV_NAME	"brcmnand-soc"

struct brcmnand_soc_ofdata {
	int (*init)(struct brcmnand_soc *soc);
	bool (*ctlrdy_ack)(struct brcmnand_soc *soc);
	void (*ctlrdy_set_enabled)(struct brcmnand_soc *soc, bool en);
	void (*apb_access_mode)(struct brcmnand_soc *soc, bool cfg_le);
};

struct bcm63138_nand_soc_priv {
	void __iomem *base;
};

#define BCM63138_NAND_INT_STATUS		0x00
#define BCM63138_NAND_INT_EN			0x04

enum {
	BCM63138_CTLRDY		= BIT(4),
};

static bool bcm63138_nand_intc_ack(struct brcmnand_soc *soc)
{
	struct bcm63138_nand_soc_priv *priv = soc->priv;
	void __iomem *mmio = priv->base + BCM63138_NAND_INT_STATUS;
	u32 val = __raw_readl(mmio);

	if (val & BCM63138_CTLRDY) {
		__raw_writel(val & ~BCM63138_CTLRDY, mmio);
		return true;
	}

	return false;
}

static void bcm63138_nand_intc_set(struct brcmnand_soc *soc, bool en)
{
	struct bcm63138_nand_soc_priv *priv = soc->priv;
	void __iomem *mmio = priv->base + BCM63138_NAND_INT_EN;
	u32 val = __raw_readl(mmio);

	if (en)
		val |= BCM63138_CTLRDY;
	else
		val &= ~BCM63138_CTLRDY;

	__raw_writel(val, mmio);
}

static int bcm63138_nand_soc_init(struct brcmnand_soc *soc)
{
	struct bcm63138_nand_soc_priv *priv;

	priv = devm_kzalloc(soc->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->base = of_io_request_and_map(soc->dn, 0, DRV_NAME);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	soc->priv = priv;

	/* Clear the interrupt */
	bcm63138_nand_intc_ack(soc);

	return 0;
}

static const struct brcmnand_soc_ofdata bcm63138_nand_soc = {
	.init			= bcm63138_nand_soc_init,
	.ctlrdy_ack		= bcm63138_nand_intc_ack,
	.ctlrdy_set_enabled	= bcm63138_nand_intc_set,
};

struct iproc_nand_soc_priv {
	void __iomem *idm_base;
	void __iomem *ext_base;
	spinlock_t idm_lock;
};

#define IPROC_NAND_CTLR_READY_OFFSET       0x10
#define IPROC_NAND_CTLR_READY              BIT(0)

#define IPROC_NAND_IO_CTRL_OFFSET          0x00
#define IPROC_NAND_APB_LE_MODE             BIT(24)
#define IPROC_NAND_INT_CTRL_READ_ENABLE    BIT(6)

static bool iproc_nand_intc_ack(struct brcmnand_soc *soc)
{
	struct iproc_nand_soc_priv *priv = soc->priv;
	void __iomem *mmio = priv->ext_base + IPROC_NAND_CTLR_READY_OFFSET;
	u32 val = __raw_readl(mmio);

	if (val & IPROC_NAND_CTLR_READY) {
		__raw_writel(IPROC_NAND_CTLR_READY, mmio);
		return true;
	}

	return false;
}

static void iproc_nand_intc_set(struct brcmnand_soc *soc, bool en)
{
	struct iproc_nand_soc_priv *priv = soc->priv;
	void __iomem *mmio = priv->idm_base + IPROC_NAND_IO_CTRL_OFFSET;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&priv->idm_lock, flags);

	val = __raw_readl(mmio);

	if (en)
		val |= IPROC_NAND_INT_CTRL_READ_ENABLE;
	else
		val &= ~IPROC_NAND_INT_CTRL_READ_ENABLE;

	__raw_writel(val, mmio);

	spin_unlock_irqrestore(&priv->idm_lock, flags);
}

static int iproc_nand_soc_init(struct brcmnand_soc *soc)
{
	struct iproc_nand_soc_priv *priv;

	priv = devm_kzalloc(soc->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->idm_lock);

	priv->idm_base = of_io_request_and_map(soc->dn, 0, DRV_NAME);
	if (IS_ERR(priv->idm_base))
		return PTR_ERR(priv->idm_base);

	priv->ext_base = of_io_request_and_map(soc->dn, 1, DRV_NAME);
	if (IS_ERR(priv->ext_base)) {
		struct resource res;

		of_address_to_resource(soc->dn, 0, &res);
		release_mem_region(res.start, resource_size(&res));
		iounmap(priv->idm_base);
		return PTR_ERR(priv->ext_base);
	}

	soc->priv = priv;
	iproc_nand_intc_ack(soc);

	return 0;
}

static void iproc_nand_apb_access(struct brcmnand_soc *soc, bool cfg_le)
{
	struct iproc_nand_soc_priv *priv = soc->priv;
	void __iomem *mmio = priv->idm_base + IPROC_NAND_IO_CTRL_OFFSET;
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&priv->idm_lock, flags);

	val = __raw_readl(mmio);

	if (cfg_le)
		val |= IPROC_NAND_APB_LE_MODE;
	else
		val &= ~IPROC_NAND_APB_LE_MODE;

	__raw_writel(val, mmio);

	spin_unlock_irqrestore(&priv->idm_lock, flags);
}

static const struct brcmnand_soc_ofdata iproc_nand_soc = {
	.init			= iproc_nand_soc_init,
	.ctlrdy_ack		= iproc_nand_intc_ack,
	.ctlrdy_set_enabled	= iproc_nand_intc_set,
	.apb_access_mode	= iproc_nand_apb_access,
};

static const struct of_device_id brcmnand_soc_ofmatch[] = {
	{
		.compatible	= "brcm,nand-soc-bcm63138",
		.data		= &bcm63138_nand_soc,
	},
	{
		.compatible	= "brcm,nand-soc-iproc",
		.data		= &iproc_nand_soc,
	},
	{},
};

struct brcmnand_soc *devm_brcmnand_probe_soc(struct device *dev,
					     struct device_node *dn)
{
	const struct brcmnand_soc_ofdata *soc_data;
	const struct of_device_id *match;
	struct brcmnand_soc *soc;
	int ret;

	match = of_match_node(brcmnand_soc_ofmatch, dn);
	if (!match)
		return NULL;

	soc_data = match->data;

	soc = devm_kzalloc(dev, sizeof(*soc), GFP_KERNEL);
	if (!soc)
		return NULL;

	soc->dev = dev;
	soc->dn = dn;
	soc->ctlrdy_ack = soc_data->ctlrdy_ack;
	soc->ctlrdy_set_enabled = soc_data->ctlrdy_set_enabled;
	soc->apb_access_mode = soc_data->apb_access_mode;
	if (soc_data->init) {
		ret = soc_data->init(soc);
		if (ret)
			return NULL;
	}

	return soc;
}
