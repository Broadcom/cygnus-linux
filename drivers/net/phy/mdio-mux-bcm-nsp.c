/*
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 */

#include <linux/device.h>
#include <linux/mdio-mux.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/phy.h>
#include <linux/platform_device.h>

#define NSP_MDIO_EXT_BUS_START_ADDR		16
#define NSP_MDIO_EXT_SELECT_BIT			BIT(9)

struct nsp_mdiomux_desc {
	void __iomem *bus_ctrl;
	void __iomem *mgmt_ctrl;
	void *mux_handle;
};

static int mdio_mux_nsp_switch_fn(int current_child, int desired_child,
				  void *priv)
{
	struct nsp_mdiomux_desc *md = priv;
	u32 data, bus_id;

	/* select internal or external bus */
	data = readl(md->mgmt_ctrl);
	if (desired_child == NSP_MDIO_EXT_BUS_START_ADDR)
		data |= NSP_MDIO_EXT_SELECT_BIT;
	else
		data &= ~NSP_MDIO_EXT_SELECT_BIT;
	writel(data, md->mgmt_ctrl);

	/* select bus number */
	if (md->bus_ctrl) {
		bus_id = desired_child & (NSP_MDIO_EXT_BUS_START_ADDR - 1);
		writel(bus_id, md->bus_ctrl);
	}

	return 0;
}

static int mdio_mux_nsp_probe(struct platform_device *pdev)
{
	struct nsp_mdiomux_desc *md;
	struct resource *res;
	int ret;

	md = devm_kzalloc(&pdev->dev, sizeof(*md), GFP_KERNEL);
	if (!md)
		return -ENOMEM;

	/* Bus control is not available in some SoC's */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "bus-ctrl");
	if (res) {
		md->bus_ctrl = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(md->bus_ctrl)) {
			dev_err(&pdev->dev, "failed to ioremap register\n");
			return PTR_ERR(md->bus_ctrl);
		}
	}

	/* Get management control */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mgmt-ctrl");
	if (!res)
		return -EINVAL;

	md->mgmt_ctrl = ioremap(res->start, resource_size(res));
	if (!md->mgmt_ctrl)
		return -ENOMEM;

	ret = mdio_mux_init(&pdev->dev, mdio_mux_nsp_switch_fn,
			    &md->mux_handle, md, NULL);
	if (ret != 0) {
		iounmap(md->mgmt_ctrl);
		return ret;
	}

	pdev->dev.platform_data = md;
	return 0;
}

static int mdio_mux_nsp_remove(struct platform_device *pdev)
{
	struct nsp_mdiomux_desc *md = dev_get_platdata(&pdev->dev);

	iounmap(md->mgmt_ctrl);
	mdio_mux_uninit(md->mux_handle);
	return 0;
}

static const struct of_device_id mdio_mux_nsp_match[] = {
	{ .compatible = "brcm,mdio-mux-nsp" },
	{},
};
MODULE_DEVICE_TABLE(of, mdio_mux_nsp_match);

static struct platform_driver mdio_mux_nsp_driver = {
	.driver = {
		.name = "mdio-mux-nsp",
		.of_match_table = mdio_mux_nsp_match,
	},
	.probe = mdio_mux_nsp_probe,
	.remove = mdio_mux_nsp_remove,
};

module_platform_driver(mdio_mux_nsp_driver);

MODULE_DESCRIPTION("NSP MDIO Mux Bus Driver");
MODULE_AUTHOR("Yendapally Reddy Dhananjaya Reddy <yendapally.reddy@broadcom.com");
MODULE_LICENSE("GPL v2");
