/*
 * Copyright (C) 2015 Broadcom Corporation
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

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/phy/iproc_mdio_phy.h>

#define MAX_PHY_ADDR                  0x1f

#define PCIE_MDCDIV_VAL               0x3e

#define PCIE_BLK_ADDR_OFFSET          0x1f
#define PCIE_BLK_ADDR_MASK            0xff00
#define PCIE_REG_ADDR_MASK            0x1f

#define PCIE_SW_CTRL0_OFFSET          0x1000
#define PCIE_SW_PWRDOWN_SHIFT         0

#define PCIE_AFE1_100MHZ_C3_OFFSET    0x2103
#define PCIE_AFE1_100MHZ_C3_VAL       0x2b1c

#define CRMU_PCIE_CFG_OFFSET          0x00
#define CRMU_PCIE1_PHY_IDDQ_SHIFT     10
#define CRMU_PCIE0_PHY_IDDQ_SHIFT     2

enum cygnus_pcie_phy_id {
	CYGNUS_PHY_PCIE0 = 0,
	CYGNUS_PHY_PCIE1,
	MAX_NUM_PHYS,
};

struct cygnus_pcie_phy_core;

/**
 * struct cygnus_pcie_phy - Cygnus PCIe PHY device
 * @core: pointer to the Cygnus PCIe PHY core control
 * @id: internal ID to identify the Cygnus PCIe PHY
 * @addr: PHY address used by MDC to communicate with the PHY
 * @phy: pointer to the kernel PHY device
 *
 */
struct cygnus_pcie_phy {
	struct cygnus_pcie_phy_core *core;
	enum cygnus_pcie_phy_id id;
	unsigned addr;
	struct phy *phy;
};

/**
 * struct cygnus_pcie_phy_core - Cygnus PCIe PHY core control
 * @dev: pointer to device
 * @crmu: CRMU register base
 * @lock: mutex to protect access to individual PHYs
 * @phys: pointer to Cygnus PHY device
 *
 */
struct cygnus_pcie_phy_core {
	struct device *dev;
	void __iomem *crmu;
	struct mutex lock;
	struct cygnus_pcie_phy phys[MAX_NUM_PHYS];
};

static int cygnus_pcie_phy_reg_read(unsigned phy_addr, unsigned reg_addr,
				    u16 *val)
{
	int ret;
	u16 addr = (u16)(reg_addr & PCIE_BLK_ADDR_MASK);

	ret = iproc_mdio_write(PCIE_MDCDIV_VAL, phy_addr, PCIE_BLK_ADDR_OFFSET,
			       addr);
	if (ret)
		return ret;

	ret = iproc_mdio_read(PCIE_MDCDIV_VAL, phy_addr,
			      reg_addr & PCIE_REG_ADDR_MASK, val);
	return ret;
}

static int cygnus_pcie_phy_reg_write(unsigned phy_addr, unsigned reg_addr,
				     u16 val)
{
	int ret;
	u16 addr = (u16)(reg_addr & PCIE_BLK_ADDR_MASK);

	ret = iproc_mdio_write(PCIE_MDCDIV_VAL, phy_addr, PCIE_BLK_ADDR_OFFSET,
			       addr);
	if (ret)
		return ret;

	ret = iproc_mdio_write(PCIE_MDCDIV_VAL, phy_addr,
			       reg_addr & PCIE_REG_ADDR_MASK, val);
	return ret;
}

static void cygnus_pcie_afe_enable_disable(void __iomem *reg,
					   enum cygnus_pcie_phy_id id,
					   bool enable)
{
	unsigned shift;
	u32 val;

	if (id == CYGNUS_PHY_PCIE0)
		shift = CRMU_PCIE0_PHY_IDDQ_SHIFT;
	else
		shift = CRMU_PCIE1_PHY_IDDQ_SHIFT;

	if (enable) {
		val = readl(reg + CRMU_PCIE_CFG_OFFSET);
		val &= ~BIT(shift);
		writel(val, reg + CRMU_PCIE_CFG_OFFSET);
	} else {
		val = readl(reg + CRMU_PCIE_CFG_OFFSET);
		val |= BIT(shift);
		writel(val, reg + CRMU_PCIE_CFG_OFFSET);
	}

	/* wait for AFE to come up or shut down */
	msleep(100);
}

static int cygnus_pcie_power_config(struct cygnus_pcie_phy *phy, bool on)
{
	struct cygnus_pcie_phy_core *core = phy->core;
	u16 val;
	int ret;

	if (phy->id != CYGNUS_PHY_PCIE0 && phy->id != CYGNUS_PHY_PCIE1)
		return -EINVAL;

	if (on) {
		/* enable AFE through the CRMU interface */
		cygnus_pcie_afe_enable_disable(core->crmu, phy->id, true);

		/* to get the reference clock configured */
		ret = cygnus_pcie_phy_reg_write(phy->addr,
						PCIE_AFE1_100MHZ_C3_OFFSET,
						PCIE_AFE1_100MHZ_C3_VAL);
		if (ret)
			goto err_disable_afe;
		cygnus_pcie_phy_reg_read(phy->addr,
					 PCIE_AFE1_100MHZ_C3_OFFSET, &val);
		if (ret)
			goto err_disable_afe;

		msleep(20);

		/* now toggle AFE */
		cygnus_pcie_afe_enable_disable(core->crmu, phy->id, false);
		cygnus_pcie_afe_enable_disable(core->crmu, phy->id, true);

		dev_info(core->dev,
			 "pcie phy on, addr:0x%02x off:0x%04x val:0x%04x\n",
			 phy->addr, PCIE_AFE1_100MHZ_C3_OFFSET, val);
	} else {
		/* disable AFE through the CRMU interface */
		cygnus_pcie_afe_enable_disable(core->crmu, phy->id, false);
		dev_info(core->dev, "pcie phy off\n");
	}

	return 0;

err_disable_afe:
	cygnus_pcie_afe_enable_disable(core->crmu, phy->id, false);
	return ret;
}

static int cygnus_pcie_phy_init(struct phy *p)
{
	return 0;
}

static int cygnus_pcie_phy_exit(struct phy *p)
{
	return 0;
}

static int cygnus_pcie_phy_power_on(struct phy *p)
{
	struct cygnus_pcie_phy *phy = phy_get_drvdata(p);
	struct cygnus_pcie_phy_core *core = phy->core;
	int ret = 0;

	mutex_lock(&core->lock);

	switch (phy->id) {
	case CYGNUS_PHY_PCIE0:
	case CYGNUS_PHY_PCIE1:
		ret = cygnus_pcie_power_config(phy, true);
		if (ret)
			dev_err(core->dev, "unable to power on PCIe PHY\n");
		break;

	default:
		dev_err(core->dev, "PHY id not supported\n");
		mutex_unlock(&core->lock);
		return -EINVAL;
	}

	mutex_unlock(&core->lock);

	return ret;
}

static int cygnus_pcie_phy_power_off(struct phy *p)
{
	struct cygnus_pcie_phy *phy = phy_get_drvdata(p);
	struct cygnus_pcie_phy_core *core = phy->core;
	int ret = 0;

	mutex_lock(&core->lock);

	switch (phy->id) {
	case CYGNUS_PHY_PCIE0:
	case CYGNUS_PHY_PCIE1:
		ret = cygnus_pcie_power_config(phy, false);
		if (ret)
			dev_err(core->dev, "unable to power off PCIe PHY\n");
		break;

	default:
		dev_err(core->dev, "PHY id not supported\n");
		mutex_unlock(&core->lock);
		break;
	}

	mutex_unlock(&core->lock);

	return ret;
}

static struct phy_ops cygnus_pcie_phy_ops = {
	.init = cygnus_pcie_phy_init,
	.exit = cygnus_pcie_phy_exit,
	.power_on = cygnus_pcie_phy_power_on,
	.power_off = cygnus_pcie_phy_power_off,
};

static struct phy *cygnus_pcie_phy_xlate(struct device *dev,
					 struct of_phandle_args *args)
{
	struct cygnus_pcie_phy_core *core;
	int id, phy_addr;

	core = dev_get_drvdata(dev);
	if (!core)
		return ERR_PTR(-EINVAL);

	id = args->args[0];
	phy_addr = args->args[1];

	if (WARN_ON(id >= MAX_NUM_PHYS))
		return ERR_PTR(-ENODEV);

	if (WARN_ON(phy_addr > MAX_PHY_ADDR))
		return ERR_PTR(-ENODEV);

	core->phys[id].addr = phy_addr;

	return core->phys[id].phy;
}

static int cygnus_pcie_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cygnus_pcie_phy_core *core;
	struct phy_provider *provider;
	struct resource *res;
	int i = 0;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	core->crmu = devm_ioremap_resource(dev, res);
	if (IS_ERR(core->crmu))
		return PTR_ERR(core->crmu);

	mutex_init(&core->lock);

	for (i = 0; i < MAX_NUM_PHYS; i++) {
		struct cygnus_pcie_phy *p = &core->phys[i];

		p->phy = devm_phy_create(dev, NULL, &cygnus_pcie_phy_ops);
		if (IS_ERR(p->phy)) {
			dev_err(dev, "failed to create PHY\n");
			return PTR_ERR(p->phy);
		}

		p->core = core;
		p->id = i;
		phy_set_drvdata(p->phy, p);
	}

	dev_set_drvdata(dev, core);

	provider = devm_of_phy_provider_register(dev, cygnus_pcie_phy_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "failed to register PHY provider\n");
		return PTR_ERR(provider);
	}

	return 0;
}

static const struct of_device_id cygnus_pcie_phy_match_table[] = {
	{ .compatible = "brcm,cygnus-pcie-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, cygnus_pcie_phy_match_table);

static struct platform_driver cygnus_pcie_phy_driver = {
	.driver = {
		.name		= "cygnus-pcie-phy",
		.of_match_table	= cygnus_pcie_phy_match_table,
	},
	.probe	= cygnus_pcie_phy_probe,
};
module_platform_driver(cygnus_pcie_phy_driver);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom Cygnus PCIe PHY driver");
MODULE_LICENSE("GPL v2");
