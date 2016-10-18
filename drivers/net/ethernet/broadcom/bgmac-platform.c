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

#define pr_fmt(fmt)		KBUILD_MODNAME ": " fmt

#include <linux/bcma/bcma.h>
#include <linux/brcmphy.h>
#include <linux/etherdevice.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include "bgmac.h"

#define NICPM_PADRING_CFG		0x00000004
#define NICPM_IOMUX_CTRL		0x00000008

#define NICPM_PADRING_CFG_INIT_VAL	0x74000000
#define NICPM_IOMUX_CTRL_INIT_VAL	0x21880000
#define NICPM_IOMUX_CTRL_INIT_VAL_BX	0x3196e800

/* Offsets from GMAC_DEVCONTROL */
/* PHY registers */
#define GPHY_EXP_DATA_REG           0x15
#define GPHY_EXP_SELECT_REG         0x17
#define GPHY_MISC_CTRL_REG          0x18  /* shadow 7 */
#define GPHY_CLK_ALIGNCTRL_REG      0x1C  /* Shadow 3 */

/* Initialization values of above PHY registers */
#define GPHY_EXP_DATA_REG_VAL                  0x11B
#define GPHY_EXP_SELECT_REG_VAL_LANE_SWAP      0x0F09
#define GPHY_EXP_SELECT_REG_VAL_BROADREACH_OFF 0x0F90
#define GPHY_MISC_CTRL_REG_SKEW_DISABLE_VAL    0xF0E7
#define GPHY_CLK_GTX_DELAY_DISALE_WR_VAL       0x8c00
#define GPHY_MISC_CTRL_REG_DELAY_DISABLE_VAL   0x7007
#define GPHY_CLK_GTX_DELAY_DISALE_RD_VAL       0x0c00

static u32 platform_bgmac_read(struct bgmac *bgmac, u16 offset)
{
	return readl(bgmac->plat.base + offset);
}

static void platform_bgmac_write(struct bgmac *bgmac, u16 offset, u32 value)
{
	writel(value, bgmac->plat.base + offset);
}

static u32 platform_bgmac_idm_read(struct bgmac *bgmac, u16 offset)
{
	return readl(bgmac->plat.idm_base + offset);
}

static void platform_bgmac_idm_write(struct bgmac *bgmac, u16 offset, u32 value)
{
	return writel(value, bgmac->plat.idm_base + offset);
}

static bool platform_bgmac_clk_enabled(struct bgmac *bgmac)
{
	if ((bgmac_idm_read(bgmac, BCMA_IOCTL) &
	     (BCMA_IOCTL_CLK | BCMA_IOCTL_FGC)) != BCMA_IOCTL_CLK)
		return false;
	if (bgmac_idm_read(bgmac, BCMA_RESET_CTL) & BCMA_RESET_CTL_RESET)
		return false;
	return true;
}

static void platform_bgmac_clk_enable(struct bgmac *bgmac, u32 flags)
{
	bgmac_idm_write(bgmac, BCMA_IOCTL,
			(BCMA_IOCTL_CLK | BCMA_IOCTL_FGC | flags));
	bgmac_idm_read(bgmac, BCMA_IOCTL);

	bgmac_idm_write(bgmac, BCMA_RESET_CTL, 0);
	bgmac_idm_read(bgmac, BCMA_RESET_CTL);
	udelay(1);

	bgmac_idm_write(bgmac, BCMA_IOCTL, (BCMA_IOCTL_CLK | flags));
	bgmac_idm_read(bgmac, BCMA_IOCTL);
	udelay(1);
}

static void platform_bgmac_cco_ctl_maskset(struct bgmac *bgmac, u32 offset,
					   u32 mask, u32 set)
{
	/* This shouldn't be encountered */
	WARN_ON(1);
}

static u32 platform_bgmac_get_bus_clock(struct bgmac *bgmac)
{
	/* This shouldn't be encountered */
	WARN_ON(1);

	return 0;
}

static void platform_bgmac_cmn_maskset32(struct bgmac *bgmac, u16 offset,
					 u32 mask, u32 set)
{
	/* This shouldn't be encountered */
	WARN_ON(1);
}

static int bgmac_phy54810_rgmii_sync(struct phy_device *phy_dev)
{
	int rc;

	rc = phy_write(phy_dev, GPHY_EXP_SELECT_REG,
		       GPHY_EXP_SELECT_REG_VAL_BROADREACH_OFF);
	if (rc < 0)
		return rc;

	rc = phy_write(phy_dev, GPHY_EXP_DATA_REG, 0);
	if (rc < 0)
		return rc;

	rc = phy_write(phy_dev, GPHY_EXP_SELECT_REG,
		       GPHY_EXP_SELECT_REG_VAL_LANE_SWAP);
	if (rc < 0)
		return rc;

	rc = phy_write(phy_dev, GPHY_EXP_DATA_REG,
		       GPHY_EXP_DATA_REG_VAL);
	if (rc < 0)
		return rc;

	rc = phy_write(phy_dev, GPHY_MISC_CTRL_REG,
		       GPHY_MISC_CTRL_REG_SKEW_DISABLE_VAL);
	if (rc < 0)
		return rc;

	rc = phy_write(phy_dev, GPHY_CLK_ALIGNCTRL_REG,
		       GPHY_CLK_GTX_DELAY_DISALE_WR_VAL);
	if (rc < 0)
		return rc;

	rc = phy_write(phy_dev, GPHY_MISC_CTRL_REG,
		       GPHY_MISC_CTRL_REG_DELAY_DISABLE_VAL);
	if (rc < 0)
		return rc;

	rc = phy_write(phy_dev, GPHY_CLK_ALIGNCTRL_REG,
		       GPHY_CLK_GTX_DELAY_DISALE_RD_VAL);
	if (rc < 0)
		return rc;

	return 0;
}

static void bgmac_phy_init(struct bgmac *bgmac)
{
	if (!bgmac->plat.nicpm_base)
		return;

	/* SET RGMII IO CONFIG */
	writel(NICPM_PADRING_CFG_INIT_VAL,
	       bgmac->plat.nicpm_base + NICPM_PADRING_CFG);

	/* Give some time so that values take effect */
	usleep_range(10, 100);

	/* SET IO MUX CONTROL */
	if (bgmac->feature_flags & BGMAC_NS2)
		writel(NICPM_IOMUX_CTRL_INIT_VAL,
		       bgmac->plat.nicpm_base + NICPM_IOMUX_CTRL);
	else if (bgmac->feature_flags & BGMAC_NS2_B0)
		writel(NICPM_IOMUX_CTRL_INIT_VAL_BX,
		       bgmac->plat.nicpm_base + NICPM_IOMUX_CTRL);
	else
		WARN_ON(1);

	usleep_range(10, 100);
}

static int platform_phy_connect(struct bgmac *bgmac,
				void (*bgmac_adjust_link)(struct net_device *))
{
	struct phy_device *phy_dev;

	bgmac_phy_init(bgmac);

	if (bgmac->feature_flags & (BGMAC_NS2 | BGMAC_NS2_B0)) {
		int rc;

		/* Register PHY BCM54810 Fix-ups */
		rc = phy_register_fixup_for_uid(PHY_ID_BCM54810, 0xfffffff0,
						bgmac_phy54810_rgmii_sync);
		if (rc) {
			dev_err(bgmac->dev, "Phy not found\n");
			return -ENODEV;
		}
	}

	phy_dev = of_phy_get_and_connect(bgmac->net_dev, bgmac->dev->of_node,
					 bgmac_adjust_link);
	if (!phy_dev) {
		dev_err(bgmac->dev, "Phy connect failed\n");
		return -ENODEV;
	}

	return 0;
}

static int platform_phy_direct_connect(struct bgmac *bgmac,
				       void (*adjust_link)(struct net_device *))
{
	struct fixed_phy_status fphy_status = {
		.link = 1,
		.speed = SPEED_1000,
		.duplex = DUPLEX_FULL,
	};
	struct phy_device *phy_dev;
	int err;

	phy_dev = fixed_phy_register(PHY_POLL, &fphy_status, -1, NULL);
	if (!phy_dev || IS_ERR(phy_dev)) {
		dev_err(bgmac->dev, "Failed to register fixed PHY device\n");
		return -ENODEV;
	}

	err = phy_connect_direct(bgmac->net_dev, phy_dev, adjust_link,
				 PHY_INTERFACE_MODE_MII);
	if (err) {
		dev_err(bgmac->dev, "Connecting PHY failed\n");
		return err;
	}

	return err;
}

static int bgmac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct bgmac *bgmac;
	struct resource *regs;
	const u8 *mac_addr;

	bgmac = devm_kzalloc(&pdev->dev, sizeof(*bgmac), GFP_KERNEL);
	if (!bgmac)
		return -ENOMEM;

	platform_set_drvdata(pdev, bgmac);

	if (of_device_is_compatible(np, "brcm,ns2-amac"))
		bgmac->feature_flags |= BGMAC_NS2;
	else if (of_device_is_compatible(np, "brcm,ns2-b0-amac"))
		bgmac->feature_flags |= BGMAC_NS2_B0;

	/* Set the features of the 4707 family */
	bgmac->feature_flags |= BGMAC_FEAT_CLKCTLST;
	bgmac->feature_flags |= BGMAC_FEAT_NO_RESET;
	bgmac->feature_flags |= BGMAC_FEAT_CMDCFG_SR_REV4;
	bgmac->feature_flags |= BGMAC_FEAT_TX_MASK_SETUP;
	bgmac->feature_flags |= BGMAC_FEAT_RX_MASK_SETUP;

	bgmac->dev = &pdev->dev;
	bgmac->dma_dev = &pdev->dev;

	mac_addr = of_get_mac_address(np);
	if (mac_addr)
		ether_addr_copy(bgmac->mac_addr, mac_addr);
	else
		dev_warn(&pdev->dev, "MAC address not present in device tree\n");

	bgmac->irq = platform_get_irq(pdev, 0);
	if (bgmac->irq < 0) {
		dev_err(&pdev->dev, "Unable to obtain IRQ\n");
		return bgmac->irq;
	}

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "amac_base");
	if (!regs) {
		dev_err(&pdev->dev, "Unable to obtain base resource\n");
		return -EINVAL;
	}

	bgmac->plat.base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(bgmac->plat.base))
		return PTR_ERR(bgmac->plat.base);

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "idm_base");
	if (!regs) {
		dev_err(&pdev->dev, "Unable to obtain idm resource\n");
		return -EINVAL;
	}

	bgmac->plat.idm_base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(bgmac->plat.idm_base))
		return PTR_ERR(bgmac->plat.idm_base);

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "nicpm_base");
	if (regs) {
		bgmac->plat.nicpm_base = devm_ioremap_resource(&pdev->dev,
							       regs);
		if (IS_ERR(bgmac->plat.nicpm_base))
			return PTR_ERR(bgmac->plat.nicpm_base);
	}

	bgmac->read = platform_bgmac_read;
	bgmac->write = platform_bgmac_write;
	bgmac->idm_read = platform_bgmac_idm_read;
	bgmac->idm_write = platform_bgmac_idm_write;
	bgmac->clk_enabled = platform_bgmac_clk_enabled;
	bgmac->clk_enable = platform_bgmac_clk_enable;
	bgmac->cco_ctl_maskset = platform_bgmac_cco_ctl_maskset;
	bgmac->get_bus_clock = platform_bgmac_get_bus_clock;
	bgmac->cmn_maskset32 = platform_bgmac_cmn_maskset32;
	if (of_parse_phandle(np, "phy-handle", 0)) {
		bgmac->phy_connect = platform_phy_connect;
	} else {
		bgmac->phy_connect = platform_phy_direct_connect;
		bgmac->feature_flags |= BGMAC_FEAT_FORCE_SPEED_2500;
	}

	return bgmac_enet_probe(bgmac);
}

static int bgmac_remove(struct platform_device *pdev)
{
	struct bgmac *bgmac = platform_get_drvdata(pdev);

	bgmac_enet_remove(bgmac);

	return 0;
}

static const struct of_device_id bgmac_of_enet_match[] = {
	{.compatible = "brcm,amac",},
	{.compatible = "brcm,nsp-amac",},
	{.compatible = "brcm,ns2-amac",},
	{.compatible = "brcm,ns2-b0-amac",},
	{},
};

MODULE_DEVICE_TABLE(of, bgmac_of_enet_match);

static struct platform_driver bgmac_enet_driver = {
	.driver = {
		.name  = "bgmac-enet",
		.of_match_table = bgmac_of_enet_match,
	},
	.probe = bgmac_probe,
	.remove = bgmac_remove,
};

module_platform_driver(bgmac_enet_driver);
MODULE_LICENSE("GPL");
