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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>

#define CDRU_USBPHY_CLK_RST_SEL_OFFSET			0x11b4
#define CDRU_USBPHY2_HOST_DEV_SEL_OFFSET		0x11b8
#define CDRU_SPARE_REG_0_OFFSET				0x1238
#define CRMU_USB_PHY_AON_CTRL_OFFSET			0x00028
#define CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET		0x1210
#define CDRU_USBPHY_P2_STATUS_OFFSET			0x1200

#define CDRU_USB_DEV_SUSPEND_RESUME_CTRL_DISABLE	0
#define PHY2_DEV_HOST_CTRL_SEL_DEVICE			0
#define PHY2_DEV_HOST_CTRL_SEL_HOST			1
#define CDRU_USBPHY_P2_STATUS__USBPHY_ILDO_ON_FLAG	1
#define CDRU_USBPHY_P2_STATUS__USBPHY_PLL_LOCK		0
#define CRMU_USBPHY_P0_AFE_CORERDY_VDDC			1
#define CRMU_USBPHY_P0_RESETB				2
#define CRMU_USBPHY_P1_AFE_CORERDY_VDDC			9
#define CRMU_USBPHY_P1_RESETB				10
#define CRMU_USBPHY_P2_AFE_CORERDY_VDDC			17
#define CRMU_USBPHY_P2_RESETB				18

#define USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET		0x0408
#define USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable	0
#define USB2_IDM_IDM_RESET_CONTROL_OFFSET		0x0800
#define USB2_IDM_IDM_RESET_CONTROL__RESET		0

#define PLL_LOCK_RETRY_COUNT	1000

struct bcm_phy_instance;

struct bcm_phy_driver {
	void __iomem *usbphy_regs;
	void __iomem *usb2h_idm_regs;
	void __iomem *usb2d_idm_regs;
	spinlock_t lock;
	int num_phys, idm_host_enabled;
	struct bcm_phy_instance *instances;
};

struct bcm_phy_instance {
	struct bcm_phy_driver *driver;
	struct phy *generic_phy;
	int port;
	int host_mode; /* 1 - Host , 0 - device */
	int power; /* 1 -powered_on 0 -powered off */
};

static inline int cdru_usbphy_p2_status_wait(int reg_bit,
					     struct bcm_phy_driver *phy_driver)
{
	/* Wait for the PLL lock status */
	int retry = PLL_LOCK_RETRY_COUNT;
	u32 reg_val;

	do {
		udelay(1);
		reg_val = readl(phy_driver->usbphy_regs +
				CDRU_USBPHY_P2_STATUS_OFFSET);
	} while (--retry > 0 && (reg_val & (1 << reg_bit)) == 0);

	if (retry == 0)
		return -EBUSY;
	else
		return 0;
}

static struct phy *bcm_usb_phy_xlate(struct device *dev,
				     struct of_phandle_args *args)
{
	struct bcm_phy_driver *phy_driver = dev_get_drvdata(dev);

	if (!phy_driver)
		return ERR_PTR(-EINVAL);

	if (WARN_ON(args->args[0] >= phy_driver->num_phys))
		return ERR_PTR(-ENODEV);

	if (WARN_ON(args->args[1] < 0 || args->args[1] > 1))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(args->args_count < 2))
		return ERR_PTR(-EINVAL);

	phy_driver->instances[args->args[0]].port = args->args[0];
	phy_driver->instances[args->args[0]].host_mode = args->args[1];

	return phy_driver->instances[args->args[0]].generic_phy;
}

static int bcm_phy_init(struct phy *generic_phy)
{
	u32 reg_val;
	unsigned long flags;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;

	spin_lock_irqsave(&phy_driver->lock, flags);

	/* Only PORT 2 is capabale of being device and host
	 * Default setting is device, check if it is set to host */
	if (instance_ptr->port == 2) {
		if (instance_ptr->host_mode == PHY2_DEV_HOST_CTRL_SEL_HOST)
			writel(PHY2_DEV_HOST_CTRL_SEL_HOST,
				phy_driver->usbphy_regs +
				CDRU_USBPHY2_HOST_DEV_SEL_OFFSET);
		else {
			/* Disable suspend/resume signals to device controller
			when a port is in device mode  */
			reg_val = readl(phy_driver->usbphy_regs +
				      CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET);
			reg_val |=
			  (1 << CDRU_USB_DEV_SUSPEND_RESUME_CTRL_DISABLE);
			writel(reg_val, phy_driver->usbphy_regs +
			       CDRU_USB_DEV_SUSPEND_RESUME_CTRL_OFFSET);
		}
	}

	spin_unlock_irqrestore(&phy_driver->lock, flags);
	return 0;
}

static int bcm_phy_shutdown(struct phy *generic_phy)
{

	u32 reg_val, powered_on_phy;
	int i, power_off_flag = 1;
	unsigned long flags;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;

	spin_lock_irqsave(&phy_driver->lock, flags);

	/* power down the phy */
	reg_val = readl(phy_driver->usbphy_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	if (instance_ptr->port == 0) {
		reg_val &= ~(1 << CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
		reg_val &= ~(1 << CRMU_USBPHY_P0_RESETB);
	} else if (instance_ptr->port == 1) {
		reg_val &= ~(1 << CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
		reg_val &= ~(1 << CRMU_USBPHY_P1_RESETB);
	} else if (instance_ptr->port == 2) {
		reg_val &= ~(1 << CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
		reg_val &= ~(1 << CRMU_USBPHY_P2_RESETB);
	}
	writel(reg_val, phy_driver->usbphy_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	instance_ptr->power = 0;

	/* if a port is configured to device and it is being shutdown,
	 * turn off the clocks to the usb device controller */
	if (instance_ptr->port == 2 &&
		instance_ptr->host_mode == PHY2_DEV_HOST_CTRL_SEL_DEVICE) {
		reg_val = readl(phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val &= ~(1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
		writel(reg_val, phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2d_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val |= (1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
	} else {

		/* If the phy being shutdown provides clock and reset to
		 * the host controller, change it do a different powered on phy
		 * If all phys are powered off, shut of the host controller
		 */
		reg_val = readl(phy_driver->usbphy_regs +
				CDRU_USBPHY_CLK_RST_SEL_OFFSET);
		powered_on_phy = reg_val;
		if (reg_val == instance_ptr->port) {
			for (i = 0; i < phy_driver->num_phys; i++) {
				if (phy_driver->instances[i].power == 1 &&
				phy_driver->instances[i].host_mode == 0) {
					power_off_flag = 0;
					powered_on_phy = i;
				}
			}
		}

		if (power_off_flag) {
			/* Put the host controller into reset
			state and disable clock */
			reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
			reg_val &=
			  ~(1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
			writel(reg_val, phy_driver->usb2h_idm_regs +
					USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

			reg_val = readl(phy_driver->usb2h_idm_regs +
					 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
			reg_val |= (1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
			writel(reg_val, phy_driver->usb2h_idm_regs +
					USB2_IDM_IDM_RESET_CONTROL_OFFSET);
			phy_driver->idm_host_enabled = 0;
		} else
			writel(powered_on_phy, phy_driver->usbphy_regs +
				   CDRU_USBPHY_CLK_RST_SEL_OFFSET);
	}

	spin_unlock_irqrestore(&phy_driver->lock, flags);
	return 0;
}

static int bcm_phy_poweron(struct phy *generic_phy)
{
	int ret, clock_reset_flag = 1;
	unsigned long flags;
	u32 reg_val;
	struct bcm_phy_instance *instance_ptr = phy_get_drvdata(generic_phy);
	struct bcm_phy_driver *phy_driver = instance_ptr->driver;

	spin_lock_irqsave(&phy_driver->lock, flags);

	/* Bring the AFE block out of reset to start powering up the PHY */
	reg_val = readl(phy_driver->usbphy_regs + CRMU_USB_PHY_AON_CTRL_OFFSET);

	if (instance_ptr->port == 0)
		reg_val |= (1 << CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
	else if (instance_ptr->port == 1)
		reg_val |= (1 << CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
	else if (instance_ptr->port == 2)
		reg_val |= (1 << CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
	writel(reg_val, phy_driver->usbphy_regs + CRMU_USB_PHY_AON_CTRL_OFFSET);

	instance_ptr->power = 1;

	/* Check if the port 2 is configured for device */
	if (instance_ptr->port == 2 &&
		instance_ptr->host_mode == PHY2_DEV_HOST_CTRL_SEL_DEVICE) {

		ret = cdru_usbphy_p2_status_wait
		       (CDRU_USBPHY_P2_STATUS__USBPHY_ILDO_ON_FLAG, phy_driver);
		if (ret < 0) {
			dev_err(&generic_phy->dev, "Timed out waiting for USBPHY_ILDO_ON_FLAG on CDRU_USBPHY_P2_STATUS");
			goto err_shutdown;
		}

		ret = cdru_usbphy_p2_status_wait
			(CDRU_USBPHY_P2_STATUS__USBPHY_PLL_LOCK, phy_driver);
		if (ret < 0) {
			dev_err(&generic_phy->dev, "Timed out waiting for USBPHY_PLL_LOCK on CDRU_USBPHY_P2_STATUS");
			goto err_shutdown;
		}


	/* Enable clock to USB device and get the USB device out of reset */
		reg_val = readl(phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val |= (1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
		writel(reg_val, phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2d_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val &= ~(1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2d_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);

	} else {
		reg_val = readl(phy_driver->usbphy_regs +
				CDRU_USBPHY_CLK_RST_SEL_OFFSET);

		/* Check if the phy that is configured
		 * to provide clock and reset is powered on*/
		if (reg_val >= 0 && reg_val < phy_driver->num_phys) {
			if (phy_driver->instances[reg_val].power == 1)
				clock_reset_flag = 0;
		}

		/* if not set the current phy */
		if (clock_reset_flag) {
			reg_val = instance_ptr->port;
			writel(reg_val, phy_driver->usbphy_regs +
			       CDRU_USBPHY_CLK_RST_SEL_OFFSET);
		}
	}

	if (phy_driver->idm_host_enabled != 1) {
		/* Enable clock to USB and get the USB out of reset */
		reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);
		reg_val |= (1 << USB2_IDM_IDM_IO_CONTROL_DIRECT__clk_enable);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_IO_CONTROL_DIRECT_OFFSET);

		reg_val = readl(phy_driver->usb2h_idm_regs +
				USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		reg_val &= ~(1 << USB2_IDM_IDM_RESET_CONTROL__RESET);
		writel(reg_val, phy_driver->usb2h_idm_regs +
				 USB2_IDM_IDM_RESET_CONTROL_OFFSET);
		phy_driver->idm_host_enabled = 1;
	}

	spin_unlock_irqrestore(&phy_driver->lock, flags);
	return 0;

err_shutdown:
	spin_unlock_irqrestore(&phy_driver->lock, flags);
	bcm_phy_shutdown(generic_phy);
	return ret;
}

static struct phy_ops ops = {
	.init		= bcm_phy_init,
	.power_on	= bcm_phy_poweron,
	.power_off	= bcm_phy_shutdown,
};

static const struct of_device_id bcm_phy_dt_ids[] = {
	{ .compatible = "brcm,cygnus-usb-phy", },
	{ }
};

static int bcm_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource res;
	struct bcm_phy_driver *phy_driver;
	struct phy_provider *phy_provider;
	int ret, num_phys, i;
	u32 reg_val;

	/* get number of phy cells */
	ret = of_property_read_u32(dev->of_node, "num-phys", &num_phys);
	if (ret) {
		dev_err(dev, "Failed to obtain device tree resource num-phys\n");
		return ret;
	}

	/* allocate memory for each phy instance */
	phy_driver = devm_kzalloc(dev, sizeof(struct bcm_phy_driver),
				  GFP_KERNEL);
	if (!phy_driver)
		return -ENOMEM;

	phy_driver->instances = devm_kcalloc(dev, num_phys,
					     sizeof(struct bcm_phy_instance),
					     GFP_KERNEL);
	phy_driver->num_phys = num_phys;
	spin_lock_init(&phy_driver->lock);

	ret = of_address_to_resource(dev->of_node, 0, &res);
	if (ret) {
		dev_err(dev, "Failed to obtain device tree resource usbphy_regs\n");
		return ret;
	}
	phy_driver->usbphy_regs  = devm_ioremap_nocache(dev, res.start,
						resource_size(&res));
	if (!phy_driver->usbphy_regs) {
		dev_err(dev, "Failed to remap usbphy_regs\n");
		return -ENOMEM;
	}

	ret = of_address_to_resource(dev->of_node, 1, &res);
	if (ret) {
		dev_err(dev, "Failed to obtain device tree resource usb2h_idm_regs\n");
		return ret;
	}
	phy_driver->usb2h_idm_regs = devm_ioremap_nocache(dev, res.start,
						  resource_size(&res));
	if (!phy_driver->usb2h_idm_regs) {
		dev_err(dev, "Failed to remap usb2h_idm_regs\n");
		return -ENOMEM;
	}

	ret = of_address_to_resource(dev->of_node, 2, &res);
	if (ret) {
		dev_err(dev, "Failed to obtain device tree resource usb2d_idm_regs\n");
		return ret;
	}
	phy_driver->usb2d_idm_regs = devm_ioremap_nocache(dev, res.start,
						   resource_size(&res));
	if (!phy_driver->usb2d_idm_regs) {
		dev_err(dev, "Failed to remap usb2d_idm_regs\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, phy_driver);
	phy_driver->idm_host_enabled = 0;

	/* Shutdown all ports. They can be powered up as
	 * required */
	reg_val = readl(phy_driver->usbphy_regs +
			CRMU_USB_PHY_AON_CTRL_OFFSET);
	reg_val &= ~(1 << CRMU_USBPHY_P0_AFE_CORERDY_VDDC);
	reg_val &= ~(1 << CRMU_USBPHY_P0_RESETB);
	reg_val &= ~(1 << CRMU_USBPHY_P1_AFE_CORERDY_VDDC);
	reg_val &= ~(1 << CRMU_USBPHY_P1_RESETB);
	reg_val &= ~(1 << CRMU_USBPHY_P2_AFE_CORERDY_VDDC);
	reg_val &= ~(1 << CRMU_USBPHY_P2_RESETB);
	writel(reg_val, phy_driver->usbphy_regs +
		CRMU_USB_PHY_AON_CTRL_OFFSET);

	for (i = 0; i < phy_driver->num_phys; i++) {
		struct bcm_phy_instance *instance_ptr =
					&phy_driver->instances[i];
		instance_ptr->generic_phy =
				devm_phy_create(dev, NULL, &ops);

		if (IS_ERR(instance_ptr->generic_phy)) {
			dev_err(dev, "Failed to create usb phy %d", i);
			return PTR_ERR(instance_ptr->generic_phy);
		}
		instance_ptr->driver = phy_driver;
		phy_set_drvdata(instance_ptr->generic_phy, instance_ptr);
	}

	phy_provider = devm_of_phy_provider_register(dev,
					bcm_usb_phy_xlate);

	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register as phy provider\n");
		return PTR_ERR(phy_provider);
	}

	platform_set_drvdata(pdev, phy_driver);

	return 0;

}

MODULE_DEVICE_TABLE(of, bcm_phy_dt_ids);

static struct platform_driver bcm_phy_driver = {
	.probe = bcm_phy_probe,
	.driver = {
		.name = "bcm-cygnus-usbphy",
		.of_match_table = of_match_ptr(bcm_phy_dt_ids),
	},
};

module_platform_driver(bcm_phy_driver);

MODULE_ALIAS("platform:bcm-cygnus-usbphy");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Cygnus USB PHY driver");
MODULE_LICENSE("GPL V2");
