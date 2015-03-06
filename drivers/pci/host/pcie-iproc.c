/*
 * Copyright (C) 2014 Hauke Mehrtens <hauke@hauke-m.de>
 * Copyright (C) 2015 Broadcom Corporatcommon ion
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
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mbus.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>

#include "pcie-iproc.h"

#define CLK_CONTROL_OFFSET           0x000
#define EP_MODE_SURVIVE_PERST_SHIFT  1
#define EP_MODE_SURVIVE_PERST        BIT(EP_MODE_SURVIVE_PERST_SHIFT)
#define RC_PCIE_RST_OUTPUT_SHIFT     0
#define RC_PCIE_RST_OUTPUT           BIT(RC_PCIE_RST_OUTPUT_SHIFT)

#define CFG_IND_ADDR_OFFSET          0x120
#define CFG_IND_ADDR_MASK            0x00001ffc

#define CFG_IND_DATA_OFFSET          0x124

#define CFG_ADDR_OFFSET              0x1f8
#define CFG_ADDR_BUS_NUM_SHIFT       20
#define CFG_ADDR_BUS_NUM_MASK        0x0ff00000
#define CFG_ADDR_DEV_NUM_SHIFT       15
#define CFG_ADDR_DEV_NUM_MASK        0x000f8000
#define CFG_ADDR_FUNC_NUM_SHIFT      12
#define CFG_ADDR_FUNC_NUM_MASK       0x00007000
#define CFG_ADDR_REG_NUM_SHIFT       2
#define CFG_ADDR_REG_NUM_MASK        0x00000ffc
#define CFG_ADDR_CFG_TYPE_SHIFT      0
#define CFG_ADDR_CFG_TYPE_MASK       0x00000003

#define CFG_DATA_OFFSET              0x1fc

#define SYS_RC_INTX_EN               0x330
#define SYS_RC_INTX_MASK             0xf

static inline struct iproc_pcie *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

#define INVALID_ACCESS_OFFSET 0xffffffff
static u32 iproc_pcie_cfg_base(struct iproc_pcie *pcie, int busno,
			       unsigned int devfn, int where)
{
	int slot = PCI_SLOT(devfn);
	int fn = PCI_FUNC(devfn);
	u32 val;

	/* root complex access */
	if (busno == 0) {
		if (slot >= 1)
			return INVALID_ACCESS_OFFSET;
		writel(where & CFG_IND_ADDR_MASK,
		       pcie->base + CFG_IND_ADDR_OFFSET);
		return CFG_IND_DATA_OFFSET;
	}

	if (fn > 1)
		return INVALID_ACCESS_OFFSET;

	/* EP device access */
	val = (busno << CFG_ADDR_BUS_NUM_SHIFT) |
		(slot << CFG_ADDR_DEV_NUM_SHIFT) |
		(fn << CFG_ADDR_FUNC_NUM_SHIFT) |
		(where & CFG_ADDR_REG_NUM_MASK) |
		(1 & CFG_ADDR_CFG_TYPE_MASK);
	writel(val, pcie->base + CFG_ADDR_OFFSET);

	return CFG_DATA_OFFSET;
}

#define INVALID_CFG_RD 0xffffffff
static int iproc_pcie_read_conf(struct iproc_pcie *pcie, int busno,
				unsigned int devfn, int where, int size,
				u32 *val)
{
	u32 offset, mask, shift;

	*val = INVALID_CFG_RD;

	if (size != 1 && size != 2 && size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	offset = iproc_pcie_cfg_base(pcie, busno, devfn, where);
	if (offset == INVALID_ACCESS_OFFSET)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	*val = readl(pcie->base + offset);

	if (size != 4) {
		mask = (1 << (size * BITS_PER_BYTE)) - 1;
		shift = (where % 4) * BITS_PER_BYTE;
		*val = (*val >> shift) & mask;
	}

	dev_dbg(pcie->dev,
		"conf rd: busn=%d devfn=%u where=%d size=%d val=0x%08x\n",
		busno, devfn, where, size, *val);

	return PCIBIOS_SUCCESSFUL;
}

static int iproc_pcie_write_conf(struct iproc_pcie *pcie, int busno,
				 unsigned int devfn, int where, int size,
				 u32 val)
{
	u32 offset, mask, shift, data;

	if (size != 1 && size != 2 && size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	offset = iproc_pcie_cfg_base(pcie, busno, devfn, where);
	if (offset == INVALID_ACCESS_OFFSET)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	data = readl(pcie->base + offset);

	if (size != 4) {
		mask = (1 << (size * BITS_PER_BYTE)) - 1;
		shift = (where % 4) * BITS_PER_BYTE;
		data &= ~(mask << shift);
		data |= (val & mask) << shift;
	} else {
		data = val;
	}

	writel(data, pcie->base + offset);

	dev_dbg(pcie->dev,
		"conf wr: busn=%d devfn=%u where=%d size=%d data=0x%08x\n",
		busno, devfn, where, size, data);

	return PCIBIOS_SUCCESSFUL;
}

static int iproc_pcie_read(struct pci_bus *bus, unsigned int devfn,
			   int where, int size, u32 *val)
{
	struct iproc_pcie *pcie = sys_to_pcie(bus->sysdata);

	return iproc_pcie_read_conf(pcie, bus->number, devfn, where, size,
				    val);
}

static int iproc_pcie_write(struct pci_bus *bus, unsigned int devfn,
			    int where, int size, u32 val)
{
	struct iproc_pcie *pcie = sys_to_pcie(bus->sysdata);

	return iproc_pcie_write_conf(pcie, bus->number, devfn, where, size,
				     val);
}

static struct pci_ops iproc_pcie_ops = {
	.read = iproc_pcie_read,
	.write = iproc_pcie_write,
};

static void iproc_pcie_reset(struct iproc_pcie *pcie)
{
	u32 val;

	/*
	 * Configure the PCIe controller as root complex and send a downstream
	 * reset
	 */
	val = EP_MODE_SURVIVE_PERST | RC_PCIE_RST_OUTPUT;
	writel(val, pcie->base + CLK_CONTROL_OFFSET);
	udelay(250);
	val &= ~EP_MODE_SURVIVE_PERST;
	writel(val, pcie->base + CLK_CONTROL_OFFSET);
	msleep(250);
}

static int iproc_pcie_check_link(struct iproc_pcie *pcie, struct pci_bus *bus)
{
	u8 hdr_type;
	u32 link_ctrl;
	u16 pos, link_status;
	int link_is_active = 0;

	/* make sure we are not in EP mode */
	pci_bus_read_config_byte(bus, 0, PCI_HEADER_TYPE, &hdr_type);
	if ((hdr_type & 0x7f) != PCI_HEADER_TYPE_BRIDGE) {
		dev_err(pcie->dev, "in EP mode, hdr=0x08%x\n", hdr_type);
		return -EFAULT;
	}

	/* force class to PCI_CLASS_BRIDGE_PCI (0x0604) */
	pci_bus_write_config_word(bus, 0, PCI_CLASS_DEVICE,
				  PCI_CLASS_BRIDGE_PCI);

	/* check link status to see if link is active */
	pos = pci_bus_find_capability(bus, 0, PCI_CAP_ID_EXP);
	pci_bus_read_config_word(bus, 0, pos + PCI_EXP_LNKSTA, &link_status);
	if (link_status & PCI_EXP_LNKSTA_NLW)
		link_is_active = 1;

	if (!link_is_active) {
		/* try GEN 1 link speed */
#define PCI_LINK_STATUS_CTRL_2_OFFSET 0x0dc
#define PCI_TARGET_LINK_SPEED_MASK    0xf
#define PCI_TARGET_LINK_SPEED_GEN2    0x2
#define PCI_TARGET_LINK_SPEED_GEN1    0x1
		pci_bus_read_config_dword(bus, 0,
					  PCI_LINK_STATUS_CTRL_2_OFFSET,
					  &link_ctrl);
		if ((link_ctrl & PCI_TARGET_LINK_SPEED_MASK) ==
		    PCI_TARGET_LINK_SPEED_GEN2) {
			link_ctrl &= ~PCI_TARGET_LINK_SPEED_MASK;
			link_ctrl |= PCI_TARGET_LINK_SPEED_GEN1;
			pci_bus_write_config_dword(bus, 0,
					   PCI_LINK_STATUS_CTRL_2_OFFSET,
					   link_ctrl);
			msleep(100);

			pos = pci_bus_find_capability(bus, 0, PCI_CAP_ID_EXP);
			pci_bus_read_config_word(bus, 0, pos + PCI_EXP_LNKSTA,
						 &link_status);
			if (link_status & PCI_EXP_LNKSTA_NLW)
				link_is_active = 1;
		}
	}

	dev_info(pcie->dev, "link: %s\n", link_is_active ? "UP" : "DOWN");

	return link_is_active ? 0 : -ENODEV;
}

static void iproc_pcie_enable(struct iproc_pcie *pcie)
{
	writel(SYS_RC_INTX_MASK, pcie->base + SYS_RC_INTX_EN);
}

int iproc_pcie_setup(struct iproc_pcie *pcie)
{
	int ret;
	struct pci_bus *bus;

	if (!pcie || !pcie->dev || !pcie->base)
		return -EINVAL;

	if (pcie->phy) {
		ret = phy_init(pcie->phy);
		if (ret) {
			dev_err(pcie->dev, "unable to initialize PCIe PHY\n");
			return ret;
		}

		ret = phy_power_on(pcie->phy);
		if (ret) {
			dev_err(pcie->dev, "unable to power on PCIe PHY\n");
			goto err_exit_phy;
		}

	}

	iproc_pcie_reset(pcie);

	pcie->sysdata.private_data = pcie;

	bus = pci_create_root_bus(pcie->dev, 0, &iproc_pcie_ops,
				  &pcie->sysdata, pcie->resources);
	if (!bus) {
		dev_err(pcie->dev, "unable to create PCI root bus\n");
		ret = -ENOMEM;
		goto err_power_off_phy;
	}

	ret = iproc_pcie_check_link(pcie, bus);
	if (ret) {
		dev_err(pcie->dev, "no PCIe EP device detected\n");
		goto err_rm_root_bus;
	}

	iproc_pcie_enable(pcie);

	pci_scan_child_bus(bus);
	pci_assign_unassigned_bus_resources(bus);
	pci_bus_add_devices(bus);

	pci_fixup_irqs(pci_common_swizzle, of_irq_parse_and_map_pci);

	return 0;

err_rm_root_bus:
	pci_lock_rescan_remove();
	pci_stop_root_bus(bus);
	pci_remove_root_bus(bus);
	pci_unlock_rescan_remove();

err_power_off_phy:
	if (pcie->phy)
		phy_power_off(pcie->phy);
err_exit_phy:
	if (pcie->phy)
		phy_exit(pcie->phy);

	return ret;
}

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom iPROC PCIe common driver");
MODULE_LICENSE("GPL v2");
