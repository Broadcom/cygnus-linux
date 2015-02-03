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

/* max number of MSI event queues */
#define MAX_MSI_EQ 6
#define MAX_IRQS MAX_MSI_EQ

#define MDIO_TIMEOUT_USEC 100

#define OPCODE_WRITE 1
#define OPCODE_READ  2

#define MII_TA_VAL 2
#define MII_MDCDIV 62

#define MII_MGMT_CTRL_OFFSET         0x000
#define MII_MGMT_CTRL_MDCDIV_SHIFT   0
#define MII_MGMT_CTRL_PRE_SHIFT      7
#define MII_MGMT_CTRL_BUSY_SHIFT     8
#define MII_MGMT_CTRL_EXT_SHIFT      9
#define MII_MGMT_CTRL_BTP_SHIFT      10

#define MII_MGMT_CMD_DATA_OFFSET     0x004
#define MII_MGMT_CMD_DATA_SHIFT      0
#define MII_MGMT_CMD_TA_SHIFT        16
#define MII_MGMT_CMD_RA_SHIFT        18
#define MII_MGMT_CMD_PA_SHIFT        23
#define MII_MGMT_CMD_OP_SHIFT        28
#define MII_MGMT_CMD_SB_SHIFT        30
#define MII_MGMT_CMD_DATA_MASK       0xFFFF

#define CLK_CONTROL_OFFSET           0x000
#define EP_PERST_SOURCE_SELECT_SHIFT 2
#define EP_PERST_SOURCE_SELECT       (1 << EP_PERST_SOURCE_SELECT_SHIFT)
#define EP_MODE_SURVIVE_PERST_SHIFT  1
#define EP_MODE_SURVIVE_PERST        (1 << EP_MODE_SURVIVE_PERST_SHIFT)
#define RC_PCIE_RST_OUTPUT_SHIFT     0
#define RC_PCIE_RST_OUTPUT           (1 << RC_PCIE_RST_OUTPUT_SHIFT)

#define CFG_IND_ADDR_OFFSET          0x120
#define CFG_IND_ADDR_MASK            0x00001FFC

#define CFG_IND_DATA_OFFSET          0x124

#define CFG_ADDR_OFFSET              0x1F8
#define CFG_ADDR_BUS_NUM_SHIFT       20
#define CFG_ADDR_BUS_NUM_MASK        0x0FF00000
#define CFG_ADDR_DEV_NUM_SHIFT       15
#define CFG_ADDR_DEV_NUM_MASK        0x000F8000
#define CFG_ADDR_FUNC_NUM_SHIFT      12
#define CFG_ADDR_FUNC_NUM_MASK       0x00007000
#define CFG_ADDR_REG_NUM_SHIFT       2
#define CFG_ADDR_REG_NUM_MASK        0x00000FFC
#define CFG_ADDR_CFG_TYPE_SHIFT      0
#define CFG_ADDR_CFG_TYPE_MASK       0x00000003

#define CFG_DATA_OFFSET              0x1FC

#define SYS_EQ_PAGE_OFFSET           0x200
#define SYS_MSI_PAGE_OFFSET          0x204

#define SYS_MSI_INTS_EN_OFFSET       0x208

#define SYS_MSI_CTRL_0_OFFSET        0x210
#define SYS_MSI_INTR_EN_SHIFT        11
#define SYS_MSI_INTR_EN              (1 << SYS_MSI_INTR_EN_SHIFT)
#define SYS_MSI_INT_N_EVENT_SHIFT    1
#define SYS_MSI_INT_N_EVENT          (1 << SYS_MSI_INT_N_EVENT_SHIFT)
#define SYS_MSI_EQ_EN_SHIFT          0
#define SYS_MSI_EQ_EN                (1 << SYS_MSI_EQ_EN_SHIFT)

#define SYS_EQ_HEAD_0_OFFSET         0x250
#define SYS_EQ_TAIL_0_OFFSET         0x254
#define SYS_EQ_TAIL_0_MASK           0x3F

#define SYS_RC_INTX_EN               0x330
#define SYS_RC_INTX_MASK             0xF

#define SYS_RC_INTX_CSR              0x334
#define SYS_RC_INTX_MASK             0xF

#define OARR_0_OFFSET                0xD20
#define OAAR_0_ADDR_MASK             0xF0000000
#define OAAR_0_VALID_SHIFT           0
#define OAAR_0_VALID                 (1 << OAAR_0_VALID_SHIFT)
#define OAAR_0_UPPER_OFFSET          0xD24
#define OAAR_0_UPPER_ADDR_MASK       0x0000000F

#define PCIE_SYS_RC_INTX_EN_OFFSET   0x330

#define OMAP_0_LOWER_OFFSET          0xD40
#define OMAP_0_LOWER_ADDR_MASK       0xF0000000
#define OMAP_0_UPPER_OFFSET          0x0D44

#define PCIE_LINK_STATUS_OFFSET      0xF0C
#define PCIE_PHYLINKUP_SHITF         3
#define PCIE_PHYLINKUP               (1 << PCIE_PHYLINKUP_SHITF)

#define STRAP_STATUS_OFFSET          0xF10
#define STRAP_1LANE_SHIFT            2
#define STRAP_1LANE                  (1 << STRAP_1LANE_SHIFT)
#define STRAP_IF_ENABLE_SHIFT        1
#define STRAP_IF_ENABLE              (1 << STRAP_IF_ENABLE_SHIFT)
#define STRAP_RC_MODE_SHIFT          0
#define STRAP_RC_MODE                (1 << STRAP_RC_MODE_SHIFT)

struct iproc_pcie;

/**
 * iProc MSI
 * @pcie: pointer to the iProc PCIe data structure
 * @irq_in_use: bitmap of MSI IRQs that are in use
 * @domain: MSI IRQ domain
 * @chip: MSI controller
 * @eq_page: memory page to store the iProc MSI event queue
 * @msi_page: memory page for MSI posted writes
 */
struct iproc_msi {
	struct iproc_pcie *pcie;
	DECLARE_BITMAP(irq_in_use, MAX_IRQS);
	struct irq_domain *domain;
	struct msi_controller chip;
	unsigned long eq_page;
	unsigned long msi_page;
};

/**
 * iProc PCIe
 * @dev: pointer to the device
 * @mii: MII/MDIO management I/O register base
 * @reg: PCIe I/O register base
 * @io: PCIe I/O resource
 * @mem: PCIe memory resource
 * @busn: PCIe bus resource
 * @phy_addr: MIDO PHY address
 * @irqs: Array that stores IRQs
 * @msi: MSI related info
 */
struct iproc_pcie {
	struct device *dev;

	void __iomem *mii;
	void __iomem *reg;

	struct resource io;
	struct resource mem;
	struct resource busn;

	u32 phy_addr;
	int irqs[MAX_IRQS];

	struct iproc_msi msi;
};

static inline int mdio_wait_idle(struct iproc_pcie *pcie)
{
	int timeout = MDIO_TIMEOUT_USEC;

	while (readl(pcie->mii + MII_MGMT_CTRL_OFFSET) &
			(1 << MII_MGMT_CTRL_BUSY_SHIFT)) {
		udelay(1);
		if (timeout-- <= 0)
			return -EBUSY;
	}
	return 0;
}

static void mdio_init(struct iproc_pcie *pcie)
{
	u32 val;

	val = MII_MDCDIV << MII_MGMT_CTRL_MDCDIV_SHIFT;
	val |= (1 << MII_MGMT_CTRL_PRE_SHIFT);
	writel(val, pcie->mii + MII_MGMT_CTRL_OFFSET);

	WARN_ON(mdio_wait_idle(pcie));
}

static u16 mdio_read(struct iproc_pcie *pcie, unsigned int phy_addr,
		unsigned int reg_addr)
{
	u32 val;

	WARN_ON(mdio_wait_idle(pcie));

	val = (MII_TA_VAL << MII_MGMT_CMD_TA_SHIFT);
	val |= (reg_addr << MII_MGMT_CMD_RA_SHIFT);
	val |= (phy_addr << MII_MGMT_CMD_PA_SHIFT);
	val |= (OPCODE_READ << MII_MGMT_CMD_OP_SHIFT);
	val |= (1 << MII_MGMT_CMD_SB_SHIFT);
	writel(val, pcie->mii + MII_MGMT_CMD_DATA_OFFSET);

	WARN_ON(mdio_wait_idle(pcie));

	val = readl(pcie->mii + MII_MGMT_CMD_DATA_OFFSET) &
		MII_MGMT_CMD_DATA_MASK;

	return (u16)val;
}

static void mdio_write(struct iproc_pcie *pcie, unsigned int phy_addr,
		unsigned int reg_addr, u16 wr_data)
{
	u32 val;

	WARN_ON(mdio_wait_idle(pcie));

	val = (MII_TA_VAL << MII_MGMT_CMD_TA_SHIFT);
	val |= (reg_addr << MII_MGMT_CMD_RA_SHIFT);
	val |= (phy_addr << MII_MGMT_CMD_PA_SHIFT);
	val |= (OPCODE_WRITE << MII_MGMT_CMD_OP_SHIFT);
	val |= (1 << MII_MGMT_CMD_SB_SHIFT);
	val |= ((u32)wr_data & MII_MGMT_CMD_DATA_MASK);
	writel(val, pcie->mii + MII_MGMT_CMD_DATA_OFFSET);

	WARN_ON(mdio_wait_idle(pcie));
}

#define PCIE_PHY_BLK_ADDR_OFFSET 0x1F
#define PCIE_PHY_BLK_ADDR_MASK   0xFFF0
#define PCIE_PHY_REG_ADDR_MASK   0xF
static u16 iproc_pcie_phy_reg_read(struct iproc_pcie *pcie,
		unsigned int phy_addr, unsigned int reg_addr)
{
	u16 val;

	mdio_write(pcie, phy_addr, PCIE_PHY_BLK_ADDR_OFFSET,
			reg_addr & PCIE_PHY_BLK_ADDR_MASK);
	val = mdio_read(pcie, phy_addr, reg_addr & PCIE_PHY_REG_ADDR_MASK);

	dev_dbg(pcie->dev, "phy rd: phy: 0x%0x reg: 0x%4x data: 0x%4x\n",
			phy_addr, reg_addr, val);

	return val;
}

static void iproc_pcie_phy_reg_write(struct iproc_pcie *pcie,
		unsigned int phy_addr, unsigned int reg_addr, u16 val)
{
	mdio_write(pcie, phy_addr, PCIE_PHY_BLK_ADDR_OFFSET,
			reg_addr & PCIE_PHY_BLK_ADDR_MASK);
	mdio_write(pcie, phy_addr, reg_addr & PCIE_PHY_REG_ADDR_MASK, val);

	dev_dbg(pcie->dev, "phy wr: phy: 0x%0x reg: 0x%4x data: 0x%4x\n",
			phy_addr, reg_addr, val);
}

static inline struct iproc_pcie *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static void iproc_pcie_reset(struct iproc_pcie *pcie)
{
	u32 val;

	/* send a downstream reset */
	val = EP_MODE_SURVIVE_PERST | RC_PCIE_RST_OUTPUT;
	writel(val, pcie->reg + CLK_CONTROL_OFFSET);
	udelay(250);
	val &= ~EP_MODE_SURVIVE_PERST;
	writel(val, pcie->reg + CLK_CONTROL_OFFSET);
	mdelay(250);
}

#define INVALID_ACCESS_OFFSET 0xFFFFFFFF
static u32 iproc_pcie_conf_access(struct iproc_pcie *pcie, struct pci_bus *bus,
		unsigned int devfn, int where)
{
	int busno = bus->number;
	int slot = PCI_SLOT(devfn);
	int fn = PCI_FUNC(devfn);
	u32 val;

	/* root complex access */
	if (busno == 0) {
		if (slot)
			return INVALID_ACCESS_OFFSET;
		writel(where & CFG_IND_ADDR_MASK,
				pcie->reg + CFG_IND_ADDR_OFFSET);
		return CFG_IND_DATA_OFFSET;
	}

	if (fn > 1)
		return INVALID_ACCESS_OFFSET;

	/* access of EP device */
	val = (bus->number << CFG_ADDR_BUS_NUM_SHIFT) |
		(PCI_SLOT(devfn) << CFG_ADDR_DEV_NUM_SHIFT) |
		(PCI_FUNC(devfn) << CFG_ADDR_FUNC_NUM_SHIFT) |
		(where & CFG_ADDR_REG_NUM_MASK) |
		(1 & CFG_ADDR_CFG_TYPE_MASK);
	writel(val, pcie->reg + CFG_ADDR_OFFSET);

	return CFG_DATA_OFFSET;
}

#define INVALID_CFG_RD 0xFFFFFFFF
static int iproc_pci_read_conf(struct pci_bus *bus, unsigned int devfn,
		int where, int size, u32 *val)
{
	u32 offset;
	struct iproc_pcie *pcie = sys_to_pcie(bus->sysdata);

	*val = INVALID_CFG_RD;

	if (size != 1 && size != 2 && size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	offset = iproc_pcie_conf_access(pcie, bus, devfn, where);
	if (offset == INVALID_ACCESS_OFFSET)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	*val = readl(pcie->reg + offset);

	switch (size) {
	case 4:
		/* return raw data */
		break;
	case 2:
		*val = (*val >> (8 * (where & 3))) & 0xFFFF;
		break;
	case 1:
		*val = (*val >> (8 * (where & 3))) & 0xFF;
		break;
	default:
		BUG_ON(1);
	}

	dev_dbg(pcie->dev, "conf rd: busn=%d devfn=%d where=%d size=%d val=0x%08x\n",
			bus->number, devfn, where, size, *val);

	return PCIBIOS_SUCCESSFUL;
}

static int iproc_pci_write_conf(struct pci_bus *bus, unsigned int devfn,
		int where, int size, u32 val)
{
	int shift;
	u32 offset, data;
	struct iproc_pcie *pcie = sys_to_pcie(bus->sysdata);

	if (size != 1 && size != 2 && size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	offset = iproc_pcie_conf_access(pcie, bus, devfn, where);
	if (offset == INVALID_ACCESS_OFFSET)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	data = readl(pcie->reg + offset);

	switch (size) {
	case 4:
		data = val;
		break;
	case 2:
		shift = 8 * (where & 2);
		data &= ~(0xFFFF << shift);
		data |= ((val & 0xFFFF) << shift);
		break;
	case 1:
		shift = 8 * (where & 3);
		data &= ~(0xFF << shift);
		data |= ((val & 0xFF) << shift);
		break;
	default:
		BUG_ON(1);
	}

	writel(data, pcie->reg + offset);

	dev_dbg(pcie->dev,
		"config wr: busn=%d devfn=%d where=%d size=%d data=0x%08x\n",
		bus->number, devfn, where, size, data);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops iproc_pcie_ops = {
	.read = iproc_pci_read_conf,
	.write = iproc_pci_write_conf,
};

static int iproc_pcie_check_link(struct iproc_pcie *pcie)
{
	int ret;
	u8 nlw;
	u16 pos, tmp16;
	u32 val;
	struct pci_sys_data sys;
	struct pci_bus bus;

	val = readl(pcie->reg + PCIE_LINK_STATUS_OFFSET);
	dev_dbg(pcie->dev, "link status: 0x%08x\n", val);

	val = readl(pcie->reg + STRAP_STATUS_OFFSET);
	dev_dbg(pcie->dev, "strap status: 0x%08x\n", val);

	memset(&sys, 0, sizeof(sys));
	memset(&bus, 0, sizeof(bus));

	bus.number = 0;
	bus.ops = &iproc_pcie_ops;
	bus.sysdata = &sys;
	sys.private_data = pcie;

	ret = iproc_pci_read_conf(&bus, 0, PCI_HEADER_TYPE, 1, &val);
	if (ret != PCIBIOS_SUCCESSFUL || val != PCI_HEADER_TYPE_BRIDGE) {
		dev_err(pcie->dev, "in EP mode, val=0x08%x\n", val);
		return -EFAULT;
	}

	/*
	 * Under RC mode, write to function specific register 0x43c, to change
	 * the CLASS code in configuration space
	 *
	 * After this modification, the CLASS code in configuration space would
	 * be read as PCI_CLASS_BRIDGE_PCI(0x0604)
	 */
#define PCI_BRIDGE_CTRL_REG_OFFSET 0x43C
#define PCI_CLASS_BRIDGE_PCI_MASK  0xFF0000FF
	pci_bus_read_config_dword(&bus, 0, PCI_BRIDGE_CTRL_REG_OFFSET, &val);
	val = (val & PCI_CLASS_BRIDGE_PCI_MASK) | (PCI_CLASS_BRIDGE_PCI << 8);
	pci_bus_write_config_dword(&bus, 0, PCI_BRIDGE_CTRL_REG_OFFSET, val);

	/* check link status to see if link is active */
	pos = pci_bus_find_capability(&bus, 0, PCI_CAP_ID_EXP);
	pci_bus_read_config_word(&bus, 0, pos + PCI_EXP_LNKSTA, &tmp16);
	tmp16 &= PCI_EXP_LNKSTA_DLLLA;
	nlw = (tmp16 & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;

	if (nlw == 0) {
		/* try GEN 1 link speed */
#define PCI_LINK_STATUS_CTRL_2_OFFSET 0xDC
#define PCI_TARGET_LINK_SPEED_MASK    0xF
#define PCI_TARGET_LINK_SPEED_GEN2    0x2
#define PCI_TARGET_LINK_SPEED_GEN1    0x1
		pci_bus_read_config_dword(&bus, 0,
				PCI_LINK_STATUS_CTRL_2_OFFSET, &val);
		if ((val & PCI_TARGET_LINK_SPEED_MASK) ==
				PCI_TARGET_LINK_SPEED_GEN2) {
			val &= ~PCI_TARGET_LINK_SPEED_MASK;
			val |= PCI_TARGET_LINK_SPEED_GEN1;
			pci_bus_write_config_dword(&bus, 0,
					PCI_LINK_STATUS_CTRL_2_OFFSET, val);
			pci_bus_read_config_dword(&bus, 0,
					PCI_LINK_STATUS_CTRL_2_OFFSET, &val);
			mdelay(100);

			pos = pci_bus_find_capability(&bus, 0, PCI_CAP_ID_EXP);
			pci_bus_read_config_word(&bus, 0, pos + PCI_EXP_LNKSTA,
					&tmp16);
			nlw = (tmp16 & PCI_EXP_LNKSTA_NLW) >>
				PCI_EXP_LNKSTA_NLW_SHIFT;
		}
	}

	dev_info(pcie->dev, "link: %s\n", nlw ? "UP" : "DOWN");

	return nlw ? 0 : -ENODEV;
}

static int iproc_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct iproc_pcie *pcie = sys_to_pcie(sys);

	pci_add_resource(&sys->resources, &pcie->io);
	pci_add_resource(&sys->resources, &pcie->mem);
	pci_add_resource(&sys->resources, &pcie->busn);

	return 1;
}

static struct pci_bus *iproc_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct iproc_pcie *pcie = sys->private_data;
	struct pci_bus *bus;

	bus = pci_create_root_bus(pcie->dev, sys->busnr, &iproc_pcie_ops, sys,
			&sys->resources);
	if (!bus)
		return NULL;

	if (IS_ENABLED(CONFIG_PCI_MSI))
		bus->msi = &pcie->msi.chip;

	pci_scan_child_bus(bus);

	return bus;
}

static struct hw_pci hw;

static void iproc_pcie_enable(struct iproc_pcie *pcie)
{
	hw.nr_controllers = 1;
	hw.private_data = (void **)&pcie;
	hw.setup = iproc_pcie_setup;
	hw.scan = iproc_pcie_scan_bus;
	hw.map_irq = of_irq_parse_and_map_pci;
	hw.ops = &iproc_pcie_ops;

	/* enable root complex INTX */
	writel(SYS_RC_INTX_MASK, pcie->reg + SYS_RC_INTX_EN);

	pci_common_init_dev(pcie->dev, &hw);
#ifdef CONFIG_PCI_DOMAINS
	hw.domain++;
#endif
}

#define PCIE_PHY_REG_ADDR 0x2103
#define PCIE_PHY_DATA     0x2B1C
static void iproc_pcie_mii_phy_init(struct iproc_pcie *pcie, u32 phy_addr)
{
	unsigned int reg_addr;
	u16 val;

	mdio_init(pcie);

	reg_addr = PCIE_PHY_REG_ADDR;
	val = PCIE_PHY_DATA;
	iproc_pcie_phy_reg_write(pcie, phy_addr, reg_addr, val);
	val = iproc_pcie_phy_reg_read(pcie, phy_addr, reg_addr);
	dev_info(pcie->dev, "phy: 0x%x reg: 0x%4x val: 0x%4x\n", phy_addr,
			reg_addr, val);
}

static inline struct iproc_msi *to_iproc_msi(struct msi_controller *chip)
{
	return container_of(chip, struct iproc_msi, chip);
}

static int iproc_msi_irq_assign(struct iproc_msi *chip)
{
	int msi;

	msi = find_first_zero_bit(chip->irq_in_use, MAX_IRQS);
	if (msi < MAX_IRQS)
		set_bit(msi, chip->irq_in_use);
	else
		msi = -ENOSPC;

	return msi;
}

static void iproc_msi_irq_free(struct iproc_msi *chip, unsigned long irq)
{
	clear_bit(irq, chip->irq_in_use);
}

static int iproc_msi_setup_irq(struct msi_controller *chip,
		struct pci_dev *pdev, struct msi_desc *desc)
{
	struct iproc_msi *msi = to_iproc_msi(chip);
	struct iproc_pcie *pcie = msi->pcie;
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;

	hwirq = iproc_msi_irq_assign(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq) {
		iproc_msi_irq_free(msi, hwirq);
		return -EINVAL;
	}

	dev_dbg(pcie->dev, "mapped irq:%d\n", irq);

	irq_set_msi_desc(irq, desc);

	msg.address_lo = virt_to_phys((void *)msi->msi_page) | (hwirq * 4);
	msg.address_hi = 0x0;
	msg.data = hwirq;

	write_msi_msg(irq, &msg);

	return 0;
}

static void iproc_msi_teardown_irq(struct msi_controller *chip,
		unsigned int irq)
{
	struct iproc_msi *msi = to_iproc_msi(chip);
	struct irq_data *data = irq_get_irq_data(irq);

	iproc_msi_irq_free(msi, data->hwirq);
}

static struct irq_chip iproc_msi_irq_chip = {
	.name = "iProc PCIe MSI",
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

static int iproc_msi_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &iproc_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

static const struct irq_domain_ops iproc_msi_domain_ops = {
	.map = iproc_msi_map,
};

static irqreturn_t iproc_msi_irq(int irq, void *data)
{
	struct iproc_pcie *pcie = data;
	unsigned int eq, head, tail, num_events;

	/* Do not handle INTx interrupt */
	if ((readl(pcie->reg + SYS_RC_INTX_CSR) & SYS_RC_INTX_MASK) != 0)
		return IRQ_NONE;

	eq = irq - pcie->irqs[0];
	BUG_ON(eq >= MAX_MSI_EQ);

	irq = irq_find_mapping(pcie->msi.domain, eq);
	head = readl(pcie->reg + SYS_EQ_HEAD_0_OFFSET + (eq * 8));
	do {
		tail = readl(pcie->reg + SYS_EQ_TAIL_0_OFFSET + (eq * 8));
		tail &= SYS_EQ_TAIL_0_MASK;

		num_events = (tail < head) ?
			(64 + (tail - head)) : (tail - head);
		if (!num_events)
			break;

		generic_handle_irq(irq);

		head++;
		head %= 64;
		writel(head, pcie->reg + SYS_EQ_HEAD_0_OFFSET + (eq * 8));
	} while (true);

	return IRQ_HANDLED;
}

static int iproc_pcie_enable_msi(struct iproc_pcie *pcie)
{
	struct iproc_msi *msi = &pcie->msi;
	struct device_node *np = pcie->dev->of_node;
	int i, ret;
	u32 val;

	msi->pcie = pcie;
	msi->chip.dev = pcie->dev;
	msi->chip.setup_irq = iproc_msi_setup_irq;
	msi->chip.teardown_irq = iproc_msi_teardown_irq;

	msi->domain = irq_domain_add_linear(pcie->dev->of_node, MAX_IRQS,
			&iproc_msi_domain_ops, &msi->chip);
	if (!msi->domain) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	for (i = 0; i < MAX_IRQS; i++) {
		ret = devm_request_irq(pcie->dev, pcie->irqs[i],
			iproc_msi_irq, IRQF_SHARED, "iproc-pcie", pcie);
		if (ret < 0) {
			dev_err(pcie->dev, "failed to request IRQ: %d\n",
					pcie->irqs[i]);
			goto err_rm_irq_domain;
		}
	}

	msi->eq_page = __get_free_pages(GFP_KERNEL, 0);
	if (!msi->eq_page) {
		dev_err(pcie->dev,
			"failed to allocate memory for MSI event queue\n");
		ret = -ENOMEM;
		goto err_rm_irq_domain;
	}

	msi->msi_page = __get_free_pages(GFP_KERNEL, 0);
	if (!msi->msi_page) {
		dev_err(pcie->dev,
			"failed to allocate memory for MSI\n");
		ret = -ENOMEM;
		goto err_free_msi_eq_page;
	}

	writel(virt_to_phys((void *)msi->eq_page),
			pcie->reg + SYS_EQ_PAGE_OFFSET);
	writel(virt_to_phys((void *)msi->msi_page),
			pcie->reg + SYS_MSI_PAGE_OFFSET);

	for (i = 0; i < MAX_MSI_EQ; i++) {
		/* enable MSI event queue and interrupt */
		val = SYS_MSI_INTR_EN | SYS_MSI_INT_N_EVENT | SYS_MSI_EQ_EN;
		writel(val, pcie->reg + SYS_MSI_CTRL_0_OFFSET + (i * 4));
		/*
		 * To support legacy platforms that require the MSI interrupt
		 * enable register to be set explicitly
		 */
		if (of_find_property(np, "have-msi-inten-reg", NULL)) {
			val = readl(pcie->reg + SYS_MSI_INTS_EN_OFFSET);
			val |= (1 << i);
			writel(val, pcie->reg + SYS_MSI_INTS_EN_OFFSET);
		}
	}

	dev_info(pcie->dev, "MSI enabled\n");
	return 0;

err_free_msi_eq_page:
	free_pages(msi->eq_page, 0);

err_rm_irq_domain:
	irq_domain_remove(msi->domain);
	return ret;
}

static int __init iproc_pcie_probe(struct platform_device *pdev)
{
	struct iproc_pcie *pcie;
	struct device_node *np = pdev->dev.of_node;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	struct resource res, regs;
	int i, ret;

	pcie = devm_kzalloc(&pdev->dev, sizeof(struct iproc_pcie),
			    GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = &pdev->dev;
	platform_set_drvdata(pdev, pcie);

	if (of_pci_parse_bus_range(pdev->dev.of_node, &pcie->busn)) {
		dev_err(&pdev->dev, "failed to parse bus-range property\n");
		return -EINVAL;
	}

	/* PCIE controller registers */
	ret = of_address_to_resource(np, 0, &regs);
	if (ret) {
		dev_err(pcie->dev, "unable to obtain device resources\n");
		return -ENODEV;
	}

	pcie->reg = devm_ioremap(pcie->dev, regs.start, resource_size(&regs));
	if (!pcie->reg) {
		dev_err(pcie->dev, "unable to map device reg resources\n");
		return -ENOMEM;
	}

	/* MDIO registers */
	ret = of_address_to_resource(np, 1, &regs);
	if (ret) {
		dev_err(pcie->dev, "unable to obtain device resources\n");
		return -ENODEV;
	}

	pcie->mii = devm_ioremap(pcie->dev, regs.start, resource_size(&regs));
	if (!pcie->mii) {
		dev_err(pcie->dev, "unable to map device mii resources\n");
		return -ENOMEM;
	}

	for (i = 0; i < MAX_IRQS; i++) {
		pcie->irqs[i] = irq_of_parse_and_map(np, i);
		if (!pcie->irqs[i]) {
			dev_err(pcie->dev, "unable to parse irq index:%d\n", i);
			return -ENODEV;
		}
	}

	if (of_property_read_u32(np, "phy-addr", &pcie->phy_addr)) {
		dev_err(pcie->dev, "missing \"phy-addr\" property in DT\n");
		return -EINVAL;
	}

	if (of_pci_range_parser_init(&parser, np)) {
		dev_err(pcie->dev, "missing \"ranges\" property in DT\n");
		return -EINVAL;
	}

	/* Get the PCI memory ranges from DT */
	for_each_of_pci_range(&parser, &range) {
		of_pci_range_to_resource(&range, np, &res);

		switch (res.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			memcpy(&pcie->io, &res, sizeof(res));
			pcie->io.name = "I/O";
			break;

		case IORESOURCE_MEM:
			memcpy(&pcie->mem, &res, sizeof(res));
			pcie->mem.name = "MEM";
			break;
		}
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = iproc_pcie_enable_msi(pcie);
		if (ret < 0) {
			dev_err(pcie->dev, "failed to enable MSI support\n");
			return ret;
		}
	}

	iproc_pcie_mii_phy_init(pcie, pcie->phy_addr);

	iproc_pcie_reset(pcie);

	ret = iproc_pcie_check_link(pcie);
	if (ret) {
		dev_err(pcie->dev, "no PCIe EP device detected\n");
		return ret;
	}

	iproc_pcie_enable(pcie);
	pci_assign_unassigned_resources();

	return 0;
}

static const struct of_device_id iproc_pcie_of_match_table[] = {
	{ .compatible = "brcm,iproc-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, iproc_pcie_of_match_table);

static struct platform_driver iproc_pcie_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "iproc-pcie",
		.of_match_table =
		   of_match_ptr(iproc_pcie_of_match_table),
	},
};

static int __init iproc_pcie_init(void)
{
	return platform_driver_probe(&iproc_pcie_driver,
			iproc_pcie_probe);
}
subsys_initcall(iproc_pcie_init);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom iPROC PCIe driver");
MODULE_LICENSE("GPL v2");
