/*
 * Copyright (C) 2016 Broadcom.
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
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox_client.h>
#include <linux/bcm_iproc_mailbox.h>
#include <linux/delay.h>

#define IPROC_CRMU_MAILBOX0_OFFSET       0x0
#define IPROC_CRMU_MAILBOX1_OFFSET       0x4

#define CRMU_IPROC_MAILBOX0_OFFSET       0x8
#define CRMU_IPROC_MAILBOX1_OFFSET       0xc

#define IPROC_INTR_STATUS                0x34
#define IPROC_MAILBOX_INTR_SHIFT         0
#define IPROC_MAILBOX_INTR_MASK          0x1

#define IPROC_INTR_CLEAR                 0x3c
#define IPROC_MAILBOX_INTR_CLR_SHIFT     0

#define M0_IPC_CMD_DONE_MASK             0x80000000
#define M0_IPC_CMD_REPLY_MASK            0x3fff0000
#define M0_IPC_CMD_REPLY_SHIFT           16

/* Domains that interrupts get forwarded to. */
enum mbox_domain {
	AON_GPIO_DOMAIN = 0,
};

enum iproc_m0_cmd {
	/*
	 * Enable/disable GPIO event forwarding from M0 to A9
	 * Param - 1 to enable, 0 to disable
	 * Response - return code
	 */
	M0_IPC_M0_CMD_AON_GPIO_FORWARDING_ENABLE = 0xe,

	/*
	 * AON GPIO interrupt ("forwarded" to IPROC)
	 * Param - AON GPIO mask
	 */
	M0_IPC_HOST_CMD_AON_GPIO_EVENT = 0x102,
};

struct iproc_mbox {
	struct device         *dev;
	void __iomem          *base;
	spinlock_t            lock;
	struct irq_domain     *irq_domain;
	struct mbox_controller controller;
	u32                   num_chans;
	int                   mbox_irq;
};

static struct lock_class_key mbox_lock_class;

static void iproc_mbox_irq_handler(struct irq_desc *desc)
{
	struct iproc_mbox *mbox = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long status;
	u32 cmd, param;
	int virq;

	chained_irq_enter(chip, desc);

	/* Determine type of interrupt. */
	status = readl(mbox->base + IPROC_INTR_STATUS);
	status = (status >> IPROC_MAILBOX_INTR_SHIFT) &
		IPROC_MAILBOX_INTR_MASK;

	/* Process mailbox interrupts. */
	if (status) {
		writel(1 << IPROC_MAILBOX_INTR_CLR_SHIFT,
			mbox->base + IPROC_INTR_CLEAR);

		cmd = readl(mbox->base + CRMU_IPROC_MAILBOX0_OFFSET);
		param = readl(mbox->base + CRMU_IPROC_MAILBOX1_OFFSET);

		dev_dbg(mbox->dev,
			"Received message from M0: cmd 0x%x param 0x%x\n",
			cmd, param);

		/* Process AON GPIO interrupt - forward to GPIO handler. */
		if (cmd == M0_IPC_HOST_CMD_AON_GPIO_EVENT) {
			virq = irq_find_mapping(mbox->irq_domain,
				AON_GPIO_DOMAIN);
			generic_handle_irq(virq);
		}
	}

	chained_irq_exit(chip, desc);
}

static int iproc_mbox_send_data_m0_imp(struct iproc_mbox *mbox,
	struct iproc_mbox_msg *msg, int max_retries, int poll_period_us)
{
	unsigned long flags;
	u32 val;
	int err = 0;
	int retries;

	spin_lock_irqsave(&mbox->lock, flags);

	dev_dbg(mbox->dev, "Send msg to M0: cmd=0x%x, param=0x%x, wait_ack=%d\n",
		msg->cmd, msg->param, msg->wait_ack);

	writel(msg->cmd, mbox->base + IPROC_CRMU_MAILBOX0_OFFSET);
	writel(msg->param, mbox->base + IPROC_CRMU_MAILBOX1_OFFSET);

	if (msg->wait_ack) {
		err = msg->reply_code = -ETIMEDOUT;
		for (retries = 0; retries < max_retries; retries++) {
			val = readl(mbox->base + IPROC_CRMU_MAILBOX0_OFFSET);
			if (val & M0_IPC_CMD_DONE_MASK) {
				/*
				 * M0 replied - save reply code and
				 * clear error.
				 */
				msg->reply_code = (val &
					M0_IPC_CMD_REPLY_MASK) >>
					M0_IPC_CMD_REPLY_SHIFT;
				err = 0;
				break;
			}
			udelay(poll_period_us);
		}
	}

	spin_unlock_irqrestore(&mbox->lock, flags);

	return err;
}

static void iproc_mbox_aon_gpio_forwarding_enable(struct iproc_mbox *mbox,
	bool en)
{
	struct iproc_mbox_msg msg;
	const int max_retries = 5;
	const int poll_period_us = 200;

	msg.cmd = M0_IPC_M0_CMD_AON_GPIO_FORWARDING_ENABLE;
	msg.param = en ? 1 : 0;
	msg.wait_ack = true;

	iproc_mbox_send_data_m0_imp(mbox, &msg, max_retries, poll_period_us);
}

static void iproc_mbox_irq_unmask(struct irq_data *d)
{
	struct iproc_mbox *iproc_mbox = irq_data_get_irq_chip_data(d);

	iproc_mbox_aon_gpio_forwarding_enable(iproc_mbox, true);
}

static void iproc_mbox_irq_mask(struct irq_data *d)
{
	/* Do nothing - Mask callback is not required, since upon GPIO event,
	 * M0 disables GPIO forwarding to A9. Hence, GPIO forwarding is already
	 * disabled  when in mbox irq handler, and no other mbox events from M0
	 * to A9 are expected until GPIO forwarding is enabled following
	 * iproc_mbox_irq_unmask()
	 */
}

static struct irq_chip iproc_mbox_irq_chip = {
	.name = "bcm-iproc-mbox",
	.irq_mask = iproc_mbox_irq_mask,
	.irq_unmask = iproc_mbox_irq_unmask,
};

static int iproc_mbox_irq_map(struct irq_domain *d, unsigned int irq,
	irq_hw_number_t hwirq)
{
	int ret;

	ret = irq_set_chip_data(irq, d->host_data);
	if (ret < 0)
		return ret;
	irq_set_lockdep_class(irq, &mbox_lock_class);
	irq_set_chip_and_handler(irq, &iproc_mbox_irq_chip,
		handle_simple_irq);

	return 0;
}

static void iproc_mbox_irq_unmap(struct irq_domain *d, unsigned int irq)
{
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static struct irq_domain_ops iproc_mbox_irq_ops = {
	.map = iproc_mbox_irq_map,
	.unmap = iproc_mbox_irq_unmap,
	.xlate = irq_domain_xlate_onecell,
};

static const struct of_device_id iproc_mbox_of_match[] = {
	{ .compatible = "brcm,iproc-mailbox" },
	{ }
};
MODULE_DEVICE_TABLE(of, iproc_mbox_of_match);

/*
 * Sends a message to M0. The mailbox framework prevents multiple accesses to
 * the same channel but there is only one h/w "channel". This driver allows
 * multiple clients to create channels to the controller but must serialize
 * access to the mailbox registers used to communicate with the M0.
 */
static int iproc_mbox_send_data_m0(struct mbox_chan *chan, void *data)
{
	struct iproc_mbox *mbox = dev_get_drvdata(chan->mbox->dev);
	struct iproc_mbox_msg *msg = (struct iproc_mbox_msg *)data;
	int err = 0;
	const int poll_period_us = 5;
	int max_retries;

	if (!msg)
		return -EINVAL;

	/* At least 1 attempt for misconfigured clients. */
	if (chan->cl->tx_tout == 0)
		max_retries = 1;
	else
		max_retries = (chan->cl->tx_tout * 1000) / poll_period_us;

	err = iproc_mbox_send_data_m0_imp(mbox, msg, max_retries,
		poll_period_us);

	return err;
}

static int iproc_mbox_startup(struct mbox_chan *chan)
{
	/* Do nothing. */
	return 0;
}

static void iproc_mbox_shutdown(struct mbox_chan *chan)
{
	/* Do nothing. */
}

static struct mbox_chan_ops iproc_mbox_ops = {
	.send_data    = iproc_mbox_send_data_m0,
	.startup      = iproc_mbox_startup,
	.shutdown     = iproc_mbox_shutdown,
};

static int __init iproc_mbox_probe(struct platform_device *pdev)
{
	int virq;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct iproc_mbox *iproc_mbox;
	int err;
	struct device_node *node;
	const char *mbox_prop_name = "mboxes";

	dev_info(&pdev->dev, "Initializing iproc mailbox controller\n");

	iproc_mbox = devm_kzalloc(dev, sizeof(*iproc_mbox), GFP_KERNEL);
	if (!iproc_mbox)
		return -ENOMEM;

	iproc_mbox->dev = dev;
	spin_lock_init(&iproc_mbox->lock);

	platform_set_drvdata(pdev, iproc_mbox);

	/* Count number of "mboxes" properties to determine # channels. */
	for_each_of_allnodes(node) {
		struct property *prop = of_find_property(
			node, mbox_prop_name, NULL);
		if (prop) {
			struct device_node *mbox_phandle = of_parse_phandle(
				node, mbox_prop_name, 0);
			if (mbox_phandle == dev->of_node)
				iproc_mbox->num_chans++;
		}
	}

	/*
	 * Allocate mailbox channels. If the mailbox driver is only being used
	 * to forward interrupts to the gpio domain then there may be no
	 * clients.
	 */
	if (iproc_mbox->num_chans > 0) {
		struct mbox_chan *chans = devm_kzalloc(&pdev->dev,
			sizeof(*chans) * iproc_mbox->num_chans, GFP_KERNEL);

		if (!chans)
			return -ENOMEM;

		/* Initialize mailbox controller. */
		iproc_mbox->controller.dev = iproc_mbox->dev;
		iproc_mbox->controller.num_chans = iproc_mbox->num_chans;
		iproc_mbox->controller.chans = chans;
		iproc_mbox->controller.ops = &iproc_mbox_ops;
		iproc_mbox->controller.txdone_irq = false;
		iproc_mbox->controller.txdone_poll = false;
		err = mbox_controller_register(&iproc_mbox->controller);
		if (err) {
			dev_err(&pdev->dev, "Register mailbox failed\n");
			return err;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iproc_mbox->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(iproc_mbox->base)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		return PTR_ERR(iproc_mbox->base);
	}

	iproc_mbox->mbox_irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!iproc_mbox->mbox_irq) {
		dev_err(&pdev->dev, "irq_of_parse_and_map failed\n");
		return -ENODEV;
	}

	iproc_mbox->irq_domain = irq_domain_add_linear(dev->of_node, 1,
		&iproc_mbox_irq_ops, iproc_mbox);
	if (!iproc_mbox->irq_domain) {
		dev_err(&pdev->dev, "unable to allocate IRQ domain\n");
		err = -ENXIO;
		goto dispose_mapping;
	}

	/* Map irq for AON GPIO interrupt handling into this domain. */
	virq = irq_create_mapping(iproc_mbox->irq_domain, AON_GPIO_DOMAIN);
	if (!virq) {
		dev_err(&pdev->dev, "failed mapping irq into domain\n");
		err = -ENXIO;
		goto domain_remove;
	}
	dev_dbg(&pdev->dev, "irq for aon gpio domain: %d\n", virq);

	irq_set_chained_handler_and_data(iproc_mbox->mbox_irq,
		iproc_mbox_irq_handler, iproc_mbox);

	return 0;

domain_remove:
	irq_domain_remove(iproc_mbox->irq_domain);

dispose_mapping:
	irq_dispose_mapping(iproc_mbox->mbox_irq);

	return err;
}

#ifdef CONFIG_PM_SLEEP
static int iproc_mbox_suspend(struct device *dev)
{
	struct iproc_mbox *mbox = dev_get_drvdata(dev);

	dev_info(dev,
		"Suspending mailbox controller: disabling GPIO forwarding\n");
	iproc_mbox_aon_gpio_forwarding_enable(mbox, false);
	synchronize_irq(mbox->mbox_irq);

	return 0;
}

static int iproc_mbox_resume(struct device *dev)
{
	struct iproc_mbox *mbox = dev_get_drvdata(dev);

	dev_info(dev,
		"Resuming mailbox controller: enabling AON GPIO forwarding\n");
	iproc_mbox_aon_gpio_forwarding_enable(mbox, true);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(iproc_mbox_pm_ops, iproc_mbox_suspend,
	iproc_mbox_resume);

struct platform_driver iproc_mbox_driver = {
	.driver = {
		.name = "brcm,iproc-mailbox",
		.of_match_table = iproc_mbox_of_match,
		.pm = &iproc_mbox_pm_ops,
	},
	.probe = iproc_mbox_probe,
};

static int __init iproc_mbox_init(void)
{
	return platform_driver_register(&iproc_mbox_driver);
}
arch_initcall(iproc_mbox_init);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom iProc Mailbox Driver");
MODULE_LICENSE("GPL v2");
