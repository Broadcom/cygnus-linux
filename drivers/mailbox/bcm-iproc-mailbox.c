/*
 * Copyright (C) 2017 Broadcom.
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
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/mailbox_controller.h>
#include <linux/mailbox/bcm_iproc_mailbox.h>
#include <linux/delay.h>

#define IPROC_CRMU_MAILBOX0_OFFSET       0x0
#define IPROC_CRMU_MAILBOX1_OFFSET       0x4

#define M0_IPC_CMD_DONE_MASK             0x80000000
#define M0_IPC_CMD_REPLY_MASK            0x3fff0000
#define M0_IPC_CMD_REPLY_SHIFT           16

/* Max time the M0 will take to respond to a message. */
#define MAX_M0_TIMEOUT_MS                2

struct iproc_mbox {
	struct device         *dev;
	void __iomem          *base;
	spinlock_t            lock;
	struct mbox_controller controller;
	u32                   num_chans;
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
		unsigned long flags;
	int err = 0;
	const int poll_period_us = 5;
	const int max_retries = (MAX_M0_TIMEOUT_MS * 1000) / poll_period_us;

	if (!msg)
		return -EINVAL;

	spin_lock_irqsave(&mbox->lock, flags);

	dev_dbg(mbox->dev, "Send msg to M0: cmd=0x%x, param=0x%x, wait_ack=%d\n",
		msg->cmd, msg->param, msg->wait_ack);

	writel(msg->cmd, mbox->base + IPROC_CRMU_MAILBOX0_OFFSET);
	writel(msg->param, mbox->base + IPROC_CRMU_MAILBOX1_OFFSET);

	if (msg->wait_ack) {
		int retries;

		err = msg->reply_code = -ETIMEDOUT;
		for (retries = 0; retries < max_retries; retries++) {
			u32 val = readl(
				mbox->base + IPROC_CRMU_MAILBOX0_OFFSET);
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

static int iproc_mbox_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct iproc_mbox *iproc_mbox;
	struct device_node *node;
	const char *mbox_prop_name = "mboxes";
	struct mbox_chan *chans;

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

	if (iproc_mbox->num_chans == 0) {
		dev_err(dev, "No mailbox clients configured\n");
		return -ENODEV;
	}

	chans = devm_kzalloc(&pdev->dev,
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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iproc_mbox->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(iproc_mbox->base)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		return PTR_ERR(iproc_mbox->base);
	}

	return 0;
}

static struct platform_driver iproc_mbox_driver = {
	.driver = {
		.name = "brcm,iproc-mailbox",
		.of_match_table = iproc_mbox_of_match,
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
