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
/*
 * DESCRIPTION: The Broadcom iProc RNG200 Driver
 */

#include <linux/hw_random.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/delay.h>


/* Registers */
#define RNG_CTRL_OFFSET					0x00
#define RNG_CTRL_RNG_RBGEN_MASK				0x00001FFF
#define RNG_CTRL_RNG_RBGEN_ENABLE			0x00000001
#define RNG_CTRL_RNG_RBGEN_DISABLE			0x00000000

#define RNG_SOFT_RESET_OFFSET				0x04
#define RNG_SOFT_RESET_RNG_SOFT_RESET_MASK		0x00000001
#define RNG_SOFT_RESET_RNG_SOFT_RESET_ACTIVE		0x00000001
#define RNG_SOFT_RESET_RNG_SOFT_RESET_CLEAR		0x00000000

#define RBG_SOFT_RESET_OFFSET				0x08
#define RBG_SOFT_RESET_RNG_SOFT_RESET_MASK		0x00000001
#define RBG_SOFT_RESET_RNG_SOFT_RESET_ACTIVE		0x00000001
#define RBG_SOFT_RESET_RNG_SOFT_RESET_CLEAR		0x00000000

#define RNG_INT_STATUS_OFFSET				0x18
#define RNG_INT_STATUS_MASTER_FAIL_LOCKOUT_IRQ_MASK	0x80000000
#define RNG_INT_STATUS_STARTUP_TRANSITIONS_MET_IRQ_MASK	0x00020000
#define RNG_INT_STATUS_NIST_FAIL_IRQ_MASK		0x00000020
#define RNG_INT_STATUS_TOTAL_BITS_COUNT_IRQ_MASK	0x00000001

#define RNG_FIFO_DATA_OFFSET				0x20

#define RNG_FIFO_COUNT_OFFSET				0x24
#define RNG_FIFO_COUNT_RNG_FIFO_COUNT_MASK		0x000000FF

static void iproc_rng200_restart(void __iomem *rng_base)
{
	uint32_t val;

	/* Disable RBG */
	val = ioread32(rng_base + RNG_CTRL_OFFSET);
	val &= ~RNG_CTRL_RNG_RBGEN_MASK;
	val |= RNG_CTRL_RNG_RBGEN_DISABLE;
	iowrite32(val, rng_base + RNG_CTRL_OFFSET);

	/* Clear all interrupt status */
	iowrite32(0xFFFFFFFFUL, rng_base + RNG_INT_STATUS_OFFSET);

	/* Reset RNG and RBG */
	val = ioread32(rng_base + RBG_SOFT_RESET_OFFSET);
	val &= ~RBG_SOFT_RESET_RNG_SOFT_RESET_MASK;
	val |= RBG_SOFT_RESET_RNG_SOFT_RESET_ACTIVE;
	iowrite32(val, rng_base + RBG_SOFT_RESET_OFFSET);

	val = ioread32(rng_base + RNG_SOFT_RESET_OFFSET);
	val &= ~RNG_SOFT_RESET_RNG_SOFT_RESET_MASK;
	val |= RNG_SOFT_RESET_RNG_SOFT_RESET_ACTIVE;
	iowrite32(val, rng_base + RNG_SOFT_RESET_OFFSET);

	val = ioread32(rng_base + RNG_SOFT_RESET_OFFSET);
	val &= ~RNG_SOFT_RESET_RNG_SOFT_RESET_MASK;
	val |= RNG_SOFT_RESET_RNG_SOFT_RESET_CLEAR;
	iowrite32(val, rng_base + RNG_SOFT_RESET_OFFSET);

	val = ioread32(rng_base + RBG_SOFT_RESET_OFFSET);
	val &= ~RBG_SOFT_RESET_RNG_SOFT_RESET_MASK;
	val |= RBG_SOFT_RESET_RNG_SOFT_RESET_CLEAR;
	iowrite32(val, rng_base + RBG_SOFT_RESET_OFFSET);

	/* Enable RBG */
	val = ioread32(rng_base + RNG_CTRL_OFFSET);
	val &= ~RNG_CTRL_RNG_RBGEN_MASK;
	val |= RNG_CTRL_RNG_RBGEN_ENABLE;
	iowrite32(val, rng_base + RNG_CTRL_OFFSET);
}

static int iproc_rng200_read(struct hwrng *rng, void *buf, size_t max,
			       bool wait)
{
	uint32_t status = 0;
	uint32_t num_remaining = max;

	#define MAX_RESETS_PER_READ	1
	uint32_t num_resets = 0;

	#define MAX_IDLE_TIME	(1 * HZ)
	unsigned long idle_endtime = jiffies + MAX_IDLE_TIME;

	/* Retrieve HW RNG registers base address. */
	void __iomem *rng_base = (void __iomem *)rng->priv;

	while ((num_remaining > 0) && time_before(jiffies, idle_endtime)) {

		/* Is RNG sane? If not, reset it. */
		status = ioread32(rng_base + RNG_INT_STATUS_OFFSET);
		if ((status & (RNG_INT_STATUS_MASTER_FAIL_LOCKOUT_IRQ_MASK |
			RNG_INT_STATUS_NIST_FAIL_IRQ_MASK)) != 0) {

			if (num_resets >= MAX_RESETS_PER_READ)
				return max - num_remaining;

			iproc_rng200_restart(rng_base);
			num_resets++;
		}

		/* Are there any random numbers available? */
		if ((ioread32(rng_base + RNG_FIFO_COUNT_OFFSET) &
				RNG_FIFO_COUNT_RNG_FIFO_COUNT_MASK) > 0) {

			if (num_remaining >= sizeof(uint32_t)) {
				/* Buffer has room to store entire word */
				*(uint32_t *)buf = ioread32(rng_base +
							RNG_FIFO_DATA_OFFSET);
				buf += sizeof(uint32_t);
				num_remaining -= sizeof(uint32_t);
			} else {
				/* Buffer can only store partial word */
				uint32_t rnd_number = ioread32(rng_base +
							RNG_FIFO_DATA_OFFSET);
				memcpy(buf, &rnd_number, num_remaining);
				buf += num_remaining;
				num_remaining = 0;
			}

			/* Reset the IDLE timeout */
			idle_endtime = jiffies + MAX_IDLE_TIME;
		} else {
			if (!wait)
				/* Cannot wait, return immediately */
				return max - num_remaining;

			/* Can wait, give others chance to run */
			cpu_relax();
		}
	}

	return max - num_remaining;
}

static struct hwrng iproc_rng200_ops = {
	.name	= "iproc-rng200",
	.read	= iproc_rng200_read,
};

static int iproc_rng200_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	void __iomem *rng_base = 0;
	uint32_t val = 0;
	int err = 0;

	/* Map peripheral */
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res) {
		dev_err(dev, "failed to get rng resources\n");
		return -ENODEV;
	}

	rng_base = devm_ioremap_resource(dev, res);
	if (!rng_base) {
		dev_err(dev, "failed to remap rng regs\n");
		return -ENODEV;
	}

	iproc_rng200_ops.priv = (unsigned long)rng_base;

	/* Setup RNG. */
	val = ioread32(rng_base + RNG_CTRL_OFFSET);
	val &= ~RNG_CTRL_RNG_RBGEN_MASK;
	val |= RNG_CTRL_RNG_RBGEN_ENABLE;
	iowrite32(val, rng_base + RNG_CTRL_OFFSET);

	/* Register driver */
	err = hwrng_register(&iproc_rng200_ops);
	if (err) {
		dev_err(dev, "hwrng registration failed\n");
		return err;
	}
	dev_info(dev, "hwrng registered\n");

	return 0;
}

static int iproc_rng200_remove(struct platform_device *pdev)
{
	uint32_t val = 0;
	void __iomem *rng_base = (void __iomem *)iproc_rng200_ops.priv;

	/* Unregister driver */
	hwrng_unregister(&iproc_rng200_ops);

	/* Disable RNG hardware */
	val = ioread32(rng_base + RNG_CTRL_OFFSET);
	val &= ~RNG_CTRL_RNG_RBGEN_MASK;
	val |= RNG_CTRL_RNG_RBGEN_DISABLE;
	iowrite32(val, rng_base + RNG_CTRL_OFFSET);

	return 0;
}

static const struct of_device_id iproc_rng200_of_match[] = {
	{ .compatible = "brcm,iproc-rng200", },
	{},
};
MODULE_DEVICE_TABLE(of, iproc_rng200_of_match);

static struct platform_driver iproc_rng200_driver = {
	.driver = {
		.name = "iproc-rng200",
		.of_match_table = iproc_rng200_of_match,
	},
	.probe		= iproc_rng200_probe,
	.remove		= iproc_rng200_remove,
};
module_platform_driver(iproc_rng200_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("iProc RNG200 Random Number Generator driver");
MODULE_LICENSE("GPL v2");
