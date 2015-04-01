/*
 * Portions Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/bcma/bcma.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/clk.h>

#include "spi-bcm-mspi.h"

#define BCM_MSPI_MAX_SPI_BAUD   13500000	/* 216 MHz? */
#define SPBR_MIN                8U
#define SPBR_MAX                255U
#define MSPI_SPCR0_LSB_OFFSET   0x200
#define MSPI_SPCR0_LSB_SHIFT    0

/* The longest observed required wait was 19 ms */
#define BCM_MSPI_SPE_TIMEOUT_MS 80

struct bcm_mspi {
#ifdef CONFIG_SPI_BCMA_MSPI
	struct bcma_device *core;
#endif

	void __iomem *base;
	struct spi_master *master;
	struct clk *clk;
	size_t read_offset;
	u32    spbr;

	void (*mspi_write)(struct bcm_mspi *mspi, u16 offset, u32 value);
	u32 (*mspi_read)(struct bcm_mspi *mspi, u16 offset);
};

static inline unsigned int bcm_mspi_calc_timeout(size_t len)
{
	/* Do some magic calculation based on length and buad. Add 10% and 1. */
	return (len * 9000 / BCM_MSPI_MAX_SPI_BAUD * 110 / 100) + 1;
}

static void bcm_mspi_hw_init(struct bcm_mspi *mspi)
{
	/* Set SPBR (serial clock baud rate). */
	if (mspi->spbr) {
		mspi->mspi_write(mspi, MSPI_SPCR0_LSB_OFFSET,
			mspi->spbr << MSPI_SPCR0_LSB_SHIFT);
	}
}

static int bcm_mspi_wait(struct bcm_mspi *mspi, unsigned int timeout_ms)
{
	unsigned long deadline;
	u32 tmp;

	/* SPE bit has to be 0 before we read MSPI STATUS */
	deadline = jiffies + BCM_MSPI_SPE_TIMEOUT_MS * HZ / 1000;
	do {
		tmp = mspi->mspi_read(mspi, MSPI_SPCR2);
		if (!(tmp & MSPI_SPCR2_SPE))
			break;
		udelay(5);
	} while (!time_after_eq(jiffies, deadline));

	if (tmp & MSPI_SPCR2_SPE)
		goto spi_timeout;

	/* Check status */
	deadline = jiffies + timeout_ms * HZ / 1000;
	do {
		tmp = mspi->mspi_read(mspi, MSPI_STATUS);
		if (tmp & MSPI_STATUS_SPIF) {
			mspi->mspi_write(mspi, MSPI_STATUS, 0);
			return 0;
		}

		cpu_relax();
		udelay(100);
	} while (!time_after_eq(jiffies, deadline));

spi_timeout:
	mspi->mspi_write(mspi, MSPI_STATUS, 0);

	pr_err("Timeout waiting for SPI to be ready!\n");

	return -EBUSY;
}

static void bcm_mspi_buf_write(struct bcm_mspi *mspi, u8 *w_buf,
				 size_t len, bool cont)
{
	u32 tmp;
	int i;

	for (i = 0; i < len; i++) {
		/* Transmit Register File MSB */
		mspi->mspi_write(mspi, MSPI_TXRAM + 4 * (i * 2),
				 (unsigned int)w_buf[i]);
	}

	for (i = 0; i < len; i++) {
		tmp = MSPI_CDRAM_CONT | MSPI_CDRAM_PCS_DISABLE_ALL |
			MSPI_CDRAM_PCS_DSCK;
		if (!cont && i == len - 1)
			tmp &= ~MSPI_CDRAM_CONT;
		tmp &= ~0x1;
		/* Command Register File */
		mspi->mspi_write(mspi, MSPI_CDRAM + 4 * i, tmp);
	}

	/* Set queue pointers */
	mspi->mspi_write(mspi, MSPI_NEWQP, 0);
	mspi->mspi_write(mspi, MSPI_ENDQP, len - 1);

	if (cont)
		mspi->mspi_write(mspi, MSPI_WRITE_LOCK, 1);

	/* Start SPI transfer */
	tmp = mspi->mspi_read(mspi, MSPI_SPCR2);
	tmp |= MSPI_SPCR2_SPE;
	if (cont)
		tmp |= MSPI_SPCR2_CONT_AFTER_CMD;
	mspi->mspi_write(mspi, MSPI_SPCR2, tmp);

	/* Wait for SPI to finish */
	bcm_mspi_wait(mspi, bcm_mspi_calc_timeout(len));

	if (!cont)
		mspi->mspi_write(mspi, MSPI_WRITE_LOCK, 0);

	mspi->read_offset = len;
}

static void bcm_mspi_buf_read(struct bcm_mspi *mspi, u8 *r_buf,
				size_t len, bool cont)
{
	u32 tmp;
	int i;

	for (i = 0; i < mspi->read_offset + len; i++) {
		tmp = MSPI_CDRAM_CONT | MSPI_CDRAM_PCS_DISABLE_ALL |
		      MSPI_CDRAM_PCS_DSCK;
		if (!cont && i == mspi->read_offset + len - 1)
			tmp &= ~MSPI_CDRAM_CONT;
		tmp &= ~0x1;
		/* Command Register File */
		mspi->mspi_write(mspi, MSPI_CDRAM + 4 * i, tmp);
	}

	/* Set queue pointers */
	mspi->mspi_write(mspi, MSPI_NEWQP, 0);
	mspi->mspi_write(mspi, MSPI_ENDQP,
			 mspi->read_offset + len - 1);

	if (cont)
		mspi->mspi_write(mspi, MSPI_WRITE_LOCK, 1);

	/* Start SPI transfer */
	tmp = mspi->mspi_read(mspi, MSPI_SPCR2);
	tmp |= MSPI_SPCR2_SPE;
	if (cont)
		tmp |= MSPI_SPCR2_CONT_AFTER_CMD;
	mspi->mspi_write(mspi, MSPI_SPCR2, tmp);

	/* Wait for SPI to finish */
	bcm_mspi_wait(mspi, bcm_mspi_calc_timeout(len));

	if (!cont)
		mspi->mspi_write(mspi, MSPI_WRITE_LOCK, 0);

	for (i = 0; i < len; ++i) {
		int offset = mspi->read_offset + i;

		/* Data stored in the transmit register file LSB */
		r_buf[i] = (u8)mspi->mspi_read(mspi,
			MSPI_RXRAM + 4 * (1 + offset * 2));
	}

	mspi->read_offset = 0;
}

static int bcm_mspi_transfer_one(struct spi_master *master,
				   struct spi_device *spi,
				   struct spi_transfer *t)
{
	struct bcm_mspi *mspi = spi_master_get_devdata(master);
	u8 *buf;
	size_t left;

	if (t->tx_buf) {
		buf = (u8 *)t->tx_buf;
		left = t->len;
		while (left) {
			size_t to_write = min_t(size_t, 16, left);
			bool cont = left - to_write > 0;

			bcm_mspi_buf_write(mspi, buf, to_write, cont);
			left -= to_write;
			buf += to_write;
		}
	}

	if (t->rx_buf) {
		buf = (u8 *)t->rx_buf;
		left = t->len;
		while (left) {
			size_t to_read = min_t(size_t, 16 - mspi->read_offset,
					       left);
			bool cont = left - to_read > 0;

			bcm_mspi_buf_read(mspi, buf, to_read, cont);
			left -= to_read;
			buf += to_read;
		}
	}

	return 0;
}

/*
 * Allocate SPI master for both bcma and non bcma bus. The SPI device must be
 * configured in DT.
 */
static struct bcm_mspi *bcm_mspi_init(struct device *dev)
{
	struct bcm_mspi *data;
	struct spi_master *master;
	u32 desired_rate;

	master = spi_alloc_master(dev, sizeof(*data));
	if (!master) {
		dev_err(dev, "error allocating spi_master\n");
		return 0;
	}

	data = spi_master_get_devdata(master);
	data->master = master;

	/* SPI master will always use the SPI device(s) from DT. */
	master->dev.of_node = dev->of_node;
	master->transfer_one = bcm_mspi_transfer_one;

	/*
	 * Enable clock if provided. The frequency can be changed by setting
	 * SPBR (serial clock baud rate) based on the desired 'clock-frequency'.
	 *
	 * Baud rate is calculated as: mspi_clk / (2 * SPBR) where SPBR is a
	 * value between 1-255. If not set then it is left at the h/w default.
	 */
	data->clk = devm_clk_get(dev, "mspi_clk");
	if (!IS_ERR(data->clk)) {
		int ret = clk_prepare_enable(data->clk);

		if (ret < 0) {
			dev_err(dev, "failed to enable clock: %d\n", ret);
			return 0;
		}

		/* Calculate SPBR if clock-frequency provided. */
		if (of_property_read_u32(dev->of_node, "clock-frequency",
			&desired_rate) >= 0) {
			u32 spbr = clk_get_rate(data->clk) / (2 * desired_rate);

			if (spbr > 0)
				data->spbr = clamp_val(spbr, SPBR_MIN,
					SPBR_MAX);
		}
	}

	return data;
}

#ifdef CONFIG_SPI_BCM_MSPI

static const struct of_device_id bcm_mspi_dt[] = {
	{ .compatible = "brcm,mspi" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_mspi_dt);

static inline u32 bcm_mspi_read(struct bcm_mspi *mspi, u16 offset)
{
	return readl(mspi->base + offset);
}

static inline void bcm_mspi_write(struct bcm_mspi *mspi, u16 offset,
	u32 value)
{
	writel(value, mspi->base + offset);
}

/*
 * Probe routine for non-bcma devices.
 */
static int bcm_mspi_probe(struct platform_device *pdev)
{
	struct bcm_mspi *data;
	struct device *dev = &pdev->dev;
	int err;
	struct resource *res;

	dev_info(dev, "BCM MSPI probe\n");

	data = bcm_mspi_init(dev);
	if (!data)
		return -ENOMEM;

	/* Map base memory address. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->base)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		err = PTR_ERR(data->base);
		goto out;
	}

	data->mspi_read = bcm_mspi_read;
	data->mspi_write = bcm_mspi_write;
	platform_set_drvdata(pdev, data);

	/* Initialize SPI controller. */
	bcm_mspi_hw_init(data);

	err = devm_spi_register_master(dev, data->master);
	if (err)
		goto out;

	return 0;

out:
	spi_master_put(data->master);
	return err;
}

static struct platform_driver bcm_mspi_driver = {
	.driver = {
		.name = "bcm-mspi",
		.of_match_table = bcm_mspi_dt,
	},
	.probe = bcm_mspi_probe,
};

module_platform_driver(bcm_mspi_driver);

#endif

#ifdef CONFIG_SPI_BCMA_MSPI

static const struct bcma_device_id bcm_mspi_bcma_tbl[] = {
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_NS_QSPI, BCMA_ANY_REV,
		BCMA_ANY_CLASS),
	{},
};
MODULE_DEVICE_TABLE(bcma, bcm_mspi_bcma_tbl);

static const struct of_device_id bcm_bcma_mspi_dt[] = {
	{ .compatible = "brcm,bcma-mspi" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_mspi_dt);

static inline u32 bcm_bcma_mspi_read(struct bcm_mspi *mspi, u16 offset)
{
	return bcma_read32(mspi->core, offset);
}

static inline void bcm_bcma_mspi_write(struct bcm_mspi *mspi, u16 offset,
	u32 value)
{
	bcma_write32(mspi->core, offset, value);
}

/*
 * Probe routine for bcma devices.
 */
static int bcm_mspi_bcma_probe(struct bcma_device *core)
{
	struct bcm_mspi *data;
	int err;

	dev_info(&core->dev, "BCM MSPI BCMA probe\n");

	if (core->bus->drv_cc.core->id.rev != 42) {
		dev_err(&core->dev,
			"SPI on SoC with unsupported ChipCommon rev\n");
		return -ENOTSUPP;
	}

	data = bcm_mspi_init(&core->dev);
	if (!data)
		return -ENOMEM;

	data->mspi_read = bcm_bcma_mspi_read;
	data->mspi_write = bcm_bcma_mspi_write;
	data->core = core;

	bcma_set_drvdata(core, data);

	/* Initialize SPI controller. */
	bcm_mspi_hw_init(data);

	err = devm_spi_register_master(&core->dev, data->master);
	if (err) {
		spi_master_put(data->master);
		return err;
	}

	return 0;
}

static struct bcma_driver bcm_mspi_bcma_driver = {
	.name		= KBUILD_MODNAME,
	.drv = {
		.of_match_table = bcm_bcma_mspi_dt,
	},
	.id_table	= bcm_mspi_bcma_tbl,
	.probe		= bcm_mspi_bcma_probe,
};

static int __init bcm_mspi_bcma_module_init(void)
{
	int err;

	err = bcma_driver_register(&bcm_mspi_bcma_driver);
	if (err)
		pr_err("Failed to register bcma driver: %d\n", err);

	return err;
}

static void __exit bcm_mspi_bcma_module_exit(void)
{
	bcma_driver_unregister(&bcm_mspi_bcma_driver);
}

module_init(bcm_mspi_bcma_module_init);
module_exit(bcm_mspi_bcma_module_exit);

#endif

MODULE_DESCRIPTION("Broadcom MSPI SPI Controller driver");
MODULE_AUTHOR("Rafał Miłecki <zajec5@gmail.com>");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
