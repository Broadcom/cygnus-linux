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
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/bcma/bcma.h>
#include <linux/spi/spi.h>

#include "spi-bcm-mspi.h"

#define BCM_MSPI_MAX_SPI_BAUD   13500000	/* 216 MHz? */

/* The longest observed required wait was 19 ms */
#define BCM_MSPI_SPE_TIMEOUT_MS 80

struct bcm_mspi {
	struct bcma_device *core;
	struct spi_master *master;

	size_t read_offset;
};

static inline u32 bcm_mspi_read(struct bcm_mspi *mspi, u16 offset)
{
	return bcma_read32(mspi->core, offset);
}

static inline void bcm_mspi_write(struct bcm_mspi *mspi, u16 offset,
				    u32 value)
{
	bcma_write32(mspi->core, offset, value);
}

static inline unsigned int bcm_mspi_calc_timeout(size_t len)
{
	/* Do some magic calculation based on length and buad. Add 10% and 1. */
	return (len * 9000 / BCM_MSPI_MAX_SPI_BAUD * 110 / 100) + 1;
}

static int bcm_mspi_wait(struct bcm_mspi *mspi, unsigned int timeout_ms)
{
	unsigned long deadline;
	u32 tmp;

	/* SPE bit has to be 0 before we read MSPI STATUS */
	deadline = jiffies + BCM_MSPI_SPE_TIMEOUT_MS * HZ / 1000;
	do {
		tmp = bcm_mspi_read(mspi, MSPI_SPCR2);
		if (!(tmp & MSPI_SPCR2_SPE))
			break;
		udelay(5);
	} while (!time_after_eq(jiffies, deadline));

	if (tmp & MSPI_SPCR2_SPE)
		goto spi_timeout;

	/* Check status */
	deadline = jiffies + timeout_ms * HZ / 1000;
	do {
		tmp = bcm_mspi_read(mspi, MSPI_STATUS);
		if (tmp & MSPI_STATUS_SPIF) {
			bcm_mspi_write(mspi, MSPI_STATUS, 0);
			return 0;
		}

		cpu_relax();
		udelay(100);
	} while (!time_after_eq(jiffies, deadline));

spi_timeout:
	bcm_mspi_write(mspi, MSPI_STATUS, 0);

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
		bcm_mspi_write(mspi, MSPI_TXRAM + 4 * (i * 2),
				 (unsigned int)w_buf[i]);
	}

	for (i = 0; i < len; i++) {
		tmp = MSPI_CDRAM_CONT | MSPI_CDRAM_PCS_DISABLE_ALL |
			MSPI_CDRAM_PCS_DSCK;
		if (!cont && i == len - 1)
			tmp &= ~MSPI_CDRAM_CONT;
		tmp &= ~0x1;
		/* Command Register File */
		bcm_mspi_write(mspi, MSPI_CDRAM + 4 * i, tmp);
	}

	/* Set queue pointers */
	bcm_mspi_write(mspi, MSPI_NEWQP, 0);
	bcm_mspi_write(mspi, MSPI_ENDQP, len - 1);

	if (cont)
		bcm_mspi_write(mspi, MSPI_WRITE_LOCK, 1);

	/* Start SPI transfer */
	tmp = bcm_mspi_read(mspi, MSPI_SPCR2);
	tmp |= MSPI_SPCR2_SPE;
	if (cont)
		tmp |= MSPI_SPCR2_CONT_AFTER_CMD;
	bcm_mspi_write(mspi, MSPI_SPCR2, tmp);

	/* Wait for SPI to finish */
	bcm_mspi_wait(mspi, bcm_mspi_calc_timeout(len));

	if (!cont)
		bcm_mspi_write(mspi, MSPI_WRITE_LOCK, 0);

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
		bcm_mspi_write(mspi, MSPI_CDRAM + 4 * i, tmp);
	}

	/* Set queue pointers */
	bcm_mspi_write(mspi, MSPI_NEWQP, 0);
	bcm_mspi_write(mspi, MSPI_ENDQP, mspi->read_offset + len - 1);

	if (cont)
		bcm_mspi_write(mspi, MSPI_WRITE_LOCK, 1);

	/* Start SPI transfer */
	tmp = bcm_mspi_read(mspi, MSPI_SPCR2);
	tmp |= MSPI_SPCR2_SPE;
	if (cont)
		tmp |= MSPI_SPCR2_CONT_AFTER_CMD;
	bcm_mspi_write(mspi, MSPI_SPCR2, tmp);

	/* Wait for SPI to finish */
	bcm_mspi_wait(mspi, bcm_mspi_calc_timeout(len));

	if (!cont)
		bcm_mspi_write(mspi, MSPI_WRITE_LOCK, 0);

	for (i = 0; i < len; ++i) {
		int offset = mspi->read_offset + i;

		/* Data stored in the transmit register file LSB */
		r_buf[i] = (u8)bcm_mspi_read(mspi,
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

static struct spi_board_info bcm_mspi_info = {
	.modalias	= "bcm53xxspiflash",
};

static const struct bcma_device_id bcm_mspi_bcma_tbl[] = {
	BCMA_CORE(BCMA_MANUF_BCM, BCMA_CORE_NS_QSPI, BCMA_ANY_REV,
		BCMA_ANY_CLASS),
	{},
};
MODULE_DEVICE_TABLE(bcma, bcm_mspi_bcma_tbl);

static int bcm_mspi_bcma_probe(struct bcma_device *core)
{
	struct bcm_mspi *data;
	struct spi_master *master;
	int err;

	dev_info(&core->dev, "BCM MSPI BCMA probe\n");

	if (core->bus->drv_cc.core->id.rev != 42) {
		pr_err("SPI on SoC with unsupported ChipCommon rev\n");
		return -ENOTSUPP;
	}

	master = spi_alloc_master(&core->dev, sizeof(*data));
	if (!master)
		return -ENOMEM;

	data = spi_master_get_devdata(master);
	data->master = master;
	data->core = core;

	master->transfer_one = bcm_mspi_transfer_one;

	bcma_set_drvdata(core, data);

	err = devm_spi_register_master(&core->dev, data->master);
	if (err) {
		spi_master_put(master);
		bcma_set_drvdata(core, NULL);
		goto out;
	}

	/* Broadcom SoCs (at least with the CC rev 42) use SPI for flash only */
	spi_new_device(master, &bcm_mspi_info);

out:
	return err;
}

static void bcm_mspi_bcma_remove(struct bcma_device *core)
{
	struct bcm_mspi *mspi = bcma_get_drvdata(core);

	spi_unregister_master(mspi->master);
}

static struct bcma_driver bcm_mspi_bcma_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= bcm_mspi_bcma_tbl,
	.probe		= bcm_mspi_bcma_probe,
	.remove		= bcm_mspi_bcma_remove,
};

static int __init bcm_mspi_module_init(void)
{
	int err = 0;

	err = bcma_driver_register(&bcm_mspi_bcma_driver);
	if (err)
		pr_err("Failed to register bcma driver: %d\n", err);

	return err;
}

static void __exit bcm_mspi_module_exit(void)
{
	bcma_driver_unregister(&bcm_mspi_bcma_driver);
}

module_init(bcm_mspi_module_init);
module_exit(bcm_mspi_module_exit);

MODULE_DESCRIPTION("Broadcom MSPI SPI Controller driver");
MODULE_AUTHOR("Rafał Miłecki <zajec5@gmail.com>");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
