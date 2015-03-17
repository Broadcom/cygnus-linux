#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/of.h>

#include "spi-bcm53xx.h"

#define BCM53XXSPI_MAX_SPI_BAUD	13500000	/* 216 MHz? */

/* The longest observed required wait was 19 ms */
#define BCM53XXSPI_SPE_TIMEOUT_MS	80

struct bcm53xxspi {
	void __iomem *base;
	struct spi_master *master;

	size_t read_offset;
};

static inline u32 bcm53xxspi_read(struct bcm53xxspi *b53spi, u16 offset)
{
	return readl(b53spi->base + offset);
}

static inline void bcm53xxspi_write(struct bcm53xxspi *b53spi, u16 offset,
				    u32 value)
{
	writel(value, b53spi->base + offset);
}

static inline unsigned int bcm53xxspi_calc_timeout(size_t len)
{
	/* Do some magic calculation based on length and buad. Add 10% and 1. */
	return (len * 9000 / BCM53XXSPI_MAX_SPI_BAUD * 110 / 100) + 1;
}

static int bcm53xxspi_wait(struct bcm53xxspi *b53spi, unsigned int timeout_ms)
{
	unsigned long deadline;
	u32 tmp;

	/* SPE bit has to be 0 before we read MSPI STATUS */
	deadline = jiffies + BCM53XXSPI_SPE_TIMEOUT_MS * HZ / 1000;
	do {
		tmp = bcm53xxspi_read(b53spi, B53SPI_MSPI_SPCR2);
		if (!(tmp & B53SPI_MSPI_SPCR2_SPE))
			break;
		udelay(5);
	} while (!time_after_eq(jiffies, deadline));

	if (tmp & B53SPI_MSPI_SPCR2_SPE)
		goto spi_timeout;

	/* Check status */
	deadline = jiffies + timeout_ms * HZ / 1000;
	do {
		tmp = bcm53xxspi_read(b53spi, B53SPI_MSPI_MSPI_STATUS);
		if (tmp & B53SPI_MSPI_MSPI_STATUS_SPIF) {
			bcm53xxspi_write(b53spi, B53SPI_MSPI_MSPI_STATUS, 0);
			return 0;
		}

		cpu_relax();
		udelay(100);
	} while (!time_after_eq(jiffies, deadline));

spi_timeout:
	bcm53xxspi_write(b53spi, B53SPI_MSPI_MSPI_STATUS, 0);

	pr_err("Timeout waiting for SPI to be ready!\n");

	return -EBUSY;
}

static void bcm53xxspi_buf_write(struct bcm53xxspi *b53spi, u8 *w_buf,
				 size_t len, bool cont)
{
	u32 tmp;
	int i;

	for (i = 0; i < len; i++) {
		/* Transmit Register File MSB */
		bcm53xxspi_write(b53spi, B53SPI_MSPI_TXRAM + 4 * (i * 2),
				 (unsigned int)w_buf[i]);
	}

	for (i = 0; i < len; i++) {
		tmp = B53SPI_CDRAM_CONT | B53SPI_CDRAM_PCS_DISABLE_ALL |
		      B53SPI_CDRAM_PCS_DSCK;
		if (!cont && i == len - 1)
			tmp &= ~B53SPI_CDRAM_CONT;
		tmp &= ~0x1;
		/* Command Register File */
		bcm53xxspi_write(b53spi, B53SPI_MSPI_CDRAM + 4 * i, tmp);
	}

	/* Set queue pointers */
	bcm53xxspi_write(b53spi, B53SPI_MSPI_NEWQP, 0);
	bcm53xxspi_write(b53spi, B53SPI_MSPI_ENDQP, len - 1);

	if (cont)
		bcm53xxspi_write(b53spi, B53SPI_MSPI_WRITE_LOCK, 1);

	/* Start SPI transfer */
	tmp = bcm53xxspi_read(b53spi, B53SPI_MSPI_SPCR2);
	tmp |= B53SPI_MSPI_SPCR2_SPE;
	if (cont)
		tmp |= B53SPI_MSPI_SPCR2_CONT_AFTER_CMD;
	bcm53xxspi_write(b53spi, B53SPI_MSPI_SPCR2, tmp);

	/* Wait for SPI to finish */
	bcm53xxspi_wait(b53spi, bcm53xxspi_calc_timeout(len));

	if (!cont)
		bcm53xxspi_write(b53spi, B53SPI_MSPI_WRITE_LOCK, 0);

	b53spi->read_offset = len;
}

static void bcm53xxspi_buf_read(struct bcm53xxspi *b53spi, u8 *r_buf,
				size_t len, bool cont)
{
	u32 tmp;
	int i;

	for (i = 0; i < b53spi->read_offset + len; i++) {
		tmp = B53SPI_CDRAM_CONT | B53SPI_CDRAM_PCS_DISABLE_ALL |
		      B53SPI_CDRAM_PCS_DSCK;
		if (!cont && i == b53spi->read_offset + len - 1)
			tmp &= ~B53SPI_CDRAM_CONT;
		tmp &= ~0x1;
		/* Command Register File */
		bcm53xxspi_write(b53spi, B53SPI_MSPI_CDRAM + 4 * i, tmp);
	}

	/* Set queue pointers */
	bcm53xxspi_write(b53spi, B53SPI_MSPI_NEWQP, 0);
	bcm53xxspi_write(b53spi, B53SPI_MSPI_ENDQP,
			 b53spi->read_offset + len - 1);

	if (cont)
		bcm53xxspi_write(b53spi, B53SPI_MSPI_WRITE_LOCK, 1);

	/* Start SPI transfer */
	tmp = bcm53xxspi_read(b53spi, B53SPI_MSPI_SPCR2);
	tmp |= B53SPI_MSPI_SPCR2_SPE;
	if (cont)
		tmp |= B53SPI_MSPI_SPCR2_CONT_AFTER_CMD;
	bcm53xxspi_write(b53spi, B53SPI_MSPI_SPCR2, tmp);

	/* Wait for SPI to finish */
	bcm53xxspi_wait(b53spi, bcm53xxspi_calc_timeout(len));

	if (!cont)
		bcm53xxspi_write(b53spi, B53SPI_MSPI_WRITE_LOCK, 0);

	for (i = 0; i < len; ++i) {
		int offset = b53spi->read_offset + i;

		/* Data stored in the transmit register file LSB */
		r_buf[i] = (u8)bcm53xxspi_read(b53spi, B53SPI_MSPI_RXRAM + 4 * (1 + offset * 2));
	}

	b53spi->read_offset = 0;
}

static int bcm53xxspi_transfer_one(struct spi_master *master,
				   struct spi_device *spi,
				   struct spi_transfer *t)
{
	struct bcm53xxspi *b53spi = spi_master_get_devdata(master);
	u8 *buf;
	size_t left;

	if (t->tx_buf) {
		buf = (u8 *)t->tx_buf;
		left = t->len;
		while (left) {
			size_t to_write = min_t(size_t, 16, left);
			bool cont = left - to_write > 0;

			bcm53xxspi_buf_write(b53spi, buf, to_write, cont);
			left -= to_write;
			buf += to_write;
		}
	}

	if (t->rx_buf) {
		buf = (u8 *)t->rx_buf;
		left = t->len;
		while (left) {
			size_t to_read = min_t(size_t, 16 - b53spi->read_offset,
					       left);
			bool cont = left - to_read > 0;

			bcm53xxspi_buf_read(b53spi, buf, to_read, cont);
			left -= to_read;
			buf += to_read;
		}
	}

	return 0;
}

static int bcm53xxspi_bcma_probe(struct platform_device *pdev)
{
	struct bcm53xxspi *priv;
	struct device *dev = &pdev->dev;
	struct bcm53xxspi *b53spi;
	struct spi_master *master;
	int err;
	struct resource *res;

	dev_info(dev, "Entering BCM MSPI probe\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	master = spi_alloc_master(dev, sizeof(*b53spi));
	if (!master)
		return -ENOMEM;

	b53spi = spi_master_get_devdata(master);
	b53spi->master = master;
	b53spi->master->dev.of_node = dev->of_node;

	master->transfer_one = bcm53xxspi_transfer_one;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	b53spi->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		err = PTR_ERR(priv->base);
		goto out;
	}

	platform_set_drvdata(pdev, priv);

	err = devm_spi_register_master(dev, master);
	if (err)
		goto out;

	return 0;

out:
	spi_master_put(master);
	return err;
}

static int bcm53xxspi_bcma_remove(struct platform_device *pdev)
{
	struct bcm53xxspi *priv = platform_get_drvdata(pdev);

	spi_unregister_master(priv->master);

	return 0;
}

static const struct of_device_id bcm_mspi_dt[] = {
	{ .compatible = "brcm,mspi" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_mspi_dt);

static struct platform_driver driver = {
    .driver = {
        .name = "bcm-mspi",
		.of_match_table = bcm_mspi_dt,
    },
    .probe = bcm53xxspi_bcma_probe,
    .remove = bcm53xxspi_bcma_remove,
};

static int __init bcm53xxspi_module_init(void)
{
	platform_driver_register(&driver);

	return 0;
}

static void __exit bcm53xxspi_module_exit(void)
{
	platform_driver_unregister(&driver);
}

module_init(bcm53xxspi_module_init);
module_exit(bcm53xxspi_module_exit);

MODULE_DESCRIPTION("Broadcom BCM53xx SPI Controller driver");
MODULE_AUTHOR("Rafał Miłecki <zajec5@gmail.com>");
MODULE_LICENSE("GPL");
