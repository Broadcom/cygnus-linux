/*
 * Copyright (C) 2015 Broadcom Corporation
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
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>

#define MSPI_SPCR0_LSB_OFFSET           0x0
#define MSPI_SPCR0_LSB_SHIFT            0
#define SPBR_MIN                        8U
#define SPBR_MAX                        255U
#define MSPI_SPCR0_MSB_OFFSET           0x4
#define MSPI_SPCR0_MSB_CPHA_SHIFT       0
#define MSPI_SPCR0_MSB_CPOL_SHIFT       1
#define MSPI_NEWQP_OFFSET               0x10
#define MSPI_ENDQP_OFFSET               0x14
#define MSPI_SPCR2_OFFSET               0x18
#define MSPI_SPCR2_SPE_SHIFT            6
#define MSPI_SPCR2_SPIFIE_SHIFT         5
#define MSPI_SPCR2_CONT_AFTER_CMD_SHIFT 7
#define MSPI_STATUS_OFFSET              0x20
#define MSPI_TXRAM_OFFSET               0x40
#define MSPI_RXRAM_OFFSET               0xc0
#define MSPI_CDRAM_OFFSET               0x140
#define MSPI_CDRAM_CONT_SHIFT           7
#define MSPI_CDRAM_BITSE_SHIFT          6
#define MSPI_CDRAM_PCS_SHIFT            0
#define MSPI_WRITE_LOCK_OFFSET          0x180
#define INTERRUPT_MSPI_DONE_OFFSET      0x14

#define NUM_SLOTS                       16
#define dev_dbg dev_err
struct bcm_mspi {
	struct platform_device  *pdev;
	void __iomem            *base;
    /* Base of interrupt control regs. */
	void __iomem            *int_base;
	struct spi_master       *master;
	struct clk              *clk;
	u32                     spbr;
	struct completion       xfer_done;
	int                     rx_slot;
};

/*
 * Calculate TXRAM offset for the requested slot.
 */
static inline u32 tx_offset(int slot)
{
	BUG_ON(slot > NUM_SLOTS);
	return 4 * (slot * 2);
}

/*
 * Calculate RXRAM offset for the requested slot.
 */
static inline u32 rx_offset(int slot)
{
	BUG_ON(slot > NUM_SLOTS);
	return 4 * (1 + slot * 2);
}

/*
 * Calculate CDRAM offset for the requested slot.
 */
static inline u32 cdram_offset(int slot)
{
	BUG_ON(slot > NUM_SLOTS);
	return MSPI_CDRAM_OFFSET + (4 * slot);
}

/*
 * Start tx or rx SPI transfer and wait until complete. The queue start pointer
 * is always 0. The end queue pointer is passed in to end_slot.
 *
 * @set_cont Set the CONT bit if true, clear if false. The CONT bit must be set
 * if all slots are filled and there is more data to be transferred.
 * @end_slot The end queue pointer.
 * Returns 0 if successful, -EIO if transfer timed out.
 */
static int bcm_mspi_transfer(struct bcm_mspi *mspi, bool set_cont,
	int end_slot)
{
	int val;
	int err = 0;

	if (end_slot > NUM_SLOTS - 1)
		end_slot = NUM_SLOTS - 1;

	/* Set queue pointers for transmit. */
	writel(0, mspi->base + MSPI_NEWQP_OFFSET);
	writel(end_slot, mspi->base + MSPI_ENDQP_OFFSET);
	dev_dbg(&mspi->pdev->dev, "NEWQP: %d ENDQP: %d\n", 0, end_slot);

	reinit_completion(&mspi->xfer_done);

	writel(1, mspi->base + MSPI_WRITE_LOCK_OFFSET);

	/*
	 * Start the transfer. CONT bit is set if another tx SPI transfer
	 * is required.
	 */
	val = (1 << MSPI_SPCR2_SPE_SHIFT) | (1 << MSPI_SPCR2_SPIFIE_SHIFT);
	if (set_cont)
		val |= (1 << MSPI_SPCR2_CONT_AFTER_CMD_SHIFT);

	writel(val, mspi->base + MSPI_SPCR2_OFFSET);
	dev_dbg(&mspi->pdev->dev, "SPCR2: 0x%x\n", val);

	/* Wait for interrupt indicating transfer is complete. */
	if (!wait_for_completion_timeout(&mspi->xfer_done,
		msecs_to_jiffies(10))) {
		dev_err(&mspi->pdev->dev,
			"timeout waiting for tx MSPI interrupt\n");
		err = -ETIMEDOUT;
	}

	writel(0, mspi->base + MSPI_WRITE_LOCK_OFFSET);

	return err;
}

/*
 * Copies data from tx buffer to the h/w tx buffer (TXRAM). When all tx slots
 * have been filled, a SPI transfer is initiated to send the data to the device.
 * Continues until all data has been sent.
 *
 * Data is copied into the h/w transmit buffer starting at TXRAM0. The CONT bit
 * is set in the corresponding command register (CDRAMx) if there is another
 * byte to send. It is cleared on the last byte.
 *
 * The number of bytes sent is saved and is the start of the receive buffer for
 * the next receive transfer. See bcm_mspi_rx_data().
 */
static int bcm_mspi_tx_data(struct spi_master *master,
	struct spi_device *spi_dev, struct spi_transfer *transfer)
{
	struct bcm_mspi *mspi = spi_master_get_devdata(master);
	int slot = 0;
	const u8 *buf = transfer->tx_buf;
	u32 val;
	int bytes_processed = 0;
	int err;

	if (!transfer->tx_buf)
		return 0;

	dev_dbg(&mspi->pdev->dev, "tx %d bytes\n", transfer->len);

	while (bytes_processed < transfer->len) {
		bool last_slot = false;

		/*
		 * Write data to slots until all are filled or all
		 * bytes written.
		 */
		for (slot = 0; slot < NUM_SLOTS && !last_slot; slot++) {
			u32 txram_offset = MSPI_TXRAM_OFFSET + tx_offset(slot);
			u32 msb = *buf++;

			val = (spi_dev->chip_select << MSPI_CDRAM_PCS_SHIFT) |
				(1 << MSPI_CDRAM_CONT_SHIFT);

			writel(msb, mspi->base + txram_offset);
			dev_dbg(&mspi->pdev->dev, "TXRAM: write 0x%x to 0x%x\n",
				msb, txram_offset);

			bytes_processed++;

			if (bytes_processed >= transfer->len)
				last_slot = true;

			/*
			 * Update command register. CONT cleared
			 * on last slot.
			 */
			if (last_slot)
				val &= ~(1 << MSPI_CDRAM_CONT_SHIFT);

			writel(val, mspi->base + cdram_offset(slot));
			dev_dbg(&mspi->pdev->dev, "CDRAM: write 0x%x to 0x%x\n",
				val, cdram_offset(slot));
		}

		/* Start transfer and wait until complete. */
		err = bcm_mspi_transfer(mspi, !last_slot, slot - 1);
		if (err)
			return err;
	}

	/* The rx data will go into RXRAM0/1 + last tx length. */
	if (slot >= NUM_SLOTS)
		mspi->rx_slot = 0;
	else
		mspi->rx_slot = slot;

	return 0;
}

/*
 * Receives data from the device by configuring the command registers (CDRAMx)
 * and then initiating a receive transfer. When the transfer is complete, the
 * data is copied to the receive buffer. Continues until all all data has
 * been received.
 *
 * The received data is copied into the RXRAM buffer starting at RXRAM0 + the
 * last tx length + 1. After the transfer is complete, the data is received
 * starting at RXRAM0 again until another tx transfer is done. The CONT bit
 * is always set on the command register (CDRAMx) corresponding to the RXRAM
 * slot. The next unused slot is also configured except the CONT bit is cleared.
 * This quirk applies to rx transfers only.
 */
static int bcm_mspi_rx_data(struct spi_master *master,
	struct spi_device *spi_dev, struct spi_transfer *transfer)
{
	struct bcm_mspi *mspi = spi_master_get_devdata(master);
	int slot, rx_slot, end_slot;
	int bytes_processed = 0;
	u8 *buf = transfer->rx_buf;
	u32 val = (spi_dev->chip_select << MSPI_CDRAM_PCS_SHIFT) |
		(1 << MSPI_CDRAM_CONT_SHIFT);
	int err;
	int slots_avail;
	int bytes = 0;

	if (!transfer->rx_buf)
		return 0;

	dev_dbg(&mspi->pdev->dev, "rx %d bytes\n", transfer->len);

    /* Receive all rx data. */
	while (bytes_processed < transfer->len) {
		bool xfer_complete = false;

		/* Set command register for each slot. */
		slots_avail = NUM_SLOTS - mspi->rx_slot;
		for (slot = 0; slot < slots_avail && slot < transfer->len &&
			bytes < transfer->len; slot++) {
			/* Update command register. */
			writel(val, mspi->base + cdram_offset(slot));
			dev_dbg(&mspi->pdev->dev, "CDRAM: write 0x%x to 0x%x\n",
				val, cdram_offset(slot));

			bytes++;
		}

		/* CONT bit is cleared on the next unused slot. */
		end_slot = slot - 1;
		if (slot + 1 < slots_avail) {
			end_slot += 1;
			val &= ~(1 << MSPI_CDRAM_CONT_SHIFT);
			dev_dbg(&mspi->pdev->dev, "CDRAM + 1: write 0x%x to 0x%x\n",
				val, cdram_offset(end_slot));
			writel(val, mspi->base + cdram_offset(end_slot));
		}

		/* Start transfer and wait until complete. */
		xfer_complete = (bytes == transfer->len);
		err = bcm_mspi_transfer(mspi, !xfer_complete, end_slot);
		if (err)
			return err;

		/* Copy data from rx registers to rx buffer. */
		for (rx_slot = mspi->rx_slot; rx_slot < NUM_SLOTS; rx_slot++) {
			u32 rxram_offset = MSPI_RXRAM_OFFSET +
				rx_offset(rx_slot);
			u32 msb = readl(mspi->base + rxram_offset);

			dev_dbg(&mspi->pdev->dev, "rxram offset: %x. data = 0x%x\n",
				rxram_offset, msb);

			*buf++ = (u8)msb;
			bytes_processed++;

			if (bytes_processed >= transfer->len)
				break;
		}

		/*
		 * The read pointer always starts at RXRAM0 after an rx transfer
		 * of any length.
		 */
		mspi->rx_slot = 0;
	}

	return 0;
}

/*
 * Sets CPOL, CPHA if requested by slave.
 */
static int bcm_mspi_setup(struct spi_device *spidev)
{
	struct bcm_mspi *mspi = spi_master_get_devdata(spidev->master);
	u32 val;

	val = readl(mspi->base + MSPI_SPCR0_MSB_OFFSET);

	if (spidev->mode & SPI_CPHA)
		val |= (1 << MSPI_SPCR0_MSB_CPHA_SHIFT);
	if (spidev->mode & SPI_CPOL)
		val |= (1 << MSPI_SPCR0_MSB_CPOL_SHIFT);

	writel(val, mspi->base + MSPI_SPCR0_MSB_OFFSET);

	return 0;
}

static int bcm_mspi_transfer_one(struct spi_master *master,
	struct spi_device *spidev, struct spi_transfer *transfer)
{
	int err;

	err = bcm_mspi_tx_data(master, spidev, transfer);
	if (err)
		return err;

	err = bcm_mspi_rx_data(master, spidev, transfer);
	if (err)
		return err;

	/* Delay requested amount before next transfer. */
	udelay(transfer->delay_usecs);

	return 0;
}

/*
 * The ISR is called when a SPI transfer has completed. SPIF (MSPI finished)
 * will be set in the status register.
 */
static irqreturn_t bcm_mspi_isr(int irq, void *data)
{
	struct platform_device *pdev = data;
	struct bcm_mspi *mspi = platform_get_drvdata(pdev);
	u32 val;

	val = readl(mspi->base + MSPI_STATUS_OFFSET);
	if (val & 1) {
		/* Clear interrupt then signal completion of transfer. */
		val &= ~1;
		writel(val, mspi->base + MSPI_STATUS_OFFSET);
		if (mspi->int_base)
			writel(1, mspi->int_base + INTERRUPT_MSPI_DONE_OFFSET);

		complete(&mspi->xfer_done);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void bcm_mspi_hw_init(struct bcm_mspi *mspi)
{
	/* Set SPBR (serial clock baud rate). */
	if (mspi->spbr)
		writel(mspi->spbr << MSPI_SPCR0_LSB_SHIFT,
			mspi->base + MSPI_SPCR0_LSB_OFFSET);
}

static const struct of_device_id bcm_mspi_dt[] = {
	{ .compatible = "brcm,mspi" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_mspi_dt);

static int bcm_mspi_probe(struct platform_device *pdev)
{
	struct bcm_mspi *data;
	struct spi_master *master;
	struct device *dev = &pdev->dev;
	int err;
	struct resource *res;
	unsigned int irq;

	master = spi_alloc_master(dev, sizeof(*data));
	if (!master) {
		dev_err(dev, "error allocating spi_master\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);

	data = spi_master_get_devdata(master);
	data->master = master;
	data->pdev = pdev;
	platform_set_drvdata(pdev, data);
	init_completion(&data->xfer_done);

	/* SPI master will always use the SPI device(s) from DT. */
	master->dev.of_node = dev->of_node;
	master->transfer_one = bcm_mspi_transfer_one;
	master->setup = bcm_mspi_setup;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->flags = SPI_MASTER_HALF_DUPLEX;
	master->mode_bits = SPI_MODE_3;

	/*
	 * Enable clock if provided. The frequency can be changed by setting
	 * SPBR (serial clock baud rate) based on the desired 'clock-frequency'.
	 *
	 * Baud rate is calculated as: mspi_clk / (2 * SPBR) where SPBR is a
	 * value between 1-255. If not set then it is left at the h/w default.
	 */
	data->clk = devm_clk_get(dev, "mspi_clk");
	if (!IS_ERR(data->clk)) {
		u32 desired_rate = 0;

		err = clk_prepare_enable(data->clk);
		if (err < 0) {
			dev_err(dev, "failed to enable clock: %d\n", err);
			goto out;
		}

		/* Calculate SPBR if clock-frequency provided. */
		of_property_read_u32(dev->of_node, "clock-frequency",
			&desired_rate);
		if (desired_rate > 0) {
			u32 spbr = clk_get_rate(data->clk) / (2 * desired_rate);

			if (spbr > 0) {
				data->spbr = clamp_val(spbr, SPBR_MIN,
					SPBR_MAX);
			 } else {
				dev_err(dev, "failed to get clock rate: %d\n",
					spbr);
				err = spbr;
				goto out;
			}
		}
	} else {
		/* Don't report error if clock not specified - it's optional. */
		if (PTR_ERR(data->clk) != -ENOENT) {
			err = PTR_ERR(data->clk);
			dev_err(dev, "failed to get clock: %d\n", err);
			goto out;
		}
	}

	/* Map base memory address. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(data->base)) {
		err = PTR_ERR(data->base);
		goto out;
	}

	/* Map interrupt control base memory address. Not used on all chips. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		data->int_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(data->int_base)) {
			err = PTR_ERR(data->base);
			goto out;
		}
	}

	/* Get IRQ. */
	irq = platform_get_irq(pdev, 0);
	if (irq == 0) {
		dev_err(&pdev->dev, "could not get IRQ\n");
		err = -EIO;
		goto out;
	}

	/* Initialize SPI controller. */
	bcm_mspi_hw_init(data);

	err = devm_request_irq(&pdev->dev, irq, bcm_mspi_isr,
		IRQF_SHARED, "bcm-mspi", pdev);
	if (err)
		goto out;

	err = devm_spi_register_master(dev, data->master);
	if (err)
		goto out;

	dev_info(dev, "BCM MSPI initialized successfully\n");

	return 0;

out:
	spi_master_put(data->master);
	return err;
}

static struct platform_driver bcm_mspi_driver = {
	.driver = {
		.name = "brcm,mspi-v0",
		.of_match_table = bcm_mspi_dt,
	},
	.probe = bcm_mspi_probe,
};

module_platform_driver(bcm_mspi_driver);

MODULE_DESCRIPTION("Broadcom MSPI SPI Controller driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
