/*
 * Copyright 2015 Broadcom Corporation.  All rights reserved.
 *
 * Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * This driver implements the Broadcom DTE Digital Timing Engine Driver (DTE).
 * The DTE allows for hardware time stamping of network packets, audio/video
 * samples and/or GPIO inputs in a common adjustable time base.
 */

#include <linux/hw_random.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/bcm_cygnus_dte.h>

#define DTE_SW_FIFO_SIZE 64
#define DTE_HW_FIFO_SIZE 16
#define DTE_MAX_DEVS 1
#define DTE_DEVICE_NAME	"dte"

/* Registers */
#define DTE_CTRL_REG_BASE                        0x600
#define DTE_CTRL_REG__INTERRUPT                  16
#define DTE_NEXT_SOI_REG_BASE                    0x610
#define DTE_ILEN_REG_BASE                        0x614
#define DTE_LTS_FIFO_REG_BASE                    0x640
#define DTE_LTS_CSR_REG_BASE                     0x644
#define DTE_LTS_CSR_REG__FIFO_EMPTY              4
#define DTE_LTS_CSR_REG__FIFO_OVERFLOW           3
#define DTE_LTS_CSR_REG__FIFO_UNDERFLOW          2
#define DTE_NCO_LOW_TIME_REG_BASE                0x650
#define DTE_NCO_TIME_REG_BASE                    0x654
#define DTE_NCO_OVERFLOW_REG_BASE                0x658
#define DTE_NCO_INC_REG_BASE                     0x65c
#define DTE_LTS_DIV_54_REG_BASE                  0x660
#define DTE_LTS_DIV_76_REG_BASE                  0x664
#define DTE_LTS_DIV_98_REG_BASE                  0x668
#define DTE_LTS_DIV_1110_REG_BASE                0x66c
#define DTE_LTS_DIV_1312_REG_BASE                0x670
#define DTE_LTS_DIV_14_REG_BASE                  0x674
#define DTE_LTS_DIV_MASK                         0xffff
#define DTE_LTS_DIV_54_REG__DIV_VAL_4_R          0
#define DTE_LTS_DIV_54_REG__DIV_VAL_5_R          16
#define DTE_LTS_DIV_76_REG__DIV_VAL_6_R          0
#define DTE_LTS_DIV_76_REG__DIV_VAL_7_R          16
#define DTE_LTS_DIV_98_REG__DIV_VAL_8_R          0
#define DTE_LTS_DIV_98_REG__DIV_VAL_9_R          16
#define DTE_LTS_DIV_1110_REG__DIV_VAL_10_R       0
#define DTE_LTS_DIV_1110_REG__DIV_VAL_11_R       16
#define DTE_LTS_DIV_1312_REG__DIV_VAL_12_R       0
#define DTE_LTS_DIV_1312_REG__DIV_VAL_13_R       16
#define DTE_LTS_DIV_14_REG__DIV_VAL_15_R         0
#define DTE_LTS_DIV_14_REG__DIV_VAL_14_R         16
#define DTE_LTS_SRC_EN_REG_BASE                  0x680
#define DTE_INTR_STATUS_REG                      0x6A0
#define DTE_INTR_STATUS_ISO_INTR_SHIFT           0
#define DTE_INTR_STATUS_ISO_INTR_MASK            0x1
#define DTE_INTR_STATUS_FIFO_LEVEL_INTR_SHIFT    1
#define DTE_INTR_STATUS_FIFO_LEVEL_INTR_MASK     0x7
#define DTE_INTR_STATUS_TIMEOUT_INTR_SHIFT       4
#define DTE_INTR_STATUS_TIMEOUT_INTR_MASK        0x1
#define DTE_INTR_MASK_REG                        0x6A4
#define ISO_INTR_MASK_SHIFT                      0

/*
 * There are 3 time registers in the DTE block;
 * NCO_OVERFLOW[7:0] (sum3)
 * NCO_TIME[31:0] (sum2)
 * NCO_LOW_TIME[31:0] (sum1).
 * These can be considered as a 72 bit overall timestamp[71:0]
 * in ns with a format of 44.28
 *
 * The DTE block runs at 125MHz, ie; every 8ns,
 * NCO_INC is added to the timestamp[71:0].
 * NCO_INC represents fractional ns in 4.28 format.
 *
 * The DTE client Timstamps have a resoultion of ns and is constructed
 * from the 28 LS bits of sum2 and 4 MS bits of sum1.
 *
 * Register            |     sum3    |          sum2     |         sum1       |
 *                     |7           0|31  28            0|31    28:27        0|
 * Timestamp[71:0]     |71                                      28:27        0|
 * NCO_INC[31:0]                                         |31    28:27        0|
 * DTE client TS[31:0]                      |31                  0|
 */

/* NCO nominal increment = 8ns DTE operates at 125 MHz */
#define NCO_INC_NOMINAL 0x80000000

struct bcm_cygnus_dte {
	struct list_head node;
	struct platform_device *pdev;
	struct cdev dte_cdev;
	struct class *dte_class;
	struct kfifo recv_fifo[DTE_CLIENT_MAX];
	dev_t  devt;
	uint32_t fifoof;
	uint32_t fifouf;
	uint32_t kfifoof[DTE_CLIENT_MAX];
	spinlock_t lock;
	struct mutex mutex;
	struct timespec ts_ref;           /* Reference base timespec */
	uint32_t timestamp_overflow_last; /* Last timestamp overflow */
	void __iomem *audioeav;
};

static LIST_HEAD(dtedev_list);

struct dte_client_mapping {
	enum dte_client client_index;
	int lts_index;
	uint32_t reg_offset;
	int shift;
};

#define DTE_CLIENT_MAPPING_ENTRY(client, lts_index, regbase) \
	{ DTE_CLIENT_##client, \
	  lts_index,\
	  DTE_LTS_##regbase##_BASE, \
	  DTE_LTS_##regbase##__DIV_VAL_##lts_index##_R }

static const struct dte_client_mapping dte_client_map_table[DTE_CLIENT_MAX] = {
	DTE_CLIENT_MAPPING_ENTRY(I2S0_BITCLOCK,  4, DIV_54_REG),
	DTE_CLIENT_MAPPING_ENTRY(I2S1_BITCLOCK,  5, DIV_54_REG),
	DTE_CLIENT_MAPPING_ENTRY(I2S2_BITCLOCK,  6, DIV_76_REG),
	DTE_CLIENT_MAPPING_ENTRY(I2S0_WORDCLOCK, 7, DIV_76_REG),
	DTE_CLIENT_MAPPING_ENTRY(I2S1_WORDCLOCK, 8, DIV_98_REG),
	DTE_CLIENT_MAPPING_ENTRY(I2S2_WORDCLOCK, 9, DIV_98_REG),
	DTE_CLIENT_MAPPING_ENTRY(LCD_CLFP,      10, DIV_1110_REG),
	DTE_CLIENT_MAPPING_ENTRY(LCD_CLLP,      11, DIV_1110_REG),
	DTE_CLIENT_MAPPING_ENTRY(GPIO14,        12, DIV_1312_REG),
	DTE_CLIENT_MAPPING_ENTRY(GPIO15,        13, DIV_1312_REG),
	DTE_CLIENT_MAPPING_ENTRY(GPIO22,        14, DIV_14_REG),
	DTE_CLIENT_MAPPING_ENTRY(GPIO23,        15, DIV_14_REG)
};

struct bcm_cygnus_dte *dte_get_dev_from_devname(const char *devname)
{
	struct bcm_cygnus_dte *cygnus_dte = NULL;
	bool found = false;

	if (!devname)
		return NULL;

	list_for_each_entry(cygnus_dte, &dtedev_list, node) {
		if (!strcmp(dev_name(&cygnus_dte->pdev->dev), devname)) {
			/* Matched on device name */
			found = true;
			break;
		}
	}

	return found ? cygnus_dte : NULL;
}
EXPORT_SYMBOL(dte_get_dev_from_devname);

int dte_enable_timestamp(
		struct bcm_cygnus_dte *cygnus_dte,
		enum dte_client client,
		int enable)
{
	int src_ena;

	if (!cygnus_dte)
		return -EINVAL;

	if (client < DTE_CLIENT_MIN || client >= DTE_CLIENT_MAX)
		return -EINVAL;

	spin_lock(&cygnus_dte->lock);

	src_ena = readl(cygnus_dte->audioeav +
		DTE_LTS_SRC_EN_REG_BASE);
	if (enable)
		src_ena |= BIT(dte_client_map_table[client].lts_index);
	else
		src_ena &= ~BIT(dte_client_map_table[client].lts_index);

	writel(src_ena,
		cygnus_dte->audioeav + DTE_LTS_SRC_EN_REG_BASE);

	spin_unlock(&cygnus_dte->lock);

	return 0;
}
EXPORT_SYMBOL(dte_enable_timestamp);

int dte_set_client_divider(
		struct bcm_cygnus_dte *cygnus_dte,
		enum dte_client client,
		int divider)
{
	int lts_div;

	if (!cygnus_dte)
		return -EINVAL;

	if (client < DTE_CLIENT_MIN || client >= DTE_CLIENT_MAX)
		return -EINVAL;

	/* Divider not supported for Divider 15 (GPIO23) */
	if (client == DTE_CLIENT_GPIO23)
		return -EINVAL;

	/* Check for maximum divider size */
	if (divider > DTE_LTS_DIV_MASK)
		return -EINVAL;

	spin_lock(&cygnus_dte->lock);

	lts_div = readl(cygnus_dte->audioeav +
			dte_client_map_table[client].reg_offset);
	lts_div &= ~(DTE_LTS_DIV_MASK << dte_client_map_table[client].shift);
	lts_div |= (divider << dte_client_map_table[client].shift);
	writel(lts_div, cygnus_dte->audioeav +
		dte_client_map_table[client].reg_offset);

	spin_unlock(&cygnus_dte->lock);

	return 0;
}
EXPORT_SYMBOL(dte_set_client_divider);

int dte_set_irq_interval(
		struct bcm_cygnus_dte *cygnus_dte,
		uint32_t nanosec)
{
	int next_soi;
	int current_time;
	int intr_mask;

	if (!cygnus_dte)
		return -EINVAL;

	spin_lock(&cygnus_dte->lock);

	/*
	 * To ensure proper operation of the isochronous time interval
	 * generation (ITG) timing pulse requires programming of
	 * 1) Next Start of Interrupt Time (ns) (NEXT_SOI)
	 * 2) Interval Length (ns) (ILEN)
	 * The block first compares the value of the NEXT_SOI with
	 * that of the 29-bit value of time DTE_NCO_TIME_REG_BASE
	 * that has been left-shifted by 4 or multiplied by 16 for
	 * generating the first interrupt.  Thereafter upon completion
	 * of every ILEN number of nano-seconds it generates an ITG
	 * interrupt and the NEXT_SOI register is auto-incremented by ILEN
	 */

	/* Get the current time (sum2) (units of 16ns) */
	current_time = readl(cygnus_dte->audioeav + DTE_NCO_TIME_REG_BASE);

	/*
	 * Set the Start of Next Interval (units of ns) to trigger on next
	 * interval
	 */
	next_soi = (current_time << 4) + (nanosec);
	writel(next_soi, cygnus_dte->audioeav + DTE_NEXT_SOI_REG_BASE);

	/* configure interval length (units of ns) */
	writel(nanosec, cygnus_dte->audioeav + DTE_ILEN_REG_BASE);

	/* enable isochronous interrupt */
	intr_mask = readl(cygnus_dte->audioeav + DTE_INTR_MASK_REG);
	intr_mask &= ~(1 << ISO_INTR_MASK_SHIFT);
	writel(intr_mask, cygnus_dte->audioeav + DTE_INTR_MASK_REG);

	spin_unlock(&cygnus_dte->lock);

	return 0;
}
EXPORT_SYMBOL(dte_set_irq_interval);

int dte_get_timestamp(
		struct bcm_cygnus_dte *cygnus_dte,
		enum dte_client client,
		struct timespec *ts)
{

	if (!cygnus_dte)
		return -EINVAL;

	if (client < DTE_CLIENT_MIN || client >= DTE_CLIENT_MAX)
		return -EINVAL;

	if (kfifo_out(
		&cygnus_dte->recv_fifo[client],
		ts, sizeof(struct timespec)) == 0)
		return -EPERM;

	return 0;
}
EXPORT_SYMBOL(dte_get_timestamp);

int dte_set_time(
		struct bcm_cygnus_dte *cygnus_dte,
		struct timespec *ts)
{
	int nco_incr;

	if (!cygnus_dte)
		return -EINVAL;

	spin_lock(&cygnus_dte->lock);

	/* Disable NCO Increment */
	nco_incr = readl(cygnus_dte->audioeav + DTE_NCO_INC_REG_BASE);
	writel(0, cygnus_dte->audioeav + DTE_NCO_INC_REG_BASE);

	/* Reset Timers */
	writel(0, cygnus_dte->audioeav + DTE_NCO_LOW_TIME_REG_BASE);
	writel(0, cygnus_dte->audioeav + DTE_NCO_TIME_REG_BASE);
	writel(0, cygnus_dte->audioeav + DTE_NCO_OVERFLOW_REG_BASE);

	/* Initialize last overflow value to track wrap condition */
	cygnus_dte->timestamp_overflow_last = 0;

	/* Initialize Reference timespec */
	cygnus_dte->ts_ref = *ts;

	/* re-enable nco increment */
	writel(nco_incr, cygnus_dte->audioeav + DTE_NCO_INC_REG_BASE);

	spin_unlock(&cygnus_dte->lock);

	return 0;
}
EXPORT_SYMBOL(dte_set_time);

int dte_get_time(
		struct bcm_cygnus_dte *cygnus_dte,
		struct timespec *ts)
{
	uint32_t current_time_sum1;
	uint32_t current_time_sum2;
	uint32_t current_time_sum3;
	uint32_t timestamp_overflow;
	uint64_t ns;

	if (!cygnus_dte)
		return -EINVAL;

	spin_lock(&cygnus_dte->lock);

	/* Read Timers */
	current_time_sum1 = readl(cygnus_dte->audioeav +
				DTE_NCO_LOW_TIME_REG_BASE);
	current_time_sum2 = readl(cygnus_dte->audioeav +
				DTE_NCO_TIME_REG_BASE);
	current_time_sum3 = readl(cygnus_dte->audioeav +
				DTE_NCO_OVERFLOW_REG_BASE);
/*
 * Register            |     sum3    |          sum2     |         sum1       |
 *                     |7           0|31  28            0|31    28:27        0|
 * Timestamp[71:0]     |71                                      28:27        0|
 */

	/* Current time in units of ns */
	ns = (((uint64_t)(current_time_sum3 & 0xff) << 36) |
		((uint64_t)current_time_sum2 << 4) |
		(uint64_t)((current_time_sum1 >> 28) & 0xf));

	/*
	 * Determined if wraparound has occurred
	 * Timestamp overflow only includes the bottom 8 bits of sum3
	 * and the top 4 bits of sum2
	 * Units of 2^32 ns = 4.294967296 sec
	 */
	timestamp_overflow = (ns >> 32) & 0xfff;
	if (timestamp_overflow < cygnus_dte->timestamp_overflow_last)
		/*
		 * Wrap around has occurred but not yet reflected in
		 * the reference timespec
		 * Full wrap around amount is 44bits in ns
		 * Precisely 17592.186044416 sec = ~4.887 hrs
		 */
		ns += (uint64_t)0x1<<44;

	/* Convert current timestamp(ns) to timespec */
	*ts = ns_to_timespec(ns);

	/* Add the current time to the reference time */
	*ts = timespec_add(cygnus_dte->ts_ref, *ts);

	spin_unlock(&cygnus_dte->lock);

	return 0;
}
EXPORT_SYMBOL(dte_get_time);

int dte_adj_time(
		struct bcm_cygnus_dte *cygnus_dte,
		int64_t delta)
{
	struct timespec ts_delta;

	if (!cygnus_dte)
		return -EINVAL;

	ts_delta = ns_to_timespec(delta);

	spin_lock(&cygnus_dte->lock);

	/* Update Reference timespec */
	cygnus_dte->ts_ref = timespec_add(cygnus_dte->ts_ref, ts_delta);

	spin_unlock(&cygnus_dte->lock);

	return 0;
}
EXPORT_SYMBOL(dte_adj_time);

int dte_adj_freq(
		struct bcm_cygnus_dte *cygnus_dte,
		int32_t ppb)
{
	uint64_t diff;
	uint32_t mult = NCO_INC_NOMINAL;
	uint32_t nco_incr;
	int neg_adj = 0;

	if (!cygnus_dte)
		return -EINVAL;

	if (ppb < 0) {
		ppb = -ppb;
		neg_adj = 1;
	}

	diff = mult;
	diff *= ppb;
	diff = div_u64(diff, 1000000000ULL);

	nco_incr = neg_adj ? mult - diff : mult + diff;

	spin_lock(&cygnus_dte->lock);

	/* Update NCO Increment */
	writel(nco_incr, cygnus_dte->audioeav + DTE_NCO_INC_REG_BASE);

	spin_unlock(&cygnus_dte->lock);

	return 0;
}
EXPORT_SYMBOL(dte_adj_freq);

static int dte_open(struct inode *pnode, struct file *fp)
{
	/* look up device info for this device file */
	struct bcm_cygnus_dte *cygnus_dte = container_of(pnode->i_cdev,
			struct bcm_cygnus_dte, dte_cdev);

	fp->private_data = cygnus_dte;

	return 0;
}

static long dte_ioctl(struct file *filep,
	unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct dte_data dte_data;
	struct dte_timestamp dte_timestamp;
	struct timespec ts;
	int64_t delta;
	int32_t ppb;
	struct bcm_cygnus_dte *cygnus_dte = filep->private_data;

	mutex_lock(&cygnus_dte->mutex);
	switch (cmd) {
	case DTE_IOCTL_SET_DIVIDER:
		if (copy_from_user(&dte_data, (struct dte_data *)arg,
				sizeof(struct dte_data))) {
			ret = -EACCES;
			goto err_out;
		}
		if (dte_set_client_divider(cygnus_dte,
				dte_data.client, dte_data.data))
			ret = -EPERM;
		break;

	case DTE_IOCTL_ENABLE_TIMESTAMP:
		if (copy_from_user(&dte_data, (struct dte_data *)arg,
				sizeof(struct dte_data))) {
			ret = -EACCES;
			goto err_out;
		}
		if (dte_enable_timestamp(cygnus_dte,
				dte_data.client, dte_data.data))
			ret = -EPERM;
		break;

	case DTE_IOCTL_SET_IRQ_INTERVAL:
		if (copy_from_user(&dte_data, (struct dte_data *)arg,
				sizeof(struct dte_data))) {
			ret = -EACCES;
			goto err_out;
		}
		if (dte_set_irq_interval(cygnus_dte, dte_data.data))
			ret = -EPERM;
		break;

	case DTE_IOCTL_GET_TIMESTAMP:
		if (copy_from_user(&dte_timestamp, (struct dte_timestamp *)arg,
				sizeof(struct dte_timestamp))) {
			ret = -EACCES;
			goto err_out;
		}
		if (dte_get_timestamp(cygnus_dte,
				dte_timestamp.client,
				&dte_timestamp.ts)) {
			ret = -EPERM;
			goto err_out;
		}
		if (copy_to_user((struct dte_timestamp *)arg, &dte_timestamp,
				sizeof(struct dte_timestamp))) {
			ret = -EACCES;
			goto err_out;
		}
		break;

	case DTE_IOCTL_SET_TIME:
		if (copy_from_user(&ts, (struct timespec *)arg,
				sizeof(struct timespec))) {
			ret = -EACCES;
			goto err_out;
		}
		if (dte_set_time(cygnus_dte, &ts))
			ret = -EPERM;
		break;

	case DTE_IOCTL_GET_TIME:
		if (dte_get_time(cygnus_dte, &ts))
			ret = -EPERM;
		if (copy_to_user((struct timespec *)arg, &ts,
				sizeof(struct timespec))) {
			ret = -EACCES;
			goto err_out;
		}
		break;

	case DTE_IOCTL_ADJ_TIME:
		if (copy_from_user(&delta, (int64_t *)arg,
				sizeof(int64_t))) {
			ret = -EACCES;
			goto err_out;
		}
		if (dte_adj_time(cygnus_dte, delta))
			ret = -EPERM;
		break;

	case DTE_IOCTL_ADJ_FREQ:
		if (copy_from_user(&ppb, (int32_t *)arg,
				sizeof(int32_t))) {
			ret = -EACCES;
			goto err_out;
		}
		if (dte_adj_freq(cygnus_dte, ppb))
			ret = -EPERM;
		break;

	default:
		ret = -EINVAL;
		goto err_out;
	}

err_out:
	mutex_unlock(&cygnus_dte->mutex);
	return ret;
}

static const struct file_operations dte_cdev_fops = {
	.open           = dte_open,
	.unlocked_ioctl = dte_ioctl,
};

static irqreturn_t bcm_cygnus_dte_isr_threaded(int irq, void *drv_ctx)
{
	uint32_t fifo_csr;
	uint32_t rd_data;
	uint32_t current_time_sum2;
	uint32_t current_time_sum3;
	uint32_t i = 0;
	uint32_t active_clients;
	int client;
	int inlen;
	struct platform_device *pdev = (struct platform_device *)drv_ctx;
	struct bcm_cygnus_dte *cygnus_dte;
	uint32_t timestamp_overflow;
	uint32_t timestamp_ns;
	uint64_t ns;
	struct timespec ts_stamp;
	uint32_t status;

	cygnus_dte = (struct bcm_cygnus_dte *)platform_get_drvdata(pdev);

	/* clear interrupt bit */
	writel(1<<DTE_CTRL_REG__INTERRUPT,
		cygnus_dte->audioeav + DTE_CTRL_REG_BASE);

	/* clear all interrupts in status register */
	status = (DTE_INTR_STATUS_ISO_INTR_MASK <<
		DTE_INTR_STATUS_ISO_INTR_SHIFT) |
		(DTE_INTR_STATUS_FIFO_LEVEL_INTR_MASK <<
		DTE_INTR_STATUS_FIFO_LEVEL_INTR_SHIFT) |
		(DTE_INTR_STATUS_TIMEOUT_INTR_MASK <<
		DTE_INTR_STATUS_TIMEOUT_INTR_SHIFT);
	writel(status, cygnus_dte->audioeav + DTE_INTR_STATUS_REG);

	/* get data from dte fifo */
	do {
		fifo_csr = readl(cygnus_dte->audioeav +
			DTE_LTS_CSR_REG_BASE);
		/* fifo empty */
		if (fifo_csr & BIT(DTE_LTS_CSR_REG__FIFO_EMPTY))
			break;

		/* overflow error */
		if (fifo_csr & BIT(DTE_LTS_CSR_REG__FIFO_OVERFLOW))
			break;

		spin_lock(&cygnus_dte->lock);

		/* first event contains active clients */
		active_clients = readl(cygnus_dte->audioeav +
			DTE_LTS_FIFO_REG_BASE);

		/* then timestamp */
		rd_data = readl(cygnus_dte->audioeav +
			DTE_LTS_FIFO_REG_BASE);

		/* Save the actual timestamp */
		timestamp_ns = rd_data;

		/* Get the timestamp overflow counters */
		current_time_sum2 = readl(cygnus_dte->audioeav +
			DTE_NCO_TIME_REG_BASE);
		current_time_sum3 = readl(cygnus_dte->audioeav +
			DTE_NCO_OVERFLOW_REG_BASE);
		spin_unlock(&cygnus_dte->lock);

		/*
		 * Timestamp overflow only includes the bottom
		 * 8 bits of sum3 and the top 4 bits of sum2.
		 *
		 * Combine sum3 and sum2 to the timestamp_overflow.
		 * Units of 2^32 ns = 4.294967296 sec
		 */
		timestamp_overflow =
			((current_time_sum3 & 0xff) << 4) |
			((current_time_sum2 >> 28) & 0xffffff);
		/*
		 * If the current time is less than the fifo timestamped time
		 * then wrap around must have occurred.
		 * Convert current_time_sum2 to ns before comparison
		 */
		if ((current_time_sum2 << 4) <= timestamp_ns) {
			if (timestamp_overflow > 0)
				/*
				 * Decrement the overflow counter since the fifo
				 * timestamp occurred before overflow counter
				 * had incremented
				 */
				timestamp_overflow--;
			else
				/*
				 * If the overflow counter is zero *and*
				 * the timestamp wrapped then the overflow
				 * counter also wrapped.
				 * Set to max value of 12 bits wide.
				 * sum3 (8 bits) and sum2 (top 4 bits)
				 */
				timestamp_overflow = 0xfff;
		}

		spin_lock(&cygnus_dte->lock);
		/*
		 * Update reference timespec if wrap of the timestamp overflow
		 * occurred
		 */
		if (timestamp_overflow < cygnus_dte->timestamp_overflow_last) {
			struct timespec ts_wrapamount;
			/*
			 * Wrap around occurred.
			 * Full wrap around amount is 44 bits in ns.
			 * Includes sum3 (8bits), sum2 (32 bits), sum1 (4 bits)
			 * Precisely 17592.186044416 sec = ~4.887 hrs
			 */
			ts_wrapamount = ns_to_timespec((uint64_t)0x1<<44);
			/* Increment the reference */
			cygnus_dte->ts_ref =
				timespec_add(cygnus_dte->ts_ref, ts_wrapamount);
		}
		/* Track the last timestamp overflow value */
		cygnus_dte->timestamp_overflow_last = timestamp_overflow;

		/* Convert current timestamp(ns) to timespec */
		ns = ((uint64_t)timestamp_overflow << 32) +
			timestamp_ns;
		ts_stamp = ns_to_timespec(ns);

		/* Add the current time to the reference time */
		ts_stamp = timespec_add(cygnus_dte->ts_ref, ts_stamp);
		spin_unlock(&cygnus_dte->lock);

		/*
		 * The valid timestamp may be valid for more than one client
		 * Examine all the active clients and insert timestamp
		 * to the appropriate software fifo for each client asserted
		 * Clients[0:3] are unused.
		 * Shift down by 4 to zero index the first valid input client
		 */
		active_clients >>= 4;

		for_each_set_bit(client,
			(unsigned long *)&active_clients, DTE_CLIENT_MAX) {
			/* Insert full timestamps into software fifo */
			inlen = kfifo_in(
				&cygnus_dte->recv_fifo[client],
				&ts_stamp,
				sizeof(struct timespec));

			if (inlen != sizeof(struct timespec)) {
				cygnus_dte->kfifoof[client]++;
				dev_err(&cygnus_dte->pdev->dev,
					"kfifoof[%d] = %d\n",
					client,
					cygnus_dte->kfifoof[client]);
			}
		}
	} while (++i < DTE_HW_FIFO_SIZE); /* at most get FIFO size */

	if (fifo_csr & BIT(DTE_LTS_CSR_REG__FIFO_UNDERFLOW)) {
		cygnus_dte->fifouf++;
		dev_err(&cygnus_dte->pdev->dev,
			"fifouf = %d\n", cygnus_dte->fifouf);
	}
	if (fifo_csr & BIT(DTE_LTS_CSR_REG__FIFO_OVERFLOW)) {
		cygnus_dte->fifoof++;
		dev_err(&cygnus_dte->pdev->dev,
			"fifoof =%d\n", cygnus_dte->fifoof);
	}

	/* overflow/underflow error, reset fifo */
	if (fifo_csr & 0xc) {
		writel(0, cygnus_dte->audioeav + DTE_LTS_CSR_REG_BASE);
		dev_err(&cygnus_dte->pdev->dev, "Resetting HW FIFO\n");
	}

	return IRQ_HANDLED;
}

static const struct of_device_id bcm_cygnus_dte_of_match[] = {
	{ .compatible = "brcm,cygnus-dte", },
	{},
};
MODULE_DEVICE_TABLE(of, bcm_cygnus_dte_of_match);

static int bcm_cygnus_dte_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct bcm_cygnus_dte *cygnus_dte;
	int ret = -ENOMEM;
	dev_t devt;
	struct device *retdev;
	struct class *dte_class;
	int client;
	int irq;

	dev_info(dev, "Entering bcm_cygnus_dte_probe\n");

	cygnus_dte = devm_kzalloc(dev, sizeof(*cygnus_dte), GFP_KERNEL);
	if (!cygnus_dte)
		return -ENOMEM;

	mutex_init(&cygnus_dte->mutex);

	/* Audio DTE memory mapped registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cygnus_dte->audioeav = devm_ioremap_resource(dev, res);
	if (IS_ERR(cygnus_dte->audioeav)) {
		dev_err(&pdev->dev, "%s IO remap audioeav failed\n", __func__);
		return PTR_ERR(cygnus_dte->audioeav);
	}

	spin_lock_init(&cygnus_dte->lock);

	/* get interrupt */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "%s platform_get_irq failed\n", __func__);
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq,
				NULL, bcm_cygnus_dte_isr_threaded,
				IRQF_ONESHOT, pdev->name, pdev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq error %d\n", ret);
		return ret;
	}

	for (client = DTE_CLIENT_MIN; client < DTE_CLIENT_MAX; client++) {
		ret = kfifo_alloc(&cygnus_dte->recv_fifo[client],
				DTE_SW_FIFO_SIZE * sizeof(struct timespec),
				GFP_KERNEL);
		if (ret) {
			dev_err(dev, "Failed kfifo alloc\n");
			goto err_free_kfifo;
		}
	}

	/* create device */
	cdev_init(&cygnus_dte->dte_cdev, &dte_cdev_fops);

	cygnus_dte->pdev = pdev;

	/* create class for mdev auto create node /dev/dte */
	dte_class = class_create(THIS_MODULE, DTE_DEVICE_NAME);
	if (IS_ERR(dte_class)) {
		ret = PTR_ERR(dte_class);
		dev_err(&pdev->dev, "can't register static dte class\n");
		goto err_free_kfifo;
	}
	cygnus_dte->dte_class = dte_class;

	ret = alloc_chrdev_region(&devt, 0, DTE_MAX_DEVS, DTE_DEVICE_NAME);
	if (ret) {
		dev_err(&pdev->dev,
			"%s Alloc char device region failed\n", __func__);
		goto err_class_destroy;
	}

	ret = cdev_add(&cygnus_dte->dte_cdev, devt, 1);
	if (ret) {
		dev_err(dev, "Failed adding dte cdev file\n");
		goto err_unreg_chardev;
	}

	retdev = device_create(cygnus_dte->dte_class, NULL,
			devt, NULL, DTE_DEVICE_NAME);
	if (IS_ERR(retdev)) {
		dev_err(&pdev->dev, "can't create dte device\n ");
		ret = PTR_ERR(retdev);
		goto err_cdev_delete;
	}

	list_add_tail(&cygnus_dte->node, &dtedev_list);
	platform_set_drvdata(pdev, cygnus_dte);
	cygnus_dte->devt = devt;

	dev_info(dev, "Exiting bcm_cygnus_dte_probe\n");

	return 0;

err_cdev_delete:
	cdev_del(&cygnus_dte->dte_cdev);
err_unreg_chardev:
	unregister_chrdev_region(devt, DTE_MAX_DEVS);
err_class_destroy:
	class_destroy(cygnus_dte->dte_class);
err_free_kfifo:
	for (client = DTE_CLIENT_MIN; client < DTE_CLIENT_MAX; client++)
		kfifo_free(&cygnus_dte->recv_fifo[client]);
	return ret;
}

static int bcm_cygnus_dte_remove(struct platform_device *pdev)
{
	int client;
	int intr_mask;
	struct bcm_cygnus_dte *cygnus_dte = platform_get_drvdata(pdev);

	/* disable isochronous interrupt */
	intr_mask = readl(cygnus_dte->audioeav + DTE_INTR_MASK_REG);
	intr_mask |= 1 << ISO_INTR_MASK_SHIFT;
	writel(intr_mask, cygnus_dte->audioeav + DTE_INTR_MASK_REG);

	list_del(&cygnus_dte->node);
	device_destroy(cygnus_dte->dte_class, cygnus_dte->devt);
	class_destroy(cygnus_dte->dte_class);
	unregister_chrdev_region(cygnus_dte->devt, DTE_MAX_DEVS);
	platform_set_drvdata(pdev, NULL);
	cdev_del(&cygnus_dte->dte_cdev);
	for (client = DTE_CLIENT_MIN; client < DTE_CLIENT_MAX; client++)
		kfifo_free(&cygnus_dte->recv_fifo[client]);

	return 0;
}

static struct platform_driver bcm_cygnus_dte_driver = {
	.driver = {
		.name = "cygnus-dte",
		.of_match_table = bcm_cygnus_dte_of_match,
	},
	.probe    = bcm_cygnus_dte_probe,
	.remove   = bcm_cygnus_dte_remove,
};
module_platform_driver(bcm_cygnus_dte_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom Cygnus DTE Digital Timing Engine driver");
MODULE_LICENSE("GPL v2");
