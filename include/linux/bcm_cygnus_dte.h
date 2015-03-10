/*
 * Copyright (C) 2015, Broadcom Corporation. All Rights Reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <linux/ioctl.h>
#include <linux/types.h>

#ifndef _BCM_CYGNUS_DTE_H_
#define _BCM_CYGNUS_DTE_H_

/**
 * DTE Client
 */
enum dte_client {
	DTE_CLIENT_MIN = 0,
	DTE_CLIENT_I2S0_BITCLOCK = 0,
	DTE_CLIENT_I2S1_BITCLOCK,
	DTE_CLIENT_I2S2_BITCLOCK,
	DTE_CLIENT_I2S0_WORDCLOCK,
	DTE_CLIENT_I2S1_WORDCLOCK,
	DTE_CLIENT_I2S2_WORDCLOCK,
	DTE_CLIENT_LCD_CLFP,
	DTE_CLIENT_LCD_CLLP,
	DTE_CLIENT_GPIO14,
	DTE_CLIENT_GPIO15,
	DTE_CLIENT_GPIO22,
	DTE_CLIENT_GPIO23,
	DTE_CLIENT_MAX,
};

#define DTE_IOCTL_BASE          'd'
#define DTE_IO(nr)              _IO(DTE_IOCTL_BASE, nr)
#define DTE_IOR(nr, type)       _IOR(DTE_IOCTL_BASE, nr, type)
#define DTE_IOW(nr, type)       _IOW(DTE_IOCTL_BASE, nr, type)
#define DTE_IOWR(nr, type)      _IOWR(DTE_IOCTL_BASE, nr, type)

#define DTE_IOCTL_SET_DIVIDER       DTE_IOW(0x00, struct dte_data)
#define DTE_IOCTL_ENABLE_TIMESTAMP  DTE_IOW(0x01, struct dte_data)
#define DTE_IOCTL_SET_IRQ_INTERVAL  DTE_IOW(0x02, struct dte_data)
#define DTE_IOCTL_GET_TIMESTAMP     DTE_IOWR(0x03, struct dte_timestamp)
#define DTE_IOCTL_SET_TIME          DTE_IOW(0x04, struct timespec)
#define DTE_IOCTL_GET_TIME          DTE_IOR(0x05, struct timespec)
#define DTE_IOCTL_ADJ_TIME          DTE_IOW(0x06, int64_t)
#define DTE_IOCTL_ADJ_FREQ          DTE_IOW(0x07, int32_t)

struct dte_data {
	enum dte_client client;
	unsigned int data;
};

struct dte_timestamp {
	enum dte_client client;
	struct timespec ts;
};

struct bcm_cygnus_dte;

extern struct bcm_cygnus_dte *dte_get_dev_from_devname(
		const char *devname);
extern int dte_enable_timestamp(
		struct bcm_cygnus_dte *cygnus_dte,
		enum dte_client client,
		int enable);
extern int dte_set_client_divider(
		struct bcm_cygnus_dte *cygnus_dte,
		enum dte_client client,
		int divider);
extern int dte_set_irq_interval(
		struct bcm_cygnus_dte *cygnus_dte,
		uint32_t nanosec);
extern int dte_get_timestamp(
		struct bcm_cygnus_dte *cygnus_dte,
		enum dte_client client,
		struct timespec *ts);
extern int dte_set_time(
		struct bcm_cygnus_dte *cygnus_dte,
		struct timespec *ts);
extern int dte_get_time(
		struct bcm_cygnus_dte *cygnus_dte,
		struct timespec *ts);
extern int dte_adj_time(
		struct bcm_cygnus_dte *cygnus_dte,
		int64_t delta);
extern int dte_adj_freq(
		struct bcm_cygnus_dte *cygnus_dte,
		int32_t ppb);
#endif
