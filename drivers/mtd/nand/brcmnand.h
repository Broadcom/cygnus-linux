/*
 * Copyright © 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __BRCMNAND_H__
#define __BRCMNAND_H__

#include <linux/types.h>

struct device;
struct device_node;

struct brcmnand_intc {
	struct device *dev; /* parent device */
	struct device_node *dn;
	bool (*ctlrdy_ack)(struct brcmnand_intc *intc);
	void (*ctlrdy_set_enabled)(struct brcmnand_intc *intc, bool en);
	void *priv;
};

/**
 * Probe for a custom Broadcom NAND interrupt controller
 *
 * @dev: device to bind devres objects to
 * @dn: DT node for the interrupt controller
 *
 * Return a new intc struct if successful. Should be freed with
 * brcmnand_remove_intc().
 * Return NULL for all other errors
 */
struct brcmnand_intc *devm_brcmnand_probe_intc(struct device *dev,
					       struct device_node *dn);

#endif /* __BRCMNAND_H__ */
