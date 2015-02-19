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

#include <linux/device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "brcmnand.h"

#define DRV_NAME	"brcmnand-intc"

struct brcmnand_intc_ofdata {
	int (*init)(struct brcmnand_intc *intc);
	bool (*ctlrdy_ack)(struct brcmnand_intc *intc);
	void (*ctlrdy_set_enabled)(struct brcmnand_intc *intc, bool en);
};

static const struct of_device_id brcmnand_intc_ofmatch[] = {
	/* TODO: fill in */
	{},
};

struct brcmnand_intc *devm_brcmnand_probe_intc(struct device *dev,
					       struct device_node *dn)
{
	const struct brcmnand_intc_ofdata *intc_data;
	const struct of_device_id *match;
	struct brcmnand_intc *intc;
	int ret;

	match = of_match_node(brcmnand_intc_ofmatch, dn);
	if (!match)
		return NULL;

	intc_data = match->data;

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return NULL;

	intc->dev = dev;
	intc->dn = dn;
	intc->ctlrdy_ack = intc_data->ctlrdy_ack;
	intc->ctlrdy_set_enabled = intc_data->ctlrdy_set_enabled;
	if (intc_data->init) {
		ret = intc_data->init(intc);
		if (ret)
			return NULL;
	}

	return intc;
}
