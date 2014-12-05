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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/ioport.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/irqchip/chained_irq.h>

#define CYGNUS_GPIO_DATA_IN_OFFSET   0x00
#define CYGNUS_GPIO_DATA_OUT_OFFSET  0x04
#define CYGNUS_GPIO_OUT_EN_OFFSET    0x08
#define CYGNUS_GPIO_IN_TYPE_OFFSET   0x0c
#define CYGNUS_GPIO_INT_DE_OFFSET    0x10
#define CYGNUS_GPIO_INT_EDGE_OFFSET  0x14
#define CYGNUS_GPIO_INT_MSK_OFFSET   0x18
#define CYGNUS_GPIO_INT_STAT_OFFSET  0x1c
#define CYGNUS_GPIO_INT_MSTAT_OFFSET 0x20
#define CYGNUS_GPIO_INT_CLR_OFFSET   0x24
#define CYGNUS_GPIO_PAD_RES_OFFSET   0x34
#define CYGNUS_GPIO_RES_EN_OFFSET    0x38

/* drive strength control for ASIU GPIO */
#define CYGNUS_GPIO_ASIU_DRV0_CTRL_OFFSET 0x58

/* drive strength control for CCM GPIO */
#define CYGNUS_GPIO_CCM_DRV0_CTRL_OFFSET  0x00

#define GPIO_BANK_SIZE 0x200
#define NGPIOS_PER_BANK 32
#define GPIO_BANK(pin) ((pin) / NGPIOS_PER_BANK)

#define CYGNUS_GPIO_REG(pin, reg) (GPIO_BANK(pin) * GPIO_BANK_SIZE + (reg))
#define CYGNUS_GPIO_SHIFT(pin) ((pin) % NGPIOS_PER_BANK)

#define GPIO_FLAG_BIT_MASK           0xffff
#define GPIO_PULL_BIT_SHIFT          16
#define GPIO_PULL_BIT_MASK           0x3

#define GPIO_DRV_STRENGTH_BIT_SHIFT  20
#define GPIO_DRV_STRENGTH_BITS       3
#define GPIO_DRV_STRENGTH_BIT_MASK   ((1 << GPIO_DRV_STRENGTH_BITS) - 1)

/*
 * For GPIO internal pull up/down registers
 */
enum gpio_pull {
	GPIO_PULL_NONE = 0,
	GPIO_PULL_UP,
	GPIO_PULL_DOWN,
	GPIO_PULL_INVALID,
};

/*
 * GPIO drive strength
 */
enum gpio_drv_strength {
	GPIO_DRV_STRENGTH_2MA = 0,
	GPIO_DRV_STRENGTH_4MA,
	GPIO_DRV_STRENGTH_6MA,
	GPIO_DRV_STRENGTH_8MA,
	GPIO_DRV_STRENGTH_10MA,
	GPIO_DRV_STRENGTH_12MA,
	GPIO_DRV_STRENGTH_14MA,
	GPIO_DRV_STRENGTH_16MA,
	GPIO_DRV_STRENGTH_INVALID,
};

struct cygnus_gpio {
	struct device *dev;
	void __iomem *base;
	void __iomem *io_ctrl;
	spinlock_t lock;
	struct gpio_chip gc;
	unsigned num_banks;
	int irq;
	struct irq_domain *irq_domain;
};

static struct cygnus_gpio *to_cygnus_gpio(struct gpio_chip *gc)
{
	return container_of(gc, struct cygnus_gpio, gc);
}

static u32 cygnus_readl(struct cygnus_gpio *cygnus_gpio, unsigned int offset)
{
	return readl(cygnus_gpio->base + offset);
}

static void cygnus_writel(struct cygnus_gpio *cygnus_gpio,
			  unsigned int offset, u32 val)
{
	writel(val, cygnus_gpio->base + offset);
}

/**
 *  cygnus_set_bit - set or clear one bit (corresponding to the GPIO pin) in a
 *  Cygnus GPIO register
 *
 *  @cygnus_gpio: Cygnus GPIO device
 *  @reg: register offset
 *  @gpio: GPIO pin
 *  @set: set or clear. 1 - set; 0 -clear
 */
static void cygnus_set_bit(struct cygnus_gpio *cygnus_gpio,
			   unsigned int reg, unsigned gpio, int set)
{
	unsigned int offset = CYGNUS_GPIO_REG(gpio, reg);
	unsigned int shift = CYGNUS_GPIO_SHIFT(gpio);
	u32 val;

	val = cygnus_readl(cygnus_gpio, offset);
	if (set)
		val |= BIT(shift);
	else
		val &= ~BIT(shift);
	cygnus_writel(cygnus_gpio, offset, val);
}

static int cygnus_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct cygnus_gpio *cygnus_gpio = to_cygnus_gpio(gc);

	return irq_find_mapping(cygnus_gpio->irq_domain, offset);
}

static void cygnus_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct cygnus_gpio *cygnus_gpio;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int i, bit;

	chained_irq_enter(chip, desc);

	cygnus_gpio = irq_get_handler_data(irq);

	/* go through the entire GPIO banks and handle all interrupts */
	for (i = 0; i < cygnus_gpio->num_banks; i++) {
		unsigned long val = cygnus_readl(cygnus_gpio,
				(i * GPIO_BANK_SIZE) +
				CYGNUS_GPIO_INT_MSTAT_OFFSET);

		for_each_set_bit(bit, &val, NGPIOS_PER_BANK) {
			unsigned pin = NGPIOS_PER_BANK * i + bit;
			int child_irq =
				cygnus_gpio_to_irq(&cygnus_gpio->gc, pin);

			/*
			 * Clear the interrupt before invoking the
			 * handler, so we do not leave any window
			 */
			cygnus_writel(cygnus_gpio, (i * GPIO_BANK_SIZE) +
				CYGNUS_GPIO_INT_CLR_OFFSET, BIT(bit));

			generic_handle_irq(child_irq);
		}
	}

	chained_irq_exit(chip, desc);
}


static void cygnus_gpio_irq_ack(struct irq_data *d)
{
	struct cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	unsigned int offset = CYGNUS_GPIO_REG(gpio,
			CYGNUS_GPIO_INT_CLR_OFFSET);
	unsigned int shift = CYGNUS_GPIO_SHIFT(gpio);
	u32 val = BIT(shift);

	cygnus_writel(cygnus_gpio, offset, val);
}

/**
 *  cygnus_gpio_irq_set_mask - mask/unmask a GPIO interrupt
 *
 *  @d: IRQ chip data
 *  @mask: mask/unmask GPIO interrupt. 0 - mask (disable); 1 - unmask (enable)
 */
static void cygnus_gpio_irq_set_mask(struct irq_data *d, int mask)
{
	struct cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;

	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_INT_MSK_OFFSET, gpio, mask);
}

static void cygnus_gpio_irq_mask(struct irq_data *d)
{
	struct cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	spin_lock_irqsave(&cygnus_gpio->lock, flags);
	cygnus_gpio_irq_set_mask(d, 0);
	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);
}

static void cygnus_gpio_irq_unmask(struct irq_data *d)
{
	struct cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	spin_lock_irqsave(&cygnus_gpio->lock, flags);
	cygnus_gpio_irq_set_mask(d, 1);
	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);
}

static int cygnus_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct cygnus_gpio *cygnus_gpio = irq_data_get_irq_chip_data(d);
	unsigned gpio = d->hwirq;
	int int_type = 0, dual_edge = 0, edge_lvl = 0;
	unsigned long flags;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		edge_lvl = 1;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		break;

	case IRQ_TYPE_EDGE_BOTH:
		dual_edge = 1;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		int_type = 1;
		edge_lvl = 1;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		int_type = 1;
		break;

	default:
		dev_err(cygnus_gpio->dev, "invalid GPIO IRQ type 0x%x\n",
				type);
		return -EINVAL;
	}

	spin_lock_irqsave(&cygnus_gpio->lock, flags);
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_IN_TYPE_OFFSET, gpio,
			int_type);
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_INT_DE_OFFSET, gpio,
			dual_edge);
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_INT_EDGE_OFFSET, gpio,
			edge_lvl);
	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev,
		"gpio:%u set int_type:%d dual_edge:%d edge_lvl:%d\n",
		gpio, int_type, dual_edge, edge_lvl);

	return 0;
}

static struct irq_chip cygnus_gpio_irq_chip = {
	.name = "bcm-cygnus-gpio",
	.irq_ack = cygnus_gpio_irq_ack,
	.irq_mask = cygnus_gpio_irq_mask,
	.irq_unmask = cygnus_gpio_irq_unmask,
	.irq_set_type = cygnus_gpio_irq_set_type,
};

static int cygnus_gpio_direction_input(struct gpio_chip *gc, unsigned gpio)
{
	struct cygnus_gpio *cygnus_gpio = to_cygnus_gpio(gc);
	unsigned long flags;

	spin_lock_irqsave(&cygnus_gpio->lock, flags);
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_OUT_EN_OFFSET, gpio, 0);
	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev, "gpio:%u set input\n", gpio);

	return 0;
}

static int cygnus_gpio_direction_output(struct gpio_chip *gc,
		unsigned gpio, int value)
{
	struct cygnus_gpio *cygnus_gpio = to_cygnus_gpio(gc);
	unsigned long flags;

	spin_lock_irqsave(&cygnus_gpio->lock, flags);
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_OUT_EN_OFFSET, gpio, 1);
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_DATA_OUT_OFFSET, gpio, value);
	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev, "gpio:%u set output, value:%d\n",
			gpio, value);

	return 0;
}

static void cygnus_gpio_set(struct gpio_chip *gc, unsigned gpio,
		int value)
{
	struct cygnus_gpio *cygnus_gpio = to_cygnus_gpio(gc);
	unsigned long flags;

	spin_lock_irqsave(&cygnus_gpio->lock, flags);
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_DATA_OUT_OFFSET, gpio, value);
	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev, "gpio:%u set, value:%d\n", gpio, value);
}

static int cygnus_gpio_get(struct gpio_chip *gc, unsigned gpio)
{
	struct cygnus_gpio *cygnus_gpio = to_cygnus_gpio(gc);
	unsigned int offset = CYGNUS_GPIO_REG(gpio,
			CYGNUS_GPIO_DATA_IN_OFFSET);
	unsigned int shift = CYGNUS_GPIO_SHIFT(gpio);
	u32 val;

	val = cygnus_readl(cygnus_gpio, offset);
	val = (val >> shift) & 1;

	dev_dbg(cygnus_gpio->dev, "gpio:%u get, value:%d\n", gpio, val);

	return val;
}

static struct lock_class_key gpio_lock_class;

static int cygnus_gpio_irq_map(struct irq_domain *d, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	int ret;

	ret = irq_set_chip_data(irq, d->host_data);
	if (ret < 0)
		return ret;
	irq_set_lockdep_class(irq, &gpio_lock_class);
	irq_set_chip_and_handler(irq, &cygnus_gpio_irq_chip,
			handle_simple_irq);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

static void cygnus_gpio_irq_unmap(struct irq_domain *d, unsigned int irq)
{
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static struct irq_domain_ops cygnus_irq_ops = {
	.map = cygnus_gpio_irq_map,
	.unmap = cygnus_gpio_irq_unmap,
	.xlate = irq_domain_xlate_twocell,
};

#ifdef CONFIG_OF_GPIO
static void cygnus_gpio_set_pull(struct cygnus_gpio *cygnus_gpio,
				 unsigned gpio, enum gpio_pull pull)
{
	int pullup;
	unsigned long flags;

	switch (pull) {
	case GPIO_PULL_UP:
		pullup = 1;
		break;
	case GPIO_PULL_DOWN:
		pullup = 0;
		break;
	case GPIO_PULL_NONE:
	case GPIO_PULL_INVALID:
	default:
		return;
	}

	spin_lock_irqsave(&cygnus_gpio->lock, flags);
	/* set pull up/down */
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_PAD_RES_OFFSET, gpio, pullup);
	/* enable pad */
	cygnus_set_bit(cygnus_gpio, CYGNUS_GPIO_RES_EN_OFFSET, gpio, 1);
	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev, "gpio:%u set pullup:%d\n", gpio, pullup);
}

static void cygnus_gpio_set_strength(struct cygnus_gpio *cygnus_gpio,
		unsigned gpio, enum gpio_drv_strength strength)
{
	struct device *dev = cygnus_gpio->dev;
	void __iomem *base;
	unsigned int i, offset, shift;
	u32 val;
	unsigned long flags;

	/* some GPIO controllers do not support drive strength configuration */
	if (of_find_property(dev->of_node, "no-drv-strength", NULL))
		return;

	/*
	 * Some GPIO controllers use a different register block for drive
	 * strength control
	 */
	if (cygnus_gpio->io_ctrl) {
		base = cygnus_gpio->io_ctrl;
		offset = CYGNUS_GPIO_CCM_DRV0_CTRL_OFFSET;
	} else {
		base = cygnus_gpio->base;
		offset = CYGNUS_GPIO_REG(gpio,
				CYGNUS_GPIO_ASIU_DRV0_CTRL_OFFSET);
	}

	shift = CYGNUS_GPIO_SHIFT(gpio);

	spin_lock_irqsave(&cygnus_gpio->lock, flags);

	for (i = 0; i < GPIO_DRV_STRENGTH_BITS; i++) {
		val = readl(base + offset);
		val &= ~BIT(shift);
		val |= ((strength >> i) & 0x1) << shift;
		writel(val, base + offset);
		offset += 4;
	}

	spin_unlock_irqrestore(&cygnus_gpio->lock, flags);

	dev_dbg(cygnus_gpio->dev,
			"gpio:%u set drive strength:%d\n", gpio, strength);
}

static int cygnus_gpio_of_xlate(struct gpio_chip *gc,
		const struct of_phandle_args *gpiospec, u32 *flags)
{
	struct cygnus_gpio *cygnus_gpio = to_cygnus_gpio(gc);
	enum gpio_pull pull;
	enum gpio_drv_strength strength;

	if (gc->of_gpio_n_cells < 2)
		return -EINVAL;

	if (WARN_ON(gpiospec->args_count < gc->of_gpio_n_cells))
		return -EINVAL;

	if (gpiospec->args[0] >= gc->ngpio)
		return -EINVAL;

	pull = (gpiospec->args[1] >> GPIO_PULL_BIT_SHIFT) & GPIO_PULL_BIT_MASK;
	if (WARN_ON(pull >= GPIO_PULL_INVALID))
		return -EINVAL;

	strength = (gpiospec->args[1] >> GPIO_DRV_STRENGTH_BIT_SHIFT) &
		GPIO_DRV_STRENGTH_BIT_MASK;

	if (flags)
		*flags = gpiospec->args[1] & GPIO_FLAG_BIT_MASK;

	cygnus_gpio_set_pull(cygnus_gpio, gpiospec->args[0], pull);
	cygnus_gpio_set_strength(cygnus_gpio, gpiospec->args[0], strength);

	return gpiospec->args[0];
}
#endif

static const struct of_device_id cygnus_gpio_of_match[] = {
	{ .compatible = "brcm,cygnus-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, cygnus_gpio_of_match);

static int cygnus_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct cygnus_gpio *cygnus_gpio;
	struct gpio_chip *gc;
	u32 i, ngpios;
	int ret;

	cygnus_gpio = devm_kzalloc(dev, sizeof(*cygnus_gpio), GFP_KERNEL);
	if (!cygnus_gpio)
		return -ENOMEM;

	cygnus_gpio->dev = dev;
	platform_set_drvdata(pdev, cygnus_gpio);

	if (of_property_read_u32(dev->of_node, "ngpios", &ngpios)) {
		dev_err(&pdev->dev, "missing ngpios DT property\n");
		return -ENODEV;
	}
	cygnus_gpio->num_banks = (ngpios + NGPIOS_PER_BANK - 1) /
		NGPIOS_PER_BANK;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cygnus_gpio->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cygnus_gpio->base)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		return PTR_ERR(cygnus_gpio->base);
	}

	/*
	 * Only certain types of Cygnus GPIO interfaces have I/O control
	 * registers
	 */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		cygnus_gpio->io_ctrl = devm_ioremap_resource(dev, res);
		if (IS_ERR(cygnus_gpio->io_ctrl)) {
			dev_err(&pdev->dev, "unable to map I/O memory\n");
			return PTR_ERR(cygnus_gpio->io_ctrl);
		}
	}

	spin_lock_init(&cygnus_gpio->lock);

	gc = &cygnus_gpio->gc;
	gc->base = -1;
	gc->ngpio = ngpios;
	gc->label = dev_name(dev);
	gc->dev = dev;
#ifdef CONFIG_OF_GPIO
	gc->of_node = dev->of_node;
	gc->of_gpio_n_cells = 2;
	gc->of_xlate = cygnus_gpio_of_xlate;
#endif
	gc->direction_input = cygnus_gpio_direction_input;
	gc->direction_output = cygnus_gpio_direction_output;
	gc->set = cygnus_gpio_set;
	gc->get = cygnus_gpio_get;
	gc->to_irq = cygnus_gpio_to_irq;

	ret = gpiochip_add(gc);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to add GPIO chip\n");
		return ret;
	}

	/*
	 * Some of the GPIO interfaces do not have interrupt wired to the main
	 * processor
	 */
	cygnus_gpio->irq = platform_get_irq(pdev, 0);
	if (cygnus_gpio->irq < 0) {
		ret = cygnus_gpio->irq;
		if (ret == -EPROBE_DEFER)
			goto err_rm_gpiochip;

		dev_info(&pdev->dev, "no interrupt hook\n");
	}

	cygnus_gpio->irq_domain = irq_domain_add_linear(dev->of_node,
			gc->ngpio, &cygnus_irq_ops, cygnus_gpio);
	if (!cygnus_gpio->irq_domain) {
		dev_err(&pdev->dev, "unable to allocate IRQ domain\n");
		ret = -ENXIO;
		goto err_rm_gpiochip;
	}

	for (i = 0; i < gc->ngpio; i++) {
		int irq = irq_create_mapping(cygnus_gpio->irq_domain, i);

		irq_set_lockdep_class(irq, &gpio_lock_class);
		irq_set_chip_data(irq, cygnus_gpio);
		irq_set_chip_and_handler(irq, &cygnus_gpio_irq_chip,
				handle_simple_irq);
		set_irq_flags(irq, IRQF_VALID);
	}

	irq_set_chained_handler(cygnus_gpio->irq, cygnus_gpio_irq_handler);
	irq_set_handler_data(cygnus_gpio->irq, cygnus_gpio);

	return 0;

err_rm_gpiochip:
	gpiochip_remove(gc);
	return ret;
}

static struct platform_driver cygnus_gpio_driver = {
	.driver = {
		.name = "bcm-cygnus-gpio",
		.owner = THIS_MODULE,
		.of_match_table = cygnus_gpio_of_match,
	},
	.probe = cygnus_gpio_probe,
};

module_platform_driver(cygnus_gpio_driver);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom Cygnus GPIO Driver");
MODULE_LICENSE("GPL v2");
