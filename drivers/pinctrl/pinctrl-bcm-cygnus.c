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

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/slab.h>

#include "core.h"
#include "pinctrl-utils.h"

/*
 * Alternate function configuration
 *
 * @name: name of the alternate function
 * @group_names: array of strings of group names that can be supported by this
 * alternate function
 * @num_groups: total number of groups that can be supported by this alternate
 * function
 * @mux: mux setting for this alternate function to be programed
 */
struct cygnus_pin_function {
	const char *name;
	const char * const *group_names;
	const unsigned num_groups;
	unsigned int mux;
};

/*
 * Cygnus allows group based pinmux configuration
 *
 * @name: name of the group
 * @pins: array of pins used by this group
 * @num_pins: total number of pins used by this group
 * @offset: register offset for pinmux configuration of this group
 * @shift: bit shift for pinmux configuration of this group
 */
struct cygnus_pin_group {
	const char *name;
	const unsigned *pins;
	const unsigned num_pins;
	const unsigned int offset;
	const unsigned int shift;
};

/*
 * Cygnus pinctrl core
 *
 * @pctl: pointer to pinctrl_dev
 * @dev: pointer to the device
 * @base: I/O register base for Cygnus pinctrl configuration
 *
 */
struct cygnus_pinctrl {
	struct pinctrl_dev *pctl;
	struct device *dev;
	void __iomem *base;

	const struct pinctrl_pin_desc *pins;
	unsigned num_pins;

	const struct cygnus_pin_group *groups;
	unsigned num_groups;

	const struct cygnus_pin_function *functions;
	unsigned num_functions;
};

#define CYGNUS_PIN_GROUP(group_name, off, sh)		\
{							\
	.name = #group_name,				\
	.pins = group_name##_pins,			\
	.num_pins = ARRAY_SIZE(group_name##_pins),	\
	.offset = off,					\
	.shift = sh,					\
}

/*
 * The following pin description is based on Cygnus I/O MUX spreadsheet
 */
static const struct pinctrl_pin_desc cygnus_pinctrl_pins[] = {
	PINCTRL_PIN(0, "ext_device_reset_n"),
	PINCTRL_PIN(1, "chip_mode0"),
	PINCTRL_PIN(2, "chip_mode1"),
	PINCTRL_PIN(3, "chip_mode2"),
	PINCTRL_PIN(4, "chip_mode3"),
	PINCTRL_PIN(5, "chip_mode4"),
	PINCTRL_PIN(6, "bsc0_scl"),
	PINCTRL_PIN(7, "bsc0_sda"),
	PINCTRL_PIN(8, "bsc1_scl"),
	PINCTRL_PIN(9, "bsc1_sda"),
	PINCTRL_PIN(10, "d1w_dq"),
	PINCTRL_PIN(11, "d1wowstz_l"),
	PINCTRL_PIN(12, "gpio0"),
	PINCTRL_PIN(13, "gpio1"),
	PINCTRL_PIN(14, "gpio2"),
	PINCTRL_PIN(15, "gpio3"),
	PINCTRL_PIN(16, "gpio4"),
	PINCTRL_PIN(17, "gpio5"),
	PINCTRL_PIN(18, "gpio6"),
	PINCTRL_PIN(19, "gpio7"),
	PINCTRL_PIN(20, "gpio8"),
	PINCTRL_PIN(21, "gpio9"),
	PINCTRL_PIN(22, "gpio10"),
	PINCTRL_PIN(23, "gpio11"),
	PINCTRL_PIN(24, "gpio12"),
	PINCTRL_PIN(25, "gpio13"),
	PINCTRL_PIN(26, "gpio14"),
	PINCTRL_PIN(27, "gpio15"),
	PINCTRL_PIN(28, "gpio16"),
	PINCTRL_PIN(29, "gpio17"),
	PINCTRL_PIN(30, "gpio18"),
	PINCTRL_PIN(31, "gpio19"),
	PINCTRL_PIN(32, "gpio20"),
	PINCTRL_PIN(33, "gpio21"),
	PINCTRL_PIN(34, "gpio22"),
	PINCTRL_PIN(35, "gpio23"),
	PINCTRL_PIN(36, "mdc"),
	PINCTRL_PIN(37, "mdio"),
	PINCTRL_PIN(38, "pwm0"),
	PINCTRL_PIN(39, "pwm1"),
	PINCTRL_PIN(40, "pwm2"),
	PINCTRL_PIN(41, "pwm3"),
	PINCTRL_PIN(42, "sc0_clk"),
	PINCTRL_PIN(43, "sc0_cmdvcc_l"),
	PINCTRL_PIN(44, "sc0_detect"),
	PINCTRL_PIN(45, "sc0_fcb"),
	PINCTRL_PIN(46, "sc0_io"),
	PINCTRL_PIN(47, "sc0_rst_l"),
	PINCTRL_PIN(48, "sc1_clk"),
	PINCTRL_PIN(49, "sc1_cmdvcc_l"),
	PINCTRL_PIN(50, "sc1_detect"),
	PINCTRL_PIN(51, "sc1_fcb"),
	PINCTRL_PIN(52, "sc1_io"),
	PINCTRL_PIN(53, "sc1_rst_l"),
	PINCTRL_PIN(54, "spi0_clk"),
	PINCTRL_PIN(55, "spi0_mosi"),
	PINCTRL_PIN(56, "spi0_miso"),
	PINCTRL_PIN(57, "spi0_ss"),
	PINCTRL_PIN(58, "spi1_clk"),
	PINCTRL_PIN(59, "spi1_mosi"),
	PINCTRL_PIN(60, "spi1_miso"),
	PINCTRL_PIN(61, "spi1_ss"),
	PINCTRL_PIN(62, "spi2_clk"),
	PINCTRL_PIN(63, "spi2_mosi"),
	PINCTRL_PIN(64, "spi2_miso"),
	PINCTRL_PIN(65, "spi2_ss"),
	PINCTRL_PIN(66, "spi3_clk"),
	PINCTRL_PIN(67, "spi3_mosi"),
	PINCTRL_PIN(68, "spi3_miso"),
	PINCTRL_PIN(69, "spi3_ss"),
	PINCTRL_PIN(70, "uart0_cts"),
	PINCTRL_PIN(71, "uart0_rts"),
	PINCTRL_PIN(72, "uart0_rx"),
	PINCTRL_PIN(73, "uart0_tx"),
	PINCTRL_PIN(74, "uart1_cts"),
	PINCTRL_PIN(75, "uart1_dcd"),
	PINCTRL_PIN(76, "uart1_dsr"),
	PINCTRL_PIN(77, "uart1_dtr"),
	PINCTRL_PIN(78, "uart1_ri"),
	PINCTRL_PIN(79, "uart1_rts"),
	PINCTRL_PIN(80, "uart1_rx"),
	PINCTRL_PIN(81, "uart1_tx"),
	PINCTRL_PIN(82, "uart3_rx"),
	PINCTRL_PIN(83, "uart3_tx"),
	PINCTRL_PIN(84, "sdio1_clk_sdcard"),
	PINCTRL_PIN(85, "sdio1_cmd"),
	PINCTRL_PIN(86, "sdio1_data0"),
	PINCTRL_PIN(87, "sdio1_data1"),
	PINCTRL_PIN(88, "sdio1_data2"),
	PINCTRL_PIN(89, "sdio1_data3"),
	PINCTRL_PIN(90, "sdio1_wp_n"),
	PINCTRL_PIN(91, "sdio1_card_rst"),
	PINCTRL_PIN(92, "sdio1_led_on"),
	PINCTRL_PIN(93, "sdio1_cd"),
	PINCTRL_PIN(94, "sdio0_clk_sdcard"),
	PINCTRL_PIN(95, "sdio0_cmd"),
	PINCTRL_PIN(96, "sdio0_data0"),
	PINCTRL_PIN(97, "sdio0_data1"),
	PINCTRL_PIN(98, "sdio0_data2"),
	PINCTRL_PIN(99, "sdio0_data3"),
	PINCTRL_PIN(100, "sdio0_wp_n"),
	PINCTRL_PIN(101, "sdio0_card_rst"),
	PINCTRL_PIN(102, "sdio0_led_on"),
	PINCTRL_PIN(103, "sdio0_cd"),
	PINCTRL_PIN(104, "sflash_clk"),
	PINCTRL_PIN(105, "sflash_cs_l"),
	PINCTRL_PIN(106, "sflash_mosi"),
	PINCTRL_PIN(107, "sflash_miso"),
	PINCTRL_PIN(108, "sflash_wp_n"),
	PINCTRL_PIN(109, "sflash_hold_n"),
	PINCTRL_PIN(110, "nand_ale"),
	PINCTRL_PIN(111, "nand_ce0_l"),
	PINCTRL_PIN(112, "nand_ce1_l"),
	PINCTRL_PIN(113, "nand_cle"),
	PINCTRL_PIN(114, "nand_dq0"),
	PINCTRL_PIN(115, "nand_dq1"),
	PINCTRL_PIN(116, "nand_dq2"),
	PINCTRL_PIN(117, "nand_dq3"),
	PINCTRL_PIN(118, "nand_dq4"),
	PINCTRL_PIN(119, "nand_dq5"),
	PINCTRL_PIN(120, "nand_dq6"),
	PINCTRL_PIN(121, "nand_dq7"),
	PINCTRL_PIN(122, "nand_rb_l"),
	PINCTRL_PIN(123, "nand_re_l"),
	PINCTRL_PIN(124, "nand_we_l"),
	PINCTRL_PIN(125, "nand_wp_l"),
	PINCTRL_PIN(126, "lcd_clac"),
	PINCTRL_PIN(127, "lcd_clcp"),
	PINCTRL_PIN(128, "lcd_cld0"),
	PINCTRL_PIN(129, "lcd_cld1"),
	PINCTRL_PIN(130, "lcd_cld10"),
	PINCTRL_PIN(131, "lcd_cld11"),
	PINCTRL_PIN(132, "lcd_cld12"),
	PINCTRL_PIN(133, "lcd_cld13"),
	PINCTRL_PIN(134, "lcd_cld14"),
	PINCTRL_PIN(135, "lcd_cld15"),
	PINCTRL_PIN(136, "lcd_cld16"),
	PINCTRL_PIN(137, "lcd_cld17"),
	PINCTRL_PIN(138, "lcd_cld18"),
	PINCTRL_PIN(139, "lcd_cld19"),
	PINCTRL_PIN(140, "lcd_cld2"),
	PINCTRL_PIN(141, "lcd_cld20"),
	PINCTRL_PIN(142, "lcd_cld21"),
	PINCTRL_PIN(143, "lcd_cld22"),
	PINCTRL_PIN(144, "lcd_cld23"),
	PINCTRL_PIN(145, "lcd_cld3"),
	PINCTRL_PIN(146, "lcd_cld4"),
	PINCTRL_PIN(147, "lcd_cld5"),
	PINCTRL_PIN(148, "lcd_cld6"),
	PINCTRL_PIN(149, "lcd_cld7"),
	PINCTRL_PIN(150, "lcd_cld8"),
	PINCTRL_PIN(151, "lcd_cld9"),
	PINCTRL_PIN(152, "lcd_clfp"),
	PINCTRL_PIN(153, "lcd_clle"),
	PINCTRL_PIN(154, "lcd_cllp"),
	PINCTRL_PIN(155, "lcd_clpower"),
	PINCTRL_PIN(156, "camera_vsync"),
	PINCTRL_PIN(157, "camera_trigger"),
	PINCTRL_PIN(158, "camera_strobe"),
	PINCTRL_PIN(159, "camera_standby"),
	PINCTRL_PIN(160, "camera_reset_n"),
	PINCTRL_PIN(161, "camera_pixdata9"),
	PINCTRL_PIN(162, "camera_pixdata8"),
	PINCTRL_PIN(163, "camera_pixdata7"),
	PINCTRL_PIN(164, "camera_pixdata6"),
	PINCTRL_PIN(165, "camera_pixdata5"),
	PINCTRL_PIN(166, "camera_pixdata4"),
	PINCTRL_PIN(167, "camera_pixdata3"),
	PINCTRL_PIN(168, "camera_pixdata2"),
	PINCTRL_PIN(169, "camera_pixdata1"),
	PINCTRL_PIN(170, "camera_pixdata0"),
	PINCTRL_PIN(171, "camera_pixclk"),
	PINCTRL_PIN(172, "camera_hsync"),
	PINCTRL_PIN(173, "camera_pll_ref_clk"),
	PINCTRL_PIN(174, "usb_id_indication"),
	PINCTRL_PIN(175, "usb_vbus_indication"),
	PINCTRL_PIN(176, "gpio0_3p3"),
	PINCTRL_PIN(177, "gpio1_3p3"),
	PINCTRL_PIN(178, "gpio2_3p3"),
	PINCTRL_PIN(179, "gpio3_3p3"),
};

/*
 * List of groups of pins
 */
static const unsigned gpio0_pins[] = { 12 };
static const unsigned gpio1_pins[] = { 13 };
static const unsigned gpio2_pins[] = { 14 };
static const unsigned gpio3_pins[] = { 15 };
static const unsigned gpio4_pins[] = { 16 };
static const unsigned gpio5_pins[] = { 17 };
static const unsigned gpio6_pins[] = { 18 };
static const unsigned gpio7_pins[] = { 19 };
static const unsigned gpio8_pins[] = { 20 };
static const unsigned gpio9_pins[] = { 21 };
static const unsigned gpio10_pins[] = { 22 };
static const unsigned gpio11_pins[] = { 23 };
static const unsigned gpio12_pins[] = { 24 };
static const unsigned gpio13_pins[] = { 25 };
static const unsigned gpio14_pins[] = { 26 };
static const unsigned gpio15_pins[] = { 27 };
static const unsigned gpio16_pins[] = { 28 };
static const unsigned gpio17_pins[] = { 29 };
static const unsigned gpio18_pins[] = { 30 };
static const unsigned gpio19_pins[] = { 31 };
static const unsigned gpio20_pins[] = { 32 };
static const unsigned gpio21_pins[] = { 33 };
static const unsigned gpio22_pins[] = { 34 };
static const unsigned gpio23_pins[] = { 35 };
static const unsigned pwm0_pins[] = { 38 };
static const unsigned pwm1_pins[] = { 39 };
static const unsigned pwm2_pins[] = { 40 };
static const unsigned pwm3_pins[] = { 41 };
static const unsigned sdio0_pins[] = { 94, 95, 96, 97, 98, 99 };
static const unsigned smart_card0_pins[] = { 42, 43, 44, 46, 47 };
static const unsigned smart_card1_pins[] = { 48, 49, 50, 52, 53 };
static const unsigned spi0_pins[] = { 54, 55, 56, 57 };
static const unsigned spi1_pins[] = { 58, 59, 60, 61 };
static const unsigned spi2_pins[] = { 62, 63, 64, 65 };
static const unsigned spi3_pins[] = { 66, 67, 68, 69 };
static const unsigned d1w_pins[] = { 10, 11 };
static const unsigned lcd_pins[] = { 126, 127, 128, 129, 130, 131, 132,	133,
	134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147,
	148, 149, 150, 151, 152, 153, 154, 155 };
static const unsigned uart0_pins[] = { 70, 71, 72, 73 };
static const unsigned uart1_dte_pins[] = { 75, 76, 77, 78 };
static const unsigned uart1_pins[] = { 74, 79, 80, 81 };
static const unsigned uart3_pins[] = { 82, 83 };
static const unsigned qspi_pins[] = { 104, 105, 106, 107 };
static const unsigned nand_pins[] = { 110, 111, 112, 113, 114, 115, 116, 117,
	118, 119, 120, 121, 122, 123, 124, 125 };
static const unsigned sdio0_cd_pins[] = { 103 };
static const unsigned sdio0_mmc_pins[] = { 100, 101, 102 };
static const unsigned can0_spi4_pins[] = { 86, 87 };
static const unsigned can1_spi4_pins[] = { 88, 89 };
static const unsigned sdio1_cd_pins[] = { 93 };
static const unsigned sdio1_led_pins[] = { 84, 85 };
static const unsigned sdio1_mmc_pins[] = { 90, 91, 92 };
static const unsigned camera_led_pins[] = { 156, 157, 158, 159, 160 };
static const unsigned camera_rgmii_pins[] = { 169, 170, 171, 169, 170, 171,
	172, 173 };
static const unsigned camera_sram_rgmii_pins[] = { 161, 162, 163, 164, 165,
	166, 167, 168 };
static const unsigned qspi_gpio_pins[] = { 108, 109 };
static const unsigned smart_card0_fcb_pins[] = { 45 };
static const unsigned smart_card1_fcb_pins[] = { 51 };
static const unsigned gpio0_3p3_pins[] = { 176 };
static const unsigned gpio1_3p3_pins[] = { 177 };
static const unsigned gpio2_3p3_pins[] = { 178 };

/*
 * List of groups names. Need to match the order in cygnus_pin_groups
 */
static const char * const cygnus_pin_group_names[] = {
	"gpio0",
	"gpio1",
	"gpio2",
	"gpio3",
	"gpio4",
	"gpio5",
	"gpio6",
	"gpio7",
	"gpio8",
	"gpio9",
	"gpio10",
	"gpio11",
	"gpio12",
	"gpio13",
	"gpio14",
	"gpio15",
	"gpio16",
	"gpio17",
	"gpio18",
	"gpio19",
	"gpio20",
	"gpio21",
	"gpio22",
	"gpio23",
	"pwm0",
	"pwm1",
	"pwm2",
	"pwm3",
	"sdio0",
	"smart_card0",
	"smart_card1",
	"spi0",
	"spi1",
	"spi2",
	"spi3",
	"d1w",
	"lcd",
	"uart0",
	"uart1_dte",
	"uart1",
	"uart3",
	"qspi",
	"nand",
	"sdio0_cd",
	"sdio0_mmc",
	"can0_spi4",
	"can1_spi4",
	"sdio1_cd",
	"sdio1_led",
	"sdio1_mmc",
	"camera_led",
	"camera_rgmii",
	"camera_sram_rgmii",
	"qspi_gpio",
	"smart_card0_fcb",
	"smart_card1_fcb",
	"gpio0_3p3",
	"gpio1_3p3",
	"gpio2_3p3",
};

/*
 * List of groups. Need to match the order in cygnus_pin_group_names
 */
static const struct cygnus_pin_group cygnus_pin_groups[] = {
	CYGNUS_PIN_GROUP(gpio0, 0x0, 0),
	CYGNUS_PIN_GROUP(gpio1, 0x0, 4),
	CYGNUS_PIN_GROUP(gpio2, 0x0, 8),
	CYGNUS_PIN_GROUP(gpio3, 0x0, 12),
	CYGNUS_PIN_GROUP(gpio4, 0x0, 16),
	CYGNUS_PIN_GROUP(gpio5, 0x0, 20),
	CYGNUS_PIN_GROUP(gpio6, 0x0, 24),
	CYGNUS_PIN_GROUP(gpio7, 0x0, 28),
	CYGNUS_PIN_GROUP(gpio8, 0x4, 0),
	CYGNUS_PIN_GROUP(gpio9, 0x4, 4),
	CYGNUS_PIN_GROUP(gpio10, 0x4, 8),
	CYGNUS_PIN_GROUP(gpio11, 0x4, 12),
	CYGNUS_PIN_GROUP(gpio12, 0x4, 16),
	CYGNUS_PIN_GROUP(gpio13, 0x4, 20),
	CYGNUS_PIN_GROUP(gpio14, 0x4, 24),
	CYGNUS_PIN_GROUP(gpio15, 0x4, 28),
	CYGNUS_PIN_GROUP(gpio16, 0x8, 0),
	CYGNUS_PIN_GROUP(gpio17, 0x8, 4),
	CYGNUS_PIN_GROUP(gpio18, 0x8, 8),
	CYGNUS_PIN_GROUP(gpio19, 0x8, 12),
	CYGNUS_PIN_GROUP(gpio20, 0x8, 16),
	CYGNUS_PIN_GROUP(gpio21, 0x8, 20),
	CYGNUS_PIN_GROUP(gpio22, 0x8, 24),
	CYGNUS_PIN_GROUP(gpio23, 0x8, 28),
	CYGNUS_PIN_GROUP(pwm0, 0xc, 0),
	CYGNUS_PIN_GROUP(pwm1, 0xc, 4),
	CYGNUS_PIN_GROUP(pwm2, 0xc, 8),
	CYGNUS_PIN_GROUP(pwm3, 0xc, 12),
	CYGNUS_PIN_GROUP(sdio0, 0xc, 16),
	CYGNUS_PIN_GROUP(smart_card0, 0xc, 20),
	CYGNUS_PIN_GROUP(smart_card1, 0xc, 24),
	CYGNUS_PIN_GROUP(spi0, 0x10, 0),
	CYGNUS_PIN_GROUP(spi1, 0x10, 4),
	CYGNUS_PIN_GROUP(spi2, 0x10, 8),
	CYGNUS_PIN_GROUP(spi3, 0x10, 12),
	CYGNUS_PIN_GROUP(d1w, 0x10, 16),
	CYGNUS_PIN_GROUP(lcd, 0x10, 20),
	CYGNUS_PIN_GROUP(uart0, 0x14, 0),
	CYGNUS_PIN_GROUP(uart1_dte, 0x14, 4),
	CYGNUS_PIN_GROUP(uart1, 0x14, 8),
	CYGNUS_PIN_GROUP(uart3, 0x14, 12),
	CYGNUS_PIN_GROUP(qspi, 0x14, 16),
	CYGNUS_PIN_GROUP(nand, 0x14, 20),
	CYGNUS_PIN_GROUP(sdio0_cd, 0x18, 0),
	CYGNUS_PIN_GROUP(sdio0_mmc, 0x18, 4),
	CYGNUS_PIN_GROUP(can0_spi4, 0x18, 8),
	CYGNUS_PIN_GROUP(can1_spi4, 0x18, 12),
	CYGNUS_PIN_GROUP(sdio1_cd, 0x18, 16),
	CYGNUS_PIN_GROUP(sdio1_led, 0x18, 20),
	CYGNUS_PIN_GROUP(sdio1_mmc, 0x18, 24),
	CYGNUS_PIN_GROUP(camera_led, 0x1c, 0),
	CYGNUS_PIN_GROUP(camera_rgmii, 0x1c, 4),
	CYGNUS_PIN_GROUP(camera_sram_rgmii, 0x1c, 8),
	CYGNUS_PIN_GROUP(qspi_gpio, 0x1c, 12),
	CYGNUS_PIN_GROUP(smart_card0_fcb, 0x20, 0),
	CYGNUS_PIN_GROUP(smart_card1_fcb, 0x20, 4),
	CYGNUS_PIN_GROUP(gpio0_3p3, 0x28, 0),
	CYGNUS_PIN_GROUP(gpio1_3p3, 0x28, 4),
	CYGNUS_PIN_GROUP(gpio2_3p3, 0x28, 8),
};

#define CYGNUS_PIN_FUNCTION(fcn_name, mux_val)			\
{								\
	.name = #fcn_name,					\
	.group_names = cygnus_pin_group_names,			\
	.num_groups = ARRAY_SIZE(cygnus_pin_group_names),	\
	.mux = mux_val,						\
}

/*
 * Cygnus has 4 alternate functions. All groups can be configured to any of
 * the 4 alternate functions
 */
static const struct cygnus_pin_function cygnus_pin_functions[] = {
	CYGNUS_PIN_FUNCTION(alt1, 0),
	CYGNUS_PIN_FUNCTION(alt2, 1),
	CYGNUS_PIN_FUNCTION(alt3, 2),
	CYGNUS_PIN_FUNCTION(alt4, 3),
};

static int cygnus_get_groups_count(struct pinctrl_dev *pctrl_dev)
{
	struct cygnus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->num_groups;
}

static const char *cygnus_get_group_name(struct pinctrl_dev *pctrl_dev,
		unsigned selector)
{
	struct cygnus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->groups[selector].name;
}

static int cygnus_get_group_pins(struct pinctrl_dev *pctrl_dev,
		unsigned selector, const unsigned **pins,
		unsigned *num_pins)
{
	struct cygnus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	*pins = pinctrl->groups[selector].pins;
	*num_pins = pinctrl->groups[selector].num_pins;

	return 0;
}

static void cygnus_pin_dbg_show(struct pinctrl_dev *pctrl_dev,
		struct seq_file *s, unsigned offset)
{
	seq_printf(s, " %s", dev_name(pctrl_dev->dev));
}

static int find_matched_function(const char *function_name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cygnus_pin_functions); i++) {
		if (!strcmp(cygnus_pin_functions[i].name, function_name))
			return (int)cygnus_pin_functions[i].mux;
	}

	return -EINVAL;
}

static int cygnus_dt_node_to_map(struct pinctrl_dev *pctrl_dev,
		struct device_node *np, struct pinctrl_map **map,
		unsigned *num_maps)
{
	int ret, num_groups;
	unsigned reserved_maps = 0;
	struct property *prop;
	const char *group_name, *function_name;

	*map = NULL;
	*num_maps = 0;

	num_groups = of_property_count_strings(np, "brcm,groups");
	if (num_groups < 0) {
		dev_err(pctrl_dev->dev,
			"could not parse property brcm,groups\n");
		return -EINVAL;
	}

	ret = of_property_read_string(np, "brcm,function", &function_name);
	if (ret < 0) {
		dev_err(pctrl_dev->dev,
			"could not parse property brcm,function\n");
		return -EINVAL;
	}

	/* make sure it's a valid alternate function */
	ret = find_matched_function(function_name);
	if (ret < 0) {
		dev_err(pctrl_dev->dev, "invalid function name: %s\n",
				function_name);
	}

	ret = pinctrl_utils_reserve_map(pctrl_dev, map, &reserved_maps,
			num_maps, num_groups);
	if (ret) {
		dev_err(pctrl_dev->dev, "unable to reserve map\n");
		return ret;
	}

	of_property_for_each_string(np, "brcm,groups", prop, group_name) {
		ret = pinctrl_utils_add_map_mux(pctrl_dev, map,
				&reserved_maps, num_maps, group_name,
				function_name);
		if (ret) {
			dev_err(pctrl_dev->dev, "can't add map: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static struct pinctrl_ops cygnus_pinctrl_ops = {
	.get_groups_count = cygnus_get_groups_count,
	.get_group_name = cygnus_get_group_name,
	.get_group_pins = cygnus_get_group_pins,
	.pin_dbg_show = cygnus_pin_dbg_show,
	.dt_node_to_map = cygnus_dt_node_to_map,
	.dt_free_map = pinctrl_utils_dt_free_map,
};

static int cygnus_get_functions_count(struct pinctrl_dev *pctrl_dev)
{
	struct cygnus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->num_functions;
}

static const char *cygnus_get_function_name(struct pinctrl_dev *pctrl_dev,
		unsigned selector)
{
	struct cygnus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	return pinctrl->functions[selector].name;
}

static int cygnus_get_function_groups(struct pinctrl_dev *pctrl_dev,
	unsigned selector, const char * const **groups,
	unsigned * const num_groups)
{
	struct cygnus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);

	*groups = pinctrl->functions[selector].group_names;
	*num_groups = pinctrl->functions[selector].num_groups;

	return 0;
}

static int cygnus_pinmux_set_mux(struct pinctrl_dev *pctrl_dev,
		unsigned function_selector, unsigned group_selector)
{
	struct cygnus_pinctrl *pinctrl = pinctrl_dev_get_drvdata(pctrl_dev);
	const struct cygnus_pin_function *function =
			&pinctrl->functions[function_selector];
	const struct cygnus_pin_group *group =
			&pinctrl->groups[group_selector];
	u32 val, mask = 0x7;

	dev_dbg(pctrl_dev->dev,
	"group:%s with offset:0x%08x shift:%u set to function: %s mux:%u\n",
		group->name, group->offset, group->shift, function->name,
		function->mux);

	val = readl(pinctrl->base + group->offset);
	val &= ~(mask << group->shift);
	val |= function->mux << group->shift;
	writel(val, pinctrl->base + group->offset);

	return 0;
}

static struct pinmux_ops cygnus_pinmux_ops = {
	.get_functions_count = cygnus_get_functions_count,
	.get_function_name = cygnus_get_function_name,
	.get_function_groups = cygnus_get_function_groups,
	.set_mux = cygnus_pinmux_set_mux,
};

static struct pinctrl_desc cygnus_pinctrl_desc = {
	.pctlops = &cygnus_pinctrl_ops,
	.pmxops = &cygnus_pinmux_ops,
	.owner = THIS_MODULE,
};

static int cygnus_pinctrl_probe(struct platform_device *pdev)
{
	struct cygnus_pinctrl *pinctrl;
	struct resource *res;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl) {
		dev_err(&pdev->dev, "unable to allocate memory\n");
		return -ENOMEM;
	}
	pinctrl->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "unable to get resource\n");
		return -ENOENT;
	}

	pinctrl->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pinctrl->base)) {
		dev_err(&pdev->dev, "unable to map I/O space\n");
		return PTR_ERR(pinctrl->base);
	}

	pinctrl->pins = cygnus_pinctrl_pins;
	pinctrl->num_pins = ARRAY_SIZE(cygnus_pinctrl_pins);
	pinctrl->groups = cygnus_pin_groups;
	pinctrl->num_groups = ARRAY_SIZE(cygnus_pin_groups);
	pinctrl->functions = cygnus_pin_functions;
	pinctrl->num_functions = ARRAY_SIZE(cygnus_pin_functions);

	cygnus_pinctrl_desc.name = dev_name(&pdev->dev);
	cygnus_pinctrl_desc.pins = cygnus_pinctrl_pins;
	cygnus_pinctrl_desc.npins = ARRAY_SIZE(cygnus_pinctrl_pins);

	pinctrl->pctl = pinctrl_register(&cygnus_pinctrl_desc, &pdev->dev,
			pinctrl);
	if (!pinctrl->pctl) {
		dev_err(&pdev->dev, "unable to register cygnus pinctrl\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, pinctrl);

	return 0;
}

static int cygnus_pinctrl_remove(struct platform_device *pdev)
{
	struct cygnus_pinctrl *pinctrl = platform_get_drvdata(pdev);

	pinctrl_unregister(pinctrl->pctl);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct of_device_id cygnus_pinctrl_of_match[] = {
	{ .compatible = "brcm,cygnus-pinctrl", },
	{ },
};

static struct platform_driver cygnus_pinctrl_driver = {
	.driver = {
		.name = "cygnus-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = cygnus_pinctrl_of_match,
	},
	.probe = cygnus_pinctrl_probe,
	.remove = cygnus_pinctrl_remove,
};

static int __init cygnus_pinctrl_init(void)
{
	return platform_driver_register(&cygnus_pinctrl_driver);
}
arch_initcall(cygnus_pinctrl_init);

static void __exit cygnus_pinctrl_exit(void)
{
	platform_driver_unregister(&cygnus_pinctrl_driver);
}
module_exit(cygnus_pinctrl_exit);

MODULE_AUTHOR("Ray Jui <rjui@broadcom.com>");
MODULE_DESCRIPTION("Broadcom Cygnus pinctrl driver");
MODULE_LICENSE("GPL v2");
