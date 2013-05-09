/*
 * OMAP Generic PMIC Regulator
 *
 * Idea based on arch/arm/mach-omap2/omap_twl.c
 * Copyright (C) 2010 Texas Instruments Incorporated.
 * Thara Gopinath
 * Copyright (C) 2009 Texas Instruments Incorporated.
 * Nishanth Menon
 * Copyright (C) 2009 Nokia Corporation
 * Paul Walmsley
 *
 * Copyright (C) 2013 Texas Instruments Incorporated
 * Grygorii Strashko
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/omap-pmic-regulator.h>

#define DRIVER_NAME	"omap-pmic"

static DEFINE_MUTEX(omap_pmic_cops_mutex);
static struct omap_pmic_controller_ops *pmic_cops;

/**
 * omap_pmic_register_controller_ops() - Register voltage operations
 * @cops:	voltage operations
 *
 * It is expected that appropriate controller register it's functions
 * with this driver using this interface, If this is not done, the probe
 * for the corresponding device will defer till it fails.
 *
 * Return: -EBUSY if already registered, else returns 0
 */
int omap_pmic_register_controller_ops(struct omap_pmic_controller_ops *cops)
{
	int ret = 0;

	mutex_lock(&omap_pmic_cops_mutex);
	if (pmic_cops) {
		pr_err("Controller operations already registered\n");
		ret = -EBUSY;
		goto out;
	}
	if (!cops->devm_pmic_register || !cops->voltage_get_range ||
	    !cops->setup || !cops->voltage_get_vsel ||
	    !cops->voltage_set_vsel) {
		pr_err("Missing operations!\n");
		ret = -EINVAL;
		goto out;
	}

	pmic_cops = cops;
out:
	mutex_unlock(&omap_pmic_cops_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(omap_pmic_register_controller_ops);

/**
 * omap_pmic_map_voltage() - Map a voltage range to step accuracy for vsel
 * @rdev:	regulator device
 * @min_uv:	minimum voltage in micro volts requested
 * @max_uv:	maximum voltage in micro volts requested
 *
 * Return: >=0 if conversion was sucessful and contains selector. else < 0
 * as the appropriate error value for failure.
 */
static int omap_pmic_map_voltage(struct regulator_dev *rdev,
				 int min_uv, int max_uv)
{
	const struct omap_pmic_info *info;
	struct omap_pmic *pmic = rdev_get_drvdata(rdev);
	int ret = 0;

	if (!pmic || !rdev) {
		pr_err("Bad parameters/not ready pmic=%p rdev=%p!\n",
		       pmic, rdev);
		return -EINVAL;
	}
	info = pmic->info;

	/*
	 * At least ensure that the max voltage is at next step voltage
	 * to avoid min and max inside step boundary, so do the best voltage
	 * we can. Let the regulator core handle the constraints on system.
	 */
	if (max_uv % info->step_size_uV)
		max_uv += info->step_size_uV;

	if (min_uv || !info->voltage_selector_zero)
		ret = regulator_map_voltage_linear(rdev, min_uv, max_uv);

	dev_err(pmic->dev, "%s: uv=[%d-%d] res=%d\n", __func__,
		min_uv, max_uv, ret);
	return ret;
}


/**
 * omap_pmic_uv_to_vsel() - Convert microvolts to voltage selector(vsel)
 * @pmic:	pointer to pmic struct
 * @uv:		voltage in micro volts
 * @vsel:	If conversion is successful, voltage selector(vsel)
 *
 * Return: 0 if conversion is successful and *vsel has proper value, else
 * appropriate error value for failure.
 */
static int omap_pmic_uv_to_vsel(struct omap_pmic *pmic, u32 uv, u8 *vsel)
{
	int ret;
	struct regulator_dev *rdev = dev_get_drvdata(pmic->dev);

	ret = omap_pmic_map_voltage(rdev, uv, uv);
	if (ret >= 0) {
		*vsel = (u8) ret;
		*vsel |= pmic->info->voltage_selector_setbits;
		ret = 0;
	}
	return ret;
}

static struct omap_pmic_ops omap_generic_pmic_ops = {
	.uv_to_vsel = omap_pmic_uv_to_vsel,
};

/**
 * omap_pmic_set_voltage() - regulator interface to set voltage
 * @rdev:	regulator device
 * @sel:	selector
 *
 * Return: -ERANGE for out of range values, appropriate error code if conversion
 * fails, else returns 0.
 */
static int omap_pmic_set_voltage_sel(struct regulator_dev *rdev, unsigned sel)
{
	struct omap_pmic *pmic = rdev_get_drvdata(rdev);

	if (sel > 0xFF) {
		WARN(1, "out of range sel=0x%08x\n", sel);
		return -EINVAL;
	}
	sel |= pmic->info->voltage_selector_setbits;
	return pmic_cops->voltage_set_vsel(pmic->v_dev, (u8)sel);
}

/**
 * omap_pmic_get_voltage() - regulator interface to get voltage
 * @rdev: regulator device
 *
 * Return: current voltage set on PMIC OR appropriate error value
 */
static int omap_pmic_get_voltage_sel(struct regulator_dev *rdev)
{
	struct omap_pmic *pmic = rdev_get_drvdata(rdev);
	int ret;
	u8 sel;

	ret = pmic_cops->voltage_get_vsel(pmic->v_dev, &sel);
	if (ret)
		return ret;

	sel &= ~pmic->info->voltage_selector_setbits;
	return sel;
}

static struct regulator_ops omap_pmic_reg_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = omap_pmic_map_voltage,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,

	.set_voltage_sel = omap_pmic_set_voltage_sel,
	.get_voltage_sel = omap_pmic_get_voltage_sel,
};

/**
 * omap_pmic_parse_of() - Do DT OF node parsing
 * @pmic:	pointer to PMIC
 */
static int omap_pmic_parse_of(struct omap_pmic *pmic)
{
	struct device *dev = pmic->dev;
	struct device_node *node = dev->of_node;
	u32 val = 0;
	char *pname;
	int ret;

	pname = "ti,boot-voltage-micro-volts";
	ret = of_property_read_u32(node, pname, &val);
	if (!ret) {
		if (!val)
			goto invalid_of_property;
		pmic->boot_voltage_uV = val;
	}

	return ret;

invalid_of_property:
	if (!ret) {
		dev_err(dev, "Invalid value 0x%x[%d] in '%s' property.\n",
			val, val, pname);
		ret = -EINVAL;
	} else {
		dev_err(dev, "Missing/Invalid '%s' property - error(%d)\n",
			pname, ret);
	}
	return ret;
}

/**
 * omap_pmic_of_setup_gpios() - Setup GPIO array if needed.
 * @dev:	device to pick up the gpios from
 */
static int omap_pmic_of_setup_gpios(struct device *dev)
{
	struct device_node *node = dev->of_node;
	int num_gpios, i, ret;

	num_gpios = of_gpio_count(node);
	if (num_gpios < 0)
		return 0;

	for (i = 0; i < num_gpios; i++) {
		int gpio, level;
		enum of_gpio_flags flags;

		gpio = of_get_gpio_flags(node, i, &flags);
		if (!gpio_is_valid(gpio)) {
			dev_err(dev, "Invalid GPIO[%d]: %d\n", i, gpio);
			return -EINVAL;
		}

		ret = devm_gpio_request(dev, gpio, dev_name(dev));
		if (ret) {
			dev_err(dev, "Unable to get GPIO %d (%d)\n", gpio, ret);
			return ret;
		}
		level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = gpio_direction_output(gpio, level);
		if (ret) {
			dev_err(dev, "Failed to set GPIO %d to %d (%d)\n",
				gpio, level, ret);
			return ret;
		}
		dev_dbg(dev, "GPIO=%d set_to=%d flags=0x%08x\n", gpio,
			level, flags);
	}

	return 0;
}

static const struct omap_pmic_info omap_twl4030_vdd1 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x00,
	.cmd_reg_addr = 0x00,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 4000,
	.step_size_uV = 12500,
	.min_uV = 600000,
	.max_uV = 1450000,
	.voltage_selector_offset = 0,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = false,
};

static const struct omap_pmic_info omap_twl4030_vdd2 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x01,
	.cmd_reg_addr = 0x01,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 4000,
	.step_size_uV = 12500,
	.min_uV = 600000,
	.max_uV = 1450000,
	.voltage_selector_offset = 0,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = false,
};

static const struct omap_pmic_info omap_twl6030_vcore1 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x55,
	.cmd_reg_addr = 0x56,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 9000,
	.step_size_uV = 12660,
	.min_uV = 709000,
	.max_uV = 1418000,
	.voltage_selector_offset = 0x1,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct omap_pmic_info omap_twl6030_vcore2 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x5b,
	.cmd_reg_addr = 0x5c,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 9000,
	.step_size_uV = 12660,
	.min_uV = 709000,
	.max_uV = 1418000,
	.voltage_selector_offset = 0x1,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct omap_pmic_info omap_twl6030_vcore3 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x61,
	.cmd_reg_addr = 0x62,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 9000,
	.step_size_uV = 12660,
	.min_uV = 709000,
	.max_uV = 1418000,
	.voltage_selector_offset = 0x1,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct omap_pmic_setup_commands omap_tps62361_cmds[] = {
	{.reg = 0x06, .cmd_val = 0x06},	/* TPS6236X_RAMP_CTRL 32mV/uS */
	{.reg = 0x04, .cmd_val = 0xc0},	/* TPS6236X_CTRL VSEL0 pull down */
	{.reg = 0x05, .cmd_val = 0x00}, /* REG_TPS6236X_TEMP enable tshut */
};

static const struct omap_pmic_info omap_tps62361 = {
	.slave_addr = 0x60,
	.voltage_reg_addr = 0x01,
	.cmd_reg_addr = 0x01,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 32000,
	.step_size_uV = 10000,
	.min_uV = 500000,
	.max_uV = 1770000,
	.voltage_selector_offset = 0x0,
	.voltage_selector_setbits = 0x80, /* PFM mode */
	.voltage_selector_zero = false,
	.setup_command_list = omap_tps62361_cmds,
	.setup_num_commands = ARRAY_SIZE(omap_tps62361_cmds),
};

static const struct omap_pmic_info omap_twl6032_smps1 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x55,
	.cmd_reg_addr = 0x56,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 9000,
	.step_size_uV = 12660,
	.min_uV = 709000,
	.max_uV = 1418000,
	.voltage_selector_offset = 0x1,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct omap_pmic_info omap_twl6032_smps2 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x5b,
	.cmd_reg_addr = 0x5c,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 9000,
	.step_size_uV = 12660,
	.min_uV = 709000,
	.max_uV = 1418000,
	.voltage_selector_offset = 0x1,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct omap_pmic_info omap_twl6032_smps5 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x49,
	.cmd_reg_addr = 0x4a,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 9000,
	.step_size_uV = 12660,
	.min_uV = 709000,
	.max_uV = 1418000,
	.voltage_selector_offset = 0x1,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct omap_pmic_info omap_twl6035_smps1 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x23,
	.cmd_reg_addr = 0x22,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 220,
	.step_size_uV = 10000,
	.min_uV = 500000,
	.max_uV = 1650000,
	.voltage_selector_offset = 0x6,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct omap_pmic_info omap_twl6035_smps4 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x2b,
	.cmd_reg_addr = 0x2a,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 220,
	.step_size_uV = 10000,
	.min_uV = 500000,
	.max_uV = 1650000,
	.voltage_selector_offset = 0x6,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct omap_pmic_info omap_twl6035_smps8 = {
	.slave_addr = 0x12,
	.voltage_reg_addr = 0x37,
	.cmd_reg_addr = 0x36,
	.i2c_timeout_us = 200,
	.slew_rate_uV = 220,
	.step_size_uV = 10000,
	.min_uV = 500000,
	.max_uV = 1650000,
	.voltage_selector_offset = 0x6,
	.voltage_selector_setbits = 0x0,
	.voltage_selector_zero = true,
};

static const struct of_device_id omap_pmic_of_match_tbl[] = {
	{.compatible = "ti,omap-twl4030-vdd1", .data = &omap_twl4030_vdd1,},
	{.compatible = "ti,omap-twl4030-vdd2", .data = &omap_twl4030_vdd2,},
	{.compatible = "ti,omap-twl6030-vcore1", .data = &omap_twl6030_vcore1,},
	{.compatible = "ti,omap-twl6030-vcore2", .data = &omap_twl6030_vcore2,},
	{.compatible = "ti,omap-twl6030-vcore3", .data = &omap_twl6030_vcore3,},
	{.compatible = "ti,omap-tps62361", .data = &omap_tps62361,},
	{.compatible = "ti,omap-twl6032-smps1", .data = &omap_twl6032_smps1,},
	{.compatible = "ti,omap-twl6032-smps2", .data = &omap_twl6032_smps2,},
	{.compatible = "ti,omap-twl6032-smps5", .data = &omap_twl6032_smps5,},
	{.compatible = "ti,omap-twl6035-smps1", .data = &omap_twl6035_smps1,},
	{.compatible = "ti,omap-twl6035-smps4", .data = &omap_twl6035_smps4,},
	{.compatible = "ti,omap-twl6035-smps8", .data = &omap_twl6035_smps8,},
	{},
};
MODULE_DEVICE_TABLE(of, omap_pmic_of_match_tbl);

static int omap_pmic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;
	struct omap_pmic *pmic;
	struct regulator_desc *desc;
	struct regulation_constraints *c;
	struct regulator_config config = { };
	struct regulator_init_data *initdata = NULL;
	struct regulator_dev *rdev = NULL;
	int ret = 0;
	bool ops_ready;

	if (!node) {
		dev_err(dev, "%s: missing device tree nodes?\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&omap_pmic_cops_mutex);
	ops_ready = pmic_cops ? true : false;
	mutex_unlock(&omap_pmic_cops_mutex);
	if (!ops_ready) {
		dev_dbg(dev, "Voltage Operations not ready yet..\n");
		return -EPROBE_DEFER;
	}

	match = of_match_device(omap_pmic_of_match_tbl, dev);
	if (!match) {
		/* We do not expect this to happen */
		dev_err(dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}
	if (!match->data) {
		dev_err(dev, "%s: Bad data in match\n", __func__);
		return -EINVAL;
	}

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc) {
		dev_err(dev, "%s: unable to allocate desc\n", __func__);
		return -ENOMEM;
	}

	pmic = devm_kzalloc(dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		dev_err(dev, "%s: unable to allocate pmic\n", __func__);
		return -ENOMEM;
	}

	/* Read mandatory OF parameters */
	pmic->dev = dev;
	pmic->ops = &omap_generic_pmic_ops;
	pmic->info = match->data;

	initdata = of_get_regulator_init_data(dev, node);
	if (!initdata) {
		dev_err(dev, "%s: Unable to alloc regulator init data\n",
			__func__);
		return -ENOMEM;
	}
	c = &initdata->constraints;

	/* Constraint to PMIC limits */
	if (pmic->info->min_uV > c->min_uV)
		c->min_uV = pmic->info->min_uV;
	if (pmic->info->max_uV < c->max_uV)
		c->max_uV = pmic->info->max_uV;

	ret = omap_pmic_parse_of(pmic);
	if (ret)
		return ret;

	ret = omap_pmic_of_setup_gpios(dev);
	if (ret)
		return ret;

	pmic->v_dev = pmic_cops->devm_pmic_register(dev, pmic);
	if (IS_ERR(pmic->v_dev)) {
		dev_dbg(dev, "Registration of pmic failed (%d)\n", ret);
		ret = PTR_ERR(pmic->v_dev);
		return ret;
	}
	desc->name = dev_name(dev);
	desc->owner = THIS_MODULE;
	desc->type = REGULATOR_VOLTAGE;
	desc->ops = &omap_pmic_reg_ops;
	desc->min_uV = pmic->info->min_uV;
	desc->uV_step = pmic->info->step_size_uV;
	desc->ramp_delay = pmic->info->slew_rate_uV;
	desc->linear_min_sel = pmic->info->voltage_selector_offset;
	desc->n_voltages = pmic->info->max_uV - pmic->info->min_uV;
	desc->n_voltages /= desc->uV_step;
	desc->n_voltages += desc->linear_min_sel;

	c->valid_ops_mask |= REGULATOR_CHANGE_VOLTAGE;
	c->always_on = true;
	ret = pmic_cops->voltage_get_range(pmic->v_dev, &c->min_uV, &c->max_uV);
	if (ret) {
		dev_err(dev, "Voltage Range get failed (%d)\n", ret);
		return ret;
	}

	config.dev = dev;
	config.init_data = initdata;
	config.driver_data = pmic;
	config.of_node = node;

	rdev = regulator_register(desc, &config);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(dev, "%s: failed to register regulator(%d)\n",
			__func__, ret);
		return ret;
	}

	platform_set_drvdata(pdev, rdev);

	ret = pmic_cops->setup(pmic->v_dev);
	if (ret)
		goto out;

	if (!pmic->boot_voltage_uV)
		goto out;
	ret = regulator_map_voltage_linear(rdev, pmic->boot_voltage_uV,
					   pmic->boot_voltage_uV +
					   pmic->info->step_size_uV);
	if (ret < 0) {
		dev_err(dev, "%s: failed to map boot voltage(%d) res=%d\n",
			__func__, pmic->boot_voltage_uV, ret);
		goto out;
	}

	ret = omap_pmic_set_voltage_sel(rdev, ret);
	if (ret) {
		dev_err(dev, "%s: failed to set boot voltage(%d) res=%d\n",
			__func__, pmic->boot_voltage_uV, ret);
	}

out:
	if (ret)
		regulator_unregister(rdev);

	return ret;
}

static struct platform_driver omap_pmic_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(omap_pmic_of_match_tbl),
		   },
	.probe = omap_pmic_probe,
};
module_platform_driver(omap_pmic_driver);

MODULE_DESCRIPTION("OMAP Generic PMIC Regulator");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc.");
