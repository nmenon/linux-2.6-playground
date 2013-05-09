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
	if (!cops->devm_pmic_register || !cops->voltage_set ||
	    !cops->voltage_get || !cops->voltage_get_range) {
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
 * omap_pmic_vsel_to_uv() - Convert voltage selector(vsel) to microvolts
 * @pmic:	pointer to pmic struct
 * @vsel:	voltage selector(vsel)
 * @uv:		If conversion is successful, returns the voltage in micro volts
 *
 * Return: 0 if conversion is successful and *uv has proper value, else
 * appropriate error value for failure.
 */
static int omap_pmic_vsel_to_uv(struct omap_pmic *pmic, u8 vsel, u32 *uv)
{
	u32 tmp = vsel;

	if (!pmic || !uv) {
		pr_err("Bad parameters pmic=%p uv=%p!\n", pmic, uv);
		return -EINVAL;
	}

	if (pmic->voltage_selector_mask) {
		tmp &= pmic->voltage_selector_mask;
		tmp >>= __ffs(pmic->voltage_selector_mask);
	}

	if (!tmp && pmic->voltage_selector_zero)
		goto out;

	tmp -= pmic->voltage_selector_offset;
	tmp *= pmic->step_size_uV;
	tmp += pmic->min_uV;

	if (tmp < pmic->min_uV || tmp > pmic->max_uV) {
		dev_dbg(pmic->dev, "%s: Out of range 0x%02x[%d] (%d <-> %d)\n",
			__func__, vsel, tmp, pmic->min_uV, pmic->max_uV);
		return -ERANGE;
	}

out:
	*uv = tmp;
	dev_dbg(pmic->dev, "%s: uv=%d vsel=0x%02x\n", __func__, *uv, vsel);

	return 0;
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
	u32 tmp = uv;

	if (!pmic || !vsel) {
		pr_err("Bad parameters pmic=%p vsel=%p!\n", pmic, vsel);
		return -EINVAL;
	}

	if (!tmp && pmic->voltage_selector_zero)
		goto skip_convert;

	if (tmp > pmic->max_uV)
		goto skip_convert;

	tmp -= pmic->min_uV;
	tmp = DIV_ROUND_UP(tmp, pmic->step_size_uV);

	tmp += pmic->voltage_selector_offset;

skip_convert:
	if (tmp > 0xFF) {
		dev_dbg(pmic->dev, "%s: Out of range 0x%04x[%d] (%d - %d)\n",
			__func__, tmp, uv, pmic->min_uV, pmic->max_uV);
		return -ERANGE;
	}
	if (pmic->voltage_selector_mask) {
		tmp <<= __ffs(pmic->voltage_selector_mask);
		if (tmp > 0xFF) {
			dev_warn(pmic->dev, "%s: Out of range 0x%04x[%d]\n",
				 __func__, tmp, uv);
			return -ERANGE;
		}
		tmp &= pmic->voltage_selector_mask;
	}

	tmp |= pmic->voltage_selector_setbits;

	*vsel = tmp;
	dev_dbg(pmic->dev, "%s: uv=%d vsel=0x%02x\n", __func__, uv, *vsel);

	return 0;
}

/**
 * omap_pmic_of_read_setup_commands() - read setup commands from OF
 * @dev:	device to pick up setup commands from
 * @pmic:	pointer to pmic structure
 */
static int omap_pmic_of_read_setup_commands(struct device *dev,
					    struct omap_pmic *pmic)
{
	char *pname = "ti,setup_commands";
	const struct property *prop;
	const __be32 *v;
	struct omap_pmic_setup_commands *setup;
	const u8 num_values = 2;
	u32 num_entries;
	int i;

	prop = of_find_property(dev->of_node, pname, NULL);
	if (!prop)
		return 0;

	if (!prop->value) {
		dev_err(dev, "Empty '%s' property?\n", pname);
		return -ENODATA;
	}

	/* Each setup command is a tuple consisting of reg_addr, value */
	num_entries = prop->length / sizeof(u32);
	if (!num_entries || (num_entries % num_values)) {
		dev_err(dev, "All '%s' list entries need %d vals\n", pname,
			num_values);
		return -EINVAL;
	}
	num_entries /= num_values;

	setup = devm_kzalloc(dev, sizeof(*setup) * num_entries, GFP_KERNEL);
	if (!setup) {
		dev_err(dev, "Can't allocate info table for '%s' property\n",
			pname);
		return -ENOMEM;
	}

	pmic->setup_command_list = setup;
	pmic->setup_num_commands = num_entries;

	v = prop->value;
	for (i = 0; i < num_entries; i++, setup++) {
		u32 reg, cmd_val;

		/* NOTE: num_values should equal to entries picked up here */
		reg = be32_to_cpup(v++);
		cmd_val = be32_to_cpup(v++);

		/* Setup commands and registers are 8 bit wide. */
		if (reg > 0xFF || cmd_val > 0xFF) {
			dev_err(dev, "Bad entries in '%s' property idx=%d\n",
				pname, i);
			return -EINVAL;
		}

		setup->reg = reg;
		setup->cmd_val = cmd_val;
		dev_dbg(dev, "[setup cmd %d] reg=0x%02x val=0x%02x\n", i,
			setup->reg, setup->cmd_val);
	}

	return 0;
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
		int gpio;
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
		ret = gpio_direction_output(gpio,
					    !!(flags & OF_GPIO_ACTIVE_LOW));
		if (ret) {
			dev_err(dev, "Failed to set GPIO %d (%d)\n", gpio, ret);
			return ret;
		}
		dev_dbg(dev, "GPIO=%d set_to=%d\n", gpio,
			!!(flags & OF_GPIO_ACTIVE_LOW));
	}

	return 0;
}

/**
 * omap_pmic_set_voltage() - regulator interface to set voltage
 * @rdev:	regulator device
 * @min_uV:	min voltage in micro-volts
 * @max_uV:	max voltage in micro-volts
 * @unused:	unused.. we dont use sel
 *
 * Return: -ERANGE for out of range values, appropriate error code if conversion
 * fails, else returns 0.
 */
static int omap_pmic_set_voltage(struct regulator_dev *rdev, int min_uV,
				 int max_uV, unsigned *unused)
{
	struct omap_pmic *pmic = rdev_get_drvdata(rdev);

	return pmic_cops->voltage_set(pmic->v_dev, min_uV);
}

/**
 * omap_pmic_get_voltage() - regulator interface to get voltage
 * @rdev: regulator device
 *
 * Return: current voltage set on PMIC OR appropriate error value
 */
static int omap_pmic_get_voltage(struct regulator_dev *rdev)
{
	struct omap_pmic *pmic = rdev_get_drvdata(rdev);
	int ret;
	u32 uv;

	ret = pmic_cops->voltage_get(pmic->v_dev, &uv);
	if (ret)
		return ret;

	return uv;
}

static struct omap_pmic_ops omap_generic_pmic_ops = {
	.vsel_to_uv = omap_pmic_vsel_to_uv,
	.uv_to_vsel = omap_pmic_uv_to_vsel,
};

static struct regulator_ops omap_pmic_reg_ops = {
	.list_voltage = regulator_list_voltage_linear,

	.set_voltage = omap_pmic_set_voltage,
	.get_voltage = omap_pmic_get_voltage,
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

	pname = "ti,i2c-slave-address";
	ret = of_property_read_u32(node, pname, &val);
	/* Only 7 bit addressing allowed for slave address */
	if (ret || val >= 0x80)
		goto invalid_of_property;
	pmic->slave_addr = val;

	pname = "ti,i2c-voltage-register";
	ret = of_property_read_u32(node, pname, &val);
	if (ret || val >= 0xFF)
		goto invalid_of_property;
	pmic->voltage_reg_addr = val;

	pname = "ti,i2c-command-register";
	ret = of_property_read_u32(node, pname, &val);
	if (ret || val >= 0xFF)
		goto invalid_of_property;
	pmic->cmd_reg_addr = val;

	pname = "ti,slew-rate-microvolt";
	ret = of_property_read_u32(node, pname, &val);
	if (ret || !val)
		goto invalid_of_property;
	pmic->slew_rate_uV = val;

	pname = "ti,step-size-microvolt";
	ret = of_property_read_u32(node, pname, &val);
	if (ret || !val)
		goto invalid_of_property;
	pmic->step_size_uV = val;

	/* Optional parameters */
	pmic->voltage_selector_zero =
	    !of_property_read_bool(node, "ti,non-zero-voltage-selector");

	pname = "ti,boot-voltage-micro-volts";
	ret = of_property_read_u32(node, pname, &val);
	if (!ret) {
		if (!val)
			goto invalid_of_property;
		pmic->boot_voltage_uV = val;
	}

	ret = of_property_read_u32(node, "ti,i2c-timeout-microsecond",
				   &pmic->i2c_timeout_us);
	/* If we dont have custom parameter, use arbitary un-realistic value */
	if (ret)
		pmic->i2c_timeout_us = 200;

	pname = "ti,voltage-selector-offset";
	ret = of_property_read_u32(node, pname, &val);
	if (!ret) {
		if (val > 0xFF)
			goto invalid_of_property;
		pmic->voltage_selector_offset = val;
	}
	pname = "ti,voltage-selector-mask";
	ret = of_property_read_u32(node, pname, &val);
	if (!ret) {
		if (val > 0xFF)
			goto invalid_of_property;
		pmic->voltage_selector_mask = val;
	}
	pname = "ti,voltage-selector-set-bits";
	ret = of_property_read_u32(node, pname, &val);
	if (!ret) {
		if (val > 0xFF)
			goto invalid_of_property;
		pmic->voltage_selector_setbits = val;
	}

	ret = omap_pmic_of_read_setup_commands(dev, pmic);

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

static int omap_pmic_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
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

	initdata = of_get_regulator_init_data(dev, node);
	if (!initdata) {
		dev_err(dev, "%s: Unable to alloc regulator init data\n",
			__func__);
		return -ENOMEM;
	}
	c = &initdata->constraints;
	if (!c->max_uV) {
		dev_err(dev, "regulator-max-microvolt is set as 0?\n");
		return -EINVAL;
	}

	pmic->min_uV = c->min_uV;
	pmic->max_uV = c->max_uV;

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
	desc->uV_step = pmic->step_size_uV;
	desc->ramp_delay = pmic->slew_rate_uV;

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

	return ret;
}

static const struct of_device_id omap_pmic_of_match_tbl[] = {
	{.compatible = "ti,omap-pmic",},
	{},
};
MODULE_DEVICE_TABLE(of, omap_pmic_of_match_tbl);

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
