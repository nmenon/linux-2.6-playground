/*
 * Texas Instruments' Palmas Power Button Input Driver
 *
 * Copyright (C) 2012-2014 Texas Instruments Incorporated - http://www.ti.com/
 *	Girish S Ghongdemath
 *	Nishanth Menon
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
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/palmas.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/slab.h>

#define PALMAS_LPK_TIME_MASK		0x0c
#define PALMAS_PWRON_DEBOUNCE_MASK	0x03
#define PALMAS_PWR_KEY_PRESS		0x01
#define PALMAS_PWR_KEY_Q_TIME_MS	20

/**
 * struct palmas_pwron - Palmas power on data
 * @palmas:		pointer to palmas device
 * @input_dev:		pointer to input device
 * @irq:		irq that we are hooked on to
 * @input_work:		work for detecting release of key
 * @current_state:	key current state
 * @key_recheck_ms:	duration for recheck of key (in milli-seconds)
 */
struct palmas_pwron {
	struct palmas *palmas;
	struct input_dev *input_dev;
	int irq;
	struct delayed_work input_work;
	int current_state;
	u32 key_recheck_ms;
};

/**
 * struct palmas_pwron_config - configuration of palmas power on
 * @long_press_time_val:	value for long press h/w shutdown event
 * @pwron_debounce_val:		value for debounce of power button
 */
struct palmas_pwron_config {
	u8 long_press_time_val;
	u8 pwron_debounce_val;
};

/**
 * palmas_get_pwr_state() - read button state
 * @pwron: pointer to pwron struct
 *
 * Return: 0 for no press, PALMAS_PWR_KEY_PRESS for keypress
 * and on error, appropriate error value.
 */
static int palmas_get_pwr_state(struct palmas_pwron *pwron)
{
	struct input_dev *input_dev = pwron->input_dev;
	struct device *dev = input_dev->dev.parent;
	unsigned int reg = 0;
	int ret;

	ret = palmas_read(pwron->palmas, PALMAS_INTERRUPT_BASE,
			  PALMAS_INT1_LINE_STATE, &reg);
	if (ret) {
		dev_err(dev, "%s:Cannot read palmas PWRON status(%d)\n",
			__func__, ret);
		return ret;
	}

	/* PWRON line state is BIT(1) of the register */
	return reg & BIT(1) ? 0 : PALMAS_PWR_KEY_PRESS;
}

/**
 * palmas_power_button_work() - Detects the button release event
 * @work:	work item to detect button release
 */
static void palmas_power_button_work(struct work_struct *work)
{
	struct palmas_pwron *pwron = container_of((struct delayed_work *)work,
						  struct palmas_pwron,
						  input_work);
	struct input_dev *input_dev = pwron->input_dev;
	int next_state;

	next_state = palmas_get_pwr_state(pwron);
	if (next_state < 0)
		return;

	/*
	 * If the state did not change then schedule a work item to check the
	 * status of the power button line
	 */
	if (next_state == pwron->current_state) {
		schedule_delayed_work(&pwron->input_work,
				      msecs_to_jiffies(pwron->key_recheck_ms));
		return;
	}

	pwron->current_state = next_state;
	input_report_key(input_dev, KEY_POWER, pwron->current_state);
	input_sync(input_dev);
}

/**
 * pwron_irq() - button press isr
 * @irq:		irq
 * @palmas_pwron:	pwron struct
 *
 * Return: IRQ_HANDLED
 */
static irqreturn_t pwron_irq(int irq, void *palmas_pwron)
{
	struct palmas_pwron *pwron = palmas_pwron;
	struct input_dev *input_dev = pwron->input_dev;

	pwron->current_state = PALMAS_PWR_KEY_PRESS;

	input_report_key(input_dev, KEY_POWER, pwron->current_state);
	pm_wakeup_event(input_dev->dev.parent, 0);
	input_sync(input_dev);

	mod_delayed_work(system_wq, &pwron->input_work,
			 msecs_to_jiffies(pwron->key_recheck_ms));

	return IRQ_HANDLED;
}

/**
 * palmas_pwron_params_ofinit() - device tree parameter parser
 * @dev:	palmas button device
 * @config:	configuration params that this fills up
 */
static void palmas_pwron_params_ofinit(struct device *dev,
				       struct palmas_pwron_config *config)
{
	struct device_node *np;
	u32 val;
	int i;
	u8 lpk_times[] = { 6, 8, 10, 12 };
	int pwr_on_deb_ms[] = { 15, 100, 500, 1000 };

	/* Legacy boot? */
	if (!of_have_populated_dt())
		return;

	np = of_node_get(dev->of_node);
	/* Mixed boot? */
	if (!np)
		return;

	val = 0;
	of_property_read_u32(np, "ti,palmas-long-press-seconds", &val);
	config->long_press_time_val = ARRAY_SIZE(lpk_times) - 1;
	for (i = 0; i < ARRAY_SIZE(lpk_times); i++) {
		if (val <= lpk_times[i]) {
			config->long_press_time_val = i;
			break;
		}
	}

	val = 0;
	of_property_read_u32(np, "ti,palmas-pwron-debounce-milli-seconds",
			     &val);
	config->pwron_debounce_val = 0;
	for (i = 0; i < ARRAY_SIZE(pwr_on_deb_ms); i++) {
		if (val <= pwr_on_deb_ms[i]) {
			config->pwron_debounce_val = i;
			break;
		}
	}

	dev_info(dev, "h/w controlled shutdown duration=%d seconds\n",
		 lpk_times[config->long_press_time_val]);

	of_node_put(np);
}

/**
 * palmas_pwron_probe() - probe
 * @pdev:	platform device for the button
 *
 * Return: 0 for successful probe else appropriate error
 */
static int palmas_pwron_probe(struct platform_device *pdev)
{
	struct palmas *palmas = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct input_dev *input_dev;
	struct palmas_pwron *pwron;
	int irq, ret, val;
	struct palmas_pwron_config config = { 0 };

	palmas_pwron_params_ofinit(dev, &config);

	pwron = devm_kzalloc(dev, sizeof(*pwron), GFP_KERNEL);
	if (!pwron)
		return -ENOMEM;

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev) {
		dev_err(dev, "Can't allocate power button\n");
		return -ENOMEM;
	}

	/*
	 * Setup default hardware shutdown option (long key press)
	 * and debounce.
	 */
	val = config.long_press_time_val << __ffs(PALMAS_LPK_TIME_MASK);
	val |= config.pwron_debounce_val << __ffs(PALMAS_PWRON_DEBOUNCE_MASK);
	ret = palmas_update_bits(palmas, PALMAS_PMU_CONTROL_BASE,
				 PALMAS_LONG_PRESS_KEY,
				 PALMAS_LPK_TIME_MASK |
				 PALMAS_PWRON_DEBOUNCE_MASK, val);
	if (ret < 0) {
		dev_err(dev, "LONG_PRESS_KEY_UPDATE failed!\n");
		return ret;
	}

	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	input_dev->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	input_dev->name = "palmas_pwron";
	input_dev->phys = "palmas_pwron/input0";
	input_dev->dev.parent = dev;

	pwron->palmas = palmas;
	pwron->input_dev = input_dev;

	INIT_DELAYED_WORK(&pwron->input_work, palmas_power_button_work);

	irq = platform_get_irq(pdev, 0);
	ret = request_threaded_irq(irq, NULL, pwron_irq,
				   IRQF_TRIGGER_HIGH |
				   IRQF_TRIGGER_LOW, dev_name(dev), pwron);
	if (ret < 0) {
		dev_err(dev, "Can't get IRQ for pwron: %d\n", ret);
		return ret;
	}

	device_init_wakeup(dev, true);

	ret = input_register_device(input_dev);
	if (ret) {
		free_irq(irq, pwron);
		cancel_delayed_work_sync(&pwron->input_work);
		dev_dbg(dev, "Can't register power button: %d\n", ret);
		return ret;
	}
	pwron->irq = irq;

	pwron->key_recheck_ms = PALMAS_PWR_KEY_Q_TIME_MS;

	platform_set_drvdata(pdev, pwron);

	return 0;
}

/**
 * palmas_pwron_remove() - Cleanup on removal
 * @pdev:	platform device for the button
 *
 * Return: 0
 */
static int palmas_pwron_remove(struct platform_device *pdev)
{
	struct palmas_pwron *pwron = platform_get_drvdata(pdev);

	free_irq(pwron->irq, pwron);
	cancel_delayed_work_sync(&pwron->input_work);
	input_unregister_device(pwron->input_dev);

	return 0;
}

/**
 * palmas_pwron_suspend() - suspend handler
 * @dev:	power button device
 *
 * Cancel all pending work items for the power button, setup irq for wakeup
 *
 * Return: 0
 */
static int palmas_pwron_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct palmas_pwron *pwron = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&pwron->input_work);

	if (device_may_wakeup(dev))
		enable_irq_wake(pwron->irq);

	return 0;
}

/**
 * palmas_pwron_resume() - resume handler
 * @dev:	power button device
 *
 * Just disable the wakeup capability of irq here.
 *
 * Return: 0
 */
static int palmas_pwron_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct palmas_pwron *pwron = platform_get_drvdata(pdev);

	if (device_may_wakeup(dev))
		disable_irq_wake(pwron->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(palmas_pwron_pm,
			 palmas_pwron_suspend, palmas_pwron_resume);

#ifdef CONFIG_OF
static struct of_device_id of_palmas_pwr_match[] = {
	{.compatible = "ti,palmas-pwrbutton"},
	{},
};

MODULE_DEVICE_TABLE(of, of_palmas_pwr_match);
#endif

static struct platform_driver palmas_pwron_driver = {
	.probe = palmas_pwron_probe,
	.remove = palmas_pwron_remove,
	.driver = {
		   .name = "palmas_pwrbutton",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(of_palmas_pwr_match),
		   .pm = &palmas_pwron_pm,
	},
};
module_platform_driver(palmas_pwron_driver);

MODULE_ALIAS("platform:palmas-pwrbutton");
MODULE_DESCRIPTION("Palmas Power Button");
MODULE_LICENSE("GPL V2");
MODULE_AUTHOR("Texas Instruments Inc.");
