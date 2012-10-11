/*
 * OMAP SoC Coprocessor devfreq driver
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/suspend.h>
#include <linux/pm_opp.h>
#include <linux/devfreq.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/clk.h>

/**
 * struct omap_devfreq_data - Co processor private data
 * @profile:	devfreq profile specific for this device
 * @dev:	device pointer
 * @devfreq:	devfreq for this device
 * @stat:	my current statistics
 *
 * TBD: write something nice here.
 */
struct omap_devfreq_data {
	struct devfreq_dev_profile profile;
	struct device *dev;
	struct devfreq *devfreq;
	struct devfreq_dev_status stat;
};

static struct clk *dev_clk;
static char *dev_clk_name = "dpll2_ck";
#define DEV_NAME "iva"

static int omap_device_getrate(struct device *dev, unsigned long *rate)
{
	if (!dev_clk)
		dev_clk = clk_get(dev, dev_clk_name);
	*rate = clk_get_rate(dev_clk);
	return 0;
}

static int omap_device_scale(struct device *dev, unsigned long rate)
{
	if (!dev_clk)
		dev_clk = clk_get(dev, dev_clk_name);
	return clk_set_rate(dev_clk, rate);
}

/**
 * omap_coproc_target() - devfreq handle for setting a rate
 * @dev:	device
 * @req_freq:	requested frequency
 * @flags:	what directional flag
 *
 */
static int omap_coproc_target(struct device *dev, unsigned long *req_freq,
			      u32 flags)
{
	struct omap_devfreq_data *d;
	unsigned long new_freq, old_freq;
	struct dev_pm_opp *opp;
	int r = 0;

	if (IS_ERR_OR_NULL(dev) || IS_ERR_OR_NULL(req_freq)) {
		WARN("Bad pointer(s): dev=%p freq=%p\n",
		     (void *)dev, (void *)req_freq);
		return -EINVAL;
	}
	d = dev_get_drvdata(dev);

	rcu_read_lock();
	opp = devfreq_recommended_opp(dev, req_freq, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		dev_err(dev, "%s: %pF Unable to find OPP for freq%ld\n",
			__func__, (void *)_RET_IP_, *req_freq);
		r = PTR_ERR(opp);
		goto out;
	}
	new_freq = dev_pm_opp_get_freq(opp);
	rcu_read_unlock();

	/*
	 * We could potentially optimize this a bit by storing current_freq
	 * BUT, we dont know about external influences - e.g. thermal throttle
	 */
	r = omap_device_getrate(d->dev, &old_freq);
	if (r) {
		dev_err(d->dev, "%s: unable to get rate(%d)\n", __func__, r);
		goto out;
	}

	if (old_freq != new_freq) {
		r = omap_device_scale(d->dev, new_freq);
		dev_err(d->dev, "%s: Did a setrate(%d) n=%ld o=%ld r=%ld\n",
			__func__, r, new_freq, old_freq, *req_freq);
	}
	if (r) {
		dev_err(d->dev, "%s: fail set rate(%d) n=%ld o=%ld r=%ld\n",
			__func__, r, new_freq, old_freq, *req_freq);
		/* Fall through to exit */
	}
	dev_dbg(dev, "%s:target -  %ld %d\n", __func__, new_freq, r);
out:
	return r;
}

/**
 * omap_coproc_get_dev_status() - devfreq handle to know about the device
 * @dev:	device we are interested in
 * @stat:	return back statistics for the device
 */
static int omap_coproc_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	struct omap_devfreq_data *d;
	int r;

	if (IS_ERR_OR_NULL(dev) || IS_ERR_OR_NULL(stat)) {
		WARN("Bad pointer(s): dev=%p stat=%p\n",
		     (void *)dev, (void *)stat);
		return -EINVAL;
	}

	d = dev_get_drvdata(dev);
	/* Completely BOGUS values */
	stat->total_time = d->stat.total_time;
	stat->busy_time = d->stat.busy_time;
	r = omap_device_getrate(d->dev, &stat->current_frequency);
	if (r)
		dev_err(dev, "%s: bad get rate req = %d\n", __func__, r);
	dev_dbg(dev, "%s:polled -  %ld\n", __func__, stat->current_frequency);

	return r;
}

/**
 * omap_coproc_exit() - done using the device
 * @dev:	device
 */
static void omap_coproc_exit(struct device *dev)
{
	struct omap_devfreq_data *d = dev_get_drvdata(dev);

	devfreq_unregister_opp_notifier(dev, d->devfreq);
}

static ssize_t store_stat(const char *buf, size_t count, unsigned long *store)
{
	int ret;
	unsigned long value;
	ret = sscanf(buf, "%lu", &value);
	if (ret == 0) {
		ret = -EINVAL;
		goto out;
	}
	*store = value;
	ret = count;
out:
	return ret;
}

static ssize_t show_stat_total_time(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct omap_devfreq_data *d = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", d->stat.total_time);
}

static ssize_t store_stat_total_time(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct omap_devfreq_data *d = dev_get_drvdata(dev);
	return store_stat(buf, count, &d->stat.total_time);
}

static ssize_t show_stat_busy_time(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct omap_devfreq_data *d = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", d->stat.busy_time);
}

static ssize_t store_stat_busy_time(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct omap_devfreq_data *d = dev_get_drvdata(dev);
	return store_stat(buf, count, &d->stat.busy_time);
}

static struct device_attribute omap_debug_attr[] = {
	__ATTR(stat_total_time, S_IRUSR | S_IWUSR, show_stat_total_time,
	       store_stat_total_time),
	__ATTR(stat_busy_time, S_IRUSR | S_IWUSR, show_stat_busy_time,
	       store_stat_busy_time),
	{},
};

/**
 * omap_coproc_probe() - probe when we find a device to control
 * @pdev:	which platform device to control?
 */
static int omap_coproc_probe(struct platform_device *pdev)
{
	struct omap_devfreq_data *d;
	struct device *dev = &pdev->dev;
	int err = 0;
	struct device_attribute *attr = omap_debug_attr;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (d == NULL) {
		dev_err(dev, "%s: Cannot allocate memory.\n", __func__);
		err = -ENOMEM;
		goto out;
	}

	d->dev = dev;
	err = omap_device_getrate(dev, &d->profile.initial_freq);
	if (err) {
		dev_err(dev, "%s: Cannot get freq(%d).\n", __func__, err);
		goto out_mem;
	}

	d->profile.target = omap_coproc_target;
	d->profile.get_dev_status = omap_coproc_get_dev_status;
	d->profile.exit = omap_coproc_exit;
	/* NOTE: the following should be based per device */
	d->profile.polling_ms = 50;
	dev_info(dev, "%s init rate = %ld\n", __func__,
		 d->profile.initial_freq);

	/* NOTE: option needs to be provided for governor to be selected */
	d->devfreq = devfreq_add_device(dev, &d->profile, "simple_ondemand",
					NULL);
	if (IS_ERR(d->devfreq)) {
		err = PTR_ERR(d->devfreq);
		dev_err(dev, "%s: Cannot add to devfreq(%d).\n", __func__, err);
		goto out_mem;
	}

	err = devfreq_register_opp_notifier(dev, d->devfreq);
	if (err) {
		dev_err(dev, "%s: Cannot add to devfreq opp notifier(%d).\n",
			__func__, err);
		goto out_remove;
	}

	platform_set_drvdata(pdev, d);

	/* Completely BOGUS values */
	d->stat.total_time = 100;
	d->stat.busy_time = 1;

	while (attr->show || attr->store) {
		int k = device_create_file(dev, attr);
		dev_err(dev, "attribute res = %d\n", k);
		attr++;
	}

	/* All good.. */
	goto out;

out_remove:
	devfreq_remove_device(d->devfreq);
out_mem:
	kfree(d);
out:
	dev_info(dev, "%s result=%d", __func__, err);
	return err;
}

/**
 * omap_coproc_remove() - remove the device
 * @pdev:	device we are removing from system
 */
static int omap_coproc_remove(struct platform_device *pdev)
{
	struct omap_devfreq_data *d = platform_get_drvdata(pdev);

	/*
	 * unregister_pm_notifier(&d->pm_notifier);
	 */
	devfreq_remove_device(d->devfreq);

	kfree(d);

	dev_err(&pdev->dev, "%s\n", __func__);
	return 0;
}

/**
 * omap_coproc_suspend() - dummy hook for suspend
 * @dev: write something
 */
static int omap_coproc_suspend(struct device *dev)
{
	dev_err(dev, "suspend");
	return 0;
}

/**
 * omap_coproc_resume() - dummy hook for resume
 * @dev: write something
 */
static int omap_coproc_resume(struct device *dev)
{
	dev_err(dev, "resume");
	return 0;
}

/* Device power management hooks */
static const struct dev_pm_ops omap_coproc_pm = {
	.suspend = omap_coproc_suspend,
	.resume = omap_coproc_resume,
};

/* Any generic devices that exist on ALL OMAPs - currently example devices */
static const struct platform_device_id omap_coproc_generic_device_id_list[] = {
	{DEV_NAME, 0},
	{},
};

static struct platform_driver omap_coproc_driver = {
	.probe = omap_coproc_probe,
	.remove = omap_coproc_remove,
	.id_table = omap_coproc_generic_device_id_list,
	.driver = {
		   .name = "omap_coproc_pm",
		   .owner = THIS_MODULE,
		   .pm = &omap_coproc_pm,
		   },
};
module_platform_driver(omap_coproc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OMAP Co-processor DEVFREQ Driver");
MODULE_AUTHOR("Nishanth Menon <nm@ti.com>");
