/*
 * OMAP Power Management debug routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 * Richard Woodruff <r-woodruff2@ti.com>
 * Tony Lindgren
 * Juha Yrjola
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Igor Stoppa <igor.stoppa@nokia.com>
 * Jouni Hogander
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "clock.h"
#include "powerdomain.h"
#include "clockdomain.h"
#include "omap-pm.h"

#include "soc.h"
#include "cm2xxx_3xxx.h"
#include "prm2xxx_3xxx.h"
#include "pm.h"

u32 enable_off_mode;

static int _check_hwmod(struct omap_hwmod *oh, void *data)
{
	struct clockdomain *clkdm = data;
	bool functional = false;
	bool in_transition = false;
	bool if_idle = false;
	bool disabled = false;
	char *state = "unknown";
	bool print = true;

	/* Skip if we dont have clkdm match */
	if (clkdm != oh->clkdm)
		return 0;

	if (!clkdm_current_status(clkdm, oh, &functional, &in_transition, &if_idle, &disabled)) {
		if (disabled) {
			print = false;
			state = "disabled";
		} else {
			if (functional)
				state = "functional";
			else if (in_transition)
				state = "stuck in transition";
			else if (if_idle)
				state = "i/f clk idled - if iclk = fclk for this module, consider as disabled";
		}
	}

	if (print)
		pr_err("\t\t%s: %s\n", oh->name, state);

	return 0;
}

static int _check_clkdm_state(struct powerdomain *pwrdm, struct clockdomain *clkdm)
{
	char sflags[255] = {0};
	char hflags[255] = {0};
	bool disable_auto = false;
	bool force_sleep = false;
	bool force_wakeup = false;
	bool enable_auto = false;


	if (clkdm->flags & CLKDM_CAN_HWSUP_SWSUP)
		strncat(sflags, "HWSUP_SWSUP ", sizeof(sflags));

	else if (clkdm->flags & CLKDM_CAN_HWSUP)
		strncat(sflags, "HWSUP ", sizeof(sflags));
	else if (clkdm->flags & CLKDM_CAN_SWSUP)
		strncat(sflags, "SWSUP ", sizeof(sflags));
	if (clkdm->flags & CLKDM_NO_AUTODEPS)
		strncat(sflags, "NO_AUTODEPS ", sizeof(sflags));
	if (clkdm->flags & CLKDM_ACTIVE_WITH_MPU)
		strncat(sflags, "ACTIVE_WITH_MPU ", sizeof(sflags));
	if (clkdm->flags & CLKDM_MISSING_IDLE_REPORTING)
		strncat(sflags, "MISSING_IDLE_REPORTING ", sizeof(sflags));
	if (clkdm->_flags & _CLKDM_FLAG_HWSUP_ENABLED)
		strncat(sflags, "HWSUP_ENABLED(int) ", sizeof(sflags));

	if (!clkdm_control_status(clkdm, &disable_auto, &force_sleep, &force_wakeup, &enable_auto) ){
		if (disable_auto)
			strncat(hflags, "disable_auto ", sizeof(hflags));
		if (enable_auto)
			strncat(hflags, "enable_auto ", sizeof(hflags));
		if (force_sleep)
			strncat(hflags, "force_sleep ", sizeof(hflags));
		if (force_wakeup)
			strncat(hflags, "force_wakeup ", sizeof(hflags));
	}

	pr_info("\t %s: s/w flags(%s) h/w control(%s)\n",
		clkdm->name, sflags, hflags);
	/* Need to check per hwmod for match */
	omap_hwmod_for_each(_check_hwmod, clkdm);


	return 0;
}

void pm_debug_check_pwrdm(struct powerdomain *pwrdm)
{
	pwrdm_for_each_clkdm(pwrdm,_check_clkdm_state);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int pm_dbg_init_done;

static int pm_dbg_init(void);

enum {
	DEBUG_FILE_COUNTERS = 0,
	DEBUG_FILE_TIMERS,
};

static const char pwrdm_state_names[][PWRDM_MAX_PWRSTS] = {
	"OFF",
	"RET",
	"INA",
	"ON"
};

void pm_dbg_update_time(struct powerdomain *pwrdm, int prev)
{
	s64 t;

	if (!pm_dbg_init_done)
		return ;

	/* Update timer for previous state */
	t = sched_clock();

	pwrdm->state_timer[prev] += t - pwrdm->timer;

	pwrdm->timer = t;
}

static int clkdm_dbg_show_counter(struct clockdomain *clkdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;

	if (strcmp(clkdm->name, "emu_clkdm") == 0 ||
		strcmp(clkdm->name, "wkup_clkdm") == 0 ||
		strncmp(clkdm->name, "dpll", 4) == 0)
		return 0;

	seq_printf(s, "%s->%s (%d)\n", clkdm->name, clkdm->pwrdm.ptr->name,
		   clkdm->usecount);

	return 0;
}

static int pwrdm_dbg_show_counter(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	if (pwrdm->state != pwrdm_read_pwrst(pwrdm))
		printk(KERN_ERR "pwrdm state mismatch(%s) %d != %d\n",
			pwrdm->name, pwrdm->state, pwrdm_read_pwrst(pwrdm));

	seq_printf(s, "%s (%s)", pwrdm->name,
			pwrdm_state_names[pwrdm->state]);
	for (i = 0; i < PWRDM_MAX_PWRSTS; i++)
		seq_printf(s, ",%s:%d", pwrdm_state_names[i],
			pwrdm->state_counter[i]);

	seq_printf(s, ",RET-LOGIC-OFF:%d", pwrdm->ret_logic_off_counter);
	for (i = 0; i < pwrdm->banks; i++)
		seq_printf(s, ",RET-MEMBANK%d-OFF:%d", i + 1,
				pwrdm->ret_mem_off_counter[i]);

	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_timer(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	pwrdm_state_switch(pwrdm);

	seq_printf(s, "%s (%s)", pwrdm->name,
		pwrdm_state_names[pwrdm->state]);

	for (i = 0; i < 4; i++)
		seq_printf(s, ",%s:%lld", pwrdm_state_names[i],
			pwrdm->state_timer[i]);

	seq_printf(s, "\n");
	return 0;
}

static int pm_dbg_show_counters(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_counter, s);
	clkdm_for_each(clkdm_dbg_show_counter, s);

	return 0;
}

static int pm_dbg_show_timers(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_timer, s);
	return 0;
}

static int pm_dbg_open(struct inode *inode, struct file *file)
{
	switch ((int)inode->i_private) {
	case DEBUG_FILE_COUNTERS:
		return single_open(file, pm_dbg_show_counters,
			&inode->i_private);
	case DEBUG_FILE_TIMERS:
	default:
		return single_open(file, pm_dbg_show_timers,
			&inode->i_private);
	}
}

static const struct file_operations debug_fops = {
	.open           = pm_dbg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int pwrdm_suspend_get(void *data, u64 *val)
{
	int ret = -EINVAL;

	if (cpu_is_omap34xx())
		ret = omap3_pm_get_suspend_state((struct powerdomain *)data);
	*val = ret;

	if (ret >= 0)
		return 0;
	return *val;
}

static int pwrdm_suspend_set(void *data, u64 val)
{
	if (cpu_is_omap34xx())
		return omap3_pm_set_suspend_state(
			(struct powerdomain *)data, (int)val);
	return -EINVAL;
}

DEFINE_SIMPLE_ATTRIBUTE(pwrdm_suspend_fops, pwrdm_suspend_get,
			pwrdm_suspend_set, "%llu\n");

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *dir)
{
	int i;
	s64 t;
	struct dentry *d;

	t = sched_clock();

	for (i = 0; i < 4; i++)
		pwrdm->state_timer[i] = 0;

	pwrdm->timer = t;

	if (strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	d = debugfs_create_dir(pwrdm->name, (struct dentry *)dir);
	if (d)
		(void) debugfs_create_file("suspend", S_IRUGO|S_IWUSR, d,
			(void *)pwrdm, &pwrdm_suspend_fops);

	return 0;
}

static int option_get(void *data, u64 *val)
{
	u32 *option = data;

	*val = *option;

	return 0;
}

static int option_set(void *data, u64 val)
{
	u32 *option = data;

	*option = val;

	if (option == &enable_off_mode) {
		if (val)
			omap_pm_enable_off_mode();
		else
			omap_pm_disable_off_mode();
		if (cpu_is_omap34xx())
			omap3_pm_off_mode_enable(val);
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_dbg_option_fops, option_get, option_set, "%llu\n");

static int __init pm_dbg_init(void)
{
	struct dentry *d;

	if (pm_dbg_init_done)
		return 0;

	d = debugfs_create_dir("pm_debug", NULL);
	if (!d)
		return -EINVAL;

	(void) debugfs_create_file("count", S_IRUGO,
		d, (void *)DEBUG_FILE_COUNTERS, &debug_fops);
	(void) debugfs_create_file("time", S_IRUGO,
		d, (void *)DEBUG_FILE_TIMERS, &debug_fops);

	pwrdm_for_each(pwrdms_setup, (void *)d);

	(void) debugfs_create_file("enable_off_mode", S_IRUGO | S_IWUSR, d,
				   &enable_off_mode, &pm_dbg_option_fops);
	pm_dbg_init_done = 1;

	return 0;
}
omap_arch_initcall(pm_dbg_init);

#endif
