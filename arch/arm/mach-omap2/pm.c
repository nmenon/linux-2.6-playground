/*
 * pm.c - Common OMAP2+ power management-related code
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Copyright (C) 2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/pm_opp.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/cpu.h>

#include <asm/system_misc.h>

#include "omap-pm.h"
#include "omap_device.h"
#include "common.h"

#include "soc.h"
#include "prcm-common.h"
#include "voltage.h"
#include "powerdomain.h"
#include "clockdomain.h"
#include "pm.h"

/*
 * omap_pm_suspend: points to a function that does the SoC-specific
 * suspend work
 */
int (*omap_pm_suspend)(void);

#ifdef CONFIG_PM
/**
 * struct omap2_oscillator - Describe the board main oscillator latencies
 * @startup_time: oscillator startup latency
 * @shutdown_time: oscillator shutdown latency
 */
struct omap2_oscillator {
	u32 startup_time;
	u32 shutdown_time;
};

static struct omap2_oscillator oscillator = {
	.startup_time = ULONG_MAX,
	.shutdown_time = ULONG_MAX,
};

void omap_pm_setup_oscillator(u32 tstart, u32 tshut)
{
	oscillator.startup_time = tstart;
	oscillator.shutdown_time = tshut;
}

void omap_pm_get_oscillator(u32 *tstart, u32 *tshut)
{
	if (!tstart || !tshut)
		return;

	*tstart = oscillator.startup_time;
	*tshut = oscillator.shutdown_time;
}
#endif

int __init omap_pm_clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	/* XXX The usecount test is racy */
	if ((clkdm->flags & CLKDM_CAN_ENABLE_AUTO) &&
	    !(clkdm->flags & CLKDM_MISSING_IDLE_REPORTING))
		clkdm_allow_idle(clkdm);
	else if (clkdm->flags & CLKDM_CAN_FORCE_SLEEP &&
		 clkdm->usecount == 0)
		clkdm_sleep(clkdm);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int omap_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	if (!omap_pm_suspend)
		return -ENOENT; /* XXX doublecheck */

	switch (suspend_state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int omap_pm_begin(suspend_state_t state)
{
	cpu_idle_poll_ctrl(true);
	if (cpu_is_omap34xx())
		omap_prcm_irq_prepare();
	return 0;
}

static void omap_pm_end(void)
{
	cpu_idle_poll_ctrl(false);
}

static void omap_pm_finish(void)
{
	if (cpu_is_omap34xx())
		omap_prcm_irq_complete();
}

static const struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap_pm_begin,
	.end		= omap_pm_end,
	.enter		= omap_pm_enter,
	.finish		= omap_pm_finish,
	.valid		= suspend_valid_only_mem,
};

#endif /* CONFIG_SUSPEND */

static inline void omap_init_cpufreq(void)
{
	struct platform_device_info devinfo = {
		.name = "cpufreq-cpu0",
	};

	platform_device_register_full(&devinfo);
}

static int __init omap2_common_pm_init(void)
{
	omap_pm_if_init();

	return 0;
}
omap_postcore_initcall(omap2_common_pm_init);

int __init omap2_common_pm_late_init(void)
{
	/* cpufreq dummy device instantiation */
	omap_init_cpufreq();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&omap_pm_ops);
#endif

	return 0;
}
