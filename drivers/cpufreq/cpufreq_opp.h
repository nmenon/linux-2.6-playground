/*
 * Generic OPP Interface for CPUFREQ drivers
 *
 * Copyright (C) 2009-2014 Texas Instruments Incorporated.
 *	Nishanth Menon
 *	Romit Dasgupta
 *	Kevin Hilman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __CPUFREQ_OPP_H__
#define __CPUFREQ_OPP_H__

#include <linux/cpufreq.h>
#include <linux/pm_opp.h>

#if defined(CONFIG_CPU_FREQ) && defined(CONFIG_PM_OPP)
int dev_pm_opp_init_cpufreq_table(struct device *dev,
				  struct cpufreq_frequency_table **table);
void dev_pm_opp_free_cpufreq_table(struct device *dev,
				   struct cpufreq_frequency_table **table);
#else
static inline int dev_pm_opp_init_cpufreq_table(struct device *dev,
						struct cpufreq_frequency_table
						**table)
{
	return -EINVAL;
}

static inline void dev_pm_opp_free_cpufreq_table(struct device *dev,
						 struct cpufreq_frequency_table
						 **table)
{
}
#endif				/* CONFIG_CPU_FREQ */

#endif				/* __CPUFREQ_OPP_H__ */
