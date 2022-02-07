/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2013 NVIDIA Corporation
 * Copyright (C) 2018 Cadence Design Systems Inc.
 */

#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/time64.h>

#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>

#define PSEC_PER_SEC	1000000000000LL

/*
 * Minimum D-PHY timings based on MIPI D-PHY specification. Derived
 * from the valid ranges specified in Section 6.9, Table 14, Page 41
 * of the D-PHY specification (v2.1).
 */
int phy_mipi_dphy_get_default_config(unsigned long pixel_clock,
				     unsigned int bpp,
				     unsigned int lanes,
				     struct phy_configure_opts_mipi_dphy *cfg)
{
	unsigned long long hs_clk_rate;
	unsigned long long ui;

	if (!cfg)
		return -EINVAL;

	hs_clk_rate = pixel_clock * bpp;
	do_div(hs_clk_rate, lanes);

	ui = ALIGN(PSEC_PER_SEC, hs_clk_rate);
	do_div(ui, hs_clk_rate);

	cfg->clk_miss = 0;
	cfg->clk_post = 60000 + 52 * ui;
	cfg->clk_pre = 8000;
	cfg->clk_prepare = 38000;
	cfg->clk_settle = 95000;
	cfg->clk_term_en = 0;
	cfg->clk_trail = 60000;
	cfg->clk_zero = 262000;
	cfg->d_term_en = 0;
	cfg->eot = 0;
	cfg->hs_exit = 100000;
	cfg->hs_prepare = 40000 + 4 * ui;
	cfg->hs_zero = 105000 + 6 * ui;
	cfg->hs_settle = 85000 + 6 * ui;
	cfg->hs_skip = 40000;

	/*
	 * The MIPI D-PHY specification (Section 6.9, v1.2, Table 14, Page 40)
	 * contains this formula as:
	 *
	 *     T_HS-TRAIL = max(n * 8 * ui, 60 + n * 4 * ui)
	 *
	 * where n = 1 for forward-direction HS mode and n = 4 for reverse-
	 * direction HS mode. There's only one setting and this function does
	 * not parameterize on anything other that ui, so this code will
	 * assumes that reverse-direction HS mode is supported and uses n = 4.
	 */
	cfg->hs_trail = max(4 * 8 * ui, 60000 + 4 * 4 * ui);

	cfg->init = 100;
	cfg->lpx = 60000;
	cfg->ta_get = 5 * cfg->lpx;
	cfg->ta_go = 4 * cfg->lpx;
	cfg->ta_sure = 2 * cfg->lpx;
	cfg->wakeup = 1000;

	cfg->hs_clk_rate = hs_clk_rate;
	cfg->lanes = lanes;

	return 0;
}
EXPORT_SYMBOL(phy_mipi_dphy_get_default_config);

/*
 * Validate D-PHY configuration according to MIPI D-PHY specification
 * (v1.2, Section Section 6.9 "Global Operation Timing Parameters").
 */
int phy_mipi_dphy_config_validate(struct phy_configure_opts_mipi_dphy *cfg)
{
	unsigned long long ui;

	pr_err("%s: entry\n", __func__ );
	if (!cfg)
		return -EINVAL;

	ui = ALIGN(PSEC_PER_SEC, cfg->hs_clk_rate);
	do_div(ui, cfg->hs_clk_rate);

	if (cfg->clk_miss > 60000) {
		pr_err("%s: clk_miss %d \n", __func__, cfg->clk_miss);
		return -EINVAL;
	}

	if (cfg->clk_post < (60000 + 52 * ui)) {
		pr_err("%s: clk_post %d\n", __func__, cfg->clk_post);
		return -EINVAL;
	}

	if (cfg->clk_pre < 8000) {
		pr_err("%s: clk_pre %d\n", __func__, cfg->clk_pre);
		return -EINVAL;
	}

	if (cfg->clk_prepare < 38000 || cfg->clk_prepare > 95000) {
		pr_err("%s: clk_prepare %d\n", __func__, cfg->clk_prepare);
		return -EINVAL;
	}

	if (cfg->clk_settle < 95000 || cfg->clk_settle > 300000) {
		pr_err("%s: clk_settle %d\n", __func__, cfg->clk_settle);
		return -EINVAL;
	}

	if (cfg->clk_term_en > 38000) {
		pr_err("%s: clk_term_en %d\n",__func__, cfg->clk_term_en);
		return -EINVAL;
	}

	if (cfg->clk_trail < 60000) {
		pr_err("%s: clk_trail %d\n", __func__, cfg->clk_trail);
		return -EINVAL;
	}

	if ((cfg->clk_prepare + cfg->clk_zero) < 300000) {
		pr_err("%s: clk_zero %d\n", __func__, cfg->clk_zero);
		return -EINVAL;
	}

	if (cfg->d_term_en > (35000 + 4 * ui)) {
		pr_err("%s: d_term_en %d\n", __func__, cfg->d_term_en);
		return -EINVAL;
	}

	if (cfg->eot > (105000 + 12 * ui)) {
		pr_err("%s: eot %d\n", __func__, cfg->eot);
		return -EINVAL;
	}

	if (cfg->hs_exit < 100000) {
		pr_err("%s: hs_exit %d\n", __func__, cfg->hs_exit);
		return -EINVAL;
	}

	if (cfg->hs_prepare < (40000 + 4 * ui) ||
	    cfg->hs_prepare > (85000 + 6 * ui)) {
		pr_err("%s: hs_prepare %d\n", __func__, cfg->hs_prepare);
		return -EINVAL;
	}

	if ((cfg->hs_prepare + cfg->hs_zero) < (145000 + 10 * ui)) {
		pr_err("%s: hs_zero %d\n", __func__, cfg->hs_zero);
		return -EINVAL;
	}

	if ((cfg->hs_settle < (85000 + 6 * ui)) ||
	    (cfg->hs_settle > (145000 + 10 * ui))) {
		pr_err("%s/%d: hs_settle %d\n", __func__, __LINE__, cfg->hs_settle);
		return -EINVAL;
	}

	if (cfg->hs_skip < 40000 || cfg->hs_skip > (55000 + 4 * ui)) {
		pr_err("%s/%d: hs_skip %d\n", __func__, __LINE__, cfg->hs_skip);
		return -EINVAL;
	}

	if (cfg->hs_trail < max(8 * ui, 60000 + 4 * ui)) {
		pr_err("%s/%d: hs_trail %d\n", __func__, __LINE__, cfg->hs_trail);
		return -EINVAL;
	}

	if (cfg->init < 100) {
		pr_err("%s/%d: init %d\n", __func__, __LINE__, cfg->init);
		return -EINVAL;
	}

	if (cfg->lpx < 50000) {
		pr_err("%s/%d: lpx %d\n", __func__, __LINE__, cfg->lpx);
		return -EINVAL;
	}

	if (cfg->ta_get != (5 * cfg->lpx)) {
		pr_err("%s/%d: ta_get %d\n", __func__, __LINE__, cfg->ta_get);
		return -EINVAL;
	}

	if (cfg->ta_go != (4 * cfg->lpx)) {
		pr_err("%s/%d: ta_go %d\n", __func__, __LINE__, cfg->ta_go);
		return -EINVAL;
	}

	if (cfg->ta_sure < cfg->lpx || cfg->ta_sure > (2 * cfg->lpx)) {
		pr_err("%s/%d: ta_sure %d\n", __func__, __LINE__, cfg->ta_sure);
		return -EINVAL;
	}

	/*if (cfg->wakeup < 1000) {
		pr_err("%s/%d: wakeup %d\n", __func__, __LINE__, cfg->wakeup);
		return -EINVAL;
	}*/

	pr_err("%s/%d: all good %d\n", __func__, __LINE__, 0);
	return 0;
}
EXPORT_SYMBOL(phy_mipi_dphy_config_validate);
