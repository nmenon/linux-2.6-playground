/*
 * OMAP L3 Interconnect  error handling driver header
 *
 * Copyright (C) 2011-2014 Texas Instruments Incorporated - http://www.ti.com/
 *	Santosh Shilimkar <santosh.shilimkar@ti.com>
 *	sricharan <r.sricharan@ti.com>
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
#ifndef __OMAP_L3_NOC_H
#define __OMAP_L3_NOC_H

#define MAX_L3_MODULES			3
#define MAX_CLKDM_TARGETS		30

#define CLEAR_STDERR_LOG		(1 << 31)
#define CUSTOM_ERROR			0x2
#define STANDARD_ERROR			0x0
#define INBAND_ERROR			0x0
#define L3_APPLICATION_ERROR		0x0
#define L3_DEBUG_ERROR			0x1

/* L3 TARG register offsets */
#define L3_TARG_STDERRLOG_MAIN		0x48
#define L3_TARG_STDERRLOG_SLVOFSLSB	0x5c
#define L3_TARG_STDERRLOG_MSTADDR	0x68
#define L3_FLAGMUX_REGERR0		0xc
#define L3_FLAGMUX_MASK0		0x8

#define L3_TARGET_NOT_SUPPORTED		NULL

#define L3_BASE_IS_SUBMODULE		((void __iomem *)(1 << 0))

/**
 * struct l3_masters_data - L3 Master information
 * @id:		ID of the L3 Master
 * @name:	master name
 */
struct l3_masters_data {
	u32 id;
	char *name;
};

/**
 * struct l3_target_data - L3 Target information
 * @offset:	Offset from base for L3 Target
 * @name:	Target name
 *
 * Target information is organized indexed by bit field definitions.
 */
struct l3_target_data {
	u32 offset;
	char *name;
};

/**
 * struct l3_flagmux_data - Flag Mux information
 * @offset:	offset from base for flagmux register
 * @l3_targ:	array indexed by flagmux index (bit offset) pointing to the
 *		target data. unsupported ones are marked with
 *		L3_TARGET_NOT_SUPPORTED
 * @num_targ_data: number of entries in target data
 */
struct l3_flagmux_data {
	u32 offset;
	struct l3_target_data *l3_targ;
	u8 num_targ_data;
};


/**
 * struct omap_l3 - Description of data relevant for L3 bus.
 * @dev:	device representing the bus (populated runtime)
 * @l3_base:	base addresses of modules (populated runtime if 0)
 *		if set to L3_BASE_IS_SUBMODULE, then uses previous
 *		module index as the base address
 * @l3_flag_mux: array containing flag mux data per module
 *		 offset from corresponding module base indexed per
 *		 module.
 * @num_modules: number of clock domains / modules.
 * @l3_masters:	array pointing to master data containing name and register
 *		offset for the master.
 * @num_master: number of masters
 * @debug_irq:	irq number of the debug interrupt (populated runtime)
 * @app_irq:	irq number of the application interrupt (populated runtime)
 */
struct omap_l3 {
	struct device *dev;

	void __iomem *l3_base[MAX_L3_MODULES];
	struct l3_flagmux_data **l3_flagmux;
	int num_modules;

	struct l3_masters_data *l3_masters;
	int num_masters;

	int debug_irq;
	int app_irq;
};

static struct l3_target_data omap_l3_target_data_clk1[] = {
	{0x100,	"DMM1",},
	{0x200,	"DMM2",},
	{0x300,	"ABE",},
	{0x400,	"L4CFG",},
	{0x600,	"CLK2PWRDISC",},
	{0x0,	"HOSTCLK1",},
	{0x900,	"L4WAKEUP",},
};

static struct l3_flagmux_data omap_l3_flagmux_clk1 = {
	.offset = 0x500,
	.l3_targ = omap_l3_target_data_clk1,
	.num_targ_data = ARRAY_SIZE(omap_l3_target_data_clk1),
};


static struct l3_target_data omap_l3_target_data_clk2[] = {
	{0x500,	"CORTEXM3",},
	{0x300,	"DSS",},
	{0x100,	"GPMC",},
	{0x400,	"ISS",},
	{0x700,	"IVAHD",},
	{0xD00,	"AES1",},
	{0x900,	"L4PER0",},
	{0x200,	"OCMRAM",},
	{0x100,	"GPMCsERROR",},
	{0x600,	"SGX",},
	{0x800,	"SL2",},
	{0x1600, "C2C",},
	{0x1100, "PWRDISCCLK1",},
	{0xF00,	"SHA1",},
	{0xE00,	"AES2",},
	{0xC00,	"L4PER3",},
	{0xA00,	"L4PER1",},
	{0xB00,	"L4PER2",},
	{0x0,	"HOSTCLK2",},
	{0x1800, "CAL",},
	{0x1700, "LLI",},
};

static struct l3_flagmux_data omap_l3_flagmux_clk2 = {
	.offset = 0x1000,
	.l3_targ = omap_l3_target_data_clk2,
	.num_targ_data = ARRAY_SIZE(omap_l3_target_data_clk2),
};


static struct l3_target_data omap_l3_target_data_clk3[] = {
	{0x0100, "EMUSS",},
	{0x0300, "DEBUG SOURCE",},
	{0x0,	"HOST CLK3",},
};

static struct l3_flagmux_data omap_l3_flagmux_clk3 = {
	.offset = 0x0200,
	.l3_targ = omap_l3_target_data_clk3,
	.num_targ_data = ARRAY_SIZE(omap_l3_target_data_clk3),
};

static struct l3_masters_data omap_l3_masters[] = {
	{ 0x0 , "MPU"},
	{ 0x10, "CS_ADP"},
	{ 0x14, "xxx"},
	{ 0x20, "DSP"},
	{ 0x30, "IVAHD"},
	{ 0x40, "ISS"},
	{ 0x44, "DucatiM3"},
	{ 0x48, "FaceDetect"},
	{ 0x50, "SDMA_Rd"},
	{ 0x54, "SDMA_Wr"},
	{ 0x58, "xxx"},
	{ 0x5C, "xxx"},
	{ 0x60, "SGX"},
	{ 0x70, "DSS"},
	{ 0x80, "C2C"},
	{ 0x88, "xxx"},
	{ 0x8C, "xxx"},
	{ 0x90, "HSI"},
	{ 0xA0, "MMC1"},
	{ 0xA4, "MMC2"},
	{ 0xA8, "MMC6"},
	{ 0xB0, "UNIPRO1"},
	{ 0xC0, "USBHOSTHS"},
	{ 0xC4, "USBOTGHS"},
	{ 0xC8, "USBHOSTFS"}
};

static struct l3_flagmux_data *omap_l3_flagmux[] = {
	&omap_l3_flagmux_clk1,
	&omap_l3_flagmux_clk2,
	&omap_l3_flagmux_clk3,
};

static const struct omap_l3 omap_l3_data = {
	.l3_flagmux = omap_l3_flagmux,
	.num_modules = ARRAY_SIZE(omap_l3_flagmux),
	.l3_masters = omap_l3_masters,
	.num_masters = ARRAY_SIZE(omap_l3_masters),
};

/* DRA7 data */
static struct l3_target_data dra_l3_target_data_clk1[] = {
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0x0200, "DMM_P1",},
	{0x0600, "DSP2_SDMA",},
	{0x0b00, "EVE2",},
	{0x1300, "DMM_P2",},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0x0300, "DSP1_SDMA",},
	{0x0a00, "EVE1",},
	{0x0c00, "EVE3",},
	{0x0d00, "EVE4",},
	{0x2900, "DSS",},
	{0x0100, "GPMC",},
	{0x3700, "PCIE1",},
	{0x1600, "IVA_CONFIG",},
	{0x1800, "IVA_SL2IF",},
	{0x0500, "L4_CFG",},
	{0x1d00, "L4_WKUP",},
	{0x3800, "PCIE2",},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0x1200, "GPU",},
	{0x1000, "IPU1",},
	{0x1100, "IPU2",},
	{0x2000, "TPCC_EDMA",},
	{0x2e00, "TPTC1_EDMA",},
	{0x2b00, "TPTC2_EDMA",},
	{0x0700, "VCP1",},
	{0x2500, "L4_PER2_P3",},
	{0x0e00, "L4_PER3_P3",},
	{0x2200, "MMU1",},
	{0x1400, "PRUSS1",},
};

static struct l3_flagmux_data dra_l3_flagmux_clk1 = {
	.offset = 0x803500,
	.l3_targ = dra_l3_target_data_clk1,
	.num_targ_data = ARRAY_SIZE(dra_l3_target_data_clk1),
};

static struct l3_target_data dra_l3_target_data_clk2[] = {
	{0x0,   "RT_CLK1_1",},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0x0900, "BB2D",},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0x2100, "L4_PER1_P3",},
	{0x1c00, "L4_PER1_P1",},
	{0x1f00, "L4_PER1_P2",},
	{0x2300, "L4_PER2_P1",},
	{0x2400, "L4_PER2_P2",},
	{0x2600, "L4_PER3_P1",},
	{0x2700, "L4_PER3_P2",},
	{0x2f00, "MCASP1",},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0x2800, "MMU2",},
	{0x0f00, "OCMC_RAM1",},
	{0x1700, "OCMC_RAM2",},
	{0x1900, "OCMC_RAM3",},
	{0xdead, L3_TARGET_NOT_SUPPORTED,},
	{0x3900, "QSPI",},
};

static struct l3_flagmux_data dra_l3_flagmux_clk2 = {
	.offset = 0x803600,
	.l3_targ = dra_l3_target_data_clk2,
	.num_targ_data = ARRAY_SIZE(dra_l3_target_data_clk2),
};

static struct l3_target_data dra_l3_target_data_clk3[] = {
	{0x0100, "L3_INSTR"},
	{0x0300, "DEBUGSS_CT_TBR"},
	{0x0,    "HOST CLK3"},
};

static struct l3_flagmux_data dra_l3_flagmux_clk3 = {
	.offset = 0x200,
	.l3_targ = dra_l3_target_data_clk3,
	.num_targ_data = ARRAY_SIZE(dra_l3_target_data_clk3),
};

static struct l3_masters_data dra_l3_masters[] = {
	{ 0x0, "MPU" },
	{ 0x4, "CS_DAP" },
	{ 0x8, "DSP1_MDMA" },
	{ 0x9, "DSP1_CFG" },
	{ 0xA, "DSP1_DMA" },
	{ 0xB, "DSP2_MDMA" },
	{ 0xC, "DSP2_CFG" },
	{ 0xD, "DSP2_DMA" },
	{ 0xE, "IVA" },
	{ 0x10, "EVE1_P1" },
	{ 0x11, "EVE2_P1" },
	{ 0x12, "EVE3_P1" },
	{ 0x13, "EVE4_P1" },
	{ 0x14, "PRU1" },
	{ 0x15, "PRU2" },
	{ 0x18, "IPU1" },
	{ 0x1C, "TC1_EDMA" },
	{ 0x1D, "TC2_EDMA" },
	{ 0x20, "DSS" },
	{ 0x21, "MMU1" },
	{ 0x22, "PCIE1" },
	{ 0x23, "MMU2" },
	{ 0x24, "VIP1" },
	{ 0x25, "VIP2" },
	{ 0x26, "VIP3" },
	{ 0x27, "VPE" },
	{ 0x28, "GPU_P1" },
	{ 0x29, "GPU_P2" },
	{ 0x2B, "GMAC_SW" },
	{ 0x34, "EVE1_P2" },
	{ 0x35, "EVE2_P2" },
	{ 0x36, "EVE3_P2" },
	{ 0x37, "EVE4_P2" }
};

static struct l3_flagmux_data *dra_l3_flagmux[] = {
	&dra_l3_flagmux_clk1,
	&dra_l3_flagmux_clk2,
	&dra_l3_flagmux_clk3,
};

static const struct omap_l3 dra_l3_data = {
	.l3_base = { [1] = L3_BASE_IS_SUBMODULE },
	.l3_flagmux = dra_l3_flagmux,
	.num_modules = ARRAY_SIZE(dra_l3_flagmux),
	.l3_masters = dra_l3_masters,
	.num_masters = ARRAY_SIZE(dra_l3_masters),
};

#endif	/* __OMAP_L3_NOC_H */
