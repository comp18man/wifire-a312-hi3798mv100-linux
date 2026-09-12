// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hi3798MV100 Clock and Reset Generator Driver
 *
 * Copyright (c) 2016 HiSilicon Technologies Co., Ltd.
 */

#include <dt-bindings/clock/histb-clock.h>
#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "clk.h"
#include "crg.h"
#include "reset.h"

/* hi3798MV100 core CRG */
#define HI3798MV100_FIXED_24M			65
#define HI3798MV100_FIXED_25M			66
#define HI3798MV100_FIXED_50M			67
#define HI3798MV100_FIXED_75M			68
#define HI3798MV100_FIXED_100M			69
#define HI3798MV100_FIXED_150M			70
#define HI3798MV100_FIXED_200M			71
#define HI3798MV100_FIXED_250M			72
#define HI3798MV100_FIXED_300M			109
#define HI3798MV100_FIXED_345P6M		110
#define HI3798MV100_FIXED_375M			111
#define HI3798MV100_FIXED_400M			112
#define HI3798MV100_FIXED_432M			113
#define HI3798MV100_MMC_MUX			75
#define HI3798MV100_ETH_PUB_CLK			76
#define HI3798MV100_ETH_BUS_CLK			77
#define HI3798MV100_FIXED_12M			81
#define HI3798MV100_FIXED_48M			82
#define HI3798MV100_FIXED_60M			83
#define HI3798MV100_FIXED_166P5M		84
#define HI3798MV100_SDIO0_MUX			85
#define HI3798MV100_FIXED_3M			87
#define HI3798MV100_FIXED_15M			88
#define HI3798MV100_FIXED_83P3M			89
#define HI3798MV100_GPU_PP0_CLK			90
#define HI3798MV100_GPU_PP1_CLK			91
#define HI3798MV100_GPU_MUX			114

#define HI3798MV100_PERI_CRG18			0x48
#define HI3798MV100_CPU_FREQ_SEL_MASK		GENMASK(2, 0)
#define HI3798MV100_CPU_BEGIN_CFG_BYPASS	BIT(9)
#define HI3798MV100_CPU_SW_BEGIN		BIT(10)

#define HI3798MV100_PERI_CRG_GPU_LOWPOWER	0x124
#define HI3798MV100_PERI_CRG_GPU_STATUS		0x154
#define HI3798MV100_GPU_FREQ_SEL_MASK		GENMASK(2, 0)
#define HI3798MV100_GPU_BEGIN_CFG_BYPASS	BIT(9)
#define HI3798MV100_GPU_SW_BEGIN		BIT(10)
#define HI3798MV100_GPU_CLK_MUX_MASK		GENMASK(7, 5)

#define HI3798MV100_SYSCTRL_SC_GEN17		0xc4
#define HI3798MV100_SYSCTRL_SC_SYSID		0xee0
#define HI3798MV100_PERICTRL_SOC_FUSE_0		0x840

#define HI3798MV100_SC_SYSID			0x37980100
#define HI3798MV100_CORNER_MASK			GENMASK(31, 24)
#define HI3798MV100_CORNER_FF			0x1
#define HI3798MV100_CORNER_SS			0x3
#define HI3798MV100_CHIP_ID_MASK		GENMASK(20, 16)
#define HI3798MV100_CHIP_ID_QFP_216		0x7

/* MAX_FREQ from the SDK's clock_mpu.c: this SoC is never clocked above it. */
#define HI3798MV100_CPU_MAX_RATE		1200000000UL

#define HI3798MV100_CRG_NR_CLKS			128

struct hi3798mv100_cpu_rate {
	u8 sel;
	unsigned long rate;
};

struct hi3798mv100_cpu_clk {
	struct clk_hw hw;
	void __iomem *reg;
	bool cap_1200_to_1000;
};

struct hi3798mv100_gpu_mux_clk {
	struct clk_hw hw;
	void __iomem *cfg_reg;
	void __iomem *status_reg;
};

static const struct hi3798mv100_cpu_rate hi3798mv100_cpu_rates[] = {
	{ .sel = 6, .rate = 400000000, },
	{ .sel = 7, .rate = 600000000, },
	{ .sel = 1, .rate = 800000000, },
	{ .sel = 0, .rate = 1000000000, },
	{ .sel = 5, .rate = 1200000000, },
	{ .sel = 3, .rate = 1500000000, },
};

static DEFINE_SPINLOCK(hi3798mv100_cpu_clk_lock);
static DEFINE_SPINLOCK(hi3798mv100_gpu_clk_lock);

static struct hi3798mv100_cpu_clk *to_hi3798mv100_cpu_clk(struct clk_hw *hw)
{
	return container_of(hw, struct hi3798mv100_cpu_clk, hw);
}

static struct hi3798mv100_gpu_mux_clk *to_hi3798mv100_gpu_mux_clk(struct clk_hw *hw)
{
	return container_of(hw, struct hi3798mv100_gpu_mux_clk, hw);
}

static unsigned long hi3798mv100_cpu_sanitize_rate(struct hi3798mv100_cpu_clk *cpu_clk,
						    unsigned long rate)
{
	/*
	 * The mux offers 1.5GHz, but no vendor setup picks it: it corrupts
	 * memory on a binned-down part.
	 */
	if (rate > HI3798MV100_CPU_MAX_RATE)
		rate = HI3798MV100_CPU_MAX_RATE;

	if (cpu_clk->cap_1200_to_1000 && rate == 1200000000)
		return 1000000000;

	return rate;
}

static bool hi3798mv100_cpu_needs_1200_cap(struct platform_device *pdev)
{
	struct regmap *sysctrl;
	struct regmap *perictrl;
	unsigned int sysid;
	unsigned int gen17;
	unsigned int fuse0;
	unsigned int corner_type;
	unsigned int chip_id;
	int ret;

	sysctrl = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						  "hisilicon,sysctrl");
	if (IS_ERR(sysctrl))
		return false;

	perictrl = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						   "hisilicon,perictrl");
	if (IS_ERR(perictrl))
		return false;

	ret = regmap_read(sysctrl, HI3798MV100_SYSCTRL_SC_SYSID, &sysid);
	if (ret)
		return false;

	if (sysid != HI3798MV100_SC_SYSID)
		return false;

	ret = regmap_read(sysctrl, HI3798MV100_SYSCTRL_SC_GEN17, &gen17);
	if (ret)
		return false;

	corner_type = FIELD_GET(HI3798MV100_CORNER_MASK, gen17);
	if (corner_type == HI3798MV100_CORNER_FF ||
	    corner_type == HI3798MV100_CORNER_SS)
		return true;

	ret = regmap_read(perictrl, HI3798MV100_PERICTRL_SOC_FUSE_0, &fuse0);
	if (ret)
		return false;

	chip_id = FIELD_GET(HI3798MV100_CHIP_ID_MASK, fuse0);
	return chip_id == HI3798MV100_CHIP_ID_QFP_216;
}

static const struct hi3798mv100_cpu_rate *hi3798mv100_cpu_rate_from_sel(u8 sel)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hi3798mv100_cpu_rates); i++) {
		if (hi3798mv100_cpu_rates[i].sel == sel)
			return &hi3798mv100_cpu_rates[i];
	}

	return NULL;
}

static const struct hi3798mv100_cpu_rate *hi3798mv100_cpu_rate_from_rate(unsigned long rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hi3798mv100_cpu_rates); i++) {
		if (hi3798mv100_cpu_rates[i].rate == rate)
			return &hi3798mv100_cpu_rates[i];
	}

	return NULL;
}

static unsigned long hi3798mv100_cpu_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct hi3798mv100_cpu_clk *cpu_clk = to_hi3798mv100_cpu_clk(hw);
	const struct hi3798mv100_cpu_rate *cpu_rate;
	u32 val;

	val = readl_relaxed(cpu_clk->reg);
	cpu_rate = hi3798mv100_cpu_rate_from_sel(val & HI3798MV100_CPU_FREQ_SEL_MASK);
	if (!cpu_rate)
		return 1000000000;

	return cpu_rate->rate;
}

static long hi3798mv100_cpu_round_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long *parent_rate)
{
	struct hi3798mv100_cpu_clk *cpu_clk = to_hi3798mv100_cpu_clk(hw);
	int i;

	for (i = 0; i < ARRAY_SIZE(hi3798mv100_cpu_rates); i++) {
		if (rate <= hi3798mv100_cpu_rates[i].rate)
			return hi3798mv100_cpu_sanitize_rate(cpu_clk,
							     hi3798mv100_cpu_rates[i].rate);
	}

	return hi3798mv100_cpu_sanitize_rate(cpu_clk,
					     hi3798mv100_cpu_rates[ARRAY_SIZE(hi3798mv100_cpu_rates) - 1].rate);
}

static int hi3798mv100_cpu_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct hi3798mv100_cpu_clk *cpu_clk = to_hi3798mv100_cpu_clk(hw);
	const struct hi3798mv100_cpu_rate *cpu_rate;
	unsigned long flags;
	u32 val;

	rate = hi3798mv100_cpu_sanitize_rate(cpu_clk, rate);

	cpu_rate = hi3798mv100_cpu_rate_from_rate(rate);
	if (!cpu_rate)
		return -EINVAL;

	spin_lock_irqsave(&hi3798mv100_cpu_clk_lock, flags);

	val = readl_relaxed(cpu_clk->reg);
	val &= ~HI3798MV100_CPU_SW_BEGIN;
	writel_relaxed(val, cpu_clk->reg);

	val &= ~HI3798MV100_CPU_FREQ_SEL_MASK;
	val |= cpu_rate->sel;
	writel_relaxed(val, cpu_clk->reg);

	val |= HI3798MV100_CPU_SW_BEGIN;
	writel_relaxed(val, cpu_clk->reg);

	spin_unlock_irqrestore(&hi3798mv100_cpu_clk_lock, flags);

	return 0;
}

static int hi3798mv100_cpu_init(struct clk_hw *hw)
{
	struct hi3798mv100_cpu_clk *cpu_clk = to_hi3798mv100_cpu_clk(hw);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&hi3798mv100_cpu_clk_lock, flags);

	val = readl_relaxed(cpu_clk->reg);
	val |= HI3798MV100_CPU_BEGIN_CFG_BYPASS;
	writel_relaxed(val, cpu_clk->reg);

	spin_unlock_irqrestore(&hi3798mv100_cpu_clk_lock, flags);

	return 0;
}

static const struct clk_ops hi3798mv100_cpu_clk_ops = {
	.recalc_rate = hi3798mv100_cpu_recalc_rate,
	.round_rate = hi3798mv100_cpu_round_rate,
	.set_rate = hi3798mv100_cpu_set_rate,
	.init = hi3798mv100_cpu_init,
};

static int hi3798mv100_register_cpu_clk(struct platform_device *pdev,
					struct hisi_clock_data *clk_data)
{
	struct hi3798mv100_cpu_clk *cpu_clk;
	struct clk_init_data init = { };
	int ret;

	cpu_clk = devm_kzalloc(&pdev->dev, sizeof(*cpu_clk), GFP_KERNEL);
	if (!cpu_clk)
		return -ENOMEM;

	init.name = "clk_cpu";
	init.ops = &hi3798mv100_cpu_clk_ops;
	init.flags = CLK_GET_RATE_NOCACHE;
	init.num_parents = 0;
	cpu_clk->hw.init = &init;
	cpu_clk->reg = clk_data->base + HI3798MV100_PERI_CRG18;
	cpu_clk->cap_1200_to_1000 = hi3798mv100_cpu_needs_1200_cap(pdev);
	if (cpu_clk->cap_1200_to_1000)
		dev_info(&pdev->dev,
			 "1.2GHz CPU clock is capped to 1.0GHz for this chip bin\n");

	ret = devm_clk_hw_register(&pdev->dev, &cpu_clk->hw);
	if (ret)
		return ret;

	clk_data->clk_data.clks[HISTB_CPU_CLK] = cpu_clk->hw.clk;

	return 0;
}

static const struct hisi_fixed_rate_clock hi3798mv100_fixed_rate_clks[] = {
	{ HISTB_OSC_CLK, "clk_osc", NULL, 0, 24000000, },
	{ HISTB_APB_CLK, "clk_apb", NULL, 0, 100000000, },
	{ HISTB_AHB_CLK, "clk_ahb", NULL, 0, 200000000, },
	{ HI3798MV100_FIXED_3M, "3m", NULL, 0, 3000000, },
	{ HI3798MV100_FIXED_12M, "12m", NULL, 0, 12000000, },
	{ HI3798MV100_FIXED_15M, "15m", NULL, 0, 15000000, },
	{ HI3798MV100_FIXED_24M, "24m", NULL, 0, 24000000, },
	{ HI3798MV100_FIXED_25M, "25m", NULL, 0, 25000000, },
	{ HI3798MV100_FIXED_48M, "48m", NULL, 0, 48000000, },
	{ HI3798MV100_FIXED_50M, "50m", NULL, 0, 50000000, },
	{ HI3798MV100_FIXED_60M, "60m", NULL, 0, 60000000, },
	{ HI3798MV100_FIXED_75M, "75m", NULL, 0, 75000000, },
	{ HI3798MV100_FIXED_83P3M, "83p3m", NULL, 0, 83333333, },
	{ HI3798MV100_FIXED_100M, "100m", NULL, 0, 100000000, },
	{ HI3798MV100_FIXED_150M, "150m", NULL, 0, 150000000, },
	{ HI3798MV100_FIXED_166P5M, "166p5m", NULL, 0, 165000000, },
	{ HI3798MV100_FIXED_200M, "200m", NULL, 0, 200000000, },
	{ HI3798MV100_FIXED_250M, "250m", NULL, 0, 250000000, },
	{ HI3798MV100_FIXED_300M, "300m", NULL, 0, 300000000, },
	{ HI3798MV100_FIXED_345P6M, "345p6m", NULL, 0, 345600000, },
	{ HI3798MV100_FIXED_375M, "375m", NULL, 0, 375000000, },
	{ HI3798MV100_FIXED_400M, "400m", NULL, 0, 400000000, },
	{ HI3798MV100_FIXED_432M, "432m", NULL, 0, 432000000, },
};

static const char *const hi3798mv100_mmc_mux_p[] = {
	"75m", "100m", "50m", "15m"
};
static u32 hi3798mv100_mmc_mux_table[] = { 0, 1, 2, 3 };

static const char *const hi3798mv100_gpu_mux_p[] = {
	"432m", "400m", "375m", "345p6m", "300m", "250m", "200m", "150m"
};
static u32 hi3798mv100_gpu_mux_table[] = { 0, 1, 2, 3, 4, 5, 6, 7 };

static int hi3798mv100_gpu_mux_init(struct clk_hw *hw)
{
	struct hi3798mv100_gpu_mux_clk *gpu_mux = to_hi3798mv100_gpu_mux_clk(hw);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&hi3798mv100_gpu_clk_lock, flags);

	/* Software-controlled mode: transitions go through SW_BEGIN. */
	val = readl_relaxed(gpu_mux->cfg_reg);
	val |= HI3798MV100_GPU_BEGIN_CFG_BYPASS;
	writel_relaxed(val, gpu_mux->cfg_reg);

	spin_unlock_irqrestore(&hi3798mv100_gpu_clk_lock, flags);

	return 0;
}

static unsigned long hi3798mv100_gpu_mux_recalc_rate(struct clk_hw *hw,
						      unsigned long parent_rate)
{
	return parent_rate;
}

static int hi3798mv100_gpu_mux_determine_rate(struct clk_hw *hw,
					      struct clk_rate_request *req)
{
	return __clk_mux_determine_rate(hw, req);
}

static u8 hi3798mv100_gpu_mux_get_parent(struct clk_hw *hw)
{
	struct hi3798mv100_gpu_mux_clk *gpu_mux = to_hi3798mv100_gpu_mux_clk(hw);
	u32 val;
	int index;

	val = readl_relaxed(gpu_mux->status_reg);
	val = FIELD_GET(HI3798MV100_GPU_CLK_MUX_MASK, val);

	index = clk_mux_val_to_index(hw, hi3798mv100_gpu_mux_table, 0, val);
	if (index < 0)
		return 0;

	return index;
}

static int hi3798mv100_gpu_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct hi3798mv100_gpu_mux_clk *gpu_mux = to_hi3798mv100_gpu_mux_clk(hw);
	u32 val = clk_mux_index_to_val(hi3798mv100_gpu_mux_table, 0, index);
	unsigned long flags;
	u32 cfg;
	u32 status;
	int ret;

	if (index >= ARRAY_SIZE(hi3798mv100_gpu_mux_table))
		return -EINVAL;

	spin_lock_irqsave(&hi3798mv100_gpu_clk_lock, flags);

	cfg = readl_relaxed(gpu_mux->cfg_reg);
	cfg &= ~HI3798MV100_GPU_SW_BEGIN;
	cfg &= ~HI3798MV100_GPU_FREQ_SEL_MASK;
	cfg |= HI3798MV100_GPU_BEGIN_CFG_BYPASS;
	cfg |= FIELD_PREP(HI3798MV100_GPU_FREQ_SEL_MASK, val);
	writel_relaxed(cfg, gpu_mux->cfg_reg);

	cfg |= HI3798MV100_GPU_SW_BEGIN;
	writel_relaxed(cfg, gpu_mux->cfg_reg);

	spin_unlock_irqrestore(&hi3798mv100_gpu_clk_lock, flags);

	ret = readl_poll_timeout_atomic(gpu_mux->status_reg, status,
					FIELD_GET(HI3798MV100_GPU_CLK_MUX_MASK,
						  status) == val,
					1, 100);
	if (ret)
		return ret;

	return 0;
}

static const struct clk_ops hi3798mv100_gpu_mux_clk_ops = {
	.init = hi3798mv100_gpu_mux_init,
	.recalc_rate = hi3798mv100_gpu_mux_recalc_rate,
	.determine_rate = hi3798mv100_gpu_mux_determine_rate,
	.get_parent = hi3798mv100_gpu_mux_get_parent,
	.set_parent = hi3798mv100_gpu_mux_set_parent,
};

static struct hisi_mux_clock hi3798mv100_mux_clks[] = {
	{ HI3798MV100_MMC_MUX, "mmc_mux",
		hi3798mv100_mmc_mux_p, ARRAY_SIZE(hi3798mv100_mmc_mux_p),
		CLK_SET_RATE_PARENT, 0xa0, 8, 2, 0, hi3798mv100_mmc_mux_table, },
	{ HI3798MV100_SDIO0_MUX, "sdio0_mux",
		hi3798mv100_mmc_mux_p, ARRAY_SIZE(hi3798mv100_mmc_mux_p),
		CLK_SET_RATE_PARENT, 0x9c, 8, 2, 0, hi3798mv100_mmc_mux_table, },
};

static int hi3798mv100_register_gpu_mux_clk(struct platform_device *pdev,
					    struct hisi_clock_data *clk_data)
{
	struct hi3798mv100_gpu_mux_clk *gpu_mux;
	struct clk_init_data init = { };
	int ret;

	gpu_mux = devm_kzalloc(&pdev->dev, sizeof(*gpu_mux), GFP_KERNEL);
	if (!gpu_mux)
		return -ENOMEM;

	init.name = "gpu_mux";
	init.ops = &hi3798mv100_gpu_mux_clk_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = hi3798mv100_gpu_mux_p;
	init.num_parents = ARRAY_SIZE(hi3798mv100_gpu_mux_p);

	gpu_mux->hw.init = &init;
	gpu_mux->cfg_reg = clk_data->base + HI3798MV100_PERI_CRG_GPU_LOWPOWER;
	gpu_mux->status_reg = clk_data->base + HI3798MV100_PERI_CRG_GPU_STATUS;

	ret = devm_clk_hw_register(&pdev->dev, &gpu_mux->hw);
	if (ret)
		return ret;

	clk_data->clk_data.clks[HI3798MV100_GPU_MUX] = gpu_mux->hw.clk;

	return 0;
}

static u32 hi3798mv100_mmc_phase_regvals[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
static u32 hi3798mv100_mmc_phase_degrees[] = {
	0, 45, 90, 135, 180, 225, 270, 315
};

static struct hisi_phase_clock hi3798mv100_phase_clks[] = {
	{ HISTB_MMC_SAMPLE_CLK, "mmc_sample", "clk_mmc_ciu",
		CLK_SET_RATE_PARENT, 0xa0, 12, 3,
		hi3798mv100_mmc_phase_degrees, hi3798mv100_mmc_phase_regvals,
		ARRAY_SIZE(hi3798mv100_mmc_phase_regvals), },
	{ HISTB_MMC_DRV_CLK, "mmc_drive", "clk_mmc_ciu",
		CLK_SET_RATE_PARENT, 0xa0, 16, 3,
		hi3798mv100_mmc_phase_degrees, hi3798mv100_mmc_phase_regvals,
		ARRAY_SIZE(hi3798mv100_mmc_phase_regvals), },
};

static const struct hisi_gate_clock hi3798mv100_gate_clks[] = {
	/* UART */
	{ HISTB_UART1_CLK, "clk_uart1", "3m",
		CLK_SET_RATE_PARENT | CLK_IS_CRITICAL, 0x68, 0, 0, },
	{ HISTB_UART2_CLK, "clk_uart2", "83p3m",
		CLK_SET_RATE_PARENT, 0x68, 4, 0, },
	/* I2C */
	{ HISTB_I2C0_CLK, "clk_i2c0", "clk_apb",
		CLK_SET_RATE_PARENT, 0x6c, 4, 0, },
	{ HISTB_I2C1_CLK, "clk_i2c1", "clk_apb",
		CLK_SET_RATE_PARENT, 0x6c, 8, 0, },
	{ HISTB_I2C2_CLK, "clk_i2c2", "clk_apb",
		CLK_SET_RATE_PARENT, 0x6c, 12, 0, },
	/* SPI */
	{ HISTB_SPI0_CLK, "clk_spi0", "clk_apb",
		CLK_SET_RATE_PARENT, 0x70, 0, 0, },
	/* SDIO */
	{ HISTB_SDIO0_BIU_CLK, "clk_sdio0_biu", "200m",
		CLK_SET_RATE_PARENT, 0x9c, 0, 0, },
	{ HISTB_SDIO0_CIU_CLK, "clk_sdio0_ciu", "sdio0_mux",
		CLK_SET_RATE_PARENT, 0x9c, 1, 0, },
	/* EMMC */
	{ HISTB_MMC_BIU_CLK, "clk_mmc_biu", "200m",
		CLK_SET_RATE_PARENT, 0xa0, 0, 0, },
	{ HISTB_MMC_CIU_CLK, "clk_mmc_ciu", "mmc_mux",
		CLK_SET_RATE_PARENT, 0xa0, 1, 0, },
	/* Ethernet */
	{ HI3798MV100_ETH_BUS_CLK, "clk_bus", NULL,
		CLK_SET_RATE_PARENT, 0xcc, 0, 0, },
	{ HI3798MV100_ETH_PUB_CLK, "clk_pub", "clk_bus",
		CLK_SET_RATE_PARENT, 0xcc, 1, 0, },
	{ HISTB_ETH0_MAC_CLK, "clk_mac0", "clk_pub",
		CLK_SET_RATE_PARENT, 0xcc, 3, 0, },
	/* USB2 */
	{ HISTB_USB2_BUS_CLK, "clk_u2_bus", "clk_ahb",
		CLK_SET_RATE_PARENT, 0xb8, 0, 0, },
	{ HISTB_USB2_PHY_CLK, "clk_u2_phy", "60m",
		CLK_SET_RATE_PARENT, 0xb8, 4, 0, },
	{ HISTB_USB2_12M_CLK, "clk_u2_12m", "12m",
		CLK_SET_RATE_PARENT, 0xb8, 2, 0, },
	{ HISTB_USB2_48M_CLK, "clk_u2_48m", "48m",
		CLK_SET_RATE_PARENT, 0xb8, 1, 0, },
	{ HISTB_USB2_UTMI_CLK, "clk_u2_utmi", "60m",
		CLK_SET_RATE_PARENT, 0xb8, 5, 0, },
	{ HISTB_USB2_UTMI_CLK1, "clk_u2_utmi1", "60m",
		CLK_SET_RATE_PARENT, 0xb8, 6, 0, },
	{ HISTB_USB2_OTG_UTMI_CLK, "clk_u2_otg_utmi", "60m",
		CLK_SET_RATE_PARENT, 0xb8, 3, 0, },
	{ HISTB_USB2_PHY1_REF_CLK, "clk_u2_phy1_ref", "24m",
		CLK_SET_RATE_PARENT, 0xbc, 0, 0, },
	{ HISTB_USB2_PHY2_REF_CLK, "clk_u2_phy2_ref", "24m",
		CLK_SET_RATE_PARENT, 0xbc, 2, 0, },
	/* USB2 #2 */
	{ HISTB_USB2_2_BUS_CLK, "clk_u2_2_bus", "clk_ahb",
		CLK_SET_RATE_PARENT, 0x198, 0, 0, },
	{ HISTB_USB2_2_PHY_CLK, "clk_u2_2_phy", "60m",
		CLK_SET_RATE_PARENT, 0x198, 4, 0, },
	{ HISTB_USB2_2_12M_CLK, "clk_u2_2_12m", "12m",
		CLK_SET_RATE_PARENT, 0x198, 2, 0, },
	{ HISTB_USB2_2_48M_CLK, "clk_u2_2_48m", "48m",
		CLK_SET_RATE_PARENT, 0x198, 1, 0, },
	{ HISTB_USB2_2_UTMI_CLK, "clk_u2_2_utmi", "60m",
		CLK_SET_RATE_PARENT, 0x198, 5, 0, },
	{ HISTB_USB2_2_UTMI_CLK1, "clk_u2_2_utmi1", "60m",
		CLK_SET_RATE_PARENT, 0x198, 6, 0, },
	{ HISTB_USB2_2_OTG_UTMI_CLK, "clk_u2_2_otg_utmi", "60m",
		CLK_SET_RATE_PARENT, 0x198, 3, 0, },
	{ HISTB_USB2_2_PHY1_REF_CLK, "clk_u2_2_phy1_ref", "24m",
		CLK_SET_RATE_PARENT, 0x190, 0, 0, },
	{ HISTB_USB2_2_PHY2_REF_CLK, "clk_u2_2_phy2_ref", "24m",
		CLK_SET_RATE_PARENT, 0x190, 2, 0, },
	/* USB3 */
	{ HISTB_USB3_BUS_CLK, "clk_u3_bus", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 0, 0, },
	{ HISTB_USB3_UTMI_CLK, "clk_u3_utmi", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 4, 0, },
	{ HISTB_USB3_PIPE_CLK, "clk_u3_pipe", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 3, 0, },
	{ HISTB_USB3_SUSPEND_CLK, "clk_u3_suspend", NULL,
		CLK_SET_RATE_PARENT, 0xb0, 2, 0, },
	/* Display */
	{ HISTB_VO_BUS_CLK, "clk_vo_bus", NULL,
		CLK_SET_RATE_PARENT, 0xd8, 0, 0, },
	{ HISTB_VO_CLK, "clk_vo", NULL,
		CLK_SET_RATE_PARENT, 0xd8, 1, 0, },
	{ HISTB_VO_SD_CLK, "clk_vo_sd", NULL,
		CLK_SET_RATE_PARENT, 0xd8, 2, 0, },
	{ HISTB_VO_SDATE_CLK, "clk_vo_sdate", NULL,
		CLK_SET_RATE_PARENT, 0xd8, 3, 0, },
	{ HISTB_VO_HD_CLK, "clk_vo_hd", NULL,
		CLK_SET_RATE_PARENT, 0xd8, 4, 0, },
	{ HISTB_VO_HDATE_CLK, "clk_vo_hdate", NULL,
		CLK_SET_RATE_PARENT, 0xd8, 5, 0, },
	/* HDMI */
	{ HISTB_HDMI_TX_BUS_CLK, "clk_hdmitx_bus", NULL,
		CLK_SET_RATE_PARENT, 0x10c, 0, 0, },
	{ HISTB_HDMI_TX_CEC_CLK, "clk_hdmitx_cec", NULL,
		CLK_SET_RATE_PARENT, 0x10c, 1, 0, },
	{ HISTB_HDMI_TX_ID_CLK, "clk_hdmitx_id", NULL,
		CLK_SET_RATE_PARENT, 0x10c, 2, 0, },
	{ HISTB_HDMI_TX_MHL_CLK, "clk_hdmitx_mhl", NULL,
		CLK_SET_RATE_PARENT, 0x10c, 3, 0, },
	{ HISTB_HDMI_TX_OS_CLK, "clk_hdmitx_os", NULL,
		CLK_SET_RATE_PARENT, 0x10c, 4, 0, },
	{ HISTB_HDMI_TX_AS_CLK, "clk_hdmitx_as", NULL,
		CLK_SET_RATE_PARENT, 0x10c, 5, 0, },
	{ HISTB_HDMI_PHY_BUS_CLK, "clk_hdmitx_phy_bus", NULL,
		CLK_SET_RATE_PARENT, 0x110, 0, 0, },
	/* Audio */
	{ HISTB_AIAO_CLK, "clk_aiao", "100m",
		CLK_SET_RATE_PARENT, 0x118, 0, 0, },
	/* GPU */
	{ HISTB_GPU_BUS_CLK, "clk_gpu", "200m",
		CLK_SET_RATE_PARENT, 0xd4, 0, 0, },
	{ HISTB_GPU_GP_CLK, "clk_gpu_gp", "clk_gpu_pp0",
		CLK_SET_RATE_PARENT, 0xd4, 8, 0, },
	{ HI3798MV100_GPU_PP0_CLK, "clk_gpu_pp0", "clk_gpu_pp1",
		CLK_SET_RATE_PARENT, 0xd4, 9, 0, },
	{ HI3798MV100_GPU_PP1_CLK, "clk_gpu_pp1", "gpu_mux",
		CLK_SET_RATE_PARENT, 0xd4, 10, 0, },
	/* FEPHY */
	{ HISTB_FEPHY_CLK, "clk_fephy", "25m",
		CLK_SET_RATE_PARENT, 0x120, 0, 0, },
};

static struct hisi_clock_data *hi3798mv100_clk_register(
	struct platform_device *pdev)
{
	struct hisi_clock_data *clk_data;
	int ret;

	clk_data = hisi_clk_alloc(pdev, HI3798MV100_CRG_NR_CLKS);
	if (!clk_data)
		return ERR_PTR(-ENOMEM);

	/* hisi_phase_clock is resource managed */
	ret = hisi_clk_register_phase(&pdev->dev, hi3798mv100_phase_clks,
				      ARRAY_SIZE(hi3798mv100_phase_clks),
				      clk_data);
	if (ret)
		return ERR_PTR(ret);

	ret = hisi_clk_register_fixed_rate(hi3798mv100_fixed_rate_clks,
					   ARRAY_SIZE(hi3798mv100_fixed_rate_clks),
					   clk_data);
	if (ret)
		return ERR_PTR(ret);

	ret = hisi_clk_register_mux(hi3798mv100_mux_clks,
				    ARRAY_SIZE(hi3798mv100_mux_clks),
				    clk_data);
	if (ret)
		goto unregister_fixed_rate;

	ret = hi3798mv100_register_gpu_mux_clk(pdev, clk_data);
	if (ret)
		goto unregister_mux;

	ret = hisi_clk_register_gate(hi3798mv100_gate_clks,
				     ARRAY_SIZE(hi3798mv100_gate_clks),
				     clk_data);
	if (ret)
		goto unregister_mux;

	ret = hi3798mv100_register_cpu_clk(pdev, clk_data);
	if (ret)
		goto unregister_gate;

	ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				  &clk_data->clk_data);
	if (ret)
		goto unregister_gate;

	return clk_data;

unregister_gate:
	hisi_clk_unregister_gate(hi3798mv100_gate_clks,
				 ARRAY_SIZE(hi3798mv100_gate_clks),
				 clk_data);
unregister_mux:
	hisi_clk_unregister_mux(hi3798mv100_mux_clks,
				ARRAY_SIZE(hi3798mv100_mux_clks),
				clk_data);
unregister_fixed_rate:
	hisi_clk_unregister_fixed_rate(hi3798mv100_fixed_rate_clks,
				       ARRAY_SIZE(hi3798mv100_fixed_rate_clks),
				       clk_data);
	return ERR_PTR(ret);
}

static void hi3798mv100_clk_unregister(struct platform_device *pdev)
{
	struct hisi_crg_dev *crg = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);

	hisi_clk_unregister_gate(hi3798mv100_gate_clks,
				 ARRAY_SIZE(hi3798mv100_gate_clks),
				 crg->clk_data);
	hisi_clk_unregister_mux(hi3798mv100_mux_clks,
				ARRAY_SIZE(hi3798mv100_mux_clks),
				crg->clk_data);
	hisi_clk_unregister_fixed_rate(hi3798mv100_fixed_rate_clks,
				       ARRAY_SIZE(hi3798mv100_fixed_rate_clks),
				       crg->clk_data);
}

static const struct hisi_crg_funcs hi3798mv100_crg_funcs = {
	.register_clks = hi3798mv100_clk_register,
	.unregister_clks = hi3798mv100_clk_unregister,
};

/* hi3798MV100 sysctrl CRG */
#define HI3798MV100_SYSCTRL_NR_CLKS		16

static const struct hisi_gate_clock hi3798mv100_sysctrl_gate_clks[] = {
	{ HISTB_IR_CLK, "clk_ir", "24m",
		CLK_SET_RATE_PARENT, 0x48, 4, 0, },
	{ HISTB_TIMER01_CLK, "clk_timer01", "24m",
		CLK_SET_RATE_PARENT, 0x48, 6, 0, },
	{ HISTB_UART0_CLK, "clk_uart0", "83p3m",
		CLK_SET_RATE_PARENT, 0x48, 12, 0, },
};

static struct hisi_clock_data *hi3798mv100_sysctrl_clk_register(
	struct platform_device *pdev)
{
	struct hisi_clock_data *clk_data;
	int ret;

	clk_data = hisi_clk_alloc(pdev, HI3798MV100_SYSCTRL_NR_CLKS);
	if (!clk_data)
		return ERR_PTR(-ENOMEM);

	ret = hisi_clk_register_gate(hi3798mv100_sysctrl_gate_clks,
				     ARRAY_SIZE(hi3798mv100_sysctrl_gate_clks),
				     clk_data);
	if (ret)
		return ERR_PTR(ret);

	ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				  &clk_data->clk_data);
	if (ret)
		goto unregister_gate;

	return clk_data;

unregister_gate:
	hisi_clk_unregister_gate(hi3798mv100_sysctrl_gate_clks,
				 ARRAY_SIZE(hi3798mv100_sysctrl_gate_clks),
				 clk_data);
	return ERR_PTR(ret);
}

static void hi3798mv100_sysctrl_clk_unregister(struct platform_device *pdev)
{
	struct hisi_crg_dev *crg = platform_get_drvdata(pdev);

	of_clk_del_provider(pdev->dev.of_node);

	hisi_clk_unregister_gate(hi3798mv100_sysctrl_gate_clks,
				 ARRAY_SIZE(hi3798mv100_sysctrl_gate_clks),
				 crg->clk_data);
}

static const struct hisi_crg_funcs hi3798mv100_sysctrl_funcs = {
	.register_clks = hi3798mv100_sysctrl_clk_register,
	.unregister_clks = hi3798mv100_sysctrl_clk_unregister,
};

static const struct of_device_id hi3798mv100_crg_match_table[] = {
	{ .compatible = "hisilicon,hi3798mv100-crg",
		.data = &hi3798mv100_crg_funcs, },
	{ .compatible = "hisilicon,hi3798mv100-sysctrl",
		.data = &hi3798mv100_sysctrl_funcs, },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798mv100_crg_match_table);

static int hi3798mv100_crg_probe(struct platform_device *pdev)
{
	struct hisi_crg_dev *crg;

	crg = devm_kmalloc(&pdev->dev, sizeof(*crg), GFP_KERNEL);
	if (!crg)
		return -ENOMEM;

	crg->funcs = of_device_get_match_data(&pdev->dev);
	if (!crg->funcs)
		return -ENOENT;

	crg->rstc = hisi_reset_init(pdev);
	if (!crg->rstc)
		return -ENOMEM;

	crg->clk_data = crg->funcs->register_clks(pdev);
	if (IS_ERR(crg->clk_data)) {
		hisi_reset_exit(crg->rstc);
		return PTR_ERR(crg->clk_data);
	}

	platform_set_drvdata(pdev, crg);
	return 0;
}

static void hi3798mv100_crg_remove(struct platform_device *pdev)
{
	struct hisi_crg_dev *crg = platform_get_drvdata(pdev);

	hisi_reset_exit(crg->rstc);
	crg->funcs->unregister_clks(pdev);
}

static struct platform_driver hi3798mv100_crg_driver = {
	.probe	= hi3798mv100_crg_probe,
	.remove	= hi3798mv100_crg_remove,
	.driver = {
		.name = "hi3798mv100-crg",
		.of_match_table = hi3798mv100_crg_match_table,
	},
};

static int __init hi3798mv100_crg_init(void)
{
	return platform_driver_register(&hi3798mv100_crg_driver);
}
core_initcall(hi3798mv100_crg_init);

static void __exit hi3798mv100_crg_exit(void)
{
	platform_driver_unregister(&hi3798mv100_crg_driver);
}
module_exit(hi3798mv100_crg_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("HiSilicon Hi3798MV100 CRG Driver");
