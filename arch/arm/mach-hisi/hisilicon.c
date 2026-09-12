// SPDX-License-Identifier: GPL-2.0-only
/*
 * (HiSilicon's SoC based) flattened device tree enabled machine
 *
 * Copyright (c) 2012-2013 HiSilicon Ltd.
 * Copyright (c) 2012-2013 Linaro Ltd.
 *
 * Author: Haojian Zhuang <haojian.zhuang@linaro.org>
*/

#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/of.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#define HI3620_SYSCTRL_PHYS_BASE		0xfc802000
#define HI3620_SYSCTRL_VIRT_BASE		0xfe802000

/*
 * This table is only for optimization. Since ioremap() could always share
 * the same mapping if it's defined as static IO mapping.
 *
 * Without this table, system could also work. The cost is some virtual address
 * spaces wasted since ioremap() may be called multi times for the same
 * IO space.
 */
static struct map_desc hi3620_io_desc[] __initdata = {
	{
		/* sysctrl */
		.pfn		= __phys_to_pfn(HI3620_SYSCTRL_PHYS_BASE),
		.virtual	= HI3620_SYSCTRL_VIRT_BASE,
		.length		= 0x1000,
		.type		= MT_DEVICE,
	},
};

static void __init hi3620_map_io(void)
{
	debug_ll_io_init();
	iotable_init(hi3620_io_desc, ARRAY_SIZE(hi3620_io_desc));
}

static const char *const hi3xxx_compat[] __initconst = {
	"hisilicon,hi3620-hi4511",
	NULL,
};

DT_MACHINE_START(HI3620, "Hisilicon Hi3620 (Flattened Device Tree)")
	.map_io		= hi3620_map_io,
	.dt_compat	= hi3xxx_compat,
MACHINE_END

#define S5_IOCH1_PHYS_BASE		0xf8000000
#define S5_IOCH1_VIRT_BASE		0xf9000000
#define S5_IOCH1_SIZE			0x02000000

#define S5_IOCH2_PHYS_BASE		0xff000000
#define S5_IOCH2_VIRT_BASE		0xfb000000
#define S5_IOCH2_SIZE			0x00430000

static struct map_desc s5_io_desc[] __initdata = {
	{
		.pfn		= __phys_to_pfn(S5_IOCH1_PHYS_BASE),
		.virtual	= S5_IOCH1_VIRT_BASE,
		.length		= S5_IOCH1_SIZE,
		.type		= MT_DEVICE,
	},
	{
		.pfn		= __phys_to_pfn(S5_IOCH2_PHYS_BASE),
		.virtual	= S5_IOCH2_VIRT_BASE,
		.length		= S5_IOCH2_SIZE,
		.type		= MT_DEVICE,
	},
};

static void __init s5_map_io(void)
{
	debug_ll_io_init();
	iotable_init(s5_io_desc, ARRAY_SIZE(s5_io_desc));
}

/*
 * ACTLR as the SDK sets it in hi3798mx_init_early(), boot CPU only: bit 2
 * enables L1 prefetch, bit 8 restricts allocation to one cache way.
 */
static void __init hi3798mv100_init_early(void)
{
	u32 val;

	asm volatile(
	"mrc	p15, 0, %0, c1, c0, 1\n"
	"orr	%0, %0, #0x104\n"
	"mcr	p15, 0, %0, c1, c0, 1\n"
		: "=&r" (val)
		:
		: "cc");
}

static void __init s5_init_early(void)
{
	if (of_machine_is_compatible("hisilicon,hi3798mv100"))
		hi3798mv100_init_early();
}

static const char *const s5_compat[] __initconst = {
	"hisilicon,hi3716cv200",
	"hisilicon,hi3716mv410",
	"hisilicon,hi3798mv100",
	NULL,
};

DT_MACHINE_START(S5, "Hisilicon S5 (Flattened Device Tree)")
	.map_io		= s5_map_io,
	.init_early	= s5_init_early,
	.dt_compat	= s5_compat,
MACHINE_END
