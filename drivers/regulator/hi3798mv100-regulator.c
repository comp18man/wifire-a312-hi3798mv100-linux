// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Hi3798MV100 voltage regulator driver
 *
 * Copyright (c) Hisilicon community
 */

#define pr_fmt(fmt) "hi3798mv100-regulator: " fmt

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define HI3798MV100_PWM_STEP_MV		5
#define HI3798MV100_PWM_CLASS		2
#define HI3798MV100_PWM_PERIOD_MASK	0x0000ffff
#define HI3798MV100_PWM_DUTY_MASK	0xffff0000

struct hi3798mv100_regulator {
	struct regulator_desc desc;
	void __iomem *base;
	int min_uV;
	int max_uV;
};

static unsigned int hi3798mv100_regulator_vmin_mv(struct hi3798mv100_regulator *reg)
{
	return reg->min_uV / 1000;
}

static unsigned int hi3798mv100_regulator_vmax_mv(struct hi3798mv100_regulator *reg)
{
	return reg->max_uV / 1000;
}

static unsigned int hi3798mv100_regulator_pwm_period(struct hi3798mv100_regulator *reg)
{
	unsigned int vmax_mv = hi3798mv100_regulator_vmax_mv(reg);
	unsigned int vmin_mv = hi3798mv100_regulator_vmin_mv(reg);

	return (((vmax_mv - vmin_mv) * HI3798MV100_PWM_CLASS) /
		HI3798MV100_PWM_STEP_MV) + 1;
}

static int hi3798mv100_regulator_duty_to_mv(struct hi3798mv100_regulator *reg,
					     unsigned int duty)
{
	int vmax_mv = hi3798mv100_regulator_vmax_mv(reg);

	if (!duty)
		return vmax_mv;

	return vmax_mv - ((int)(duty - 1) * HI3798MV100_PWM_STEP_MV) /
	       HI3798MV100_PWM_CLASS;
}

static int hi3798mv100_regulator_get_voltage(struct regulator_dev *rdev)
{
	struct hi3798mv100_regulator *reg = rdev_get_drvdata(rdev);
	unsigned int raw, duty;
	int vmax_mv, vmin_mv;
	int volt_mv;

	raw = readl(reg->base);
	duty = (raw & HI3798MV100_PWM_DUTY_MASK) >> 16;
	vmax_mv = hi3798mv100_regulator_vmax_mv(reg);
	vmin_mv = hi3798mv100_regulator_vmin_mv(reg);

	volt_mv = hi3798mv100_regulator_duty_to_mv(reg, duty);
	if (volt_mv < vmin_mv || volt_mv > vmax_mv)
		return -EINVAL;

	return volt_mv * 1000;
}

static int hi3798mv100_pick_voltage(struct hi3798mv100_regulator *reg,
				    int min_uV, int max_uV,
				    unsigned int *duty_out)
{
	unsigned int vmax_mv = reg->max_uV / 1000;
	unsigned int vmin_mv = reg->min_uV / 1000;
	unsigned int req_low_mv = DIV_ROUND_UP(min_uV, 1000);
	unsigned int req_high_mv = max_uV / 1000;
	unsigned int period, duty;
	int volt_mv;

	if (req_low_mv > req_high_mv)
		return -EINVAL;

	if (req_high_mv < vmin_mv || req_low_mv > vmax_mv)
		return -EINVAL;

	req_low_mv = max_t(unsigned int, req_low_mv, vmin_mv);
	req_high_mv = min_t(unsigned int, req_high_mv, vmax_mv);
	period = hi3798mv100_regulator_pwm_period(reg);

	for (duty = 1; duty <= period; duty++) {
		volt_mv = hi3798mv100_regulator_duty_to_mv(reg, duty);
		if (volt_mv <= req_high_mv && volt_mv >= req_low_mv) {
			*duty_out = duty;
			return 0;
		}
	}

	return -EINVAL;
}

static int hi3798mv100_regulator_set_voltage(struct regulator_dev *rdev,
					      int min_uV, int max_uV,
					      unsigned int *selector)
{
	struct hi3798mv100_regulator *reg = rdev_get_drvdata(rdev);
	unsigned int duty, period, value;
	int ret;

	ret = hi3798mv100_pick_voltage(reg, min_uV, max_uV, &duty);
	if (ret)
		return ret;

	period = hi3798mv100_regulator_pwm_period(reg);
	/* Vendor format: duty in [31:16], period in [15:0]. */
	value = ((duty << 16) & HI3798MV100_PWM_DUTY_MASK) |
		(period & HI3798MV100_PWM_PERIOD_MASK);
	if (readl(reg->base) != value)
		writel(value, reg->base);

	if (selector)
		*selector = duty;

	return 0;
}

static const struct regulator_ops hi3798mv100_regulator_ops = {
	.get_voltage = hi3798mv100_regulator_get_voltage,
	.set_voltage = hi3798mv100_regulator_set_voltage,
};

static int hi3798mv100_regulator_probe(struct platform_device *pdev)
{
	struct hi3798mv100_regulator *reg;
	struct regulator_init_data *init_data;
	struct regulator_config config = { };
	struct resource *res;
	resource_size_t size;
	const char *res_name = "base-address";
	struct regulator_dev *rdev;

	reg = devm_kzalloc(&pdev->dev, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, res_name);
	if (!res)
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	size = resource_size(res);
	if (!size)
		return -EINVAL;

	/*
	 * Control words sit inside the parent syscon window, so map them
	 * without request_mem_region().
	 */
	reg->base = devm_ioremap(&pdev->dev, res->start, size);
	if (!reg->base)
		return -ENOMEM;

	reg->desc.name = dev_name(&pdev->dev);
	reg->desc.type = REGULATOR_VOLTAGE;
	reg->desc.owner = THIS_MODULE;
	reg->desc.ops = &hi3798mv100_regulator_ops;
	reg->desc.continuous_voltage_range = true;

	init_data = of_get_regulator_init_data(&pdev->dev, pdev->dev.of_node,
					       &reg->desc);
	if (!init_data)
		return -EINVAL;

	init_data->constraints.apply_uV = 0;
	reg->min_uV = init_data->constraints.min_uV;
	reg->max_uV = init_data->constraints.max_uV;
	if (reg->min_uV <= 0 || reg->max_uV <= reg->min_uV)
		return -EINVAL;

	config.dev = &pdev->dev;
	config.init_data = init_data;
	config.driver_data = reg;
	config.of_node = pdev->dev.of_node;

	rdev = devm_regulator_register(&pdev->dev, &reg->desc, &config);
	if (IS_ERR(rdev))
		return PTR_ERR(rdev);

	platform_set_drvdata(pdev, reg);
	return 0;
}

static const struct of_device_id hi3798mv100_regulator_of_match[] = {
	{ .compatible = "hisilicon,hi3798mv100-volt" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3798mv100_regulator_of_match);

static struct platform_driver hi3798mv100_regulator_driver = {
	.probe = hi3798mv100_regulator_probe,
	.driver = {
		.name = "hi3798mv100-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = hi3798mv100_regulator_of_match,
	},
};
module_platform_driver(hi3798mv100_regulator_driver);

MODULE_AUTHOR("Hisilicon community");
MODULE_DESCRIPTION("HiSilicon Hi3798MV100 voltage regulator");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hi3798mv100-regulator");
