/* Copyright (c) 2013, LGE Inc. All rights reserved.
 * Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <mach/board_lge.h>

#if defined(CONFIG_LCD_KCAL)

struct kcal_platform_data *kcal_pdata;

static ssize_t kcal_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	int ret;
	int kcal_r = 0;
	int kcal_g = 0;
	int kcal_b = 0;

	if (!count)
		return -EINVAL;

	sscanf(buf, "%d %d %d", &kcal_r, &kcal_g, &kcal_b);
	ret = kcal_pdata->set_values(kcal_r, kcal_g, kcal_b);

	if (ret)
		return -EINVAL;

	kcal_pdata->refresh_display();
	return count;
}

static ssize_t kcal_show(struct device *dev, struct device_attribute *attr,
								char *buf)
{
	int kcal_r = 0;
	int kcal_g = 0;
	int kcal_b = 0;

	kcal_pdata->get_values(&kcal_r, &kcal_g, &kcal_b);

	return sprintf(buf, "%d %d %d\n", kcal_r, kcal_g, kcal_b);
}

static DEVICE_ATTR(kcal, 0644, kcal_show, kcal_store);

static int kcal_ctrl_probe(struct platform_device *pdev)
{
	int rc = 0;

	kcal_pdata = pdev->dev.platform_data;

	if (!kcal_pdata->set_values || !kcal_pdata->get_values ||
					!kcal_pdata->refresh_display) {
		return -1;
	}

	rc = device_create_file(&pdev->dev, &dev_attr_kcal);

	if (rc)
		return -1;

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = kcal_ctrl_probe,
	.driver = {
		.name   = "kcal_ctrl",
	},
};

int __init kcal_ctrl_init(void)
{
	return platform_driver_register(&this_driver);
}

device_initcall(kcal_ctrl_init);
#endif

MODULE_DESCRIPTION("LCD KCAL Driver");
MODULE_LICENSE("GPL v2");
