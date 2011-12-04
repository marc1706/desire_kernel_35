/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include "board-htcleo.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked) {
 		gpio_direction_output(HTCLEO_GPIO_BT_RESET_N, 1);
		gpio_direction_output(HTCLEO_GPIO_BT_SHUTDOWN_N, 1);
	} else {
 		gpio_direction_output(HTCLEO_GPIO_BT_SHUTDOWN_N, 0);
		gpio_direction_output(HTCLEO_GPIO_BT_RESET_N, 0);
	}
	return 0;
}

static struct rfkill_ops htcleo_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int htcleo_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true;  /* off */

	rc = gpio_request(HTCLEO_GPIO_BT_RESET_N, "bt_reset");
	if (rc)
		goto err_gpio_reset;
	rc = gpio_request(HTCLEO_GPIO_BT_SHUTDOWN_N, "bt_shutdown");
	if (rc)
		goto err_gpio_shutdown;

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&htcleo_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	/* userspace cannot take exclusive control */

	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
	gpio_free(HTCLEO_GPIO_BT_SHUTDOWN_N);
err_gpio_shutdown:
	gpio_free(HTCLEO_GPIO_BT_RESET_N);
err_gpio_reset:
	return rc;
}

static int htcleo_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
	gpio_free(HTCLEO_GPIO_BT_SHUTDOWN_N);
	gpio_free(HTCLEO_GPIO_BT_RESET_N);

	return 0;
}

static struct platform_driver htcleo_rfkill_driver = {
	.probe = htcleo_rfkill_probe,
	.remove = htcleo_rfkill_remove,
	.driver = {
		.name = "htcleo_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init htcleo_rfkill_init(void)
{
	if (!machine_is_htcleo())
		return 0;

	return platform_driver_register(&htcleo_rfkill_driver);
}

static void __exit htcleo_rfkill_exit(void)
{
	platform_driver_unregister(&htcleo_rfkill_driver);
}

module_init(htcleo_rfkill_init);
module_exit(htcleo_rfkill_exit);
MODULE_DESCRIPTION("htcleo rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
