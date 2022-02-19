/*
 *	GPIO Hotplug Bus
 *
 *	(c) 2022 Maria Matejka <mq@jmq.cz>
 *
 *	Can be freely distributed and used under the terms of the GNU GPL 2.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

struct bus_type gpio_hotplug_bus_type = {
	.name		= "gpio_hotplug",
};

int __init gpio_hotplug_init(void)
{
	int e;
	e = bus_register(&gpio_hotplug_bus_type);
	if (e)
		return e;

	return 0;
}

void __exit gpio_hotplug_exit(void)
{
	bus_unregister(&gpio_hotplug_bus_type);
}

module_init(gpio_hotplug_init);
module_exit(gpio_hotplug_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maria Matejka <mq@jmq.cz>");
MODULE_DESCRIPTION("GPIO Hotplug");

/* vim: set ts=8 sw=8 sts=8: */
