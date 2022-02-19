/*
 *	GPIO Hotplug Bus
 *
 *	(c) 2022 Maria Matejka <mq@jmq.cz>
 *
 *	Can be freely distributed and used under the terms of the GNU GPL 2.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>

struct bus_type gpio_hotplug_bus_type = {
	.name		= "gpio_hotplug",
};

#define MAX_DATA_LINES	12

struct gpio_hotplug_socket {
	struct gpio_desc *power_gpio;
	struct gpio_desc *led_gpio;
	uint8_t data_lines_count;
	uint8_t data_lines[MAX_DATA_LINES];
};

struct gpio_hotplug_bus_device {
	struct device *dev;
	struct gpio_descs *data_gpio;
	int num_sockets;
	struct gpio_hotplug_socket sockets[];
};

static int gpio_hotplug_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	struct gpio_hotplug_bus_device *bus_dev;
	int count;

	count = device_get_child_node_count(dev);
	if (!count)
		return -ENODEV;

	bus_dev = devm_kzalloc(dev, struct_size(bus_dev, sockets, count), GFP_KERNEL);
	if (!bus_dev)
		return -ENOMEM;

	bus_dev->dev = dev;
	bus_dev->data_gpio = devm_gpiod_get_array(dev, "data", GPIOD_IN);
	if (IS_ERR(bus_dev->data_gpio))
		return PTR_ERR(bus_dev->data_gpio);

	device_for_each_child_node(dev, child) {
		struct gpio_hotplug_socket *sock = &bus_dev->sockets[bus_dev->num_sockets];
		int dlcount = 0, err, i;

		sock->power_gpio = devm_fwnode_get_gpiod_from_child(dev, "power", child, GPIOD_OUT_LOW, NULL);
		if (IS_ERR(sock->power_gpio))
			return PTR_ERR(sock->power_gpio);

		sock->led_gpio = devm_fwnode_get_gpiod_from_child(dev, "led", child, GPIOD_OUT_LOW, NULL);
		if (IS_ERR(sock->led_gpio)) {
			if (PTR_ERR(sock->led_gpio) == -ENOENT)
				sock->led_gpio = NULL;
			else
				return PTR_ERR(sock->led_gpio);
		}

		dlcount = fwnode_property_count_u8(child, "data-lines");
		if (dlcount > MAX_DATA_LINES)
			return -EINVAL;

		err = fwnode_property_read_u8_array(child, "data-lines", sock->data_lines, dlcount);
		if (err)
			return err;

		for (i = 0; i < dlcount; i++) {
			if (sock->data_lines[i] >= bus_dev->data_gpio->ndescs)
				return -EINVAL;
		}

		bus_dev->num_sockets = 0;
	}

	return 0;
}


static const struct of_device_id of_gpio_hotplug_bus_match[] = {
	{ .compatible = "gpio_hotplug,bus", },
	{},
};

MODULE_DEVICE_TABLE(of, of_gpio_hotplug_bus_match);

static struct platform_driver gpio_hotplug_bus_driver = {
	.probe		= gpio_hotplug_bus_probe,
/*	.shutdown	= gpio_hotplug_bus_shutdown, */	/* Not needed for now */
	.driver		= {
		.name	= "gpio_hotplug_bus",
		.of_match_table = of_gpio_hotplug_bus_match,
	},
};

int __init gpio_hotplug_init(void)
{
	int e = 0;
	e = bus_register(&gpio_hotplug_bus_type);
	if (e)
		goto err_bus;

	e = platform_driver_register(&gpio_hotplug_bus_driver);
	if (e)
		goto err_platform_driver;

	return 0;

err_platform_driver:
	bus_unregister(&gpio_hotplug_bus_type);
err_bus:
	return e;
}

void __exit gpio_hotplug_exit(void)
{
	platform_driver_unregister(&gpio_hotplug_bus_driver);
	bus_unregister(&gpio_hotplug_bus_type);
}

module_init(gpio_hotplug_init);
module_exit(gpio_hotplug_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maria Matejka <mq@jmq.cz>");
MODULE_DESCRIPTION("GPIO Hotplug");

/* vim: set ts=8 sw=8 sts=8: */
