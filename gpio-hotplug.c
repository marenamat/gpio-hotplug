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
	.name		= "gpio-hotplug",
};

#define GPIO_HOTPLUG_SOCKET_MAX_DATA_LINES	12

struct gpio_hotplug_socket {
	struct gpio_desc *power_gpio;
	struct gpio_desc *led_gpio;
	const char *name;
	uint8_t data_lines_count;
	uint8_t data_lines[GPIO_HOTPLUG_SOCKET_MAX_DATA_LINES];
};

struct gpio_hotplug_bus_device {
	struct device *dev;
	struct gpio_descs *data_gpio;
	int num_sockets;
	struct gpio_hotplug_socket sockets[];
};

#define CHECK(cond, ret, ...)	if (cond) { dev_err(dev, __VA_ARGS__); return ret; }

static int gpio_hotplug_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	struct gpio_hotplug_bus_device *bus_dev;
	int count;

	printk("GPIO Hotplug bus probe\n");

	count = device_get_child_node_count(dev);
	CHECK(!count, -ENODEV, "No child nodes\n");

	bus_dev = devm_kzalloc(dev, struct_size(bus_dev, sockets, count), GFP_KERNEL);
	CHECK(!bus_dev, -ENOMEM, "Not enough memory\n");
	printk("Allocated OK\n");

	bus_dev->dev = dev;
	bus_dev->data_gpio = devm_gpiod_get_array(dev, "data", GPIOD_IN);
	CHECK(IS_ERR(bus_dev->data_gpio), PTR_ERR(bus_dev->data_gpio), "Data GPIO fetch failed: %ld\n", PTR_ERR(bus_dev->data_gpio));
	printk("Have %d data gpios\n", bus_dev->data_gpio->ndescs);


#define CHECK_CHILD(cond, ret, ...)	if (cond) { fwnode_handle_put(child); dev_err(dev, __VA_ARGS__); return ret; }

	device_for_each_child_node(dev, child) {
		struct gpio_hotplug_socket *sock = &bus_dev->sockets[bus_dev->num_sockets];
		int dlcount = 0, i;
		int err;
		uint32_t data_lines[GPIO_HOTPLUG_SOCKET_MAX_DATA_LINES];

		err = fwnode_property_read_string(child, "label", &sock->name);
		CHECK_CHILD(err, err, "Socket without label\n");

		sock->power_gpio = devm_fwnode_get_gpiod_from_child(dev, "power", child, GPIOD_OUT_LOW, NULL);
		CHECK_CHILD(IS_ERR(sock->power_gpio), PTR_ERR(sock->power_gpio),
				"Socket %s power pin error: %ld\n", sock->name, PTR_ERR(sock->power_gpio));

		sock->led_gpio = devm_fwnode_get_gpiod_from_child(dev, "led", child, GPIOD_OUT_LOW, NULL);
		if (IS_ERR(sock->led_gpio)) {
			CHECK_CHILD(
				(PTR_ERR(sock->led_gpio) != -ENOENT),
				PTR_ERR(sock->led_gpio),
				"Socket %s LED pin error: %ld\n", sock->name, PTR_ERR(sock->led_gpio));

			sock->led_gpio = NULL;
		}

		dlcount = fwnode_property_count_u32(child, "data-lines");
		CHECK_CHILD(dlcount >= GPIO_HOTPLUG_SOCKET_MAX_DATA_LINES,
			-EINVAL,
			"Socket %s has too many data lines: %d\n", sock->name, dlcount);

		err = fwnode_property_read_u32_array(child, "data-lines", data_lines, dlcount);
		CHECK_CHILD(err, err,
			"Socket %s data line array read error: %d\n", sock->name, err);

		for (i = 0; i < dlcount; i++) {
			CHECK_CHILD(data_lines[i] >= bus_dev->data_gpio->ndescs,
				-EINVAL,
				"Socket %s data line %d index %d out of bounds (%d)\n",
				sock->name, i, data_lines[i], bus_dev->data_gpio->ndescs);

			sock->data_lines[i] = data_lines[i];
		}

		bus_dev->num_sockets++;
	}

	platform_set_drvdata(pdev, bus_dev);
	printk("Registered %d gpio hotplug sockets\n", bus_dev->num_sockets);

	return 0;
}


static const struct of_device_id of_gpio_hotplug_bus_match[] = {
	{ .compatible = "gpio-hotplug,bus", },
	{},
};

MODULE_DEVICE_TABLE(of, of_gpio_hotplug_bus_match);

static struct platform_driver gpio_hotplug_bus_driver = {
	.probe		= gpio_hotplug_bus_probe,
/*	.shutdown	= gpio_hotplug_bus_shutdown, */	/* Not needed for now */
	.driver		= {
		.name	= "gpio-hotplug,bus",
		.of_match_table = of_match_ptr(of_gpio_hotplug_bus_match),
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

	printk("GPIO hotplug initialized\n");
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
