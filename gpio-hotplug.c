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
#include <linux/kobject.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

static DEFINE_MUTEX(gpio_hotplug_mutex);

struct bus_type gpio_hotplug_bus_type = {
	.name		= "gpio-hotplug",
};

struct gpio_hotplug_bus_data;

struct gpio_hotplug_socket {
	struct gpio_desc *power_gpio;
	struct gpio_desc *led_gpio;
	const char *name;
	struct device dev;

	int socket_index;

	uint8_t state;

	uint8_t data_lines_count;
#define GPIO_HOTPLUG_SOCKET_MAX_DATA_LINES	12
	uint8_t data_lines[GPIO_HOTPLUG_SOCKET_MAX_DATA_LINES];

	void *child_data;
	void (*unplug)(struct gpio_hotplug_bus_data *, struct gpio_hotplug_socket *);
};

#define GPIO_HOTPLUG_SOCKET_STATE_OFF		0
#define GPIO_HOTPLUG_SOCKET_STATE_ON		1
#define GPIO_HOTPLUG_SOCKET_STATE_BLOCKED	2

const char *gpio_hotplug_socket_state_name[] = {
	[GPIO_HOTPLUG_SOCKET_STATE_OFF]	= "off",
	[GPIO_HOTPLUG_SOCKET_STATE_ON]		= "on",
	[GPIO_HOTPLUG_SOCKET_STATE_BLOCKED]	= "blocked",
};

struct gpio_hotplug_bus_data {
	struct device *dev;
	struct gpio_descs *data_gpio;
	struct gpio_hotplug_socket **data_used_by;
	int num_sockets;
	struct gpio_hotplug_socket sockets[];
};

static int gpio_hotplug_socket_state_check_off(struct gpio_hotplug_bus_data *bus_dev, struct gpio_hotplug_socket *sock)
{
	int i;

	if (sock->state != GPIO_HOTPLUG_SOCKET_STATE_OFF)
		return -EBUSY;

	/* Check line state */
	for (i = 0; i < sock->data_lines_count; i++) {
		int id = sock->data_lines[i];
		if (bus_dev->data_used_by[id] == NULL)
			continue;

		dev_err(&sock->dev, "Data line %d (idx %d) busy!\n", i, id);
		return -EIO;
	}

	return 0;
}

static void gpio_hotplug_socket_recalculate_blocked(struct gpio_hotplug_bus_data *bus_dev)
{
	int i;
	for (i = 0; i < bus_dev->num_sockets; i++)
	{
		int d;
		struct gpio_hotplug_socket *sock = &bus_dev->sockets[i];
		if (sock->state != GPIO_HOTPLUG_SOCKET_STATE_OFF)
			continue;

		/* Any data line used by other socket makes this socket blocked */
		for (d = 0; d < sock->data_lines_count; d++)
			if (bus_dev->data_used_by[sock->data_lines[d]]) {
				sock->state = GPIO_HOTPLUG_SOCKET_STATE_BLOCKED;
				break;
			}
	}
}

static void gpio_hotplug_socket_recalculate_off(struct gpio_hotplug_bus_data *bus_dev)
{
	int i;
	for (i = 0; i < bus_dev->num_sockets; i++)
	{
		int d;
		struct gpio_hotplug_socket *sock = &bus_dev->sockets[i];
		if (sock->state != GPIO_HOTPLUG_SOCKET_STATE_BLOCKED)
			continue;

		/* Any data line used by other socket keeps this socket blocked */
		for (d = 0; d < sock->data_lines_count; d++)
			if (bus_dev->data_used_by[sock->data_lines[d]])
				break;

		/* No data line used, mark as off */
		if (d == sock->data_lines_count)
			sock->state = GPIO_HOTPLUG_SOCKET_STATE_OFF;
	}
}

static void gpio_hotplug_socket_state_set_on(struct gpio_hotplug_bus_data *bus_dev, struct gpio_hotplug_socket *sock)
{
	int i;

	/* Mark the data lines busy */
	for (i = 0; i < sock->data_lines_count; i++)
		bus_dev->data_used_by[sock->data_lines[i]] = sock;

	sock->state = GPIO_HOTPLUG_SOCKET_STATE_ON;
	gpio_hotplug_socket_recalculate_blocked(bus_dev);
}

static void gpio_hotplug_socket_state_set_off(struct gpio_hotplug_bus_data *bus_dev, struct gpio_hotplug_socket *sock)
{
	int i;

	/* Mark the data lines free */
	for (i = 0; i < sock->data_lines_count; i++)
		bus_dev->data_used_by[sock->data_lines[i]] = NULL;

	sock->state = GPIO_HOTPLUG_SOCKET_STATE_OFF;
	gpio_hotplug_socket_recalculate_off(bus_dev);
}

#define find_blank(buf)	(strchr(buf, ' ') ?: strchr(buf, '\n'))

struct gpio_hotplug_led_pdev {
	struct platform_device pdev;
	struct gpio_led_platform_data pdata;
	const char *names;
	struct gpio_led leds[];
};

static void gpio_hotplug_led_unplug(struct gpio_hotplug_bus_data *bus_dev, struct gpio_hotplug_socket *sock)
{
	struct gpio_hotplug_led_pdev *pdata_packed = sock->child_data;

	printk("Device unregister before.\n");
	platform_device_unregister(&pdata_packed->pdev);
	printk("Device unregister after.\n");
	kfree(pdata_packed->names);
	kfree(pdata_packed);
}

static void gpio_hotplug_led_release(struct device *dev)
{
//	struct gpio_hotplug_led_pdev *pdata_packed =
	printk("LED release\n");
}

static ssize_t gpio_hotplug_led_create(struct gpio_hotplug_bus_data *bus_dev, struct gpio_hotplug_socket *sock, const char *buf, size_t count)
{
	int i, result;
	char *blank, *names;
	struct gpio_hotplug_led_pdev *pdata_packed;

	if (count <= 1)
		return -EINVAL;

	/* Allocate everything */
	pdata_packed = kzalloc(struct_size(pdata_packed, leds, sock->data_lines_count), GFP_KERNEL);
	if (!pdata_packed)
		return -ENOMEM;

	names = kmalloc(count+1, GFP_KERNEL);
	if (!names) {
		result = -ENOMEM;
		goto free_pdata;
	}

	/* Fill in the LED info structure */
	memcpy(names, buf, count);
	pdata_packed->names = names;

	for (i = 0; i < sock->data_lines_count; i++) {
		/* Parse another LED name */
		if (count <= 1) {
			dev_err(&sock->dev, "Not enough names\n");
			result = -EINVAL;
			goto free_names;
		}

		blank = find_blank(names);
		if ((!blank) || (blank == names)) {
			dev_err(&sock->dev, "Invalid name for LED %d\n", i);
			result = -EINVAL;
			goto free_names;
		}

		*blank = 0;

		printk("LED %d on pin %d named %s\n", i, sock->data_lines[i], names);

		pdata_packed->leds[i].name = names;
		pdata_packed->leds[i].default_trigger = "none";
		pdata_packed->leds[i].gpiod = bus_dev->data_gpio->desc[sock->data_lines[i]];

		names = blank + 1;
	}

	pdata_packed->pdata.num_leds = sock->data_lines_count;
	pdata_packed->pdata.leds = pdata_packed->leds;

	/* Fill in the platform device */
	pdata_packed->pdev.name = "leds-gpio";
	pdata_packed->pdev.id = -1;
	pdata_packed->pdev.dev.platform_data = &pdata_packed->pdata;
	pdata_packed->pdev.dev.release = gpio_hotplug_led_release;

	/* Register the device */
	result = platform_device_register(&pdata_packed->pdev);
	if (result) {
		dev_err(&sock->dev, "Failed to register LEDS: %d\n", result);
		goto free_names;
	}

	/* Store device-specific information to the socket */
	sock->child_data = pdata_packed;
	sock->unplug = gpio_hotplug_led_unplug;

	return 0;

free_names:
	kfree(names);
free_pdata:
	kfree(pdata_packed);
	return result;
}

struct {
	const char *type;
	ssize_t (*create)(struct gpio_hotplug_bus_data *, struct gpio_hotplug_socket *, const char *, size_t);
} new_device_match_table[] = {
	{ "LED", gpio_hotplug_led_create },
};

#define MATCH_TABLE_SIZE	(sizeof(new_device_match_table) / sizeof(new_device_match_table[0]))

static ssize_t new_device_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct gpio_hotplug_socket *sock = container_of(dev, struct gpio_hotplug_socket, dev);
	struct gpio_hotplug_bus_data *bus_dev = sock->dev.driver_data;
	const char *blank;
	int i;

	if (count <= 2)
		return -EINVAL;

	blank = find_blank(buf);
	if (!blank) {
		dev_err(dev, "Garbled input, no blank char.\n");
		return -EINVAL;
	}

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	for (i = 0; i < MATCH_TABLE_SIZE; i++) {
		int result;
		ssize_t (*create)(struct gpio_hotplug_bus_data *, struct gpio_hotplug_socket *, const char *, size_t);

		if (strncasecmp(buf, new_device_match_table[i].type, blank - buf))
			continue;

		create = new_device_match_table[i].create;

		if (mutex_lock_interruptible(&gpio_hotplug_mutex))
		{
			module_put(THIS_MODULE);
			return -EINTR;
		}

		/* Check that the socket is actually free */
		result = gpio_hotplug_socket_state_check_off(bus_dev, sock);
		if (result) {
			mutex_unlock(&gpio_hotplug_mutex);
			module_put(THIS_MODULE);
			return result;
		}

		/* Create the device */
		result = create(bus_dev, sock, blank + 1, count - 1 - (blank - buf));
		if (result) {
			mutex_unlock(&gpio_hotplug_mutex);
			module_put(THIS_MODULE);
			return result;
		}

		/* And switch it on */
		gpio_hotplug_socket_state_set_on(bus_dev, sock);

		mutex_unlock(&gpio_hotplug_mutex);
		return count;
	}

	dev_err(dev, "new_device: Unknown type\n");
	module_put(THIS_MODULE);
	return -EINVAL;
}
static DEVICE_ATTR_WO(new_device);

static ssize_t delete_device_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct gpio_hotplug_socket *sock = container_of(dev, struct gpio_hotplug_socket, dev);
	struct gpio_hotplug_bus_data *bus_dev = sock->dev.driver_data;

	if (mutex_lock_interruptible(&gpio_hotplug_mutex))
		return -EINTR;

	if (sock->state != GPIO_HOTPLUG_SOCKET_STATE_ON) {
		mutex_unlock(&gpio_hotplug_mutex);
		return -EINVAL;
	}

	/* Unplug the driver */
	sock->unplug(bus_dev, sock);

	/* Switch the device off */
	gpio_hotplug_socket_state_set_off(bus_dev, sock);

	/* Cleanup */
	sock->child_data = NULL;
	sock->unplug = NULL;

	mutex_unlock(&gpio_hotplug_mutex);
	module_put(THIS_MODULE);

	return count;
}
static DEVICE_ATTR_WO(delete_device);

static ssize_t state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct gpio_hotplug_socket *sock;
	ssize_t out;

	if (mutex_lock_interruptible(&gpio_hotplug_mutex))
		return -EINTR;

	sock = container_of(dev, struct gpio_hotplug_socket, dev);
	out = sprintf(buf, "%s\n", gpio_hotplug_socket_state_name[sock->state]);

	mutex_unlock(&gpio_hotplug_mutex);
	return out;
}
static DEVICE_ATTR_RO(state);

static struct attribute *gpio_hotplug_socket_attrs[] = {
	&dev_attr_state.attr,
	&dev_attr_new_device.attr,
	&dev_attr_delete_device.attr,
	NULL,
};
ATTRIBUTE_GROUPS(gpio_hotplug_socket);

static void gpio_hotplug_socket_release(struct device *dev)
{
	/* There is honestly nothing to do! */
}

struct device_type gpio_hotplug_socket_type = {
	.groups		= gpio_hotplug_socket_groups,
	.release	= gpio_hotplug_socket_release,
};

static int gpio_hotplug_bus_probe(struct platform_device *pdev)
{

#define CHECK(cond, ret, ...)				\
	if (cond) {					\
		dev_err(dev, __VA_ARGS__);		\
		mutex_unlock(&gpio_hotplug_mutex);	\
		return ret;				\
	}

	struct device *dev = &pdev->dev;
	struct fwnode_handle *child;
	struct gpio_hotplug_bus_data *bus_dev;
	int count;

	mutex_lock(&gpio_hotplug_mutex);

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

#define CHECK_CHILD(cond, ret, ...)			\
	if (cond) {					\
		fwnode_handle_put(child);		\
		dev_err(dev, __VA_ARGS__);		\
		mutex_unlock(&gpio_hotplug_mutex);	\
		return ret;				\
	}

	device_for_each_child_node(dev, child) {
		struct gpio_hotplug_socket *sock = &bus_dev->sockets[bus_dev->num_sockets];
		int dlcount = 0, i;
		int err;
		uint32_t data_lines[GPIO_HOTPLUG_SOCKET_MAX_DATA_LINES];

		sock->socket_index = bus_dev->num_sockets;

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

		sock->data_lines_count = dlcount;

		sock->dev.parent = bus_dev->dev;
		sock->dev.bus = &gpio_hotplug_bus_type;
		sock->dev.type = &gpio_hotplug_socket_type;
		sock->dev.driver_data = bus_dev;

		dev_set_name(&sock->dev, "socket-%s", sock->name);

		err = device_register(&sock->dev);
		CHECK_CHILD(err, err, "Failed to device socket %s: %d\n", sock->name, err);

		bus_dev->num_sockets++;
	}

	bus_dev->data_used_by = devm_kzalloc(dev, sizeof(*bus_dev->data_used_by) * bus_dev->num_sockets, GFP_KERNEL);

	platform_set_drvdata(pdev, bus_dev);
	printk("Registered %d gpio hotplug sockets\n", bus_dev->num_sockets);

	mutex_unlock(&gpio_hotplug_mutex);

	return 0;
#undef CHECK
#undef CHECK_CHILD
}

static int gpio_hotplug_remove_socket_dev(struct device *dev, void *_unused)
{
	struct gpio_hotplug_socket *sock = container_of(dev, struct gpio_hotplug_socket, dev);
	printk("Removing socket %s\n", sock->name);
	device_unregister(dev);
	return 0;
}

static void gpio_hotplug_bus_shutdown(struct platform_device *pdev)
{
	mutex_lock(&gpio_hotplug_mutex);
	printk("Bus shutdown: %s\n", dev_name(&pdev->dev));
	bus_for_each_dev(&gpio_hotplug_bus_type, NULL, NULL, gpio_hotplug_remove_socket_dev);
	mutex_unlock(&gpio_hotplug_mutex);
}

static const struct of_device_id of_gpio_hotplug_bus_match[] = {
	{ .compatible = "gpio-hotplug,bus", },
	{},
};

MODULE_DEVICE_TABLE(of, of_gpio_hotplug_bus_match);

static struct platform_driver gpio_hotplug_bus_driver = {
	.probe		= gpio_hotplug_bus_probe,
	.shutdown	= gpio_hotplug_bus_shutdown,
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
	printk("Module removal\n");

	mutex_lock(&gpio_hotplug_mutex);
	bus_for_each_dev(&gpio_hotplug_bus_type, NULL, NULL, gpio_hotplug_remove_socket_dev);
	mutex_unlock(&gpio_hotplug_mutex);

	platform_driver_unregister(&gpio_hotplug_bus_driver);
	bus_unregister(&gpio_hotplug_bus_type);
}

module_init(gpio_hotplug_init);
module_exit(gpio_hotplug_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maria Matejka <mq@jmq.cz>");
MODULE_DESCRIPTION("GPIO Hotplug");

/* vim: set ts=8 sw=8 sts=8: */
