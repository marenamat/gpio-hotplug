# GPIO Hotplug

In standard case, nobody should manipulate wires to GPIO when power is on.
Anyway, after adding some additional circuitry, some kind-of hotplug is
possible. The actual hardware is still in early development but this project is
going to include its design.

## Hardware outline

I'm designing a device with some ports and peripherals which are simple enough
to be connected to GPIO, like remote switches or LEDs on a long wire. The
device will consist of a main box with a socket panel and pluggable
peripherals. These peripherals may get wired differently every time as the
device must be portable.

Connectors to be used are probably some DINs. Therefore ground gets connected
first. Other pins get 100k pull-down resistors to ground on both sides to
ensure that after ground is connected, all other pins are also grounded before
connecting.

There is no feasible way to do device autodetection, therefore at least the
first version of the box will need to switch the devices on and off explicitly.

Every socket will consume:
  * the number of GPIO ports for data lines (e.g. 1 for LEDs and switches, 2 for capacity sensors)
  * one GPIO port for switching on/off
  * (maybe) one GPIO port for status LED

## Driver design

GPIO Hotplug is a bus (named gpio-hotplug), defined by a device tree (no ACPI
support for now, sorry). The device tree also explicitly defines how the
sockets are wired to GPIO. Multiple sockets may share one GPIO pin, e.g. there
may be a two-data-line socket sharing pins with two single-data-line sockets.

By default, all sockets are off. All assigned data ports are set to input,
status LEDs are green. After a device is plugged into a socket, the socket is
turned on by writing the appropriate config string into its sysfs control file.
Then all the data GPIO ports get blocked (possibly blocking more than one socket),
the device is provided with electricity and its driver is probed to take control.

When removing a device, it must be disabled first by writing the sysfs
control file, and after the LED turns back green, the device may be unplugged.

## Sysfs control files

For every socket, GPIO Hotplug creates a directory in `/sys/bus/gpio-hotplug/devices/`
containing following files:

* reading from `status` returns "off", "on" or "blocked"
* writing to `new_device` switches the port on or returns an error (mostly -EBUSY)
* writing to `delete_device` switches the port off

------

This project is currently in an early development stage, yet still available under
GNU GPL 2 on our own risk. Do your own thorough checking and testing before
using this code.
