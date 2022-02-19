obj-m += gpio-hotplug.o
#gpio-hotplug-objs := device.o
VERSION := $(shell uname -r)

all:
	make -C /lib/modules/$(VERSION)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(VERSION)/build M=$(PWD) clean

%.dtbo: %.dtso
	dtc -W no-unit_address_vs_reg -I dts -O dtb $< -o $@
