obj-m += gpio-hotplug.o
#gpio-hotplug-objs := device.o
VERSION := $(shell uname -r)

all:
	make -C /lib/modules/$(VERSION)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(VERSION)/build M=$(PWD) clean