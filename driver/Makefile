# SPDX-License-Identifier: GPL-2.0-or-later

KVERSION := $(shell uname -r)
PWD := $(shell pwd)

obj-m := leds-aravir.o

all:
	+make -C /lib/modules/$(KVERSION)/build/ M=$(PWD) modules
clean:
	+make -C /lib/modules/$(KVERSION)/build/ M=$(PWD) clean
