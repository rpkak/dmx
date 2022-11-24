obj-m += dmx.o
EXTRA_CFLAGS += -std=gnu18

test: all
	sudo rmmod dmx.ko
	sudo insmod dmx.ko

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
