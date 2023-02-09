obj-m += dmx.o
EXTRA_CFLAGS += -std=gnu18

all: dmx.dtbo
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f dmx.dtbo

%.dtbo: %.dts
	dtc -I dts -O dtb -o $@ $^
