obj-m += slirc_rpi.o

slirc_rpi.ko: slirc_rpi.c
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
