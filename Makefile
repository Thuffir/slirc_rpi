obj-m += wt440h_rx.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	gcc -O3 -s -Wall -fomit-frame-pointer wt440h_decode.c -o wt440h_decode

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm wt440h_decode