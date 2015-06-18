obj-m += wt440h_rx.o
INSTALL = sudo install -o fhem -g dialout
INSTALLDIR = /opt/fhem
GCC = gcc -O3 -s -Wall -fomit-frame-pointer
RM = rm -rf

all: wt440h_rx.ko wt440h_decode

wt440h_rx.ko: wt440h_rx.c
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

wt440h_decode: wt440h_decode.c
	$(GCC) wt440h_decode.c -o wt440h_decode

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	$(RM) wt440h_decode

install: all
	$(INSTALL) -m 644 wt440h_rx.ko $(INSTALLDIR)
	$(INSTALL) -m 755 wt440h_decode $(INSTALLDIR)
	$(INSTALL) -m 755 wt440h2fhem.sh $(INSTALLDIR)
