obj-m += wt440h_rx.o
INSTALL = sudo install -o fhem -g dialout
INSTALLDIR = /opt/fhem

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	gcc -O3 -s -Wall -fomit-frame-pointer wt440h_decode.c -o wt440h_decode

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm wt440h_decode

install: all
	$(INSTALL) -m 644 wt440h_rx.ko $(INSTALLDIR)
	$(INSTALL) -m 755 wt440h_decode $(INSTALLDIR)
	$(INSTALL) -m 755 wt440h2fhem.sh $(INSTALLDIR)
