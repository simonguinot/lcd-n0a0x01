KDIR ?= /lib/modules/$(shell uname -r)/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
