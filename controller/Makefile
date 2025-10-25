# Makefile (out-of-tree)
obj-m += controller_kernel.o

# Detect if the running kernel has KUnit enabled (y or m)
KUNIT_ENABLED := $(shell if zcat /proc/config.gz 2>/dev/null | grep -qE '^CONFIG_KUNIT=(y|m)'; then echo 1; else echo 0; fi)

ifeq ($(KUNIT_ENABLED),1)
  obj-m += tests/nodeb_kunit_test.o
  ccflags-y += -I$(PWD) -Wno-unused-function
  $(info Building KUnit tests: CONFIG_KUNIT is enabled)
else
  $(info Skipping KUnit tests: CONFIG_KUNIT is not enabled in running kernel)
endif

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
