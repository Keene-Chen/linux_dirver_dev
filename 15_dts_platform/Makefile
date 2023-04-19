# author  : KeeneChen
# date    : 2023.04.05-13:38:31
# details : dts platform driver

#>>> Linux kernel module >>>
KERNELDIR    := /home/keenechen/IMX6ULL/linux_system_porting/alientek/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek
CURRENT_PATH := $(shell pwd)
obj-m        := led_driver.o
ccflags-y    := -std=gnu99

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
	rm -f $(OBJFILE)
#<<< Linux kernel module <<<

#>>> custom targets >>>
GCC          ?= arm-linux-gnueabihf-gcc
ROOTFS_PATH  := ~/Linux/nfs/kc_rootfs/lib/modules/4.1.15
SRCFILE      := $(patsubst %_driver,%_app.c, $(basename $(obj-m)))
OBJFILE      := $(patsubst %_driver,%_app, $(basename $(obj-m)))

app: kernel_modules
	$(GCC) $(SRCFILE) -o $(OBJFILE)
	cp $(OBJFILE) $(ROOTFS_PATH)
	cp $(patsubst %.o,%.ko, $(obj-m)) $(ROOTFS_PATH)
#<<< custom targets <<<