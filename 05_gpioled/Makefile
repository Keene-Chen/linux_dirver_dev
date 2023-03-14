# author  : KeeneChen
# date    : 2023.03.11-19:33:32
# details : gpio led driver module

KERNELDIR    := /home/keenechen/IMX6ULL/linux_system_porting/alientek/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek
CURRENT_PATH := $(shell pwd)
obj-m        := gpioled.o
ccflags-y    := -std=gnu99

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean

GCC          ?= arm-linux-gnueabihf-gcc
SRCFILE      := $(patsubst %.o,%_app.c, $(obj-m))
OBJFILE      := $(patsubst %.o,%_app, $(obj-m))
ROOTFS_PATH  := ~/Linux/nfs/alientek_rootfs_1.35/lib/modules/4.1.15
app: kernel_modules
	$(GCC) $(SRCFILE) -o $(OBJFILE)
	mv $(OBJFILE) $(ROOTFS_PATH)
	cp $(patsubst %.o,%.ko, $(obj-m)) $(ROOTFS_PATH)
