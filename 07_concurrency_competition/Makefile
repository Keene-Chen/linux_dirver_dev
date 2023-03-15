# author  : KeeneChen
# date    : 2023.03.14-13:56:25
# details : concurrency and competition

#>>> Linux kernel module >>>
KERNELDIR    := /home/keenechen/IMX6ULL/linux_system_porting/alientek/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek
CURRENT_PATH := $(shell pwd)
obj-m        := atomic.o spinlock.o semaphore.o mutex.o
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
ROOTFS_PATH  := ~/Linux/nfs/alientek_rootfs_1.35/lib/modules/4.1.15
SRCFILE      := $(patsubst %.o,%_app.c, $(obj-m))
OBJFILE      := $(patsubst %.o,%_app, $(obj-m))

app: kernel_modules
	$(GCC) atomic_app.c    -o atomic_app
	$(GCC) spinlock_app.c  -o spinlock_app
	$(GCC) semaphore_app.c -o semaphore_app
	$(GCC) mutex_app.c     -o mutex_app
	cp $(OBJFILE) $(ROOTFS_PATH)
	cp $(patsubst %.o,%.ko, $(obj-m)) $(ROOTFS_PATH)
#<<< custom targets <<<
