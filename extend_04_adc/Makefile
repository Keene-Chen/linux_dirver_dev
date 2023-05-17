# author  : KeeneChen
# date    : 2023.04.03-21:12:28
# details : photosensor

#>>> custom targets >>>
GCC          ?= arm-linux-gnueabihf-gcc
ROOTFS_PATH  := ~/Linux/nfs/kc_rootfs/lib/modules/4.1.15
SRCFILE      := photosensor.c fire.c
OBJFILE      := photosensor fire
CFLAGS       := -march=armv7-a -mfpu=neon -mfloat-abi=hard -lm -std=gnu99

app:
	$(GCC) photosensor.c $(CFLAGS) -o photosensor
	$(GCC) fire.c $(CFLAGS) -o fire
	cp $(OBJFILE) $(ROOTFS_PATH)
clean:
	rm -f $(OBJFILE)
#<<< custom targets <<<