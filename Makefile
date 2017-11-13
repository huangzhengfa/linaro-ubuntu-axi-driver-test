obj-m := axi_driver.o
	CURRENT_PATH:=$(shell pwd)
	LINUX_KERNEL_PATH:=/lib/modules/4.6.0-xilinx/build
	CROSS_COMPILE := /root/CodeSourcery/Sourcery_CodeBench_Lite_for_Xilinx_GNU_Linux/bin/arm-xilinx-linux-gnueabi-
	CC   = $(CROSS_COMPILE)gcc
all:
	$(MAKE) -C $(LINUX_KERNEL_PATH) M=$(CURRENT_PATH) modules ARCH=arm
clean:
	rm -rf .*.cmd *.o *.mod.c *.ko .tmp_versions
