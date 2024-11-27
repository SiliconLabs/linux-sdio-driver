#*HEADER*******************************************************************
#Copyright (c)
#All rights reserved
#This software embodies materials and concepts that are confidential
#to Redpine Signals and is made available solely pursuant to the terms
#of a written license agreement with Redpine Signals
#
#Project name : WiSeConnect
#Module name  : Linux SDIO Driver
#File Name    : Makefile
#
#File Description:
#    This is the Makefile For RSI Linux USB Driver
#        
#
#Author :
#
#Rev History:
#Ver        By             date       Description
#---------------------------------------------------------
#1.1                            Initial version
#---------------------------------------------------------
#*END*********************************************************************

DEF_KERNEL_DIR := /lib/modules/$(shell uname -r)/build 

ifeq ($(KERNELDIR),)
	KERNELDIR := $(DEF_KERNEL_DIR)
endif


ifeq ($(DRIVER_DIR),)
DRIVER_DIR=$(PWD)
endif


SDIO_INCLUDE=$(DRIVER_DIR)/../include/
COMMON_INCLUDE=$(DRIVER_DIR)/../../common/include/
COMMON_SRC=../../common/src
EXTRA_CFLAGS += -DLINUX 
EXTRA_CFLAGS += -DSLI_SDIO_INTERFACE
EXTRA_CFLAGS += -D m32

EXTRA_CFLAGS += -I$(SDIO_INCLUDE)
EXTRA_CFLAGS += -I$(COMMON_INCLUDE)
EXTRA_CFLAGS += -I$(KERNELDIR)
EXTRA_CFLAGS += -Wimplicit
EXTRA_CFLAGS += -w
EXTRA_CFLAGS += -DRSI_AUTOMATION_ENABLE
EXTRA_CFLAGS += -Wstrict-prototypes
EXTRA_CFLAGS += -DRS9116
EXTRA_CFLAGS += -DRSI_SDIO_MULTI_BLOCK_SUPPORT

SRC_FILES :=  $(COMMON_SRC)/sli_linux_netlink.o \
              $(COMMON_SRC)/sli_net_device.o \
	            $(COMMON_SRC)/sli_linux_specific.o \
              $(COMMON_SRC)/sli_linux_data.o \
							$(COMMON_SRC)/sli_send_boot_insn.o \
							$(COMMON_SRC)/sli_lib_util.o \
	            sli_hal_mcu_ioports.o \
	            sli_hal_mcu_sdio.o \

obj-m := $(INSTALLDIR)rpssdio.o
$(INSTALLDIR)rpssdio-objs :=  ${SRC_FILES}

all:
	@echo "Compiling driver code"
	make -C $(KERNELDIR) SUBDIRS=$(PWD) modules

clean:
	@echo "Cleaning driver binary files"
	rm -rf $(INSTALLDIR)rpssdio.* 
	rm -rf $(INSTALLDIR)*.rpssdio.*.*
	rm -rf *.o *.mod.* *.symvers
	rm -rf .rps*
	rm -rf .rsi*
	rm -rf .tmp_versions
	rm -rf modules.order
	rm -rf $(COMMON_SRC)/*.o
	rm -rf $(COMMON_SRC)/*.cmd 
	rm -rf $(COMMON_SRC)/.*.cmd 
