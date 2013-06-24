# put your *.o targets here, make should handle the rest!

SRCS = main.c sys.c leds.c stepper.c inputs.c system_stm32f4xx.c 

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=stepper_drive

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
#CFLAGS += -mfloat-abi=softfp -mfpu=fpv4-sp-d16

###################################################

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

CFLAGS += -Iinc
#CFLAGS += -Ilib -Ilib/inc
#CFLAGS += -Ilib/inc/core -Ilib/inc/peripherals 
#LIBPATH += -Llib
#LIBS += -lstm32f4


SRCS += lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)


###################################################

# stm32f4_discovery lib
CFLAGS+=-Ilib/STM32F4xx_StdPeriph_Driver/inc
CFLAGS+=-Ilib/STM32F4xx_StdPeriph_Driver/inc/device_support
CFLAGS+=-Ilib/STM32F4xx_StdPeriph_Driver/inc/core_support
LIBPATH += -Llib/STM32F4xx_StdPeriph_Driver/build
LIBS += -lSTM32F4xx_StdPeriph_Driver

###################################################

#usb_conf.h
CFLAGS+=-DUSE_USB_OTG_FS=1

#STM32_USB_Device_Library
CFLAGS+=-Ilib/STM32_USB_Device_Library/Class/cdc/inc
CFLAGS+=-Ilib/STM32_USB_Device_Library/Core/inc
 
#STM32_USB_OTG_Driver
CFLAGS+=-Ilib/STM32_USB_OTG_Driver/inc

SRCS += lib/STM32_USB_OTG_Driver/src/usb_dcd_int.c \
lib/STM32_USB_OTG_Driver/src/usb_dcd.c \
lib/STM32_USB_OTG_Driver/src/usb_core.c \
usb/usb_bsp.c \
lib/STM32_USB_Device_Library/Core/src/usbd_core.c \
lib/STM32_USB_Device_Library/Core/src/usbd_req.c \
lib/STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
lib/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c \
usb/usbd_cdc_vcp.c \
usb/usbd_desc.c \
usb/usbd_usr.c \
usb/stm32f4xx_it.c \
usb/syscalls.c \

CFLAGS += -Iusb
###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C lib/STM32F4xx_StdPeriph_Driver/build/

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ $(LIBPATH) $(LIBS)
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin

distclean: clean
	$(MAKE) clean -C lib/STM32F4xx_StdPeriph_Driver/build/
