QMAKE_CFLAGS += -std=c99


# sources
SOURCES += src/main.c \
        src/sys.c \
        src/leds.c \
        src/stepper.c \
        src/inputs.c \
        src/system_stm32f4xx.c \
        src/gcode.c \
        src/usb.c \
        src/temp.c \

HEADERS += \
        src/gcode.h \
        src/sys.h \
        src/stepper.h \
        src/leds.h \
        src/inputs.h \
        src/usb.h \
        src/temp.h \


INCLUDEPATH += inc \


# startup code and std lib
SOURCES += lib/startup_stm32f4xx.s
include(lib/STM32F4xx_StdPeriph_Driver/STM32F4xx_StdPeriph_Driver.pri)

# Usb CDC class driver
DEFINES += USE_USB_OTG_FS=1

INCLUDEPATH += lib/STM32_USB_Device_Library/Class/cdc/inc \
        lib/STM32_USB_Device_Library/Core/inc \
        lib/STM32_USB_OTG_Driver/inc \
        usb

SOURCES += lib/STM32_USB_OTG_Driver/src/usb_dcd_int.c \
        lib/STM32_USB_OTG_Driver/src/usb_dcd.c \
        lib/STM32_USB_OTG_Driver/src/usb_core.c \
        usb/usb_bsp.c

SOURCES += lib/STM32_USB_Device_Library/Core/src/usbd_core.c \
        lib/STM32_USB_Device_Library/Core/src/usbd_req.c \
        lib/STM32_USB_Device_Library/Core/src/usbd_ioreq.c \
        lib/STM32_USB_Device_Library/Class/cdc/src/usbd_cdc_core.c

SOURCES += usb/usbd_cdc_vcp.c \
        usb/usbd_desc.c \
        usb/usbd_usr.c \
        usb/stm32f4xx_it.c \
        usb/syscalls.c

