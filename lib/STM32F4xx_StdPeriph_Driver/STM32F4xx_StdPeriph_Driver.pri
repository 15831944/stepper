#LIB = libSTM32F4xx_StdPeriph_Driver.a

INCLUDEPATH += $$PWD/inc \
		$$PWD/inc/device_support \
		$$PWD/inc/core_support

SOURCES += \
$$PWD/src/misc.c \
$$PWD/src/stm32f4xx_adc.c \
$$PWD/src/stm32f4xx_can.c \
$$PWD/src/stm32f4xx_crc.c \
$$PWD/src/stm32f4xx_cryp.c \
$$PWD/src/stm32f4xx_cryp_aes.c \
$$PWD/src/stm32f4xx_cryp_des.c \
$$PWD/src/stm32f4xx_cryp_tdes.c \
$$PWD/src/stm32f4xx_dac.c \
$$PWD/src/stm32f4xx_dbgmcu.c \
$$PWD/src/stm32f4xx_dcmi.c \
$$PWD/src/stm32f4xx_dma.c \
$$PWD/src/stm32f4xx_exti.c \
$$PWD/src/stm32f4xx_flash.c \
$$PWD/src/stm32f4xx_fsmc.c \
$$PWD/src/stm32f4xx_gpio.c \
$$PWD/src/stm32f4xx_hash_md5.c \
$$PWD/src/stm32f4xx_hash_sha1.c \
$$PWD/src/stm32f4xx_hash.c \
$$PWD/src/stm32f4xx_i2c.c \
$$PWD/src/stm32f4xx_iwdg.c \
$$PWD/src/stm32f4xx_pwr.c \
$$PWD/src/stm32f4xx_rcc.c \
$$PWD/src/stm32f4xx_rng.c \
$$PWD/src/stm32f4xx_rtc.c \
$$PWD/src/stm32f4xx_sdio.c \
$$PWD/src/stm32f4xx_spi.c \
$$PWD/src/stm32f4xx_syscfg.c \
$$PWD/src/stm32f4xx_tim.c \
$$PWD/src/stm32f4xx_usart.c \
$$PWD/src/stm32f4xx_wwdg.c \
#$$PWD/inc/core_support/core_cm4.c
#    $$PWD/inc/core_support/core_cm3.c

HEADERS += \
    $$PWD/inc/stm32f4xx_wwdg.h \
    $$PWD/inc/stm32f4xx_usart.h \
    $$PWD/inc/stm32f4xx_tim.h \
    $$PWD/inc/stm32f4xx_syscfg.h \
    $$PWD/inc/stm32f4xx_spi.h \
    $$PWD/inc/stm32f4xx_sdio.h \
    $$PWD/inc/stm32f4xx_rtc.h \
    $$PWD/inc/stm32f4xx_rng.h \
    $$PWD/inc/stm32f4xx_rcc.h \
    $$PWD/inc/stm32f4xx_pwr.h \
    $$PWD/inc/stm32f4xx_iwdg.h \
    $$PWD/inc/stm32f4xx_i2c.h \
    $$PWD/inc/stm32f4xx_hash.h \
    $$PWD/inc/stm32f4xx_gpio.h \
    $$PWD/inc/stm32f4xx_fsmc.h \
    $$PWD/inc/stm32f4xx_flash.h \
    $$PWD/inc/stm32f4xx_exti.h \
    $$PWD/inc/stm32f4xx_dma.h \
    $$PWD/inc/stm32f4xx_dcmi.h \
    $$PWD/inc/stm32f4xx_dbgmcu.h \
    $$PWD/inc/stm32f4xx_dac.h \
    $$PWD/inc/stm32f4xx_cryp.h \
    $$PWD/inc/stm32f4xx_crc.h \
    $$PWD/inc/stm32f4xx_can.h \
    $$PWD/inc/stm32f4xx_adc.h \
    $$PWD/inc/misc.h \
    $$PWD/inc/core_support/core_cmInstr.h \
    $$PWD/inc/core_support/core_cmFunc.h \
    $$PWD/inc/core_support/core_cm4_simd.h \
    $$PWD/inc/core_support/core_cm4.h \
    $$PWD/inc/core_support/core_cm3.h \
    $$PWD/inc/core_support/core_cm0.h \
    $$PWD/inc/core_support/arm_math.h \
    $$PWD/inc/core_support/arm_common_tables.h \
    $$PWD/inc/device_support/system_stm32f4xx.h \
    $$PWD/inc/device_support/stm32f4xx.h \
    $$PWD/inc/device_support/stm32f4_discovery.h

