# module.mk
# 
# STM32 DMX512
# Copyright (C) 2012 Erik Van Hamme, all rights reserved
# 
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
# 
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
# 

# This file is used for configuration of the build system.

SRCS += \
	stm32f4xx/cmsis/device/st/stm32f4xx/src/armgcc/startup_stm32f4xx.s \
	stm32f4xx/stdperiph/src/stm32f4xx_adc.c \
	stm32f4xx/stdperiph/src/stm32f4xx_can.c \
	stm32f4xx/stdperiph/src/stm32f4xx_crc.c \
	stm32f4xx/stdperiph/src/stm32f4xx_cryp_aes.c \
	stm32f4xx/stdperiph/src/stm32f4xx_cryp.c \
	stm32f4xx/stdperiph/src/stm32f4xx_cryp_des.c \
	stm32f4xx/stdperiph/src/stm32f4xx_cryp_tdes.c \
	stm32f4xx/stdperiph/src/stm32f4xx_dac.c \
	stm32f4xx/stdperiph/src/stm32f4xx_dbgmcu.c \
	stm32f4xx/stdperiph/src/stm32f4xx_dcmi.c \
	stm32f4xx/stdperiph/src/stm32f4xx_dma.c \
	stm32f4xx/stdperiph/src/stm32f4xx_exti.c \
	stm32f4xx/stdperiph/src/stm32f4xx_flash.c \
	stm32f4xx/stdperiph/src/stm32f4xx_fsmc.c \
	stm32f4xx/stdperiph/src/stm32f4xx_gpio.c \
	stm32f4xx/stdperiph/src/stm32f4xx_hash.c \
	stm32f4xx/stdperiph/src/stm32f4xx_hash_md5.c \
	stm32f4xx/stdperiph/src/stm32f4xx_hash_sha1.c \
	stm32f4xx/stdperiph/src/stm32f4xx_i2c.c \
	stm32f4xx/stdperiph/src/stm32f4xx_iwdg.c \
	stm32f4xx/stdperiph/src/stm32f4xx_misc.c \
	stm32f4xx/stdperiph/src/stm32f4xx_pwr.c \
	stm32f4xx/stdperiph/src/stm32f4xx_rcc.c \
	stm32f4xx/stdperiph/src/stm32f4xx_rng.c \
	stm32f4xx/stdperiph/src/stm32f4xx_rtc.c \
	stm32f4xx/stdperiph/src/stm32f4xx_sdio.c \
	stm32f4xx/stdperiph/src/stm32f4xx_spi.c \
	stm32f4xx/stdperiph/src/stm32f4xx_syscfg.c \
	stm32f4xx/stdperiph/src/stm32f4xx_tim.c \
	stm32f4xx/stdperiph/src/stm32f4xx_usart.c \
	stm32f4xx/stdperiph/src/stm32f4xx_wwdg.c

HDRS += \
	stm32f4xx/cmsis/device/st/stm32f4xx/include/stm32f4xx.h \
	stm32f4xx/cmsis/device/st/stm32f4xx/include/system_stm32f4xx.h \
	stm32f4xx/cmsis/include/arm_common_tables.h \
	stm32f4xx/cmsis/include/arm_math.h \
	stm32f4xx/cmsis/include/core_cm0.h \
	stm32f4xx/cmsis/include/core_cm3.h \
	stm32f4xx/cmsis/include/core_cm4.h \
	stm32f4xx/cmsis/include/core_cm4_simd.h \
	stm32f4xx/cmsis/include/core_cmFunc.h \
	stm32f4xx/cmsis/include/core_cmInstr.h \
	stm32f4xx/stdperiph/inc/misc.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_adc.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_can.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_conf.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_crc.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_cryp.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_dac.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_dbgmcu.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_dcmi.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_dma.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_exti.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_flash.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_fsmc.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_gpio.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_hash.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_i2c.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_iwdg.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_pwr.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_rcc.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_rng.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_rtc.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_sdio.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_spi.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_syscfg.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_tim.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_usart.h \
	stm32f4xx/stdperiph/inc/stm32f4xx_wwdg.h
