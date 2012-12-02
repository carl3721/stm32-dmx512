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

#SRCS += stm32f10x/cmsis/device/st/stm32f10x/src/armgcc/startup_stm32f10x_cl.s
#SRCS += stm32f10x/cmsis/device/st/stm32f10x/src/armgcc/startup_stm32f10x_hd.s
#SRCS += stm32f10x/cmsis/device/st/stm32f10x/src/armgcc/startup_stm32f10x_hd_vl.s
#SRCS += stm32f10x/cmsis/device/st/stm32f10x/src/armgcc/startup_stm32f10x_ld.s
#SRCS += stm32f10x/cmsis/device/st/stm32f10x/src/armgcc/startup_stm32f10x_ld_vl.s
ifeq ($(CHIP),stm32f103rb)
SRCS += stm32f10x/cmsis/device/st/stm32f10x/src/armgcc/startup_stm32f10x_md.s
endif
#SRCS += stm32f10x/cmsis/device/st/stm32f10x/src/armgcc/startup_stm32f10x_md_vl.s
#SRCS += stm32f10x/cmsis/device/st/stm32f10x/src/armgcc/startup_stm32f10x_xl.s

SRCS += \
	stm32f10x/cmsis/src/core_cm3.c \
	stm32f10x/stdperiph/src/misc.c \
	stm32f10x/stdperiph/src/stm32f10x_adc.c \
	stm32f10x/stdperiph/src/stm32f10x_bkp.c \
	stm32f10x/stdperiph/src/stm32f10x_can.c \
	stm32f10x/stdperiph/src/stm32f10x_cec.c \
	stm32f10x/stdperiph/src/stm32f10x_crc.c \
	stm32f10x/stdperiph/src/stm32f10x_dac.c \
	stm32f10x/stdperiph/src/stm32f10x_dbgmcu.c \
	stm32f10x/stdperiph/src/stm32f10x_dma.c \
	stm32f10x/stdperiph/src/stm32f10x_exti.c \
	stm32f10x/stdperiph/src/stm32f10x_flash.c \
	stm32f10x/stdperiph/src/stm32f10x_fsmc.c \
	stm32f10x/stdperiph/src/stm32f10x_gpio.c \
	stm32f10x/stdperiph/src/stm32f10x_i2c.c \
	stm32f10x/stdperiph/src/stm32f10x_iwdg.c \
	stm32f10x/stdperiph/src/stm32f10x_pwr.c \
	stm32f10x/stdperiph/src/stm32f10x_rcc.c \
	stm32f10x/stdperiph/src/stm32f10x_rtc.c \
	stm32f10x/stdperiph/src/stm32f10x_sdio.c \
	stm32f10x/stdperiph/src/stm32f10x_spi.c \
	stm32f10x/stdperiph/src/stm32f10x_tim.c \
	stm32f10x/stdperiph/src/stm32f10x_usart.c \
	stm32f10x/stdperiph/src/stm32f10x_wwdg.c

HDRS += \
	stm32f10x/cmsis/device/st/stm32f10x/include/stm32f10x.h \
	stm32f10x/cmsis/device/st/stm32f10x/include/system_stm32f10x.h \
	stm32f10x/cmsis/include/core_cm3.h \
	stm32f10x/stdperiph/inc/misc.h \
	stm32f10x/stdperiph/inc/stm32f10x_adc.h \
	stm32f10x/stdperiph/inc/stm32f10x_bkp.h \
	stm32f10x/stdperiph/inc/stm32f10x_can.h \
	stm32f10x/stdperiph/inc/stm32f10x_cec.h \
	stm32f10x/stdperiph/inc/stm32f10x_conf.h \
	stm32f10x/stdperiph/inc/stm32f10x_crc.h \
	stm32f10x/stdperiph/inc/stm32f10x_dac.h \
	stm32f10x/stdperiph/inc/stm32f10x_dbgmcu.h \
	stm32f10x/stdperiph/inc/stm32f10x_dma.h \
	stm32f10x/stdperiph/inc/stm32f10x_exti.h \
	stm32f10x/stdperiph/inc/stm32f10x_flash.h \
	stm32f10x/stdperiph/inc/stm32f10x_fsmc.h \
	stm32f10x/stdperiph/inc/stm32f10x_gpio.h \
	stm32f10x/stdperiph/inc/stm32f10x_i2c.h \
	stm32f10x/stdperiph/inc/stm32f10x_iwdg.h \
	stm32f10x/stdperiph/inc/stm32f10x_pwr.h \
	stm32f10x/stdperiph/inc/stm32f10x_rcc.h \
	stm32f10x/stdperiph/inc/stm32f10x_rtc.h \
	stm32f10x/stdperiph/inc/stm32f10x_sdio.h \
	stm32f10x/stdperiph/inc/stm32f10x_spi.h \
	stm32f10x/stdperiph/inc/stm32f10x_tim.h \
	stm32f10x/stdperiph/inc/stm32f10x_usart.h \
	stm32f10x/stdperiph/inc/stm32f10x_wwdg.h
