# module.mk
# 
# STM32 DMX512
# Copyright (C) 2011 Erik Van Hamme, all rights reserved
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

SRCS += stmlib-stm32f10x/src/misc.c \
	stmlib-stm32f10x/src/stm32f10x_adc.c \
	stmlib-stm32f10x/src/stm32f10x_bkp.c \
	stmlib-stm32f10x/src/stm32f10x_can.c \
	stmlib-stm32f10x/src/stm32f10x_cec.c \
	stmlib-stm32f10x/src/stm32f10x_crc.c \
	stmlib-stm32f10x/src/stm32f10x_dac.c \
	stmlib-stm32f10x/src/stm32f10x_dbgmcu.c \
	stmlib-stm32f10x/src/stm32f10x_dma.c \
	stmlib-stm32f10x/src/stm32f10x_exti.c \
	stmlib-stm32f10x/src/stm32f10x_flash.c \
	stmlib-stm32f10x/src/stm32f10x_fsmc.c \
	stmlib-stm32f10x/src/stm32f10x_gpio.c \
	stmlib-stm32f10x/src/stm32f10x_i2c.c \
	stmlib-stm32f10x/src/stm32f10x_iwdg.c \
	stmlib-stm32f10x/src/stm32f10x_pwr.c \
	stmlib-stm32f10x/src/stm32f10x_rcc.c \
	stmlib-stm32f10x/src/stm32f10x_rtc.c \
	stmlib-stm32f10x/src/stm32f10x_sdio.c \
	stmlib-stm32f10x/src/stm32f10x_spi.c \
	stmlib-stm32f10x/src/stm32f10x_tim.c \
	stmlib-stm32f10x/src/stm32f10x_usart.c \
	stmlib-stm32f10x/src/stm32f10x_wwdg.c  
