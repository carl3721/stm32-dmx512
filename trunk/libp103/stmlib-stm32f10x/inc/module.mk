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

HDRS += libp103/stmlib-stm32f10x/inc/stm32f10x_cec.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_bkp.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_exti.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_crc.h \
	libp103/stmlib-stm32f10x/inc/misc.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_adc.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_wwdg.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_pwr.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_flash.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_gpio.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_rtc.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_spi.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_tim.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_sdio.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_usart.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_fsmc.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_iwdg.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_dac.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_rcc.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_can.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_dma.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_dbgmcu.h \
	libp103/stmlib-stm32f10x/inc/stm32f10x_i2c.h 
