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

HDRS += stmlib/inc/stm32f10x_cec.h \
	stmlib/inc/stm32f10x_bkp.h \
	stmlib/inc/stm32f10x_exti.h \
	stmlib/inc/stm32f10x_crc.h \
	stmlib/inc/misc.h \
	stmlib/inc/stm32f10x_adc.h \
	stmlib/inc/stm32f10x_wwdg.h \
	stmlib/inc/stm32f10x_pwr.h \
	stmlib/inc/stm32f10x_flash.h \
	stmlib/inc/stm32f10x_gpio.h \
	stmlib/inc/stm32f10x_rtc.h \
	stmlib/inc/stm32f10x_spi.h \
	stmlib/inc/stm32f10x_tim.h \
	stmlib/inc/stm32f10x_sdio.h \
	stmlib/inc/stm32f10x_usart.h \
	stmlib/inc/stm32f10x_fsmc.h \
	stmlib/inc/stm32f10x_iwdg.h \
	stmlib/inc/stm32f10x_dac.h \
	stmlib/inc/stm32f10x_rcc.h \
	stmlib/inc/stm32f10x_can.h \
	stmlib/inc/stm32f10x_dma.h \
	stmlib/inc/stm32f10x_dbgmcu.h \
	stmlib/inc/stm32f10x_i2c.h 
