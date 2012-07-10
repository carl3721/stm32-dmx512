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

SRCS += \
  stmlib-stm32f4xx/misc.c \
  stmlib-stm32f4xx/stm32f4xx_adc.c \
  stmlib-stm32f4xx/stm32f4xx_can.c \
  stmlib-stm32f4xx/stm32f4xx_crc.c \
  stmlib-stm32f4xx/stm32f4xx_cryp_aes.c \
  stmlib-stm32f4xx/stm32f4xx_cryp.c \
  stmlib-stm32f4xx/stm32f4xx_cryp_des.c \
  stmlib-stm32f4xx/stm32f4xx_cryp_tdes.c \
  stmlib-stm32f4xx/stm32f4xx_dac.c \
  stmlib-stm32f4xx/stm32f4xx_dbgmcu.c \
  stmlib-stm32f4xx/stm32f4xx_dcmi.c \
  stmlib-stm32f4xx/stm32f4xx_dma.c \
  stmlib-stm32f4xx/stm32f4xx_exti.c \
  stmlib-stm32f4xx/stm32f4xx_flash.c \
  stmlib-stm32f4xx/stm32f4xx_fsmc.c \
  stmlib-stm32f4xx/stm32f4xx_gpio.c \
  stmlib-stm32f4xx/stm32f4xx_hash.c \
  stmlib-stm32f4xx/stm32f4xx_hash_md5.c \
  stmlib-stm32f4xx/stm32f4xx_hash_sha1.c \
  stmlib-stm32f4xx/stm32f4xx_i2c.c \
  stmlib-stm32f4xx/stm32f4xx_iwdg.c \
  stmlib-stm32f4xx/stm32f4xx_pwr.c \
  stmlib-stm32f4xx/stm32f4xx_rcc.c \
  stmlib-stm32f4xx/stm32f4xx_rng.c \
  stmlib-stm32f4xx/stm32f4xx_rtc.c \
  stmlib-stm32f4xx/stm32f4xx_sdio.c \
  stmlib-stm32f4xx/stm32f4xx_spi.c \
  stmlib-stm32f4xx/stm32f4xx_syscfg.c \
  stmlib-stm32f4xx/stm32f4xx_tim.c \
  stmlib-stm32f4xx/stm32f4xx_usart.c \
  stmlib-stm32f4xx/stm32f4xx_wwdg.c

HDRS += \
  stmlib-stm32f4xx/misc.h \
  stmlib-stm32f4xx/stm32f4xx_adc.h \
  stmlib-stm32f4xx/stm32f4xx_can.h \
  stmlib-stm32f4xx/stm32f4xx_crc.h \
  stmlib-stm32f4xx/stm32f4xx_cryp.h \
  stmlib-stm32f4xx/stm32f4xx_dac.h \
  stmlib-stm32f4xx/stm32f4xx_dbgmcu.h \
  stmlib-stm32f4xx/stm32f4xx_dcmi.h \
  stmlib-stm32f4xx/stm32f4xx_dma.h \
  stmlib-stm32f4xx/stm32f4xx_exti.h \
  stmlib-stm32f4xx/stm32f4xx_flash.h \
  stmlib-stm32f4xx/stm32f4xx_fsmc.h \
  stmlib-stm32f4xx/stm32f4xx_gpio.h \
  stmlib-stm32f4xx/stm32f4xx_hash.h \
  stmlib-stm32f4xx/stm32f4xx_i2c.h \
  stmlib-stm32f4xx/stm32f4xx_iwdg.h \
  stmlib-stm32f4xx/stm32f4xx_pwr.h \
  stmlib-stm32f4xx/stm32f4xx_rcc.h \
  stmlib-stm32f4xx/stm32f4xx_rng.h \
  stmlib-stm32f4xx/stm32f4xx_rtc.h \
  stmlib-stm32f4xx/stm32f4xx_sdio.h \
  stmlib-stm32f4xx/stm32f4xx_spi.h \
  stmlib-stm32f4xx/stm32f4xx_syscfg.h \
  stmlib-stm32f4xx/stm32f4xx_tim.h \
  stmlib-stm32f4xx/stm32f4xx_usart.h \
  stmlib-stm32f4xx/stm32f4xx_wwdg.h
