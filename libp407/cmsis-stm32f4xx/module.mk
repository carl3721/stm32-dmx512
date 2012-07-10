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
  cmsis-stm32f4xx/startup_stm32f4xx.c \
  cmsis-stm32f4xx/system_stm32f4xx.c

HDRS += \
  cmsis-stm32f4xx/core_cm4.h \
  cmsis-stm32f4xx/core_cm4_simd.h \
  cmsis-stm32f4xx/core_cmFunc.h \
  cmsis-stm32f4xx/core_cmInstr.h \
  cmsis-stm32f4xx/stm32f4xx.h \
  cmsis-stm32f4xx/system_stm32f4xx.h
