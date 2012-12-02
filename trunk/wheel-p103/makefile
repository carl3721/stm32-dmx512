# wheel-p103.mk
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

# The name of the project
PROJECT := wheel-p103

# List of all the library directories that should be included in the build.
# Each directory will be searched for a module.mk config file.
LIBS := stm32f10x common

# List of all the source directories that should be included in the build.
# Each directory will be searched for a module.mk config file.
SRCDIRS := src

# The compilation mode. Choose between "debug" and "release".
MODE ?= debug

# The target chip for the code. Currently supported chips are:
#   "stm32f103rb"
#   "stm32f407ig"
CHIP := stm32f103rb

# The directory holding the ARM toolchain.
# Please note that a trailing / must be included in this path
# 
# If the tools can be found on your $PATH, leave this variable empty
TOOLSDIR :=

# The directory holding the openocd binary.
# Please note that a trailing / must be included in this path
# 
# If openocd can be found on your $PATH, leave this variable empty
OOCDDIR :=

# The configuration files for openocd.
#
# Just list the files here prefixed by -f.
OOCDCFGS := -f "interface/olimex-arm-usb-ocd.cfg" -f "target/stm32f1x.cfg"

# The commands for openocd to execute.
#
# Each command must be specified in the -c "..." format.
OOCDCMDS = -c "init" \
	-c "reset halt" \
	-c "flash erase_sector 0 0 16" \
	-c "flash write_bank 0 $(BIN) 0" \
	-c "reset run" \
	-c "shutdown"

# Include the buildrules
include ../build/buildrules.mk
