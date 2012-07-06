# wheel-p103.mk
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

# The name of the project
export PROJECT_NAME := wheel-p103

# List of all the subdirectories that should be included in the build.
# Each directory will be searched for a recursive makefile.
SUBDIRS := \
	wheel-p103 \
	cmsis-stm32f10x \
	stmlib-stm32f10x \
	stmlib-stm32f10x/inc \
	stmlib-stm32f10x/src \
	common

# The compilation mode. Choose between "debug" and "release".
MODE ?= debug

# The target board that the code should run on. Choose between "p103" and "p407"
BOARD ?= p103

# The doxygen configuration file
DOXYFILE := doxyfile

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
# Just list the files here separated by spaces.
#OOCDCFGS := interface/ngxtech.cfg target/stm32f1x.cfg
OOCDCFGS := interface/olimex-arm-usb-ocd.cfg target/stm32f1x.cfg

# The commands for openocd to execute.
#
# Each command must be specified in the -c "..." format.
OOCDCMDS = -c "init" -c "reset halt" -c "stm32f1x mass_erase 0" -c "flash write_image $(HEX)" -c "reset run" -c "shutdown"

# The target architecture.
ARCH := -mcpu=cortex-m3 -mthumb

# List of external defines that will be passed to the compiler.
#
# Add the defines here including the -D tag. For example:
# -DDEFINE_VALUE=VALUE
EXT_DEFINES := -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

# Controls the internal docs handling for doxygen.
#
# The INTERNAL_DOCS tag determines if documentation
# that is typed after a @internal command is included in the docs. If the tag is set
# to NO (the default) then the documentation will be excluded.
# Set it to YES to include the internal documentation.
export INTERNAL_DOCS := NO

# Controls the behavior of undocumented methods in doxygen. 
#
# If the HIDE_UNDOC_MEMBERS tag is set to YES, Doxygen will hide all
# undocumented members of documented classes, files or namespaces.
# If set to NO (the default) these members will be included in the
# various overviews, but no documentation section is generated.
# This option has no effect if EXTRACT_ALL is enabled.
export HIDE_UNDOC_MEMBERS := NO

# Include the buildrules
include buildrules.mk
