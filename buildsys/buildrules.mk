# buildrules.mk
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

# This file holds the various build rules.

# change the project name based on the type of build
export PROJECT := $(PROJECT_NAME)

# build dir is dependent on the mode
BUILDDIR := $(MODE)

# output artefacts
ELF := $(BUILDDIR)/$(PROJECT).elf
MAP := $(BUILDDIR)/$(PROJECT).map
HEX := $(BUILDDIR)/$(PROJECT).hex
LST := $(BUILDDIR)/$(PROJECT).lst

# define the tools to be used for producing the binaries 
CC := $(TOOLSDIR)arm-none-eabi-gcc
OC := $(TOOLSDIR)arm-none-eabi-objcopy
OD := $(TOOLSDIR)arm-none-eabi-objdump
OS := $(TOOLSDIR)arm-none-eabi-size
OPENOCD := $(OOCDDIR)openocd

# doxygen executable
DOXYGEN := doxygen

# dependency generation
DEPGEN = -MMD -MP -MF"$(@:%.o=%.d)"

# release specific tool settings
ifeq ($(MODE),release)
OPTIMISATION = 3
COMMONFLAGS  = -Wall -O$(OPTIMISATION) -c $(ARCH) $(DEPGEN) -DRELEASE_MODE -ffunction-sections -fdata-sections -fno-exceptions -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fno-builtin
CFLAGS       = $(COMMONFLAGS) -std=gnu99
CPPFLAGS     = $(COMMONFLAGS) -fno-rtti
ASFLAGS      = -Wall -O$(OPTIMISATION) -c $(ARCH)
LFLAGS       = -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -fno-exceptions -Wl,-Map,$(MAP) $(ARCH)
ifeq ($(BOARD),p103)
LDSCRIPT     = link-p103.ld
endif
ifeq ($(BOARD),p407)
LDSCRIPT     = link-p407.ld
endif
endif

# debug specific tool settings
ifeq ($(MODE),debug)
OPTIMISATION = 0
COMMONFLAGS  = -Wall -O$(OPTIMISATION) -c $(ARCH) -g3 -gdwarf-2 $(DEPGEN) -DDEBUG_MODE -ffunction-sections -fdata-sections -fno-exceptions -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fno-builtin
CFLAGS       = $(COMMONFLAGS) -std=gnu99
CPPFLAGS     = $(COMMONFLAGS) -fno-rtti
ASFLAGS      = -Wall -O$(OPTIMISATION) -c $(ARCH) -g3 -gdwarf-2
LFLAGS       = -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -fno-exceptions -Wl,-Map,$(MAP) $(ARCH)
ifeq ($(BOARD),p103)
LDSCRIPT    ?= link_debug-p103.ld
endif
ifeq ($(BOARD),p407)
LDSCRIPT     = link_debug-p407.ld
endif
endif

# common tool settings
OCFLAGS := -O ihex
ODFLAGS := -h -S
OSFLAGS := 

# include folders
INCLUDE := $(patsubst %, -I %, $(SUBDIRS))

# srcs and hdrs variables will be expanded by includes of module.mk
# they are exported to allow doxygen to use them via the doxyfile
export SRCS :=
export HDRS :=

# include module.mk for all subdirs
include $(patsubst %, %/module.mk, $(SUBDIRS))

# list of object files
OBJS := $(patsubst %.c, $(BUILDDIR)/%.c.o, $(filter %.c, $(SRCS))) \
	$(patsubst %.cpp, $(BUILDDIR)/%.cpp.o, $(filter %.cpp, $(SRCS))) \
	$(patsubst %.s, $(BUILDDIR)/%.s.o, $(filter %.s, $(SRCS)))

# list of dependency files
DEPS := $(patsubst %.c, $(BUILDDIR)/%.c.d, $(filter %.c, $(SRCS))) \
	$(patsubst %.cpp, $(BUILDDIR)/%.cpp.d, $(filter %.cpp, $(SRCS)))

# doc dir
export DOCDIR := $(BUILDDIR)/doc

# list of directories needed by the build
FOLDERS := $(patsubst %, $(BUILDDIR)/%, $(SUBDIRS)) $(DOCDIR)

# command line arguments for openocd
OOCDARGS = $(patsubst %, -f %, $(OOCDCFGS)) $(OOCDCMDS)

# use default doxy file if none given
ifeq ($(DOXYFILE),)
DOXYFILE += doxyfile
endif

# phony targets
.PHONY: clean size all doc flash serialflash info gdb flashgdb

# the default target is 'all' 
all: $(HEX) size

# info target
info:
	@echo 'Build information'
	@echo 'Project name   : $(PROJECT_NAME)'
	@echo 'Build mode     : $(MODE)'
	@echo 'Target board   : $(BOARD)'
	@echo 'Linker script  : $(LDSCRIPT)'
	@echo 'Subdirectories : $(SUBDIRS)'
	@echo 'Sources        : $(SRCS)'
	@echo 'Headers        : $(HDRS)'
	@echo 'Objects        : $(OBJS)'
	@echo 'Dependencies   : $(DEPS)'

# rule for creating the hex file
$(HEX): $(ELF)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linux GNU Create Flash Image'
	$(OC) $(OCFLAGS) $(ELF) "$(HEX)"
	@echo 'Finished building: $@'
	@echo ' '

# rule for creating the listing file
$(LST): $(ELF)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linux GNU Create Listing'
	$(OD) $(ODFLAGS) $(ELF) > "$(LST)"
	@echo 'Finished building: $@'
	@echo ' '

# rule for creating the elf file (linking)
$(ELF): $(LDSCRIPT) $(OBJS) $(USER_OBJ)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linux GCC C Linker'
	$(CC) -T "$(LDSCRIPT)" $(LFLAGS) -o "$(ELF)" $(OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# target for printing the sizes
size: $(ELF)
	@echo 'Invoking: ARM Linux GNU Print Size'
	$(OS) $(OSFLAGS) $(ELF)
	@echo 'Finished building: $@'
	@echo ' '

# rule for deleting the output folder structure
clean:
	@echo 'Removing the output folder structure.'
	@echo 'Invoking: rm -rvf'
	rm -rvf $(MODE)
	@echo 'Finished removing output folder structure'
	@echo ' '

# rule for initializing the output folder structure
$(FOLDERS):
	@echo 'Initializing the output folder structure.'
	@echo 'Invoking: mkdir -p'
	mkdir -p $(FOLDERS)
	@echo 'Finished initializing the output folder structure.'
	@echo ' '

# rule to generate the documentation
doc: | $(FOLDERS)
	@echo 'Generating documentation'
	@echo 'Invoking: $(DOXYGEN) $(DOXYFILE)'
	$(DOXYGEN) $(DOXYFILE)
	@echo 'Finished generating the documentation'
	@echo ' '

# rule to flash the hex file with openocd
flash: $(HEX)
	@echo 'Flashing hex file to microcontroller.'
	@echo 'Invoking: $(OPENOCD) $(OOCDARGS)'
	$(OPENOCD) $(OOCDARGS)
	@echo 'Finished flashing hex file to microcontroller.'
	@echo ' '

# include the dependencies
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(DEPS)),)
-include $(DEPS)
endif
endif

# pattern rule for compiling .c files
$(BUILDDIR)/%.c.o : %.c | $(FOLDERS)
	@echo 'Compiling file: $<'
	@echo 'Invoking: ARM Linux GCC C Compiler'
	$(CC) $(INCLUDE) $(EXT_DEFINES) $(CFLAGS) -o "$@" "$<"
	@echo 'Finished compiling: $<'
	@echo ' '

# pattern rule for compiling .cpp files
$(BUILDDIR)/%.cpp.o : %.cpp | $(FOLDERS)
	@echo 'Compiling file: $<'
	@echo 'Invoking: ARM Linux GCC C++ Compiler'
	$(CC) $(INCLUDE) $(EXT_DEFINES) $(CPPFLAGS) -o "$@" "$<"
	@echo 'Finished compiling: $<'
	@echo ' '

# pattern rule for assembling .s files
$(BUILDDIR)/%.s.o : %.s | $(FOLDERS)
	@echo 'Assembling file: $<'
	@echo 'Invoking: ARM Linux GCC Assembler'
	$(CC) $(ASFLAGS) -o "$@" "$<"
	@echo 'Finished assembling: $<'
