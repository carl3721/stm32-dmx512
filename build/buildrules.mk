# buildrules.mk
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

# This file holds the various build rules.

# lib copy method (ln -s or cp -r)
LIBCOPY ?= ln -s -f

# arch is dependent on the chip
ifeq ($(CHIP),stm32f407ig)
ARCH := -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16
EXT_DEFINES += -DSTM32F4XX -DUSE_STDPERIPH_DRIVER
endif
ifeq ($(CHIP),stm32f103rb)
ARCH := -mcpu=cortex-m3 -mthumb
EXT_DEFINES += -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER
endif

# build dir is dependent on the mode
BUILDDIR := $(MODE)

# output artefacts
ELF := $(BUILDDIR)/$(PROJECT).elf
MAP := $(BUILDDIR)/$(PROJECT).map
BIN := $(BUILDDIR)/$(PROJECT).bin
LST := $(BUILDDIR)/$(PROJECT).lst

# define the tools to be used for producing the binaries 
CC := $(TOOLSDIR)arm-none-eabi-gcc
OC := $(TOOLSDIR)arm-none-eabi-objcopy
OD := $(TOOLSDIR)arm-none-eabi-objdump
OS := $(TOOLSDIR)arm-none-eabi-size
OPENOCD := $(OOCDDIR)openocd

# dependency generation
DEPGEN = -MMD -MP -MF"$(@:%.o=%.d)"

# release specific tool settings
ifeq ($(MODE),release)
OPTIMISATION = 3
COMMONFLAGS  = -Wall -O$(OPTIMISATION) -c $(ARCH) $(DEPGEN) -DNDEBUG -ffunction-sections -fdata-sections -fno-exceptions -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fno-builtin
CFLAGS       = $(COMMONFLAGS) -std=gnu99
CPPFLAGS     = $(COMMONFLAGS) -fno-rtti
ASFLAGS      = -Wall -O$(OPTIMISATION) -c $(ARCH)
LFLAGS       = -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -fno-exceptions -Wl,-Map,$(MAP) $(ARCH)
endif

# debug specific tool settings
ifeq ($(MODE),debug)
OPTIMISATION = 0
COMMONFLAGS  = -Wall -O$(OPTIMISATION) -c $(ARCH) -g3 -gdwarf-2 $(DEPGEN) -DDEBUG_MODE -ffunction-sections -fdata-sections -fno-exceptions -Wa,-adhlns="$@.lst" -c -fmessage-length=0 -fno-builtin
CFLAGS       = $(COMMONFLAGS) -std=gnu99
CPPFLAGS     = $(COMMONFLAGS) -fno-rtti
ASFLAGS      = -Wall -O$(OPTIMISATION) -c $(ARCH) -g3 -gdwarf-2
LFLAGS       = -nostartfiles -nodefaultlibs -nostdlib -Xlinker --gc-sections -fno-exceptions -Wl,-Map,$(MAP) $(ARCH)
endif

# common tool settings
OCFLAGS := -O binary
ODFLAGS := -h -S
OSFLAGS := 

# select linker script
LDSCRIPT := ../build/$(CHIP)-$(MODE).ld

# srcs and hdrs variables will be expanded by includes of module.mk
SRCS :=
HDRS :=

# include module.mk for all subdirs
MODULES := $(patsubst %, ../lib/%/module.mk, $(LIBS)) $(patsubst %, %/module.mk, $(SRCDIRS))
include $(MODULES)

# list of object files
OBJS := $(patsubst %.c, $(BUILDDIR)/%.c.o, $(filter %.c, $(SRCS))) \
	$(patsubst %.cpp, $(BUILDDIR)/%.cpp.o, $(filter %.cpp, $(SRCS))) \
	$(patsubst %.s, $(BUILDDIR)/%.s.o, $(filter %.s, $(SRCS)))

# list of dependency files
DEPS := $(patsubst %.c, $(BUILDDIR)/%.c.d, $(filter %.c, $(SRCS))) \
	$(patsubst %.cpp, $(BUILDDIR)/%.cpp.d, $(filter %.cpp, $(SRCS)))

# list of output folders
FOLDERS := $(sort $(dir $(OBJS)))

# compile time includes
INCLUDES := $(sort $(dir $(HDRS)))
INCLUDE := $(patsubst %,-I %, $(INCLUDES))

# openocd command line arguments
OOCDARGS := $(OOCDCFGS) $(OOCDCMDS)

# target folders for the library code
LIBFOLDERS := $(patsubst %, ../lib/%, $(LIBS))

# phonies
.PHONY: clean size flash oocd

# default target
all : $(FOLDERS) $(LIBS) $(BIN) size

$(LIBS) :
	$(LIBCOPY) $(LIBFOLDERS) .

clean:
	rm -rf $(BUILDDIR) $(LIBS)

$(FOLDERS):
	@echo 'Creating output folders:'
	mkdir -p $(FOLDERS)
	@echo ' '
	@echo ' '

$(ELF): $(LDSCRIPT) $(OBJS)
	@echo 'Building target ELF: $@'
	$(CC) -T "$(LDSCRIPT)" $(LFLAGS) -o "$(ELF)" $(OBJS)
	@echo ' '
	@echo ' '

$(BIN): $(ELF)
	@echo 'Building target HEX: $@'
	$(OC) $(OCFLAGS) $(ELF) "$(BIN)"
	@echo ' '
	@echo ' '

size: $(ELF)
	@echo 'Invoking: ARM GCC Linux GNU Print Size'
	$(OS) $(OSFLAGS) $(ELF)
	@echo ' '
	@echo ' '

flash: $(BIN)
	@echo 'Flashing bin file to microcontroller.'
	$(OPENOCD) $(OOCDARGS)
	@echo ' '
	@echo ' '

oocd: 
	@echo 'Staring openocd.'
	$(OPENOCD) $(OOCDCFGS)
	@echo ' '
	@echo ' '

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(DEPS)),)
-include $(DEPS)
endif
endif

$(BUILDDIR)/%.s.o : %.s
	@echo 'Assembling : $<'
	$(CC) $(ASFLAGS) -o "$@" "$<"
	@echo ' '
	@echo ' '

$(BUILDDIR)/%.c.o : %.c
	@echo 'Compiling  : $<'
	$(CC) $(INCLUDE) $(EXT_DEFINES) $(CFLAGS) -o "$@" "$<"
	@echo ' '
	@echo ' '

$(BUILDDIR)/%.cpp.o : %.cpp
	@echo 'Compiling  : $<'
	$(CC) $(INCLUDE) $(EXT_DEFINES) $(CPPFLAGS) -o "$@" "$<"
	@echo ' '
	@echo ' '



