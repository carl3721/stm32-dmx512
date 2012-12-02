/*
 * stm32f407ig-debug.ld
 * 
 * STM32 DMX512
 * Copyright (C) 2012 Erik Van Hamme, all rights reserved
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* This is the linker script for the stm32f407ig cpu in debug mode. */

OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
GROUP(-lgcc -lc -lm)
ENTRY(Reset_Handler)

/* Memory map for STM32F407IG */ 
MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 960K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_eram = ORIGIN( ram ) + LENGTH( ram );

/* Section Definitions */ 
SECTIONS {
	.text : {
		KEEP(*(.isr_vector .isr_vector.*))
		*(.text .text.* .gnu.linkonce.t.*)
		*(.glue_7t) *(.glue_7)
		*(.rodata .rodata* .gnu.linkonce.r.*)

		/* C++ Static constructors/destructors (elf)  */
		. = ALIGN(4);
		KEEP(*(.init))

		. = ALIGN(4);
		__preinit_array_start = .;
		KEEP (*(.preinit_array))
		__preinit_array_end = .;

		. = ALIGN(4);
		__init_array_start = .;
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array))
		__init_array_end = .;

		. = ALIGN(0x4);
		KEEP (*crtbegin.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*crtend.o(.ctors))

		. = ALIGN(4);
		KEEP(*(.fini))

		. = ALIGN(4);
		__fini_array_start = .;
		KEEP (*(.fini_array))
		KEEP (*(SORT(.fini_array.*)))
		__fini_array_end = .;

		KEEP (*crtbegin.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*crtend.o(.dtors))

	} > rom

	.ARM.extab : {
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > rom

	.ARM.exidx : {
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > rom

	. = ALIGN(4);
	_etext = .;
	_sidata = .;

	.data : AT (_etext) {
		_sdata = .;
		*(.data .data.*)
		. = ALIGN(4);
		_edata = . ;
	} > ram

	.bss (NOLOAD) : {
		_sbss = . ;
		*(.bss .bss.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = . ;
	} > ram

	_heap_start = .;

	/* 
	 * According to http://wwwold.ece.utep.edu/courses/web3376/Stack.html
	 * "The SP should [...] be initialized [...] to the last available address in RAM +1."
	 */
	_estack = _eram;
	
	
	/* after that it's only debugging information. */
	
	.stab 0 (NOLOAD) : { *(.stab) }
	.stabstr 0 (NOLOAD) : { *(.stabstr) }
	/* DWARF debug sections.
	* Symbols in the DWARF debugging sections are relative to the beginning
	* of the section so we begin them at 0.  */
	/* DWARF 1 */
	.debug          0 : { *(.debug) }
	.line           0 : { *(.line) }
	/* GNU DWARF 1 extensions */
	.debug_srcinfo  0 : { *(.debug_srcinfo) }
	.debug_sfnames  0 : { *(.debug_sfnames) }
	/* DWARF 1.1 and DWARF 2 */
	.debug_aranges  0 : { *(.debug_aranges) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	/* DWARF 2 */
	.debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
	.debug_abbrev   0 : { *(.debug_abbrev) }
	.debug_line     0 : { *(.debug_line) }
	.debug_frame    0 : { *(.debug_frame) }
	.debug_str      0 : { *(.debug_str) }
	.debug_loc      0 : { *(.debug_loc) }
	.debug_macinfo  0 : { *(.debug_macinfo) }
	/* DWARF 2.1 */
	.debug_ranges   0 : { *(.debug_ranges) }
	/* SGI/MIPS DWARF 2 extensions */
	.debug_weaknames 0 : { *(.debug_weaknames) }
	.debug_funcnames 0 : { *(.debug_funcnames) }
	.debug_typenames 0 : { *(.debug_typenames) }
	.debug_varnames  0 : { *(.debug_varnames) }

	.note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
	.ARM.attributes 0 : { KEEP (*(.ARM.attributes)) }
	/DISCARD/ : { *(.note.GNU-stack) }
}
