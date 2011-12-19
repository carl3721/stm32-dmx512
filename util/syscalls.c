/*
 * syscalls.c
 * 
 * STM32 DMX512
 * Copyright (C) 2011 Erik Van Hamme, all rights reserved
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

/* This file holds the implementation of newlib syscalls. */

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "stm32f10x.h"

#undef errno
extern int errno;
extern int _heap_start;

void abort(void) {
	/* Abort called */
	while (1)
		;
}

caddr_t _sbrk(int incr) {
	static unsigned char *heap_end = NULL;
	unsigned char *prev_heap;

	if (heap_end == NULL) {
		heap_end = (unsigned char *) &_heap_start;
	}
	
	unsigned char *stack_bottom = (unsigned char *) __get_MSP();
	
	if (heap_end + incr >  stack_bottom) {
		// _write (STDERR_FILENO, "Heap and stack collision\n", 25);
		// errno = ENOMEM;
		// return  (caddr_t) -1;
		abort();
	}
	
	prev_heap = heap_end;
	heap_end += incr;

	return (caddr_t) prev_heap;
}

int link(char *old, char *new) {
	return -1;
}

int _close(int file) {
	return -1;
}

int _fstat(int file, struct stat *st) {
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file) {
	return 1;
}

int _lseek(int file, int ptr, int dir) {
	return 0;
}

int _read(int file, char *ptr, int len) {
	return 0;
}

int _write(int file, char *ptr, int len) {
	return len;
}

// needed by __libc_init_array(), which is called in the startup code.
void _init() {
}

void _fini() {
}

/* ------------------------------- End Of File ------------------------------ */
