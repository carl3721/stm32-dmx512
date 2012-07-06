/*
 * cppdeclarations.cpp
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

/* This file holds declarations needed to program in c++. */

#include <stdlib.h>

void * operator new(size_t size) {
    return malloc(size);
}

void operator delete(void * ptr) {
    if (ptr) {
        free(ptr);
    }
}

void * operator new[](size_t size) {
    return malloc(size);
}

void operator delete[](void * ptr) {
    if (ptr) {
        free(ptr);
    }
}

__extension__ typedef int __guard __attribute__((mode(__DI__)));

extern "C" int __cxa_guard_acquire(__guard *g) {
    return !*(char *) (g);
}

extern "C" void __cxa_guard_release(__guard *g) {
    *(char *) g = 1;
}

extern "C" void __cxa_guard_abort(__guard *) {
}

extern "C" void __cxa_pure_virtual(void) {
}
