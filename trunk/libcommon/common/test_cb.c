/*
 * test_cb.c
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

/* This is a very simple test to check if the circular buffer is working. */

// compile this test with: 'gcc circularbuffer_u8.c test_cb.c -o test_cb'
// execute it on the linux command line with: './test_cb'

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "circularbuffer_u8.h"

int main (int argc, char** argv) {

	printf("Tester program for the circular buffer.\n");
        printf("The circular buffer is 16 bytes in size.\n");

	volatile uint8_t a[16];

	circularbuffer_u8_s b;

	circularbuffer_u8_init(&b, 16, a);

	printf("Init done.\n");

	printf("Empty check: %d\n", circularbuffer_u8_empty(&b));
	printf("Full check: %d\n", circularbuffer_u8_full(&b));

	printf("Push check (T)\n");

	circularbuffer_u8_push(&b, 'T');

	printf("Empty check: %d\n", circularbuffer_u8_empty(&b));
	printf("Full check: %d\n", circularbuffer_u8_full(&b));

	printf("Pop check (should be T): %c\n", circularbuffer_u8_pop(&b));	

	printf("Empty check: %d\n", circularbuffer_u8_empty(&b));
	printf("Full check: %d\n", circularbuffer_u8_full(&b));

	printf("Fill complete check 0-F (HEX)\n");
	
	circularbuffer_u8_push(&b, '0');
	circularbuffer_u8_push(&b, '1');
	circularbuffer_u8_push(&b, '2');
	circularbuffer_u8_push(&b, '3');
	circularbuffer_u8_push(&b, '4');
	circularbuffer_u8_push(&b, '5');
	circularbuffer_u8_push(&b, '6');
	circularbuffer_u8_push(&b, '7');
	circularbuffer_u8_push(&b, '8');
	circularbuffer_u8_push(&b, '9');
	circularbuffer_u8_push(&b, 'A');
	circularbuffer_u8_push(&b, 'B');
	circularbuffer_u8_push(&b, 'C');
	circularbuffer_u8_push(&b, 'D');
	circularbuffer_u8_push(&b, 'E');
	circularbuffer_u8_push(&b, 'F');

	printf("Empty check: %d\n", circularbuffer_u8_empty(&b));
	printf("Full check: %d\n", circularbuffer_u8_full(&b));

	printf("Pop check (should be 0): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 1): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 2): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 3): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 4): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 5): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 6): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 7): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 8): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be 9): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be A): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be B): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be C): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be D): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be E): %c\n", circularbuffer_u8_pop(&b));	
	printf("Pop check (should be F): %c\n", circularbuffer_u8_pop(&b));	

	printf("Empty check: %d\n", circularbuffer_u8_empty(&b));
	printf("Full check: %d\n", circularbuffer_u8_full(&b));

	return EXIT_SUCCESS;
}
