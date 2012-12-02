/*
 * main.h
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

/* This is the application main configuration file. Currently there are 
 * no specific configurations made, everything is on the default settings. */

/**
 * @brief We require 6 bytes for a set operation to be complete.
 */ 
#define PACKETIZER_LENGTH 6

/**
 * @brief A full packet should take less then 80 milliseconds to arrive.
 */
#define PACKETIZER_TIMEOUT_TICKS 2