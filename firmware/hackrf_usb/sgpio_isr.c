/*
 * Copyright 2012 Jared Boone
 * Copyright 2013 Benjamin Vernoux
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stdint.h>
#include "sgpio_isr.h"

#include <libopencm3/lpc43xx/sgpio.h>
#include "hackrf_core.h"

#include "usb_bulk_buffer.h"

void sgpio_isr_rx() {
	SGPIO_CLR_STATUS_1 = (1 << SGPIO_SLICE_A);

	uint32_t* const p = (uint32_t*)&usb_bulk_buffer[usb_bulk_buffer_offset];
	__asm__(
		"ldr r0, [%[SGPIO_REG_SS], #44]\n\t"
		"str r0, [%[p], #0]\n\t"
		"ldr r0, [%[SGPIO_REG_SS], #20]\n\t"
		"str r0, [%[p], #4]\n\t"
		"ldr r0, [%[SGPIO_REG_SS], #40]\n\t"
		"str r0, [%[p], #8]\n\t"
		"ldr r0, [%[SGPIO_REG_SS], #8]\n\t"
		"str r0, [%[p], #12]\n\t"
		"ldr r0, [%[SGPIO_REG_SS], #36]\n\t"
		"str r0, [%[p], #16]\n\t"
		"ldr r0, [%[SGPIO_REG_SS], #16]\n\t"
		"str r0, [%[p], #20]\n\t"
		"ldr r0, [%[SGPIO_REG_SS], #32]\n\t"
		"str r0, [%[p], #24]\n\t"
		"ldr r0, [%[SGPIO_REG_SS], #0]\n\t"
		"str r0, [%[p], #28]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #24]\n\t"
        "str r0, [%[p], #32]\n\t"
        "ldr r0, [%[SGPIO_REG_SS], #52]\n\t"
        "str r0, [%[p], #36]\n\t"
		:
		: [SGPIO_REG_SS] "l" (SGPIO_PORT_BASE + 0x100),
		  [p] "l" (p)
		: "r0"
	);

    /*
    // D9 - D2
    p[0] = SGPIO_REG_SS(SGPIO_SLICE_L);
    p[1] = SGPIO_REG_SS(SGPIO_SLICE_F);
    p[2] = SGPIO_REG_SS(SGPIO_SLICE_K);
    p[3] = SGPIO_REG_SS(SGPIO_SLICE_C);
    p[4] = SGPIO_REG_SS(SGPIO_SLICE_J);
    p[5] = SGPIO_REG_SS(SGPIO_SLICE_E);
    p[6] = SGPIO_REG_SS(SGPIO_SLICE_I);
    p[7] = SGPIO_REG_SS(SGPIO_SLICE_A);
    // D1
    p[8] = SGPIO_REG_SS(SGPIO_SLICE_G);
    // D0
    p[9] = SGPIO_REG_SS(SGPIO_SLICE_N);
    */

    usb_bulk_buffer_offset += 10*4;
    if (usb_bulk_buffer_offset > usb_bulk_buffer_size-10*4) {
        usb_bulk_buffer_offset = 0;
    }
}

