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

#include "usb_api_register.h"
#include <libopencm3/lpc43xx/gpio.h>
#include "hackrf_core.h"

#include <usb_queue.h>
#include "rf_path.h"

#include <stddef.h>
#include <stdint.h>

#include "sgpio.h"
#include "sgpio_isr.h"
#include "mcp4022.h"

/*
usb_request_status_t usb_vendor_request_write_spiflash(
	usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage)
{
	uint32_t addr = 0;
	uint16_t len = 0;

	//FIXME This should refuse to run if executing from SPI flash.

	if (stage == USB_TRANSFER_STAGE_SETUP) {
		addr = (endpoint->setup.value << 16) | endpoint->setup.index;
		len = endpoint->setup.length;
		if ((len > W25Q80BV_PAGE_LEN) || (addr > W25Q80BV_NUM_BYTES)
				|| ((addr + len) > W25Q80BV_NUM_BYTES)) {
			return USB_REQUEST_STATUS_STALL;
		} else {
			usb_transfer_schedule_block(endpoint->out, &spiflash_buffer[0], len,
						    NULL, NULL);
			w25q80bv_setup();
			return USB_REQUEST_STATUS_OK;
		}
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		addr = (endpoint->setup.value << 16) | endpoint->setup.index;
		len = endpoint->setup.length;
		if ((len > W25Q80BV_PAGE_LEN) || (addr > W25Q80BV_NUM_BYTES)
				|| ((addr + len) > W25Q80BV_NUM_BYTES)) {
			return USB_REQUEST_STATUS_STALL;
		} else {
			w25q80bv_program(addr, len, &spiflash_buffer[0]);
			usb_transfer_schedule_ack(endpoint->in);
			//FIXME probably should undo w25q80bv_setup()
			return USB_REQUEST_STATUS_OK;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}
*/

usb_request_status_t usb_vendor_request_write_adf4158(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = (endpoint->setup.value<<16)|(endpoint->setup.index);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        adf4158_write_register(data);
        usb_transfer_schedule_ack(endpoint->in);
        return USB_REQUEST_STATUS_OK;
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_read_adf4158(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        const uint32_t value = adf4158_read_register();
        endpoint->buffer[0] = (value >> 8*3) & 0xFF;
        endpoint->buffer[1] = (value >> 8*2) & 0xFF;
        endpoint->buffer[2] = (value >> 8*1) & 0xFF;
        endpoint->buffer[3] = (value >> 8*0) & 0xFF;
        usb_transfer_schedule_block(endpoint->in, &endpoint->buffer, 4,
                        NULL, NULL);
        usb_transfer_schedule_ack(endpoint->out);
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_mcp(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = endpoint->setup.index;

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        if (data > MCP_MAX_VALUE) {
			return USB_REQUEST_STATUS_STALL;
        } else {
            mcp_set(data);
            usb_transfer_schedule_ack(endpoint->in);
        }
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_gpio(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = (endpoint->setup.value<<16)|(endpoint->setup.index);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        if (data & BIT0) {
            enable_adf4158();
        }
        if (data & BIT1) {
            enable_adc();
        }
        if (data & BIT2) {
            enable_pa();
        }
        if (data & BIT3) {
            enable_mixer();
        }
        if (data & BIT4) {
            gpio_set(PORT_LED1_3, PIN_LED1);
        }
        if (data & BIT5) {
            sgpio_test_interface();
        }
        usb_transfer_schedule_ack(endpoint->in);
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_clear_gpio(
    usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    uint32_t data = (endpoint->setup.value<<16)|(endpoint->setup.index);

	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        if (data & BIT0) {
            disable_adf4158();
        }
        if (data & BIT1) {
            disable_adc();
        }
        if (data & BIT2) {
            disable_pa();
        }
        if (data & BIT3) {
            disable_mixer();
        }
        if (data & BIT4) {
            gpio_clear(PORT_LED1_3, PIN_LED1);
        }
        usb_transfer_schedule_ack(endpoint->in);
	}
    return USB_REQUEST_STATUS_OK;
}


usb_request_status_t usb_vendor_request_set_clock(
    usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
    // Configures ADC clock
	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        if (endpoint->setup.value > 4095) {
            return USB_REQUEST_STATUS_STALL;
        }
        sgpio_configure_clock(endpoint->setup.value, endpoint->setup.index);
        usb_transfer_schedule_ack(endpoint->in);
	}
    return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_trigger(
    usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage
) {
	if( stage == USB_TRANSFER_STAGE_SETUP ) {
        //set_trigger(endpoint->setup.value, endpoint->setup.index);
        usb_transfer_schedule_ack(endpoint->in);
	}
    return USB_REQUEST_STATUS_OK;
}
