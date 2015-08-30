/*
Copyright (c) 2012, Jared Boone <jared@sharebrained.com>
Copyright (c) 2013, Benjamin Vernoux <titanmkd@gmail.com>
Copyright (c) 2013, Michael Ossmann <mike@ossmann.com>

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the 
	documentation and/or other materials provided with the distribution.
    Neither the name of Great Scott Gadgets nor the names of its contributors may be used to endorse or promote products derived from this software
	without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "hackrf.h"

#include <stdlib.h>

#include <libusb.h>
#include <string.h>
#include <pthread.h>
#include <math.h>

#ifndef bool
typedef int bool;
#define true 1
#define false 0
#endif

//ADF4158 reference oscillator frequency
#define FPD_FREQ 30000000

#ifdef HACKRF_BIG_ENDIAN
#define TO_LE(x) __builtin_bswap32(x)
#define TO_LE64(x) __builtin_bswap64(x)
#else
#define TO_LE(x) x
#define TO_LE64(x) x
#endif

#define GPIO_ADF (1 << 0)
#define GPIO_ADC (1 << 1)
#define GPIO_PA  (1 << 2)
#define GPIO_MIXER (1 << 3)
#define GPIO_LED (1 << 4)

// TODO: Factor this into a shared #include so that firmware can use
// the same values.
typedef enum {
	HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE = 1,
	HACKRF_VENDOR_REQUEST_ADF4158_WRITE = 2,
	HACKRF_VENDOR_REQUEST_ADF4158READ = 3,
	HACKRF_VENDOR_REQUEST_SET_GPIO = 4,
	HACKRF_VENDOR_REQUEST_CLEAR_GPIO = 5,
	HACKRF_VENDOR_REQUEST_SET_MCP = 6,
	HACKRF_VENDOR_REQUEST_SET_CLOCK = 7,
	HACKRF_VENDOR_REQUEST_SPIFLASH_ERASE = 10,
	HACKRF_VENDOR_REQUEST_SPIFLASH_WRITE = 11,
	HACKRF_VENDOR_REQUEST_SPIFLASH_READ = 12,
	HACKRF_VENDOR_REQUEST_BOARD_ID_READ = 14,
	HACKRF_VENDOR_REQUEST_VERSION_STRING_READ = 15,
	HACKRF_VENDOR_REQUEST_BOARD_PARTID_SERIALNO_READ = 18,
} hackrf_vendor_request;

typedef enum {
	USB_CONFIG_STANDARD = 0x1,
	USB_CONFIG_CPLD_UPDATE  = 0x2,
} hackrf_usb_configurations;

typedef enum {
	HACKRF_TRANSCEIVER_MODE_OFF = 0,
	HACKRF_TRANSCEIVER_MODE_RECEIVE = 1,
	HACKRF_TRANSCEIVER_MODE_TRANSMIT = 2,
} hackrf_transceiver_mode;

struct hackrf_device {
	libusb_device_handle* usb_device;
	struct libusb_transfer** transfers;
	hackrf_sample_block_cb_fn callback;
	volatile bool transfer_thread_started; /* volatile shared between threads (read only) */
	pthread_t transfer_thread;
	uint32_t transfer_count;
	uint32_t buffer_size;
	volatile bool streaming; /* volatile shared between threads (read only) */
	void* rx_ctx;
	void* tx_ctx;
};

typedef struct {
	uint32_t bandwidth_hz;
} max2837_ft_t;

static const max2837_ft_t max2837_ft[] = {
	{ 1750000  },
	{ 2500000  },
	{ 3500000  },
	{ 5000000  },
	{ 5500000  },
	{ 6000000  },
	{ 7000000  },
	{ 8000000  },
	{ 9000000  },
	{ 10000000 },
	{ 12000000 },
	{ 14000000 },
	{ 15000000 },
	{ 20000000 },
	{ 24000000 },
	{ 28000000 },
	{ 0        }
};

typedef struct {
    char *name;
    int reg;
    int bit;
    int len;
} adf_reg;

const static adf_reg adf4158_regs[] = {
    { "frac_msb", 0, 3, 12 },
    { "n", 0, 15, 12 },
    { "muxout", 0, 27, 4 },
    { "ramp_on", 0, 31, 1 },
    { "frac_lsb", 1, 15, 13 },
    { "reserved1", 1, 3, 12 },
    { "reserved0", 1, 28, 4 },
    { "r_counter", 2, 15, 5 },
    { "csr_en", 2, 28, 1 },
    { "rdiv2", 2, 21, 1 },
    { "prescaler", 2, 22, 1 },
    { "reference_doubler", 2, 20, 1 },
    { "cp_current", 2, 24, 4 },
    { "clk1_divider", 2, 3, 12 },
    { "reserved3", 2, 23, 1 },
    { "reserved2", 2, 29, 3 },
    { "power_down", 3, 5, 1 },
    { "psk_enable", 3, 9, 1 },
    { "fsk_enable", 3, 8, 1 },
    { "sd_reset", 3, 14, 1 },
    { "counter_reset", 3, 3, 1 },
    { "reserved5", 3, 12, 2 },
    { "reserved4", 3, 16, 16 },
    { "ramp_mode", 3, 10, 2 },
    { "n_sel", 3, 15, 1 },
    { "cp_3state", 3, 4, 1 },
    { "lpd", 3, 7, 1 },
    { "pd_polarity", 3, 6, 1 },
    { "neg_bleed_current", 4, 23, 2 },
    { "clk_div_mode", 4, 19, 2 },
    { "clk2_divider", 4, 7, 12 },
    { "sd_mod_mode", 4, 26, 5 },
    { "reserved7", 4, 3, 4 },
    { "reserved6", 4, 25, 1 },
    { "lf_sel", 4, 31, 1 },
    { "readback_to_muxout", 4, 21, 2 },
    { "fsk_ramp_en", 5, 25, 1 },
    { "tx_ramp_clk", 5, 29, 1 },
    { "dev_offset", 5, 19, 4 },
    { "par_ramp", 5, 28, 1 },
    { "reserved8", 5, 30, 2 },
    { "dev_sel", 5, 23, 1 },
    { "interrupt", 5, 26, 2 },
    { "ramp2_en", 5, 24, 1 },
    { "deviation", 5, 3, 16 },
    { "step", 6, 3, 20 },
    { "reserved9", 6, 24, 8 },
    { "step_sel", 6, 23, 1 },
    { "reserved10", 7, 19, 13 },
    { "ramp_del", 7, 17, 1 },
    { "ramp_del_fl", 7, 18, 1 },
    { "del_start_en", 7, 15, 1 },
    { "delay_start_divider", 7, 3, 12 },
    { "del_clk_sel", 7, 16, 1 },
    { "", 0, 0, 0}
};

static int hackrf_adf4158_reg_to_device(hackrf_device* device, unsigned int reg);

static uint32_t adf4158[8] = {0};

volatile bool do_exit = false;

static const uint16_t hackrf_usb_vid = 0x1d50;
static const uint16_t hackrf_jawbreaker_usb_pid = 0x604b;
static const uint16_t hackrf_one_usb_pid = 0x6099;
static const uint16_t rad1o_usb_pid = 0xcc15;

static libusb_context* g_libusb_context = NULL;

static void request_exit(void)
{
	do_exit = true;
}

static int cancel_transfers(hackrf_device* device)
{
	uint32_t transfer_index;

	if( device->transfers != NULL )
	{
		for(transfer_index=0; transfer_index<device->transfer_count; transfer_index++)
		{
			if( device->transfers[transfer_index] != NULL )
			{
				libusb_cancel_transfer(device->transfers[transfer_index]);
			}
		}
		return HACKRF_SUCCESS;
	} else {
		return HACKRF_ERROR_OTHER;
	}
}

static int free_transfers(hackrf_device* device)
{
	uint32_t transfer_index;

	if( device->transfers != NULL )
	{
		// libusb_close() should free all transfers referenced from this array.
		for(transfer_index=0; transfer_index<device->transfer_count; transfer_index++)
		{
			if( device->transfers[transfer_index] != NULL )
			{
				libusb_free_transfer(device->transfers[transfer_index]);
				device->transfers[transfer_index] = NULL;
			}
		}
		free(device->transfers);
		device->transfers = NULL;
	}
	return HACKRF_SUCCESS;
}

static int allocate_transfers(hackrf_device* const device)
{
	if( device->transfers == NULL )
	{
		uint32_t transfer_index;
		device->transfers = (struct libusb_transfer**) calloc(device->transfer_count, sizeof(struct libusb_transfer));
		if( device->transfers == NULL )
		{
			return HACKRF_ERROR_NO_MEM;
		}

		for(transfer_index=0; transfer_index<device->transfer_count; transfer_index++)
		{
			device->transfers[transfer_index] = libusb_alloc_transfer(0);
			if( device->transfers[transfer_index] == NULL )
			{
				return HACKRF_ERROR_LIBUSB;
			}

			libusb_fill_bulk_transfer(
				device->transfers[transfer_index],
				device->usb_device,
				0,
				(unsigned char*)malloc(device->buffer_size),
				device->buffer_size,
				NULL,
				device,
				0
			);

			if( device->transfers[transfer_index]->buffer == NULL )
			{
				return HACKRF_ERROR_NO_MEM;
			}
		}
		return HACKRF_SUCCESS;
	} else {
		return HACKRF_ERROR_BUSY;
	}
}

static int prepare_transfers(
	hackrf_device* device,
	const uint_fast8_t endpoint_address,
	libusb_transfer_cb_fn callback)
{
	int error;
	uint32_t transfer_index;
	if( device->transfers != NULL )
	{
		for(transfer_index=0; transfer_index<device->transfer_count; transfer_index++)
		{
			device->transfers[transfer_index]->endpoint = endpoint_address;
			device->transfers[transfer_index]->callback = callback;

			error = libusb_submit_transfer(device->transfers[transfer_index]);
			if( error != 0 )
			{
				return HACKRF_ERROR_LIBUSB;
			}
		}
		return HACKRF_SUCCESS;
	} else {
		// This shouldn't happen.
		return HACKRF_ERROR_OTHER;
	}
}

static int detach_kernel_drivers(libusb_device_handle* usb_device_handle)
{
	int i, num_interfaces, result;
	libusb_device* dev;
	struct libusb_config_descriptor* config;

	dev = libusb_get_device(usb_device_handle);
	result = libusb_get_active_config_descriptor(dev, &config);
	if( result < 0 )
	{
		return HACKRF_ERROR_LIBUSB;
	}

	num_interfaces = config->bNumInterfaces;
	libusb_free_config_descriptor(config);
	for(i=0; i<num_interfaces; i++)
	{
		result = libusb_kernel_driver_active(usb_device_handle, i);
		if( result < 0 )
		{
			if( result == LIBUSB_ERROR_NOT_SUPPORTED ) {
				return 0;
			}
			return HACKRF_ERROR_LIBUSB;
		} else if( result == 1 ) {
			result = libusb_detach_kernel_driver(usb_device_handle, i);
			if( result != 0 )
			{
				return HACKRF_ERROR_LIBUSB;
			}
		}
	}
	return HACKRF_SUCCESS;
}

static int set_hackrf_configuration(libusb_device_handle* usb_device, int config)
{
	int result, curr_config;
	result = libusb_get_configuration(usb_device, &curr_config);
	if( result != 0 )
	{
		return HACKRF_ERROR_LIBUSB;
	}

	if(curr_config != config)
	{
		result = detach_kernel_drivers(usb_device);
		if( result != 0 )
		{
			return result;
		}
		result = libusb_set_configuration(usb_device, config);
		if( result != 0 )
		{
			return HACKRF_ERROR_LIBUSB;
		}
	}

	result = detach_kernel_drivers(usb_device);
	if( result != 0 )
	{
		return result;
	}
	return LIBUSB_SUCCESS;
}

#ifdef __cplusplus
extern "C"
{
#endif

int ADDCALL hackrf_init(void)
{
	if (g_libusb_context != NULL) {
		return HACKRF_SUCCESS;
	}

	const int libusb_error = libusb_init(&g_libusb_context);
	if( libusb_error != 0 )
	{
		return HACKRF_ERROR_LIBUSB;
	} else {
		return HACKRF_SUCCESS;
	}
}

int ADDCALL hackrf_exit(void)
{
	if( g_libusb_context != NULL )
	{
		libusb_exit(g_libusb_context);
		g_libusb_context = NULL;
	}

	return HACKRF_SUCCESS;
}

#include <stdio.h>
#include <string.h>

hackrf_device_list_t* ADDCALL hackrf_device_list()
{
	ssize_t i;
	libusb_device_handle* usb_device = NULL;
	hackrf_device_list_t* list = calloc(1, sizeof(*list));
	if ( list == NULL )
		return NULL;

	list->usb_devicecount = libusb_get_device_list(g_libusb_context, (libusb_device ***)&list->usb_devices);

	list->serial_numbers = calloc(list->usb_devicecount, sizeof(void *));
	list->usb_board_ids = calloc(list->usb_devicecount, sizeof(enum hackrf_usb_board_id));
	list->usb_device_index = calloc(list->usb_devicecount, sizeof(int));

	if ( list->serial_numbers == NULL || list->usb_board_ids == NULL || list->usb_device_index == NULL) {
		hackrf_device_list_free(list);
		return NULL;
	}

	for (i=0; i<list->usb_devicecount; i++) {
		struct libusb_device_descriptor device_descriptor;
		libusb_get_device_descriptor(list->usb_devices[i], &device_descriptor);

		if( device_descriptor.idVendor == hackrf_usb_vid ) {
			if((device_descriptor.idProduct == hackrf_one_usb_pid) ||
			   (device_descriptor.idProduct == hackrf_jawbreaker_usb_pid) ||
			   (device_descriptor.idProduct == rad1o_usb_pid)) {
				int idx = list->devicecount++;
				list->usb_board_ids[idx] = device_descriptor.idProduct;
				list->usb_device_index[idx] = i;

				const uint_fast8_t serial_descriptor_index = device_descriptor.iSerialNumber;
				if( serial_descriptor_index > 0 ) {
					if( libusb_open(list->usb_devices[i], &usb_device) != 0 ) {
						usb_device = NULL;
						continue;
					}
					char serial_number[64];
					const int serial_number_length = libusb_get_string_descriptor_ascii(usb_device, serial_descriptor_index, (unsigned char*)serial_number, sizeof(serial_number));
					if( serial_number_length == 32 ) {
						serial_number[32] = 0;
						list->serial_numbers[idx] = strdup(serial_number);
					}

					libusb_close(usb_device);
					usb_device = NULL;
				}
			}
		}
	}

	return list;
}

void ADDCALL hackrf_device_list_free(hackrf_device_list_t *list)
{
	int i;

	libusb_free_device_list((libusb_device **)list->usb_devices, 1);

	for (i = 0; i < list->devicecount; i++) {
		if (list->serial_numbers[i])
			free(list->serial_numbers[i]);
	}

	free(list->serial_numbers);
	free(list->usb_board_ids);
	free(list->usb_device_index);
	free(list);
}

libusb_device_handle* hackrf_open_usb(const char* const desired_serial_number)
{
	libusb_device_handle* usb_device = NULL;
	libusb_device** devices = NULL;
	const ssize_t list_length = libusb_get_device_list(g_libusb_context, &devices);
	int match_len = 0;
	ssize_t i;

	printf("Number of USB devices: %ld\n", list_length);

	if( desired_serial_number ) {
		/* If a shorter serial number is specified, only match against the suffix.
		 * Should probably complain if the match is not unique, currently doesn't.
		 */
		match_len = strlen(desired_serial_number);
		if ( match_len > 32 )
			return NULL;
	}

	for (i=0; i<list_length; i++) {
		struct libusb_device_descriptor device_descriptor;
		libusb_get_device_descriptor(devices[i], &device_descriptor);

		if( device_descriptor.idVendor == hackrf_usb_vid ) {
			if((device_descriptor.idProduct == hackrf_one_usb_pid) ||
			   (device_descriptor.idProduct == hackrf_jawbreaker_usb_pid) ||
			   (device_descriptor.idProduct == rad1o_usb_pid)) {
				printf("USB device %4x:%4x:", device_descriptor.idVendor, device_descriptor.idProduct);

				if( desired_serial_number != NULL ) {
					const uint_fast8_t serial_descriptor_index = device_descriptor.iSerialNumber;
					if( serial_descriptor_index > 0 ) {
						if( libusb_open(devices[i], &usb_device) != 0 ) {
							usb_device = NULL;
							continue;
						}
						char serial_number[64];
						const int serial_number_length = libusb_get_string_descriptor_ascii(usb_device, serial_descriptor_index, (unsigned char*)serial_number, sizeof(serial_number));
						if( serial_number_length == 32 ) {
							serial_number[32] = 0;
							printf(" %s", serial_number);
							if( strncmp(serial_number + 32-match_len, desired_serial_number, match_len) == 0 ) {
								printf(" match\n");
								break;
							} else {
								printf(" skip\n");
								libusb_close(usb_device);
								usb_device = NULL;
							}
						} else {
							printf(" wrong length of serial number: %d\n", serial_number_length);
							libusb_close(usb_device);
							usb_device = NULL;
						}
					}
				} else {
					printf(" default\n");
					libusb_open(devices[i], &usb_device);
					break;
				}
			}
		}
	}

	libusb_free_device_list(devices, 1);

	return usb_device;
}

static int hackrf_open_setup(libusb_device_handle* usb_device, hackrf_device** device)
{
	int result;
	hackrf_device* lib_device;

	//int speed = libusb_get_device_speed(usb_device);
	// TODO: Error or warning if not high speed USB?

	result = set_hackrf_configuration(usb_device, USB_CONFIG_STANDARD);
	if( result != LIBUSB_SUCCESS )
	{
		libusb_close(usb_device);
		return result;
	}

	result = libusb_claim_interface(usb_device, 0);
	if( result != LIBUSB_SUCCESS )
	{
		libusb_close(usb_device);
		return HACKRF_ERROR_LIBUSB;
	}

	lib_device = NULL;
	lib_device = (hackrf_device*)malloc(sizeof(*lib_device));
	if( lib_device == NULL )
	{
		libusb_release_interface(usb_device, 0);
		libusb_close(usb_device);
		return HACKRF_ERROR_NO_MEM;
	}

	lib_device->usb_device = usb_device;
	lib_device->transfers = NULL;
	lib_device->callback = NULL;
	lib_device->transfer_thread_started = false;
	/*
	lib_device->transfer_count = 1024;
	lib_device->buffer_size = 16384;
	*/
	lib_device->transfer_count = 4;
	lib_device->buffer_size = 262144; /* 1048576; */
	lib_device->streaming = false;
	do_exit = false;

	result = allocate_transfers(lib_device);
	if( result != 0 )
	{
		free(lib_device);
		libusb_release_interface(usb_device, 0);
		libusb_close(usb_device);
		return HACKRF_ERROR_NO_MEM;
	}

	*device = lib_device;

	return HACKRF_SUCCESS;
}

int ADDCALL hackrf_open(hackrf_device** device)
{
	libusb_device_handle* usb_device;
	
	if( device == NULL )
	{
		return HACKRF_ERROR_INVALID_PARAM;
	}
	
	usb_device = libusb_open_device_with_vid_pid(g_libusb_context, hackrf_usb_vid, hackrf_one_usb_pid);
	
	if( usb_device == NULL )
	{
		usb_device = libusb_open_device_with_vid_pid(g_libusb_context, hackrf_usb_vid, hackrf_jawbreaker_usb_pid);
	}
	
	if( usb_device == NULL )
	{
		usb_device = libusb_open_device_with_vid_pid(g_libusb_context, hackrf_usb_vid, rad1o_usb_pid);
	}
	
	if( usb_device == NULL )
	{
		return HACKRF_ERROR_NOT_FOUND;
	}
	
	return hackrf_open_setup(usb_device, device);
}

int ADDCALL hackrf_open_by_serial(const char* const desired_serial_number, hackrf_device** device)
{
	libusb_device_handle* usb_device;
	
	if( desired_serial_number == NULL )
	{
		return hackrf_open(device);
	}
	
	if( device == NULL )
	{
		return HACKRF_ERROR_INVALID_PARAM;
	}
	
	usb_device = hackrf_open_usb(desired_serial_number);
	
	if( usb_device == NULL )
	{
		return HACKRF_ERROR_NOT_FOUND;
	}
	
	return hackrf_open_setup(usb_device, device);
}

int ADDCALL hackrf_device_list_open(hackrf_device_list_t *list, int idx, hackrf_device** device)
{
	libusb_device_handle* usb_device;
	
	if( device == NULL || list == NULL || idx < 0 || idx >= list->devicecount )
	{
		return HACKRF_ERROR_INVALID_PARAM;
	}
	
	int i = list->usb_device_index[idx];

	if( libusb_open(list->usb_devices[i], &usb_device) != 0 ) {
		usb_device = NULL;
		return HACKRF_ERROR_LIBUSB;
	}
	
	return hackrf_open_setup(usb_device, device);
}

int ADDCALL hackrf_set_transceiver_mode(hackrf_device* device, hackrf_transceiver_mode value)
{
	int result;
	result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_SET_TRANSCEIVER_MODE,
		value,
		0,
		NULL,
		0,
		0
	);

	if( result != 0 )
	{
		return HACKRF_ERROR_LIBUSB;
	} else {
		return HACKRF_SUCCESS;
	}
}

int ADDCALL hackrf_set_sweep(hackrf_device* device, double fstart, double bw, double length) {
    unsigned int res = 0;
    unsigned int n = fstart/FPD_FREQ;
    unsigned int frac_msb = ((fstart/FPD_FREQ) -n)*(1 <<12);
    unsigned int frac_lsb = ((((fstart/FPD_FREQ) -n)*(1 << 12))-frac_msb)*(1 <<13);
    res = hackrf_set_adf4158_reg("n", n);
    res |= hackrf_set_adf4158_reg("frac_msb", frac_msb);
    res |= hackrf_set_adf4158_reg("frac_lsb", frac_lsb);
    printf("n: %d\n", n);
    printf("frac_msb: %d\n", frac_msb);
    printf("frac_lsb: %d\n", frac_lsb);
    if (res) {
        return -1;
    }

    unsigned int clk1 = (FPD_FREQ*length/(1<<20))+1;
    res |= hackrf_set_adf4158_reg("clk1_divider", clk1);
    res |= hackrf_set_adf4158_reg("clk2_divider", 1);
    if (res) {
        return -1;
    }

    unsigned int steps = FPD_FREQ*length/clk1;

    unsigned int devmax = 1 << 15;
    double fres = ((double)FPD_FREQ)/(1 << 25);
    double fdev = bw/steps;

    int dev_offset = (int)ceil(log2(fdev/(fres*devmax)));
    if (dev_offset < 0) {
        dev_offset = 0;
    }

    unsigned int dev = fdev/(fres*(1 << dev_offset));

    res |= hackrf_set_adf4158_reg("deviation", dev);
    printf("fres: %f\n", fres);
    printf("fdev: %f\n", fdev);
    printf("dev_offset: %d\n", dev_offset);
    printf("deviation: %d\n", dev);

    res |= hackrf_set_adf4158_reg("step", steps);
    res |= hackrf_set_adf4158_reg("dev_offset", dev_offset);
    res |= hackrf_set_adf4158_reg("clk_div_mode", 3); //Ramp clock divider
    res |= hackrf_set_adf4158_reg("ramp_on", 1); //Enable ramp

    res |= hackrf_set_adf4158_reg("pd_polarity", 1);
    res |= hackrf_set_adf4158_reg("prescaler", 1);
    res |= hackrf_set_adf4158_reg("r_counter", 1);
    res |= hackrf_set_adf4158_reg("csr_en", 1);

    // Readback to muxout and negative bleed current
    // can't be activated simultaneously
    if (1) {
        //Muxout control to readback to muxout
        res |= hackrf_set_adf4158_reg("muxout", 15);
        //res |= hackrf_set_adf4158_reg("neg_bleed_current", 0);
        res |= hackrf_set_adf4158_reg("readback_to_muxout", 3);
    } else {
        res |= hackrf_set_adf4158_reg("neg_bleed_current", 3);
    }

    res |= hackrf_set_adf4158_reg("ramp_mode", 1); //Triangle
    //res |= hackrf_set_adf4158_reg("ramp_del_fl", 1);
    //res |= hackrf_set_adf4158_reg("ramp_del", 1);
    //res |= hackrf_set_adf4158_reg("delay_start_divider", 500);
    if (res) {
        return res;
    }
    return hackrf_adf4158_to_device(device);
}

int ADDCALL hackrf_set_adf4158_reg(char *name, unsigned int value) {
    unsigned int i = 0;
    while (adf4158_regs[i].name[0]) {
        if (strcmp(name, adf4158_regs[i].name) == 0) {
            unsigned int reg = adf4158_regs[i].reg;
            unsigned int bit = adf4158_regs[i].bit;
            unsigned int len = adf4158_regs[i].len;
            adf4158[reg] &= ~((((1 << len)-1)) << bit);
            adf4158[reg] |= value << bit;
            return 0;
        }
        i++;
    }
    return -1;
}

static int hackrf_adf4158_reg_to_device(hackrf_device* device, unsigned int reg)
{
    int result;
    if (reg > 7) {
        return -1;
    }
    result = libusb_control_transfer(
            device->usb_device,
            LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
            HACKRF_VENDOR_REQUEST_ADF4158_WRITE,
            adf4158[reg]>>16,
            (adf4158[reg] & 0xFFF8) | reg,
            NULL,
            0,
            0
        );
    return result;
}

int ADDCALL hackrf_adf4158_to_device(hackrf_device* device)
{
    int reg;

    int i;
    for(i=7;i>=0;i--) {
        printf("%d: %u\n", i, adf4158[i]);
    }

    if( hackrf_adf4158_reg_to_device(device, 7) != 0 )
    {
        return HACKRF_ERROR_LIBUSB;
    }
    hackrf_set_adf4158_reg("step_sel", 0);
    if( hackrf_adf4158_reg_to_device(device, 6) != 0 )
    {
        return HACKRF_ERROR_LIBUSB;
    }
    hackrf_set_adf4158_reg("step_sel", 1);
    if( hackrf_adf4158_reg_to_device(device, 6) != 0 )
    {
        return HACKRF_ERROR_LIBUSB;
    }
    hackrf_set_adf4158_reg("dev_sel", 0);
    if( hackrf_adf4158_reg_to_device(device, 5) != 0 )
    {
        return HACKRF_ERROR_LIBUSB;
    }
    hackrf_set_adf4158_reg("dev_sel", 1);
    if( hackrf_adf4158_reg_to_device(device, 5) != 0 )
    {
        return HACKRF_ERROR_LIBUSB;
    }

    for(reg=4;reg>=0;reg--) {
        if( hackrf_adf4158_reg_to_device(device, reg) != 0 )
        {
            return HACKRF_ERROR_LIBUSB;
        }
    }

    return HACKRF_SUCCESS;
}

int ADDCALL hackrf_set_gpio(hackrf_device *device, uint32_t bits) {
    int result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_SET_GPIO,
		bits >> 16,
		bits & 0xFFFF,
		NULL,
		0,
		0
	);
    return result;
}

int ADDCALL hackrf_clear_gpio(hackrf_device *device, uint32_t bits) {
    int result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_CLEAR_GPIO,
		bits >> 16,
		bits & 0xFFFF,
		NULL,
		0,
		0
	);
    return result;
}

int ADDCALL hackrf_spiflash_erase(hackrf_device* device)
{
	int result;
	result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_SPIFLASH_ERASE,
		0,
		0,
		NULL,
		0,
		0
	);

	if (result != 0)
	{
		return HACKRF_ERROR_LIBUSB;
	} else {
		return HACKRF_SUCCESS;
	}
}

int ADDCALL hackrf_spiflash_write(hackrf_device* device, const uint32_t address,
		const uint16_t length, unsigned char* const data)
{
	int result;
	
	if (address > 0x0FFFFF)
	{
		return HACKRF_ERROR_INVALID_PARAM;
	}

	result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_SPIFLASH_WRITE,
		address >> 16,
		address & 0xFFFF,
		data,
		length,
		0
	);

	if (result < length)
	{
		return HACKRF_ERROR_LIBUSB;
	} else {
		return HACKRF_SUCCESS;
	}
}

int ADDCALL hackrf_spiflash_read(hackrf_device* device, const uint32_t address,
		const uint16_t length, unsigned char* data)
{
	int result;
	
	if (address > 0x0FFFFF)
	{
		return HACKRF_ERROR_INVALID_PARAM;
	}

	result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_SPIFLASH_READ,
		address >> 16,
		address & 0xFFFF,
		data,
		length,
		0
	);

	if (result < length)
	{
		return HACKRF_ERROR_LIBUSB;
	} else {
		return HACKRF_SUCCESS;
	}
}

int ADDCALL hackrf_board_id_read(hackrf_device* device, uint8_t* value)
{
	int result;
	result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_BOARD_ID_READ,
		0,
		0,
		value,
		1,
		0
	);

	if (result < 1)
	{
		return HACKRF_ERROR_LIBUSB;
	} else {
		return HACKRF_SUCCESS;
	}
}

int ADDCALL hackrf_version_string_read(hackrf_device* device, char* version,
		uint8_t length)
{
	int result;
	result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_VERSION_STRING_READ,
		0,
		0,
		(unsigned char*)version,
		length,
		0
	);

	if (result < 0)
	{
		return HACKRF_ERROR_LIBUSB;
	} else {
		version[result] = '\0';
		return HACKRF_SUCCESS;
	}
}

int ADDCALL hackrf_board_partid_serialno_read(hackrf_device* device, read_partid_serialno_t* read_partid_serialno)
{
	uint8_t length;
	int result;
	
	length = sizeof(read_partid_serialno_t);
	result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_BOARD_PARTID_SERIALNO_READ,
		0,
		0,
		(unsigned char*)read_partid_serialno,
		length,
		0
	);

	if (result < length)
	{
		return HACKRF_ERROR_LIBUSB;
	} else {

		read_partid_serialno->part_id[0] = TO_LE(read_partid_serialno->part_id[0]);
		read_partid_serialno->part_id[1] = TO_LE(read_partid_serialno->part_id[1]);
		read_partid_serialno->serial_no[0] = TO_LE(read_partid_serialno->serial_no[0]);
		read_partid_serialno->serial_no[1] = TO_LE(read_partid_serialno->serial_no[1]);
		read_partid_serialno->serial_no[2] = TO_LE(read_partid_serialno->serial_no[2]);
		read_partid_serialno->serial_no[3] = TO_LE(read_partid_serialno->serial_no[3]);

		return HACKRF_SUCCESS;
	}
}

int ADDCALL hackrf_set_mcp(hackrf_device* device, uint32_t value)
{
	int result;

	if( value > 63 )
	{
		return HACKRF_ERROR_INVALID_PARAM;
	}

	result = libusb_control_transfer(
		device->usb_device,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		HACKRF_VENDOR_REQUEST_SET_MCP,
		0,
		value,
		NULL,
		0,
		0
	);

	if( result != 0 )
	{
		return HACKRF_ERROR_INVALID_PARAM;
	} else {
		return HACKRF_SUCCESS;
	}
}

static void* transfer_threadproc(void* arg)
{
	hackrf_device* device = (hackrf_device*)arg;
	int error;
	struct timeval timeout = { 0, 500000 };

	while( (device->streaming) && (do_exit == false) )
	{
		error = libusb_handle_events_timeout(g_libusb_context, &timeout);
		if( (error != 0) && (error != LIBUSB_ERROR_INTERRUPTED) )
		{
			device->streaming = false;
		}
	}

	return NULL;
}

static void hackrf_libusb_transfer_callback(struct libusb_transfer* usb_transfer)
{
	hackrf_device* device = (hackrf_device*)usb_transfer->user_data;

	if(usb_transfer->status == LIBUSB_TRANSFER_COMPLETED)
	{
		hackrf_transfer transfer = {
			transfer.device = device,
			transfer.buffer = usb_transfer->buffer,
			transfer.buffer_length = usb_transfer->length,
			transfer.valid_length = usb_transfer->actual_length,
			transfer.rx_ctx = device->rx_ctx,
			transfer.tx_ctx = device->tx_ctx
		};

		if( device->callback(&transfer) == 0 )
		{
			if( libusb_submit_transfer(usb_transfer) < 0)
			{
				request_exit();
			}else {
				return;
			}
		}else {
			request_exit();
		}
	} else {
		/* Other cases LIBUSB_TRANSFER_NO_DEVICE
		LIBUSB_TRANSFER_ERROR, LIBUSB_TRANSFER_TIMED_OUT
		LIBUSB_TRANSFER_STALL,	LIBUSB_TRANSFER_OVERFLOW
		LIBUSB_TRANSFER_CANCELLED ...
		*/
		request_exit(); /* Fatal error stop transfer */
	}
}

static int kill_transfer_thread(hackrf_device* device)
{
	void* value;
	int result;
	
	request_exit();

	if( device->transfer_thread_started != false )
	{
		value = NULL;
		result = pthread_join(device->transfer_thread, &value);
		if( result != 0 )
		{
			return HACKRF_ERROR_THREAD;
		}
		device->transfer_thread_started = false;

		/* Cancel all transfers */
		cancel_transfers(device);
	}

	return HACKRF_SUCCESS;
}

static int create_transfer_thread(hackrf_device* device,
									const uint8_t endpoint_address,
									hackrf_sample_block_cb_fn callback)
{
	int result;
	
	if( device->transfer_thread_started == false )
	{
		device->streaming = false;

		result = prepare_transfers(
			device, endpoint_address,
			(libusb_transfer_cb_fn)hackrf_libusb_transfer_callback
		);

		if( result != HACKRF_SUCCESS )
		{
			return result;
		}

		device->streaming = true;
		device->callback = callback;
		result = pthread_create(&device->transfer_thread, 0, transfer_threadproc, device);
		if( result == 0 )
		{
			device->transfer_thread_started = true;
		}else {
			return HACKRF_ERROR_THREAD;
		}
	} else {
		return HACKRF_ERROR_BUSY;
	}

	return HACKRF_SUCCESS;
}

int ADDCALL hackrf_is_streaming(hackrf_device* device)
{
	/* return hackrf is streaming only when streaming, transfer_thread_started are true and do_exit equal false */
	
	if( (device->transfer_thread_started == true) &&
		(device->streaming == true) && 
		(do_exit == false) )
	{
		return HACKRF_TRUE;
	} else {
	
		if(device->transfer_thread_started == false)
		{
			return HACKRF_ERROR_STREAMING_THREAD_ERR;
		}

		if(device->streaming == false)
		{
			return HACKRF_ERROR_STREAMING_STOPPED;
		}

		return HACKRF_ERROR_STREAMING_EXIT_CALLED;
	}
}

int ADDCALL hackrf_start_rx(hackrf_device* device, hackrf_sample_block_cb_fn callback, void* rx_ctx)
{
	int result;
	const uint8_t endpoint_address = LIBUSB_ENDPOINT_IN | 1;
    result = hackrf_set_gpio(device, GPIO_PA|GPIO_MIXER|GPIO_ADF|GPIO_ADC);
	if( result != HACKRF_SUCCESS ) {
        return result;
    }
	result = hackrf_set_transceiver_mode(device, HACKRF_TRANSCEIVER_MODE_RECEIVE);

	if( result == HACKRF_SUCCESS )
	{
		device->rx_ctx = rx_ctx;
		result = create_transfer_thread(device, endpoint_address, callback);
	}
	return result;
}

int ADDCALL hackrf_stop_rx(hackrf_device* device)
{
	int result;
    result = hackrf_clear_gpio(device, GPIO_PA|GPIO_MIXER|GPIO_ADF|GPIO_ADC);
	result = hackrf_set_transceiver_mode(device, HACKRF_TRANSCEIVER_MODE_OFF);
	if (result != HACKRF_SUCCESS)
	{
		return result;
	}
	return kill_transfer_thread(device);
}

int ADDCALL hackrf_close(hackrf_device* device)
{
	int result1;

	result1 = HACKRF_SUCCESS;

	if( device != NULL )
	{
		result1 = hackrf_stop_rx(device);
		if( device->usb_device != NULL )
		{
			libusb_release_interface(device->usb_device, 0);
			libusb_close(device->usb_device);
			device->usb_device = NULL;
		}

		free_transfers(device);

		free(device);
	}

	return result1;
}

const char* ADDCALL hackrf_error_name(enum hackrf_error errcode)
{
	switch(errcode)
	{
	case HACKRF_SUCCESS:
		return "HACKRF_SUCCESS";

	case HACKRF_TRUE:
		return "HACKRF_TRUE";

	case HACKRF_ERROR_INVALID_PARAM:
		return "HACKRF_ERROR_INVALID_PARAM";

	case HACKRF_ERROR_NOT_FOUND:
		return "HACKRF_ERROR_NOT_FOUND";

	case HACKRF_ERROR_BUSY:
		return "HACKRF_ERROR_BUSY";

	case HACKRF_ERROR_NO_MEM:
		return "HACKRF_ERROR_NO_MEM";

	case HACKRF_ERROR_LIBUSB:
		return "HACKRF_ERROR_LIBUSB";

	case HACKRF_ERROR_THREAD:
		return "HACKRF_ERROR_THREAD";

	case HACKRF_ERROR_STREAMING_THREAD_ERR:
		return "HACKRF_ERROR_STREAMING_THREAD_ERR";

	case HACKRF_ERROR_STREAMING_STOPPED:
		return "HACKRF_ERROR_STREAMING_STOPPED";

	case HACKRF_ERROR_STREAMING_EXIT_CALLED:
		return "HACKRF_ERROR_STREAMING_EXIT_CALLED";

	case HACKRF_ERROR_OTHER:
		return "HACKRF_ERROR_OTHER";

	default:
		return "HACKRF unknown error";
	}
}

const char* ADDCALL hackrf_board_id_name(enum hackrf_board_id board_id)
{
	switch(board_id)
	{
	case BOARD_ID_JELLYBEAN:
		return "Jellybean";

	case BOARD_ID_JAWBREAKER:
		return "Jawbreaker";

	case BOARD_ID_HACKRF_ONE:
		return "HackRF One";

	case BOARD_ID_INVALID:
		return "Invalid Board ID";

	default:
		return "Unknown Board ID";
	}
}

extern ADDAPI const char* ADDCALL hackrf_usb_board_id_name(enum hackrf_usb_board_id usb_board_id)
{
	switch(usb_board_id)
	{
	case USB_BOARD_ID_JAWBREAKER:
		return "Jawbreaker";

	case USB_BOARD_ID_HACKRF_ONE:
		return "HackRF One";

	case USB_BOARD_ID_INVALID:
		return "Invalid Board ID";

	default:
		return "Unknown Board ID";
	}
}

const char* ADDCALL hackrf_filter_path_name(const enum rf_path_filter path)
{
	switch(path) {
	case RF_PATH_FILTER_BYPASS:
		return "mixer bypass";
	case RF_PATH_FILTER_LOW_PASS:
		return "low pass filter";
	case RF_PATH_FILTER_HIGH_PASS:
		return "high pass filter";
	default:
		return "invalid filter path";
	}
}

#ifdef __cplusplus
} // __cplusplus defined.
#endif

