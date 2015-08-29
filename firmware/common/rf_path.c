#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include "hackrf_core.h"

void enable_pa(void) {
    gpio_set(PORT_PA_OFF, PIN_PA_OFF);
}

void disable_pa(void) {
    gpio_clear(PORT_PA_OFF, PIN_PA_OFF);
}

void enable_mixer(void) {
    gpio_set(PORT_MIX_ENBL, PIN_MIX_ENBL);
}

void disable_mixer(void) {
    gpio_clear(PORT_MIX_ENBL, PIN_MIX_ENBL);
}

void disable_adc(void) {
    gpio_set(PORT_ADC_NOE, PIN_ADC_NOE);
}

void enable_adc(void) {
    gpio_clear(PORT_ADC_NOE, PIN_ADC_NOE);
}

void enable_adf4158(void) {
    gpio_set(PORT_ADF_CE, PIN_ADF_CE);
}

void disable_adf4158(void) {
    gpio_clear(PORT_ADF_CE, PIN_ADF_CE);
}

void rf_disable(void) {
    gpio_clear(PORT_ADF_TXDATA, PIN_ADF_TXDATA);

    disable_pa();
    disable_mixer();
    disable_adc();
    disable_adf4158();
}

void rf_enable(void) {
    enable_adf4158();
    enable_adc();
    enable_pa();
    enable_mixer();
}

void adf4158_write_register(uint32_t data) {
    uint16_t transfer[2] = {data >> 16, data & 0x0000FFFF};

    gpio_clear(PORT_ADF_LE, PIN_ADF_LE);
	ssp_transfer(SSP1_NUM, transfer[0]);
	ssp_transfer(SSP1_NUM, transfer[1]);
    gpio_set(PORT_ADF_LE, PIN_ADF_LE);
}

uint32_t adf4158_read_register(void) {
    uint32_t read = 0;

    gpio_set(PORT_ADF_LE, PIN_ADF_LE);
	read = (uint32_t)((uint16_t)ssp_transfer(SSP1_NUM, 0xFFFF) << 16);
	read |= (uint16_t)ssp_transfer(SSP1_NUM, 0xFFFF);
    gpio_set(PORT_ADF_LE, PIN_ADF_LE);
    return read;
}
