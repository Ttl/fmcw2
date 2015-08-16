#ifndef _RF_PATH_H
#define _RF_PATH_H


void enable_adc(void);
void disable_adc(void);

void enable_pa(void);
void disable_pa(void);

void enable_mixer(void);
void disable_mixer(void);

void enable_adf4158(void);
void disable_adf4158(void);

void rf_disable(void);
void rf_enable(void);

void adf4158_write_register(uint32_t data);
uint32_t adf4158_read_register(void);

#endif
