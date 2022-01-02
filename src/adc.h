#ifndef ADC_H
#define ADC_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>

extern const size_t nPins;
extern const size_t adc_buffer_len;
extern uint16_t adc_buffer[];

extern const uint8_t adc_clock_divider;
extern const uint32_t adc_timer_prescaler;
extern const uint32_t adc_timer_arr;
extern uint32_t adc_samplerate;

void setup_adc();
void adc_awd_set_threshold( uint8_t chan, uint16_t vh, uint16_t vl);
void adc_stop(void);
void adc_start(void);

#endif // ADC_H
