#ifndef BSP_H
#define BSP_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#ifdef STM32G4
#include <libopencm3/stm32/dmamux.h>
#endif
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>

extern const uint32_t baud_rate;

extern volatile uint8_t cmd_buf[];
extern volatile size_t cmd_buf_ix;

void bsp_set_pin_error(bool val);

void gpio_setup(void);
void clock_setup(void);
void usart_setup(void);


#endif // BSP_H
