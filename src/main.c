// A0 and A1,   used to meassure signal
// USART used for data, pins, A9 TX, A10 RX
// TIM1 (timer) that tells ADC to meassure
// ADC (analog 2 digital converter)  gives us 12bit measurment of voltage
// DMA (direct memory access)  shuffles them to RAM
// DMA shuffles the RAM out over serial data port to computer

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>

#include <types.h>
#include "bsp.h"
#include "adc.h"

volatile uint32_t error = NO_ERROR;

uint32_t nirqs_o = 0;

volatile bool usart_busy = false;

// AWD
// Set when AWD fired.
// TODO figure out how to tag the exact event that triggered the interrupt, possible?
volatile bool adc_awd_did_interrupt = false;

/* Dump half ADC buffer on usart */
void trigger_adc_buffer_to_usart(bool second_half)
{
  if (usart_busy)
  {
    error |= USART_BUSY_ERROR;
    return;
  }
  usart_busy = true;

  // Tag interrupt happened
  // This sucks as it will wiggle around and not be exactly when the AWD fired.
  if(adc_awd_did_interrupt) {
    if (second_half) {
      adc_buffer[adc_buffer_len / 2] |= 1<<14;
    } else {
      adc_buffer[0] |= 1<<14;
    }
    adc_awd_did_interrupt = false;
  }

  dma_channel_reset(DMA1, DMA_CHANNEL4);
#ifdef STM32G4
  dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART_TDR(USART2));
#else
  dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART_DR(USART1));
#endif
  dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)(adc_buffer + (second_half ? adc_buffer_len / 2: 0)));
  // Note; /2 because we're transfering half of it, *2 because 16bit data sent as 8bit
  dma_set_number_of_data(DMA1, DMA_CHANNEL4, (adc_buffer_len) * 2 / 2);
  dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_LOW);
  dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);

  dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL4);
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
  dma_enable_channel(DMA1, DMA_CHANNEL4);
  usart_enable_tx_dma(USART2);
}

/* Dump half ADC buffer on usart */
void dma1_channel1_isr()
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_HTIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_HTIF);
        trigger_adc_buffer_to_usart(false);
        return;
    }

    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        trigger_adc_buffer_to_usart(true);
        return;
    }

    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TEIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TEIF);
        error |= ADC_DMA_ERROR;
        return;
    }
}

void adc1_2_isr()
{
    // Disable watchdog (interrupt bit already cleared)
    adc_disable_analog_watchdog_regular(ADC1);

  // TODO come back to this, this is too error prone
  // chan_regs = dma_channel_regs(DMA1, DMA_CH1);
  // uint16_t * value = reinterpret_cast<uint16_t*>(chan_regs->CMAR);
  adc_awd_did_interrupt = true;
}

volatile bool got_usart_irq = false;

/* Reset usart busy when data has been transferred */
void dma1_channel4_isr()
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TEIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TEIF);
        error |= USART_DMA_ERROR;
    }
    
    dma_disable_transfer_error_interrupt(DMA1, DMA_CHANNEL4);    
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
#ifdef STM32G4
    usart_disable_tx_dma(USART2);
#else
    usart_disable_tx_dma(USART1);
#endif
    dma_disable_channel(DMA1, DMA_CHANNEL4);
    
    usart_busy = false;
    got_usart_irq = true;
}

void setup_timer()
{
    rcc_periph_clock_enable(RCC_TIM1);
    rcc_periph_reset_pulse(RST_TIM1);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
               TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_continuous_mode(TIM1);
    timer_set_prescaler(TIM1, adc_timer_prescaler);
    timer_set_period(TIM1, adc_timer_arr);
    timer_set_master_mode(TIM1, TIM_CR2_MMS_UPDATE);
    //timer_enable_counter(TIM1);
}

void parameter_sanity_check() {
  uint32_t channels = nPins;

  // This is 10(adc_smpr + 12.5), 12 bit
  uint32_t sampletime_cycles_x10 = 65 + 125;

  // Time it takes for adc to sample all channels, in PCLK cycles
  uint32_t sampletime = sampletime_cycles_x10 * adc_clock_divider * channels / 10;

  // Time between timer triggers
  uint32_t timer_interval = adc_timer_prescaler * adc_timer_arr;


  if (timer_interval <= sampletime) {
    error |= BAD_PARAMETERS_TIMER_FASTER_THAN_ADC;
  }

  uint32_t send_capacity_bytes_per_second = baud_rate / 10;  // 10 bits per byte (2 for star/stop bits)
  uint32_t requested_bytes_per_second = 2 * channels * (rcc_ahb_frequency / timer_interval);
  if (send_capacity_bytes_per_second <= requested_bytes_per_second) {
    error |= BAD_PARAMETERS_SERIAL_TOO_SLOW;
  }
}

void setup()
{
  rcc_periph_clock_enable(RCC_DMA1);
#ifdef STM32G4
  rcc_periph_clock_enable(RCC_DMAMUX1);
#endif
  usart_setup();
  parameter_sanity_check();

  for (size_t i = 0; i < adc_buffer_len; i++)
  {
    adc_buffer[i] = 0;
  }

  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

  setup_timer();
  setup_adc();

  adc_stop(); // TODO no!
}

void loop()
{
  if (error != NO_ERROR)
  {
    bsp_set_pin_error(true);
  }
  // Everything relies on interrupts, do nothing
  asm("wfi \n");
};

int main(void)
{
    clock_setup();
    gpio_setup();
    setup();

	while (1) {
        loop();
	}

	return 0;
}
