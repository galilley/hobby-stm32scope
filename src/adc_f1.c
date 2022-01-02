#include "adc.h"
#include "bsp.h"
#include <types.h>

extern volatile uint32_t error;

const uint8_t adc_clock_divider = 2;
const uint8_t adc_sampletime = ADC_SMPR_SMP_7DOT5CYC; // Also change in parameter_sanity_check

const uint32_t adc_timer_prescaler = 40;
const uint32_t adc_timer_arr = 64;
uint32_t adc_samplerate = 0;
// Time to *perform* one sample
// 11.6:
// Tconv  = sampling time + 12.5
// with 7.5 cycles sampling time, TConv = 19.5 adc cycles
// with 72MHz cpu clock, and 8 divider on adc
// total time = (19.5 cycles) / (72 MHz / 8) = 2.17µs
// so can go up to 460ksps with current settings, but transmission is bottleneck
//
// With adc div 8,  sample time 77.5 cycles, 72MHz clock
// 1 adc cycle = 8/72MHz
// total_time = 77.5 + 12.5 adc cycles = 90 adc cycles = 10µs
// up to 100ksps
//
// With adc div 8, sample time 239.5 cycles
// total time = 239.5 + 12.5 adc cycles = 252 adc cycles = 28µs
// up to 35ksps


//Channels to be acquired.
const uint32_t pa0[] = {GPIOA, GPIO0};
const uint32_t pa1[] = {GPIOA, GPIO1};
const uint32_t * const pins[] = {pa0, pa1};

#define NPINS 2
const size_t nPins = NPINS;

// Array for the ADC data
#define ADC_BUF_LEN 200*NPINS
const size_t adc_buffer_len = ADC_BUF_LEN;
uint16_t adc_buffer[ADC_BUF_LEN];

void setup_adc()
{
  rcc_periph_clock_enable(RCC_ADC1);
  //rcc_periph_clock_enable(RCC_ADC2);
  adc_power_off(ADC1);
  //adc_power_off(ADC2);

  // Set up our analog pin(s)
  for (size_t j = 0; j < nPins; j++)
    gpio_set_mode(pins[j][0], GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, pins[j][1] );

  // Slow ADC the fuck down, this sets the ADCPRE (hopefully)
  // 00: PCLK2/2
  // 01: PCLK2/4
  // 10: PCLK2/6
  // 11: PCLK2/8
  uint32_t div = RCC_CFGR_ADCPRE_DIV8;
  switch (adc_clock_divider) {
    case 2:
      div = RCC_CFGR_ADCPRE_DIV2;
      break;
    case 4:
      div = RCC_CFGR_ADCPRE_DIV4;
      break;
    case 6:
      div = RCC_CFGR_ADCPRE_DIV6;
      break;
    case 8:
      div = RCC_CFGR_ADCPRE_DIV8;
      break;
    default:
      error |= INVALID_ADC_CLK_DIV;
  };
  rcc_set_adcpre(div);

  adc_set_sample_time_on_all_channels(ADC1, adc_sampletime);
  //adc_set_sample_time_on_all_channels(ADC2, adc_sampletime);

  adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_TIM1_CC1);
  //adc_enable_external_trigger_regular(ADC2, ADC_CR2_EXTSEL_TIM1_CC1);
  //adc_set_resolution(ADC1, ADC_);

  //set the ADC in Scan mode (use ADC1 only for simplicity)
  adc_enable_scan_mode(ADC1);

  //set how many and which pins to convert.
  /*uint8_t ch_arr1[] = {0}; //ADC12_IN0
  uint8_t ch_arr2[] = {1}; //ADC12_IN1
  adc_set_regular_sequence(ADC1, 1, ch_arr1);
  adc_set_regular_sequence(ADC2, 1, ch_arr2);*/
  uint8_t ch_arr[] = {0,1};
  adc_set_regular_sequence(ADC1, 2, ch_arr);

  //adc_set_continuous_conversion_mode(ADC1)
  adc_set_single_conversion_mode(ADC1); //triggered by the timer

  adc_power_on(ADC1);
  //adc_power_on(ADC2);

  //set the DMA transfer for the ADC.
  //in this case we want to increment the memory side and run it in circular mode
  rcc_periph_clock_enable(RCC_DMA1);
  dma_channel_reset(DMA1, DMA_CHANNEL1);

  dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
  dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);
  dma_set_number_of_data(DMA1, DMA_CHANNEL1, adc_buffer_len);
  dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);

  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
  dma_enable_circular_mode(DMA1, DMA_CHANNEL1);

  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
  dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL1);
  dma_enable_channel(DMA1, DMA_CHANNEL1);
  nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
  adc_enable_dma(ADC1);

  // ADC calibration should be done at startup
  adc_calibrate(ADC1);
  //adc_calibrate(ADC2);

  //start the conversion.
  adc_start_conversion_direct(ADC1);

  adc_enable_awd_interrupt(ADC1);
  nvic_enable_irq(NVIC_ADC1_2_IRQ);

  adc_samplerate = rcc_ahb_frequency / adc_timer_prescaler / adc_timer_arr;
}

void adc_awd_set_threshold(uint8_t  chan, uint16_t vh, uint16_t vl)
{
    // set_awd_channel(ADC1, channel);
    adc_enable_analog_watchdog_on_selected_channel(ADC1, chan & ADC_CR1_AWDCH_MASK);

    adc_set_watchdog_low_threshold(ADC1, vl);
    adc_set_watchdog_high_threshold(ADC1, vh);

    adc_enable_analog_watchdog_regular(ADC1);
}

void adc_stop()
{
  timer_disable_counter(TIM1);
}

void adc_start()
{
  timer_enable_counter(TIM1);
  bsp_set_pin_error(false);
}
