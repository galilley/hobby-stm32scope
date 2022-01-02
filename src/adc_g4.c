#include "bsp.h"
#include "adc.h"
#include <types.h>

extern volatile uint32_t error;

const uint8_t adc_clock_divider = 4;
const uint8_t adc_sampletime = ADC_SMPR_SMP_6DOT5CYC; // Also change in parameter_sanity_check

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
  rcc_periph_clock_enable(RCC_GPIOA);
  //rcc_periph_clock_enable(RCC_ADC2);
  adc_power_off(ADC1);
  adc_disable_deeppwd(ADC1);
  adc_enable_regulator(ADC1);
  //adc_power_off(ADC2);

  // Set up our analog pin(s)
  rcc_periph_clock_enable(RCC_GPIOA);
  for (size_t j = 0; j < nPins; j++) {
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_PULLDOWN, GPIO0 | GPIO1);
  }

  uint32_t div = ADC_CCR_CKMODE_DIV4;
  switch (adc_clock_divider) {
    case 1:
      div = ADC_CCR_CKMODE_DIV1;
      break;
    case 2:
      div = ADC_CCR_CKMODE_DIV2;
      break;
    case 4:
      div = ADC_CCR_CKMODE_DIV4;
      break;
    default:
      error |= INVALID_ADC_CLK_DIV;
  };
  adc_set_clk_prescale(ADC1, div);

  adc_set_sample_time_on_all_channels(ADC1, adc_sampletime);
  //adc_set_sample_time_on_all_channels(ADC2, adc_sampletime);

  //adc_disable_external_trigger_regular(ADC1);
  adc_enable_external_trigger_regular(ADC1, ADC12_CFGR1_EXTSEL_TIM1_TRGO, ADC_CFGR1_EXTEN_RISING_EDGE);
  adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);

  //set how many and which pins to convert.
  /*uint8_t ch_arr1[] = {1}; //ADC12_IN1
  uint8_t ch_arr2[] = {2}; //ADC12_IN2
  adc_set_regular_sequence(ADC1, 1, ch_arr1);
  adc_set_regular_sequence(ADC2, 1, ch_arr2);*/
  uint8_t ch_arr[] = {1,2};
  adc_set_regular_sequence(ADC1, 2, ch_arr);

  //adc_set_continuous_conversion_mode(ADC1);
  adc_set_single_conversion_mode(ADC1); //triggered by the timer

  // ADC calibration should be done at startup
  adc_calibrate(ADC1);
  //adc_calibrate(ADC2);

  adc_power_on(ADC1);
  //adc_power_on(ADC2);

  //set the DMA transfer for the ADC.
  //in this case we want to increment the memory side and run it in circular mode
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

  dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL1);
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
  dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL1);
  
  // see DMAMUX mapping section in RM to find the right channel number
  dmamux_set_dma_channel_request(DMAMUX1, 1, DMAMUX_CxCR_DMAREQ_ID_ADC1);  
  
  dma_enable_channel(DMA1, DMA_CHANNEL1);
  nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
  adc_enable_dma_circular_mode(ADC1);
  adc_enable_dma(ADC1);

  //start the conversion.
  adc_start_conversion_regular(ADC1);

  /*adc_enable_awd_interrupt(ADC1);
  nvic_enable_irq(NVIC_ADC1_2_IRQ);*/

  adc_samplerate = rcc_ahb_frequency / adc_timer_prescaler / adc_timer_arr;
}

void adc_awd_set_threshold(uint8_t  chan, uint16_t vh, uint16_t vl)
{
    // set_awd_channel(ADC1, channel);
    /*adc_enable_analog_watchdog_on_selected_channel(ADC1, chan & ADC_CR1_AWDCH_MASK);

    adc_set_watchdog_low_threshold(ADC1, vl);
    adc_set_watchdog_high_threshold(ADC1, vh);

    adc_enable_analog_watchdog_regular(ADC1);*/
}
void adc_stop()
{
  timer_disable_counter(TIM1);
}

void adc_start()
{
  error = NO_ERROR;
  timer_enable_counter(TIM1);
  bsp_set_pin_error(false);
}
