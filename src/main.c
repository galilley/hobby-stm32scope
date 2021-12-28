// A0 and A1,   used to meassure signal
// USART used for data, pins, A9 TX, A10 RX
// TIM1 (timer) that tells ADC to meassure
// ADC (analog 2 digital converter)  gives us 12bit measurment of voltage
// DMA (direct memory access)  shuffles them to RAM
// DMA shuffles the RAM out over serial data port to computer

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <stdlib.h>
#include <stdio.h>

static const uint32_t baud_rate = 1200000;
static const uint32_t error_pin[] = {GPIOB, GPIO12};

static const uint8_t adc_clock_divider = 2;
static const uint8_t adc_sampletime = ADC_SMPR_SMP_7DOT5CYC; // Also change in parameter_sanity_check

static const uint32_t adc_timer_prescaler = 40;
static const uint32_t adc_timer_arr = 64;
static uint32_t adc_samplerate = 0;
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



enum commands
{
  CMD_STOP = 1,
  CMD_START = 2,
  CMD_GET_SAMPLERATE =  3,
  CMD_GET_ERROR = 4,
  CMD_SET_TRIGGER = 5,
};
enum cmd_state_type
{
  CMD_STATE_IDLE = 0,
  CMD_STATE_READ_ARG = 1,
};

volatile enum cmd_state_type cmd_state = CMD_STATE_IDLE;
volatile uint8_t cmd_buf[6];
volatile size_t cmd_buf_ix = 0;

enum error_type
{
  NO_ERROR = 0,
  USART_BUSY_ERROR = 1,
  ADC_DMA_ERROR = 1 << 2,
  USART_DMA_ERROR = 1 << 3,
  INVALID_ADC_CLK_DIV = 1 << 4,
  BAD_PARAMETERS_TIMER_FASTER_THAN_ADC = 1 << 5,
  BAD_PARAMETERS_SERIAL_TOO_SLOW = 1 << 6,
};
volatile uint32_t error = NO_ERROR;

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
  dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART_DR(USART1));
  dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)(adc_buffer + (second_half ? adc_buffer_len / 2: 0)));
  // Note; /2 because we're transfering half of it, *2 because 16bit data sent as 8bit
  dma_set_number_of_data(DMA1, DMA_CHANNEL4, (adc_buffer_len) * 2 / 2);
  dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_LOW);
  dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
  dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
  dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL4);
  dma_enable_channel(DMA1, DMA_CHANNEL4);
}

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
    timer_set_oc_value(TIM1, TIM_OC1, adc_timer_arr / 2);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_set_oc_polarity_low(TIM1, TIM_OC1);
    timer_enable_counter(TIM1);
}

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

void stop()
{
  timer_disable_counter(TIM1);
}

void start()
{
  timer_enable_counter(TIM1);
  gpio_clear(error_pin[0], error_pin[1]);
}

/**
 * Trigger mode
 * - Analog watchog is set up for the specified threshold
 * - When watchdog triggers, the measured value will be tagged
 * - User can then detect this tag
 * - TODO tag should go in the high unused bits, but how to target it correctly in time?
 */
void handle_trigger_cmd() {
  uint8_t cmd = cmd_buf[0];
  if (cmd != CMD_SET_TRIGGER) {
    // TODO set error
    return;
  }
  if (cmd_buf_ix < 5) {
    // Wait for more arguments
    cmd_state = CMD_STATE_READ_ARG;
    return;
  }

  uint8_t channel = cmd_buf[1];
  uint16_t value_low = (((uint16_t)cmd_buf[2]) << 8) + cmd_buf[3]; // TOD low and high, maybe multi-byte
  uint16_t value_high = (((uint16_t)cmd_buf[4]) << 8) + cmd_buf[5];

  // TODO check that arguments make sense


  // set_awd_channel(ADC1, channel);
  adc_enable_analog_watchdog_on_selected_channel(ADC1, channel & ADC_CR1_AWDCH_MASK);

  adc_set_watchdog_low_threshold(ADC1, value_low);
  adc_set_watchdog_high_threshold(ADC1, value_high);

  adc_enable_analog_watchdog_regular(ADC1);

  cmd_state = CMD_STATE_IDLE;
}

void handle_cmd()
{
  char buffer[16];
  char * pb = buffer;
  int cx = 0;
  uint8_t cmd = cmd_buf[0];
  switch (cmd)
  {
  case CMD_STOP:
    stop();
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_START:
    start();
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_GET_SAMPLERATE:
    cx = snprintf( buffer, 16, "%lu\n", adc_samplerate);
    if (cx > 0) {
        while (cx-- > 0) {
            usart_send_blocking(USART1, (*pb)++);
        }
    }
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_GET_ERROR:
    cx = snprintf( buffer, 16, "%lu\n", error);
    if (cx > 0) {
        while (cx-- > 0) {
            usart_send_blocking(USART1, (*pb)++);
        }
    }
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_SET_TRIGGER:
    handle_trigger_cmd();
    break;
  }
}

void usart1_isr(void)
{
  // Reading clears the thing
  while (usart_get_flag(USART1, USART_SR_RXNE)) {
    switch (cmd_state) {
      case CMD_STATE_IDLE:
        cmd_buf_ix = 0;
        // fallthrough
      case CMD_STATE_READ_ARG: {
        cmd_buf[cmd_buf_ix] = usart_recv(USART1);
        if (cmd_buf[0] != 0) {
          handle_cmd();
          cmd_buf_ix += 1;
        }
        break;
      }
    }
  }
}


void parameter_sanity_check() {
  uint32_t channels = nPins;

  // This is 10(adc_smpr + 12.5)
  uint32_t sampletime_cycles_x10 = 75 + 125;

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

static void usart1_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    /* Setup UART parameters. */
    usart_set_baudrate(USART1, baud_rate);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);

    // Prepare usart DMA
    usart_enable_tx_dma(USART1);

    // Interrupt on incoming
    usart_enable_rx_interrupt(USART1);
    nvic_enable_irq(NVIC_USART1_IRQ);
}

void setup()
{
  usart1_setup();
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_set_mode(error_pin[0], GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, error_pin[1]);
  parameter_sanity_check();

  for (size_t i = 0; i < adc_buffer_len; i++)
  {
    adc_buffer[i] = 0;
  }

  nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

  setup_timer();
  setup_adc();

  stop(); // TODO no!
}

void loop()
{
  if (error != NO_ERROR)
  {
    gpio_set(error_pin[0], error_pin[1]);
  }
  // Everything relies on interrupts, do nothing
  asm("wfi \n");
};

/* Set STM32 to 72 MHz. */
static void clock_setup(void)
{
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

int main(void)
{
	clock_setup();
    setup();

	while (1) {
        loop();
	}

	return 0;
}
