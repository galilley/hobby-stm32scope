#include "bsp.h"
#include <types.h>
#include "control.h"

const uint32_t baud_rate = 1200000;

volatile uint8_t cmd_buf[6];
volatile size_t cmd_buf_ix = 0;

static const uint32_t error_pin[] = {GPIOB, GPIO12};

void bsp_set_pin_error(bool val)
{
    if (val) {
        gpio_set(error_pin[0], error_pin[1]);
    }
    else {
        gpio_clear(error_pin[0], error_pin[1]);
    }
}

void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_set_mode(error_pin[0], GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, error_pin[1]);
}

/* Set STM32 to 72 MHz. */
void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

void usart1_isr(void)
{
  // Reading clears the thing
#ifdef STM32G4
  while (usart_get_flag(USART1, USART_ISR_RXNE)) {
#else
  while (usart_get_flag(USART1, USART_SR_RXNE)) {
#endif
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

void usart_setup(void)
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
