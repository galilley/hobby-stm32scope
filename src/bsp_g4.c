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
    gpio_mode_setup(error_pin[0], GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, error_pin[1]);
}

/* Set STM32 to 170 MHz. */
void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_170MHZ]);
}

void usart2_isr(void)
{
  // Reading clears the thing
  while (usart_get_flag(USART2, USART_ISR_RXNE)) {
    switch (cmd_state) {
      case CMD_STATE_IDLE:
        cmd_buf_ix = 0;
        // fallthrough
      case CMD_STATE_READ_ARG: {
        cmd_buf[cmd_buf_ix] = usart_recv(USART2);
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
    rcc_periph_clock_enable(RCC_USART2);
    
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
    
    /* Setup UART parameters. */
    usart_set_baudrate(USART2, baud_rate);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART2);

    // see DMAMUX mapping section inn RM to find the right channel number
    dmamux_set_dma_channel_request(DMAMUX1, 4, DMAMUX_CxCR_DMAREQ_ID_UART2_TX);

    // Interrupt on incoming
    usart_enable_rx_interrupt(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);
}
