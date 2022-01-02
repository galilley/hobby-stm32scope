#include <stdio.h>
#include "control.h"
#include "bsp.h"
#include "adc.h"

volatile enum cmd_state_type cmd_state = CMD_STATE_IDLE;
extern volatile uint32_t error;
extern void trigger_adc_buffer_to_usart(bool);

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
  adc_awd_set_threshold(channel, value_high, value_low);

  cmd_state = CMD_STATE_IDLE;
}

void print_uint32_blocking(uint32_t v)
{
    char buffer[16];
    char * pb = &(buffer[0]);
    int cx = snprintf(buffer, 16, "%lu\n", v);
    if (cx > 0) {
        while (cx-- > 0) {
            usart_send_blocking(USART2, *(pb)++);
        }
    }
    else {
        error |= SNPRINTF_ERROR;
    }
}

void handle_cmd()
{
  uint8_t cmd = cmd_buf[0];
  switch (cmd)
  {
  case CMD_STOP:
    adc_stop();
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_START:
    adc_start();
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_GET_SAMPLERATE:
    print_uint32_blocking(adc_samplerate);
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_GET_ERROR:
    print_uint32_blocking(error);
    cmd_state = CMD_STATE_IDLE;
    break;
  case CMD_SET_TRIGGER:
    handle_trigger_cmd();
    break;
  }
}
