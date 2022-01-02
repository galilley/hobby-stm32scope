#ifndef TYPES_H
#define TYPES_H

#include <stdlib.h>

enum commands
{
  CMD_STOP = 1,
  CMD_START = 2,
  CMD_GET_SAMPLERATE = 3,
  CMD_GET_ERROR = 4,
  CMD_SET_TRIGGER = 5,
};

enum cmd_state_type
{
  CMD_STATE_IDLE = 0,
  CMD_STATE_READ_ARG = 1,
};

enum error_type
{
  NO_ERROR = 0,
  USART_BUSY_ERROR = 1 << 0,
  SNPRINTF_ERROR = 1 << 1,
  ADC_DMA_ERROR = 1 << 2,
  USART_DMA_ERROR = 1 << 3,
  INVALID_ADC_CLK_DIV = 1 << 4,
  BAD_PARAMETERS_TIMER_FASTER_THAN_ADC = 1 << 5,
  BAD_PARAMETERS_SERIAL_TOO_SLOW = 1 << 6,
};

#endif // TYPES_H
