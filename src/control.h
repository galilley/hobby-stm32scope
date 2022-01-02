#ifndef CONTROL_H
#define CONTROL_H

#include <types.h>

extern volatile enum cmd_state_type cmd_state;

void handle_trigger_cmd(void);
void handle_cmd(void);

#endif // CONTROL_H
