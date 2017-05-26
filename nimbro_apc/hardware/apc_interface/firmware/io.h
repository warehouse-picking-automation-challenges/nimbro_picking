// APC IO control
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#ifndef APC_IO_H
#define APC_IO_H

#include <stdint.h>

namespace io
{
void init();
void init_RS485();
void set_RS485_output(bool out);
void reset_RS485_txComplete();
bool is_RS485_txComplete_set();
void set_vacuum(bool on);
void set_vacuum_power(bool on);
bool is_vacuum_on();
bool is_vacuum_power_on();
void dim_light(uint8_t duty);
uint8_t light_duty();
namespace debug
{
void toggle_board_led();
}
}


#endif
