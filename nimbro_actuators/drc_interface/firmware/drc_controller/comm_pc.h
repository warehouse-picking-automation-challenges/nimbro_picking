// Communication with PC
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COMM_PC_H
#define COMM_PC_H

#include <stdint.h>

void comm_pc_processByte(uint8_t c);
void comm_pc_send_emergency_switch_state(bool pressed);

extern bool g_softwareStop;

#endif
