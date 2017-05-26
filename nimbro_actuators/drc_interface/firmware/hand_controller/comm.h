// Communication control
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef HAND_CONTROLLER_COMM_H
#define HAND_CONTROLLER_COMM_H

void comm_init(void);

void dxlpro_slave_send(uint8_t c);
void dxl_master_send(const uint8_t* data, uint8_t len);

#endif
