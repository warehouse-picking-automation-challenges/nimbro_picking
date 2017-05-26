// DXL MX Communication
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#ifndef APC_DXL_H
#define APC_DXL_H

#include <stdint.h>
#include <dxl_master/dxl_master.h>

namespace dxl
{
struct ServoStatus
{
	uint16_t id;
	uint16_t flags;

	uint8_t torque_enable;
	uint16_t max_torque;
	int16_t goal_position;
	int16_t goal_position_offset;
	uint8_t offset_initialized;

	int16_t position;
	uint16_t speed;
	uint16_t load;
	uint8_t voltage;
	uint8_t temperature;
	uint8_t error;
};
extern ServoStatus servos[];
uint8_t servo_index(uint16_t id);
extern uint8_t valid_servos;
extern uint8_t br_servos;

void init();
extern uint8_t comm_errors;
void reset_errors();


void send_position_cmds();
void send_torque_enable();
void send_max_torque();
uint16_t read_feedback(uint16_t timeout = 500);
uint16_t read_eeprom(uint16_t timeout = 500);
extern uint16_t read_timeouts;
}


#endif
