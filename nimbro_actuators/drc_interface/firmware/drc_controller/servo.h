// Global servo array
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#include "../protocol/drc_proto_constants.h"

struct Servo
{
	uint16_t id;
	uint16_t flags;

	// Command
	uint32_t command;
	uint32_t commandLastWritten;
	bool commandLastWrittenValid;

	// Feedback
	uint32_t current_position;
	// ...
	uint16_t max_torque;
	uint32_t max_velocity;
	uint32_t fadein_velocity;
	int16_t present_current;
	uint8_t present_temperature;
	uint8_t present_voltage;

	uint8_t packet_state;

	uint16_t timeouts;

	// Error byte from the last received status packet
	uint8_t error;
};

Servo* servo_find(uint16_t id, uint8_t* index);

extern Servo g_servos[MAX_NUM_SERVOS];
extern uint8_t g_num_servos;

extern Servo g_ttl_servos[MAX_NUM_SERVOS];
extern uint8_t g_ttl_bus_address;
extern uint8_t g_num_ttl_servos;

extern uint8_t g_current_temp_read_id;

extern uint16_t g_cycles_since_last_write;
extern uint16_t g_cycles_since_estop;

extern uint8_t g_packet_errors;

extern uint8_t g_bulkread_indices[MAX_NUM_SERVOS];

extern uint16_t g_distance_sensor_value;

void servo_init();

#endif
