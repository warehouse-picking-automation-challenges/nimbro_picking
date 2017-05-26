// Global servo array
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "servo.h"

Servo g_servos[MAX_NUM_SERVOS];
uint8_t g_num_servos;
uint8_t g_packet_errors = 0;

Servo g_ttl_servos[MAX_NUM_SERVOS];

uint8_t g_num_ttl_servos = 0;
uint8_t g_ttl_bus_address;

uint16_t g_cycles_since_last_write = 0;
uint16_t g_cycles_since_estop = 0;

uint8_t g_bulkread_indices[MAX_NUM_SERVOS];

uint16_t g_distance_sensor_value;

void servo_init()
{
	g_num_servos = 0;
	g_num_ttl_servos = 0;

	for(unsigned int i = 0; i < MAX_NUM_SERVOS; ++i)
	{
		g_servos[i].current_position = 2;
		g_servos[i].packet_state = 255;
		g_servos[i].commandLastWritten = 0;
		g_servos[i].commandLastWrittenValid = false;
		g_servos[i].timeouts = 0;
		g_servos[i].error = 0;
	}

	for(unsigned int i = 0; i < MAX_NUM_SERVOS; ++i)
	{
		g_ttl_servos[i].current_position = 2;
		g_ttl_servos[i].packet_state = 255;
		g_ttl_servos[i].timeouts = 0;
		g_ttl_servos[i].error = 0;
		g_ttl_servos[i].commandLastWritten = 0;
		g_ttl_servos[i].commandLastWrittenValid = false;
	}
}

Servo* servo_find(uint16_t id, uint8_t* index)
{
	for(uint8_t i = 0; i < g_num_servos; ++i)
	{
		if(g_servos[i].id == id)
		{
			if(index) *index = i;
			return &g_servos[i];
		}
	}
	for(uint8_t i = 0; i < g_num_ttl_servos; ++i)
	{
		if(g_ttl_servos[i].id == id)
		{
			if (index) *index = i;
			return &g_ttl_servos[i];
		}
	}

	return 0;
}
