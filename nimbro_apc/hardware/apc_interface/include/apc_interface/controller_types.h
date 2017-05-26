// Feedback Types for hardware information
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#ifndef CONTROLLER_TYPES_H
#define CONTROLLER_TYPES_H

#include <stdint.h>
#include <string>

namespace apc_interface
{

struct ServoStatus
{
	enum struct Type
	{
		LINEAR,
		REVOLUTE,
	};
	std::string name;
	uint16_t id;
	uint16_t flags;
	Type type;

	int16_t goal_position_ticks;
	int16_t goal_position_offset = 0;
	double scaleToMetric = 1.0;
	uint16_t max_torque;
	bool torque_enabled;

	int16_t position_ticks;
	uint16_t velocity_ticks;
	uint16_t load;
	uint8_t voltage;
	uint8_t temperature;
	uint8_t error;

	double goal_pos;
	double position;
	double velocity;
	double effort;

};

struct ControllerStatus
{
	bool vacuum_on;
	bool vacuum_power_on;
	uint8_t light_duty;
	uint8_t status;
	uint16_t air_pressure;
	uint16_t timeouts;
	std::vector<ServoStatus> servos;
};

struct ControllerDebugStatus
{
	bool comm_statuscheck_uninitialized;
	bool comm_poscmd_uninitialized;
	bool comm_poscmd_invld_id;
	bool comm_trqen_uninitialized;
	bool comm_trqen_invld_id;
	bool dxl_bulkread_invld_id;
	bool dxl_bulkread_invld_size;
};

struct ServoPositionCmd
{
	uint16_t id;
	int16_t goal_position;
};

struct ServoTorqueEnable
{
	uint16_t id;
	uint16_t enable;
};

struct ServoMaxTorque
{
	uint16_t id;
	uint16_t torque;
};
}

#endif
