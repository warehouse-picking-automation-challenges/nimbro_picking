// Register definitions for MX servos
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DXL_MASTER_MX_H
#define DXL_MASTER_MX_H

enum MXRegister
{
	MX_MODEL_NUMBER         = 0x00,

	MX_VERSION              = 0x02,
	MX_ID                   = 0x03,
	MX_BAUD                 = 0x04,
	MX_RETURN_DELAY_TIME    = 0x05,
	MX_CW_ANGLE_LIMIT       = 0x06,

	MX_CCW_ANGLE_LIMIT      = 0x08,

	MX_TEMP_LIMIT_HIGH      = 0x0B,
	MX_VOLT_LIMIT_LOW       = 0x0C,
	MX_VOLT_LIMIT_HIGH      = 0x0D,
	MX_MAX_TORQUE           = 0x0E,

	MX_STATUS_RETURN_LEVEL  = 0x10,
	MX_ALARM_LED            = 0x11,
	MX_ALARM_SHUTDOWN       = 0x12,
	MX_MULTI_TURN_OFFSET    = 0x14,

	MX_RESOLUTION_DIVIDER   = 0x16,

	// RAM

	MX_TORQUE_ENABLE        = 0x18,
	MX_LED                  = 0x19,
	MX_D_GAIN               = 0x1A,
	MX_I_GAIN               = 0x1B,
	MX_P_GAIN               = 0x1C,
	MX_GOAL_POSITION        = 0x1E,

	MX_MOVING_SPEED         = 0x20,

	MX_TORQUE_LIMIT         = 0x22,

	MX_PRESENT_POSITION     = 0x24,

	MX_PRESENT_SPEED        = 0x26,

	MX_PRESENT_LOAD         = 0x28,

	MX_PRESENT_VOLTAGE      = 0x2A,
	MX_PRESENT_TEMPERATURE  = 0x2B,
	MX_REGISTERED           = 0x2C,
	MX_MOVING               = 0x2E,
	MX_LOCK                 = 0x2F,
	MX_PUNCH                = 0x30,

	MX_CURRENT              = 0x44,

	MX_TORQUE_CTRL_ENABLE   = 0x46,
	MX_GOAL_TORQUE          = 0x48,
};

#endif
