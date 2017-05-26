// Constants used in the DRC proto
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DRC_PROTO_CONSTANTS_H
#define DRC_PROTO_CONSTANTS_H

#include <stdint.h>

const int PROTOCOL_VERSION = 25;

enum ServoFlags
{
	SERVO_FLAG_VELOCITY_CONTROL = (1 << 0),
	SERVO_FLAG_IS_TTL = (1 << 1),
};

enum ServoFeedbackError
{
	// This is used as a bitfield!
	SERVO_FEEDBACK_ERROR_NONE = 0,
	SERVO_FEEDBACK_ERROR_NOT_CONFIGURED = 1,
	SERVO_FEEDBACK_ERROR_INVALID_ID = 2,
	SERVO_FEEDBACK_ERROR_EMERGENCY_PRESSED = 4,
};

const uint8_t SERVO_VOLTAGE_OFFSET = 100;

enum ServoFeedbackFlags
{
	SERVO_FEEDBACK_FLAG_IMU_PRESENT = (1 << 0),
};

enum FadeCommandErrorFlags
{
	FADE_COMMAND_ERROR_FLAG_EMERGENCY_STOP = (1 << 0),
};

enum ConnectMsgErrorFlags
{
	CONNECT_MSG_ERROR_FLAG_NOT_SHUT_DOWN = (1 << 0),
};

enum ConfigureMsgErrorFlags
{
	CONFIGURE_MSG_ERROR_FLAG_NOT_SHUT_DOWN = (1 << 0)
};

const int MAX_NUM_SERVOS = 16;

const uint8_t PASSTHROUGH_EXIT_KEY[] = {
	0xFF, 0xFF, 0xFD, 0xFA,
	0xFF, 0xFF, 0xFD, 0xFA,
};

const uint32_t REBOOT_KEY1 = 0xABFDFFFF;
const uint32_t REBOOT_KEY2 = 0xACFDFFFF;

#endif
