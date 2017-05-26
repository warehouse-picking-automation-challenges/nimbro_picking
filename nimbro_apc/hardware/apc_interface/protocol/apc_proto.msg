// Protocol definition for hardware communication used for the APC
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

// Protocol Constants
custom
{

struct ProtoConstants
{
	// NOTE(sebastian): Please increment version on each protocol change.
	enum
	{
		PROTOCOL_VERSION = 4,
		REBOOT_KEY1 = 0xABFDFFFF,
		REBOOT_KEY2 = 0xACFDFFFF,
		NUM_SERVOS = 3,
	};
};

struct RebootControllerMsgStatus
{
	enum
	{
		SUCCESS   = 0,
		WRONG_KEY = (1 << 0),
	};
};

struct ControllerStatusFlags
{
	enum
	{
		SERVO_IN_ERR        = 0,
		VACUUM_ON           = (1 << 0),
		BULKREAD_FAILED     = (1 << 1),
		SERVO_UNINITIALIZED = (1 << 2),
		VACUUM_POWER_ON     = (1 << 3),
	};
};

struct DbgStatusFlags
{
	enum
	{
		DXL_BR_INVLD_ID   = (0 << 0),
		DXL_BR_INVLD_SIZE = (1 << 0),
	};
	enum
	{
		COMM_STATUS_UNINITIALIZED = (0 << 0),
		COMM_POSCMD_UNINITIALIZED = (1 << 0),
		COMM_POSCMD_INVLD_ID      = (1 << 1),
		COMM_TRQEN_UNINITIALIZED  = (1 << 2),
		COMM_TRQEN_INVLD_ID       = (1 << 3),
	};
};
}

// PC -> µC: connect
msg ConnectRequestMsg
{
};

// µC -> PC : connect
msg ConnectAck
{
	uint32_t protocol_version;
};


// PC -> µC : reboot controller
msg RebootControllerMsg
{
	uint32_t key1;
	uint32_t key2;
};
// µC -> PC
msg RebootControllerAck
{
	uint8_t status;
};

// PC -> µC: Turn Vacuum On/Off
msg SwitchVacuumMsg
{
	uint8_t on;
};

msg SwitchVacuumPowerMsg
{
	uint8_t on;
};

// PC -> µC: Dim SR300 LEDs
msg DimLightMsg
{
	uint8_t duty;
};

msg ServoInitializationMsg
{
	uint16_t ids[ProtoConstants::NUM_SERVOS];
};

struct ServoPositionCmd
{
	uint16_t id;
	int16_t goal_position;
};

msg ServoPositionCmdMsg
{
	ServoPositionCmd cmds[ProtoConstants::NUM_SERVOS];
};

struct ServoMaxTorque
{
	uint16_t id;
	uint16_t torque;
};

msg ServoMaxTorqueMsg
{
	ServoMaxTorque cmds[ProtoConstants::NUM_SERVOS];
};

struct ServoTorqueEnable
{
	uint16_t id;
	uint8_t enable;
};

msg ServoTorqueEnableMsg
{
	ServoTorqueEnable cmds[ProtoConstants::NUM_SERVOS];
};


msg ControllerStatusRqst
{
};

struct ServoStatus
{
	uint16_t id;
	uint16_t flags;

	int16_t goal_position;

	int16_t position;
	uint16_t speed;
	uint16_t load;
	uint8_t voltage;
	uint8_t temperature;
	uint8_t error;
};

msg ControllerStatusMsg
{
	uint8_t status;
	uint8_t light_duty;
	uint16_t timeouts;
	uint16_t air_pressure;
	ServoStatus servos[ProtoConstants::NUM_SERVOS];
};

msg EEPROMStatusRqst
{
};

struct EEPROMStatus
{
	uint16_t id;
	uint16_t max_torque;
};

msg EEPROMStatusMsg
{
	EEPROMStatus eeprom[ProtoConstants::NUM_SERVOS];
};


msg DbgStatusRqst
{
};

msg DbgStatusMsg
{
	uint8_t dxl_errors;
	uint8_t comm_errors;
};

