
// If you change something here, please increment the PROTOCOL_VERSION
// in drc_proto_constants.h

struct ServoConfiguration
{
	uint16_t id;
	uint16_t max_velocity;
	uint32_t fadein_velocity;
	uint16_t flags;
};

struct ServoCommand
{
	uint16_t id;
	int32_t command;
};

struct ServoFeedback
{
	uint16_t id;
	int32_t position;
	int16_t torque;
	uint8_t packet_state;
	uint8_t temperature;
	uint16_t timeouts;
	uint8_t error;
	uint8_t voltage;
};

struct IMUFeedback
{
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;

	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
};

// PC -> µC: connect
msg ConnectMsg
{
};

// µC -> PC: connect reply
msg ConnectMsgReply
{
	uint32_t version;
	uint8_t error;
};

// PC -> µC: initial servo configuration
msg ServoConfigurationMsg
{
	ServoConfiguration servos[];
	uint8_t ttl_bus_address;
	uint8_t ttl_bus_base_id;
};

// µC -> PC: Servos configured
msg ServoConfiguredMsg
{
	uint8_t error;
	uint32_t servo_mask;
};

// PC -> µC: Position/Velocity commands
msg ServoCommandMsg
{
	ServoCommand servos[];

	// 1 if the software emergency stop is currently active
	uint8_t software_stop;

	// Request ID - copied to response
	uint16_t request_id;
};

// µC -> PC: Feedback from commanded servos
msg ServoFeedbackMsg
{
	uint8_t error;
	uint8_t packet_errors;
	uint8_t flags;
	uint16_t distance;
	uint16_t request_id;
	ServoFeedback servos[];
};

// PC -> µC: Fade In command
struct FadeCommand
{
	uint8_t id;
	uint16_t torque;
};

msg FadeCommandMsg
{
	uint16_t fade_time;
	FadeCommand servos[];
};

msg FadeCommandMsgReply
{
	uint8_t error;
};

// PC -> µC: Enter passthrough mode
// Passthrough mode is exited with the PASSTHROUGH_EXIT_KEY.
msg StartPassthroughMsg
{
};

msg StartPassthroughReply
{
	uint8_t active;
};

msg RebootServoMsg
{
	uint8_t id;
};

// PC -> µC: Reboot the µC (entering the bootloader for a short time)
msg RebootControllerMsg
{
	uint32_t key1; // REBOOT_KEY1
	uint32_t key2; // REBOOT_KEY2
};

// PC -> µC: Get Introspection Packet (only for debugging)
msg IntrospectionMsg
{
};

struct ServoIntrospection
{
	uint16_t id;
	uint16_t flags;
	uint32_t command;
	uint32_t commandLastWritten;
	uint8_t commandLastWrittenValid;
	uint32_t current_position;
	uint16_t max_torque;
	uint32_t max_velocity;
	int16_t present_current;
	uint8_t present_temperature;
	uint8_t present_voltage;
	uint8_t packet_state;
	uint16_t timeouts;
	uint8_t error;
};

// µC -> PC: Introspection of µC State
msg IntrospectionReplyMsg
{
	ServoIntrospection servos[];
	uint8_t faded_after_connect;
	uint16_t num_pro_servos;
	uint16_t num_ttl_servos;

};


msg SetServoParamMsg
{
	uint8_t id;
	uint16_t pgain;
};

// PC -> µC: Stop Communication to a TTL Servo
msg SetSilenceMsg
{
	uint8_t silence_sequence;
};
