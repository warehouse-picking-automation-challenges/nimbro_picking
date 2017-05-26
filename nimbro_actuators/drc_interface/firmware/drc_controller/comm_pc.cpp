// Communication with PC
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "comm_pc.h"

#include "drc_proto.h"
#include "../protocol/drc_proto_constants.h"
#include "servo.h"
#include "uart_wrapper.h"
#include "comm_dxl.h"
#include "io.h"
#include "imu.h"

#include <libucomm/envelope.h>
#include <libucomm/checksum.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

class PCUARTWriter
{
public:
	bool writeChar(uint8_t c)
	{
		ftdi_pc_putc(c);
		return true;
	}
};

static bool g_faded_after_connect = false;

typedef uc::EnvelopeWriter<uc::InvertedModSumGenerator, PCUARTWriter> EnvelopeWriter;
typedef uc::IO<EnvelopeWriter, uc::IO_W> SimpleWriter;
typedef Proto<SimpleWriter> WProto;

typedef uc::EnvelopeReader<uc::InvertedModSumGenerator, 256> EnvelopeReader;
typedef uc::IO<EnvelopeReader, uc::IO_R> SimpleReader;
typedef Proto<SimpleReader> RProto;

EnvelopeReader input;
PCUARTWriter writer;
EnvelopeWriter output(&writer);

bool g_softwareStop = false;

bool fillFeedback(WProto::ServoFeedback* fb, uint8_t i)
{
	Servo* servos;
	if(i < g_num_servos)
	{
		servos = g_servos;
		i = g_bulkread_indices[i];
	}
	else
	{
		servos = g_ttl_servos;
		i -= g_num_servos;
	}
	
	fb->id = servos[i].id;
	fb->position = servos[i].current_position;
	fb->packet_state = servos[i].packet_state;
	fb->torque = servos[i].present_current;
	fb->temperature = servos[i].present_temperature;
	fb->timeouts = servos[i].timeouts;
	fb->error = servos[i].error;
	fb->voltage = servos[i].present_voltage;

	return true;
}

bool fillIntrospection(WProto::ServoIntrospection* is, uint8_t i)
{
	Servo* servos;
	if(i < g_num_servos)
		servos = g_servos;
	else
	{
		servos = g_ttl_servos;
		i -= g_num_servos;
	}

	is->id = servos[i].id;
	is->flags = servos[i].flags;
	is->command = servos[i].command;
	is->commandLastWritten = servos[i].commandLastWritten;
	is->commandLastWrittenValid = (uint8_t) servos[i].commandLastWrittenValid;
	is->current_position = servos[i].current_position;
	is->max_torque = servos[i].max_torque;
	is->max_velocity = servos[i].max_velocity;
	is->present_current = servos[i].present_current;
	is->present_temperature = servos[i].present_temperature;
	is->present_voltage = servos[i].present_voltage;
	is->packet_state = servos[i].packet_state;
	is->timeouts = servos[i].timeouts;
	is->error = servos[i].error;
	return true;
}

void comm_pc_processByte(uint8_t c)
{
	if(input.take(c) != EnvelopeReader::NEW_MESSAGE)
		return;

	switch(input.msgCode())
	{
		case RProto::ConnectMsg::MSG_CODE:
		{
			WProto::ConnectMsgReply reply;
			reply.version = PROTOCOL_VERSION;
			// If proper fade-out was not done before disconnect, do not allow reconnect
			bool allServosOff = true;
			for(int i = 0; i < g_num_servos; ++i)
			{
				if(g_servos[i].max_torque != 0)
					allServosOff = false;
			}
			for(int i = 0; i < g_num_ttl_servos; ++i)
			{
				if(g_ttl_servos[i].max_torque != 0)
					allServosOff = false;
			}
			reply.error = 0;

			// HACK: Allow recovery from hard emergency.
			if(!allServosOff && !dxl_isInHardEmergency())
				reply.error |= CONNECT_MSG_ERROR_FLAG_NOT_SHUT_DOWN;

			if(reply.error == 0)
			{
				dxl_resetHardEmergency();
			}

			output << reply;
			g_faded_after_connect = false;
			break;
		}
		case RProto::ServoConfigurationMsg::MSG_CODE:
		{
			// Do not configure the servos if servos with enabled torque are present
			for(int i = 0; i < g_num_servos; ++i)
				if(g_servos[i].max_torque != 0)
					break;
			for(int i = 0; i < g_num_ttl_servos; ++i)
				if(g_ttl_servos[i].max_torque != 0)
					break;

			RProto::ServoConfigurationMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;

			g_num_servos = 0;
			g_num_ttl_servos = 0;
			g_ttl_bus_address = msg.ttl_bus_address;
			for(int i = 0; i < MAX_NUM_SERVOS; ++i)
				g_ttl_servos[i].current_position = 400;

			RProto::ServoConfiguration servoData;
			memset(&servoData, 0, sizeof(servoData));

			while(msg.servos.next(&servoData))
			{
				Servo* servo;

				if(servoData.flags & SERVO_FLAG_IS_TTL)
				{
					servo = &g_ttl_servos[servoData.id - msg.ttl_bus_base_id];
					g_num_ttl_servos++;
				}
				else
					servo = &g_servos[g_num_servos++];

				servo->id = servoData.id;
				servo->flags = servoData.flags;
				servo->max_torque = 0;
				servo->max_velocity = servoData.max_velocity;
				servo->fadein_velocity = servoData.fadein_velocity;
			}

			// Read Torque values for array directly from servos
			dxl_initTorqueFromServo();

			// Set mode: Position- oder Velocity-Control
			dxl_startSyncWrite(11, 1);
			for(int i = 0; i < g_num_servos; ++i)
			{
				uint8_t mode = (g_servos[i].flags & SERVO_FLAG_VELOCITY_CONTROL) ? 1 : 3;
				dxl_syncWriteData(g_servos[i].id, &mode, 1);
			}
			dxl_endSyncWrite();

			g_current_temp_read_id = 0;

			dxl_constantSyncWrite(563, 0xFF0000, 3);

			WProto::ServoConfiguredMsg ret;

			ret.error = 0;
			ret.servo_mask = 0;

			// Check for torques after reading them from servos!
			for(uint8_t i = 0; i < g_num_servos; ++i)
			{
				if(g_servos[i].max_torque != 0)
					ret.error |= CONFIGURE_MSG_ERROR_FLAG_NOT_SHUT_DOWN;
				else
					ret.servo_mask |= (1 << g_servos[i].id);
			}

			for(uint8_t i = 0; i < g_num_ttl_servos; ++i)
			{
				if(g_ttl_servos[i].max_torque != 0)
					ret.error |= CONFIGURE_MSG_ERROR_FLAG_NOT_SHUT_DOWN;
			}

			output << ret;

			break;
		}
		case RProto::ServoCommandMsg::MSG_CODE:
		{
			DDRF = 0xff;
			PORTF = 0xA5;

			EnvelopeReader::Reader reader = input.makeReader();
			uint8_t b;
			while(reader.read(&b, 1))
				PORTF = b;

			RProto::ServoCommandMsg msg;
			memset(&msg, 0, sizeof(msg));

			input >> msg;

			if(g_num_servos == 0)
			{
				WProto::ServoFeedbackMsg ret;
				ret.error = SERVO_FEEDBACK_ERROR_NOT_CONFIGURED;

				output << ret;
				return;
			}

			g_softwareStop = msg.software_stop;

			RProto::ServoCommand cmd;
			memset(&cmd, 0, sizeof(cmd));

			uint8_t blkrd_index = 0;

			while(msg.servos.next(&cmd))
			{
				// Find ID in global array
				PORTF = 0xA0;
				PORTF = cmd.id & 0xFF;
				PORTF = cmd.id >> 8;
				PORTF = 0xA1;

				Servo* servo = servo_find(cmd.id, &(g_bulkread_indices[blkrd_index]));
				
				if(!servo)
				{
					WProto::ServoFeedbackMsg ret;
					ret.error = SERVO_FEEDBACK_ERROR_INVALID_ID;

					PORTF = 0xAB;
					output << ret;
					return;
				}

				if(!(servo->flags & SERVO_FLAG_IS_TTL))
					blkrd_index++;

				servo->command = cmd.command;
			}

// 			for(uint8_t i = 0; i < g_num_servos; ++i)
// 				g_servos[i].current_position = 0;
//
// 			for(uint8_t i = 0; i < g_num_ttl_servos; ++i)
// 				g_ttl_servos[i].current_position = 0;

			// Send DXLPRO packet, receive feedback
			if(g_faded_after_connect)
				dxl_writeActuators();
			dxl_readActuators();

			WProto::ServoFeedbackMsg ret;
			ret.request_id = msg.request_id;
			ret.error = SERVO_FEEDBACK_ERROR_NONE;
			ret.distance = g_distance_sensor_value;

			if(io_emergencySwitch_soft() || io_emergencySwitch_hard())
				ret.error = SERVO_FEEDBACK_ERROR_EMERGENCY_PRESSED;

			ret.flags = 0;

			ret.servos.setCallback(&fillFeedback, g_num_servos + g_num_ttl_servos);

			PORTF = 0xAA;
			output << ret;

			break;
		}
		case RProto::FadeCommandMsg::MSG_CODE:
		{
			// Save fadein torque and velocity values
			RProto::FadeCommandMsg msg;
			memset(&msg, 0, sizeof(msg));

			input >> msg;

			RProto::FadeCommand cmd;
			memset(&cmd, 0, sizeof(cmd));

			//uint32_t fadein_velocity_limits[MAX_NUM_SERVOS];

			bool active[2*MAX_NUM_SERVOS];
			uint16_t old_torques[2*MAX_NUM_SERVOS];

			for(uint8_t i = 0; i < sizeof(active); ++i)
				active[i] = false;

			for(uint8_t i = 0; i < g_num_servos; ++i)
			{
				//fadein_velocity_limits[i] = g_servos[i].fadein_velocity;
				old_torques[i] = g_servos[i].max_torque;
			}

			for(uint8_t i = 0; i < g_num_ttl_servos; ++i)
			{
				old_torques[g_num_servos + i] = g_ttl_servos[i].max_torque;
			}


			bool ttl_torques_changed = false;
			while(msg.servos.next(&cmd))
			{
				for(int i = 0; i < g_num_servos; ++i)
				{
					if(g_servos[i].id == cmd.id)
					{
						active[i] = true;
						g_servos[i].max_torque = cmd.torque;
						//fadein_velocity_limits[i] = cmd.fadein_velocity;
					}
				}
				for(int i = 0; i < g_num_ttl_servos; ++i)
				{
					// TTL bus servos
					if(g_ttl_servos[i].id == cmd.id)
					{
						active[g_num_servos + i] = true;
						g_ttl_servos[i].max_torque = cmd.torque;
						ttl_torques_changed = true;
					}
				}
			}

			// Set status LEDs to yellow for impending action
			// (DXL PRO only)
			dxl_constantSyncWrite(563, 0x00FFFF, 3);

			// Enable/Disable Holding Torque (redundant after initial fade-in)
			// DXL PRO only, DXL handled by TTL adapter board
			dxl_startSyncWrite(DXL_REG_TORQUE_ENABLE, 1);

			for(uint8_t i = 0; i < g_num_servos; ++i)
			{
				uint8_t on = 1;
				if(g_servos[i].max_torque != 0)
					dxl_syncWriteData(g_servos[i].id, &on, 1);
			}

			dxl_endSyncWrite();

			// Before actual fade paranoidly set "old" DXL positions to try and fix random-position bug
			if(ttl_torques_changed)
				dxl_write_ttl_positions();

			// Actual fade
			PORTJ |= (1 << 7);
			const uint32_t time_per_step = 50; // in ms/step
			const uint32_t total_steps = msg.fade_time / time_per_step;
			for(uint32_t j = 1; j <= total_steps; ++j)
			{
				if(io_emergencySwitch_hard() || io_emergencySwitch_soft())
					break;

				dxl_startSyncWrite(DXL_REG_GOAL_POSITION, 4+4+2);
				for(int i = 0; i < g_num_servos; ++i)
				{
					//if(!active[i])
						//continue;

					DXLPosVelTorque pvt;
					pvt.position = g_servos[i].command;
					pvt.velocity = (g_servos[i].flags & SERVO_FLAG_VELOCITY_CONTROL) ? 0 : g_servos[i].fadein_velocity;
					//pvt.torque = (g_servos[i].max_torque == 0) ? 0 : old_torques[i] + (((g_servos[i].max_torque - old_torques[i]) * j) / total_steps); // Linear Interpolation
					pvt.torque = (g_servos[i].max_torque == 0) ? 0 : old_torques[i] + ((g_servos[i].max_torque - old_torques[i]) * j * j) / (total_steps * total_steps); // Quadratic Interpolation
					if(pvt.torque == 0)
						pvt.torque = 1;
					dxl_syncWriteData(g_servos[i].id, &pvt, 4+4+2);
				}
				dxl_endSyncWrite();

				// TTL Servos
				// No velocity limits happen here.
				dxl_startSyncWrite(0x80, 2*g_num_ttl_servos);
				uint16_t ttl_torques[MAX_NUM_SERVOS];
				for(int i = 0; i < g_num_ttl_servos; ++i)
				{
					uint16_t goal_torque = g_ttl_servos[i].max_torque;

					uint16_t old_torque = old_torques[g_num_servos + i];

					ttl_torques[i] = ((int32_t)old_torque) + (((int32_t)goal_torque) - ((int32_t)old_torque)) * j / total_steps;

					if(goal_torque == 0)
						ttl_torques[i] = 0; // FIXME: Why is this needed? Without it, the TTL servos do not fade out at all.
				}
				dxl_syncWriteData(g_ttl_bus_address, ttl_torques, 2*g_num_ttl_servos);
				dxl_endSyncWrite();

				_delay_ms(time_per_step);
// 				if(all_torques_zero)
// 					break;
			}
			PORTJ &= ~(1 << 7);

			// Enable/Disable Holding Torque (only relevant from second fade-in)
			dxl_startSyncWrite(DXL_REG_TORQUE_ENABLE, 1);

			for(int i = 0; i < g_num_servos; ++i)
			{
				uint8_t on = 0;
				if(g_servos[i].max_torque == 0)
					dxl_syncWriteData(g_servos[i].id, &on, 1);
			}

			dxl_endSyncWrite();

			//_delay_ms(1000);
			dxl_startSyncWrite(DXL_REG_GOAL_VELOCITY, 4);
			for(int i = 0; i < g_num_servos; ++i)
			{
				if(!(g_servos[i].flags & SERVO_FLAG_VELOCITY_CONTROL))
					dxl_syncWriteData(g_servos[i].id, &g_servos[i].max_velocity, 4);
			}
			dxl_endSyncWrite();
			
			// Change LED colors: blue: online, inactive, green: torque active, yellow: impending action
			// (DXL Pro only)
			dxl_startSyncWrite(563, 3);
			for(int i = 0; i < g_num_servos; ++i)
			{
				uint32_t color;
				if(g_servos[i].max_torque == 0)
					color = 0xFF0000;
				else
					color = 0x00FF00;
				dxl_syncWriteData(g_servos[i].id, &color, 3);
			}
			dxl_endSyncWrite();

			WProto::FadeCommandMsgReply ret;
			output << ret;
			g_faded_after_connect = true;
			break;
		}
		case RProto::StartPassthroughMsg::MSG_CODE:
		{
			WProto::StartPassthroughReply reply;
			reply.active = 1;
			output << reply;

			int keyIdx = 0;
			bool exitLoop = false;

			while(!exitLoop)
			{
				// DXL -> PC
				while(rs485_bytesWaiting())
				{
					ftdi_pc_putc(rs485_getc());
				}

				// PC -> DXL
				if(ftdi_pc_bytesWaiting())
				{
					io_setRS485Output(true);

					_delay_us(10);

					while(ftdi_pc_bytesWaiting())
					{
						uint8_t c = ftdi_pc_getc();
						if(c == PASSTHROUGH_EXIT_KEY[keyIdx])
						{
							keyIdx++;
							if(keyIdx == sizeof(PASSTHROUGH_EXIT_KEY))
							{
								keyIdx = 0; // prevent overrunning
								exitLoop = true;
							}
						}
						else
							keyIdx = 0;

						rs485_putc(c);
					}

					while(rs485_txWaiting() != 0)
						;

					while(!(UCSR3A & (1 << UDRE3)));
					while(!(UCSR3A & (1 << TXC3)));

					_delay_us(100);

					io_setRS485Output(false);
				}
			}

			reply.active = 0;
			output << reply;

			break;
		}
		case RProto::RebootServoMsg::MSG_CODE:
		{
			RProto::RebootServoMsg msg;
			memset(&msg, 0, sizeof(msg));

			input >> msg;

			dxl_rebootServo(msg.id);
			break;
		}
		case RProto::RebootControllerMsg::MSG_CODE:
		{
			RProto::RebootControllerMsg msg;
			memset(&msg, 0, sizeof(msg));

			input >> msg;

			WProto::RebootControllerMsg reply;
			reply.key1 = REBOOT_KEY1;
			reply.key2 = REBOOT_KEY2;
			output << reply;

			if(msg.key1 == REBOOT_KEY1 && msg.key2 == REBOOT_KEY2)
			{
				wdt_enable(WDTO_30MS);
				while(1);
			}

			break;
		}
		case RProto::IntrospectionMsg::MSG_CODE:
		{
			WProto::IntrospectionReplyMsg reply;
			reply.faded_after_connect = (uint8_t) g_faded_after_connect;
			reply.num_pro_servos = g_num_servos;
			reply.num_ttl_servos = g_num_ttl_servos;
			reply.servos.setCallback(&fillIntrospection, g_num_servos + g_num_ttl_servos);

			break;
		}
		case RProto::SetServoParamMsg::MSG_CODE:
		{
			RProto::SetServoParamMsg msg;
			memset(&msg, 0, sizeof(msg));

			input >> msg;
			dxl_startSyncWrite(594,2);
			dxl_syncWriteData(msg.id, &msg.pgain, 2);
			dxl_endSyncWrite();
			
			break;
		}
		case RProto::SetSilenceMsg::MSG_CODE:
		{
			RProto::SetSilenceMsg msg;
			memset(&msg, 0, sizeof(msg));

			input >> msg;
			const uint16_t SILENCE_REG = 0x101;
			dxl_startSyncWrite(SILENCE_REG, 1);
			dxl_syncWriteData(g_ttl_bus_address, &msg.silence_sequence, 1);
			dxl_endSyncWrite();

			break;
		}
	}
}
