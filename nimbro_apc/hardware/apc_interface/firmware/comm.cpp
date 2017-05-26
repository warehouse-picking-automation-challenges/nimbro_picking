// Hardware UART communication with Host
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#include "comm.h"

#include "apc_proto.h"
#include "uart_wrapper.h"
#include "adc.h"
#include "io.h"
#include "dxl.h"

#include <libucomm/envelope.h>
#include <libucomm/checksum.h>

#include <avr/wdt.h>
#include <util/delay.h>

namespace {
class PCUARTWriter
{
public:
	bool writeChar(uint8_t c)
	{
		ftdi_pc_putc(c);
		return true;
	}
};
}

namespace comm
{

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

typedef uc::EnvelopeWriter<uc::InvertedModSumGenerator, PCUARTWriter> EnvelopeWriter;
typedef uc::IO<EnvelopeWriter, uc::IO_W> SimpleWriter;
typedef Proto<SimpleWriter> WProto;

typedef uc::EnvelopeReader<uc::InvertedModSumGenerator, 256> EnvelopeReader;
typedef uc::IO<EnvelopeReader, uc::IO_R> SimpleReader;
typedef Proto<SimpleReader> RProto;


static EnvelopeReader input;
static PCUARTWriter writer;
static EnvelopeWriter output(&writer);

static bool servos_initialized = false;
static uint8_t dbg_errors = 0x0;

void processByte(uint8_t c)
{
	if(input.take(c) != EnvelopeReader::NEW_MESSAGE)
		return;

	switch(input.msgCode())
	{
		case RProto::ConnectRequestMsg::MSG_CODE:
		{
			WProto::ConnectAck ack;
			ack.protocol_version = ProtoConstants::PROTOCOL_VERSION;
			output << ack;
			while(ftdi_pc_txWaiting());
			break;
		}
		case RProto::ServoInitializationMsg::MSG_CODE:
		{
			RProto::ServoInitializationMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;

			dxl::valid_servos = 0;

			for(size_t i = 0; i < ProtoConstants::NUM_SERVOS; ++i)
			{
				dxl::servos[i].id = msg.ids[i];
				if(msg.ids[i] != 0)
					dxl::valid_servos++;
			}
			// HACK (sebastian): Ignore vacuum servo on bulkreads
			dxl::br_servos = 2;
			servos_initialized = true;
			break;
		};
		case RProto::RebootControllerMsg::MSG_CODE:
		{
			RProto::RebootControllerMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;

			WProto::RebootControllerAck ack;
			if(msg.key1 == ProtoConstants::REBOOT_KEY1 &&
			   msg.key2 == ProtoConstants::REBOOT_KEY2)
			{
				ack.status = RebootControllerMsgStatus::SUCCESS;
				output << ack;
				while(ftdi_pc_txWaiting());

				wdt_enable(WDTO_30MS);
				while(1);
			}
			ack.status = RebootControllerMsgStatus::WRONG_KEY;
			output << ack;
			while(ftdi_pc_txWaiting());
			break;
		}
		case RProto::ServoPositionCmdMsg::MSG_CODE:
		{
			RProto::ServoPositionCmdMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;
			if(!servos_initialized)
			{
				dbg_errors |= DbgStatusFlags::COMM_POSCMD_UNINITIALIZED;
				return;
			}
			for(size_t i = 0; i < ProtoConstants::NUM_SERVOS; ++i)
			{
				RProto::ServoPositionCmd* cmd = &msg.cmds[i];
				if(cmd->id == 0)
					continue;
				uint8_t index = dxl::servo_index(cmd->id);
				if(index > ProtoConstants::NUM_SERVOS)
				{
					dbg_errors |= DbgStatusFlags::COMM_POSCMD_INVLD_ID;
					return;
				}

				dxl::servos[index].goal_position = cmd->goal_position;
			}
			dxl::send_position_cmds();
			break;
		}
		case RProto::ServoMaxTorqueMsg::MSG_CODE:
		{
			RProto::ServoMaxTorqueMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;
			for(size_t i = 0; i < ProtoConstants::NUM_SERVOS; ++i)
			{
				RProto::ServoMaxTorque* cmd = &msg.cmds[i];
				if(cmd->id == 0)
					continue;
				uint8_t index = dxl::servo_index(cmd->id);
				if(index > ProtoConstants::NUM_SERVOS)
					return;

				dxl::servos[index].max_torque = cmd->torque;
			}
			dxl::send_max_torque();
			break;
		}
		case RProto::ServoTorqueEnableMsg::MSG_CODE:
		{
			RProto::ServoTorqueEnableMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;
			if(!servos_initialized)
			{
				dbg_errors |= DbgStatusFlags::COMM_TRQEN_UNINITIALIZED;
				return;
			}
			for(size_t i = 0; i < ProtoConstants::NUM_SERVOS; ++i)
			{
				RProto::ServoTorqueEnable* cmd = &msg.cmds[i];
				if(cmd->id == 0)
					continue;
				uint8_t index = dxl::servo_index(cmd->id);
				if(index > ProtoConstants::NUM_SERVOS)
				{
					dbg_errors |= DbgStatusFlags::COMM_TRQEN_INVLD_ID;
					return;
				}
				dxl::servos[index].torque_enable = cmd->enable;
			}
			dxl::send_torque_enable();
			break;
		}
		case RProto::ControllerStatusRqst::MSG_CODE:
		{
			WProto::ControllerStatusMsg status = {};

			status.status = 0;

			if(io::is_vacuum_on())
				status.status |= ControllerStatusFlags::VACUUM_ON;

			if(io::is_vacuum_power_on())
				status.status |= ControllerStatusFlags::VACUUM_POWER_ON;

			status.light_duty   = io::light_duty();
			status.air_pressure = adc::pressure_sync_read();

			if(!servos_initialized)
			{
				dbg_errors |= DbgStatusFlags::COMM_STATUS_UNINITIALIZED;
				status.status |= ControllerStatusFlags::SERVO_UNINITIALIZED;
				output << status;
				break;
			}
			if(!dxl::read_feedback())
			{
				status.status |= ControllerStatusFlags::BULKREAD_FAILED;
			}
			for(size_t i = 0; i < COUNT_OF(status.servos); ++i)
			{
				if(i >= ProtoConstants::NUM_SERVOS)
					break;
				WProto::ServoStatus* s = &status.servos[i];
				dxl::ServoStatus *servo = &dxl::servos[i];
				s->id            = servo->id;
				s->flags         = servo->flags;
				s->goal_position = servo->goal_position;
				s->position      = servo->position;
				s->speed         = servo->speed;
				s->load          = servo->load;
				s->voltage       = servo->voltage;
				s->temperature   = servo->temperature;
				s->error         = servo->error;
			}
			status.timeouts = dxl::read_timeouts;
			output << status;
			break;
		}
		case RProto::EEPROMStatusRqst::MSG_CODE:
		{
			WProto::EEPROMStatusMsg msg;
			if(!dxl::read_eeprom())
			{
				return;
			}
			for(size_t i = 0; i < COUNT_OF(msg.eeprom); ++i)
			{
				if(i >= ProtoConstants::NUM_SERVOS)
					break;
				WProto::EEPROMStatus* s = &msg.eeprom[i];
				dxl::ServoStatus* servo = &dxl::servos[i];
				s->id = servo->id;
				s->max_torque = servo->max_torque;
			}
			output << msg;
			break;
		}
		case RProto::DbgStatusRqst::MSG_CODE:
		{
			WProto::DbgStatusMsg msg = {};
			msg.dxl_errors = dxl::comm_errors;
			msg.comm_errors = dbg_errors;
			output << msg;
			break;
		}
		case RProto::DimLightMsg::MSG_CODE:
		{
			RProto::DimLightMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;
			io::dim_light(msg.duty);
			break;
		}
		case RProto::SwitchVacuumMsg::MSG_CODE:
		{
			RProto::SwitchVacuumMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;
			io::set_vacuum(!!msg.on);
			break;
		}
		case RProto::SwitchVacuumPowerMsg::MSG_CODE:
		{
			RProto::SwitchVacuumPowerMsg msg;
			memset(&msg, 0, sizeof(msg));
			input >> msg;
			io::set_vacuum_power(!!msg.on);
			break;
		}
	}

}

}
