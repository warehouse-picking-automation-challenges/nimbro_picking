// DXL MX Communication
// Author: Sebastian Schüller<schuell1@cs.uni-bonn.de>

#include "dxl.h"
#include "io.h"
#include "uart_wrapper.h"

#include <apc_proto.h>
#include <string.h>
#include <util/delay.h>

#include <dxl_master/mx.h>


#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))


namespace dxl
{
/// dxl globals
ServoStatus servos[ProtoConstants::NUM_SERVOS];
uint16_t read_timeouts;
uint8_t comm_errors;
uint8_t valid_servos;
uint8_t br_servos;
///

/// command buffers
static uint8_t write_buf[64];
static uint8_t br_buf[DXL_MASTER_BRPACKET_SIZE(COUNT_OF(servos))];

/// Bulkread sizes in bytes, doubles down as ids
static const uint8_t DEFAULT_BULKREAD_SIZE = 8;
static const uint8_t EEPROM_BULKREAD_SIZE = 2;

static uint16_t s_servo_feedback_mask;
static void reset_servo_feedback_mask()
{
	for(size_t i = 0; i < COUNT_OF(servos); ++i)
		s_servo_feedback_mask |= (1 << i);
}

uint8_t servo_index(uint16_t id)
{
	size_t i;
	for(i = 0; i < COUNT_OF(servos); ++i)
		if(id == servos[i].id)
			return i;
	return ProtoConstants::NUM_SERVOS + 1;
}

static uint16_t get_feedback(uint16_t timeout)
{
	reset_servo_feedback_mask();
	while(1)
	{
		while(rs485_bytesWaiting() == 0)
		{
			_delay_us(10);
			timeout--;
			if(timeout == 0)
			{
				read_timeouts++;
				return 0;
			}
		}
		while(rs485_bytesWaiting())
		{
			uint8_t c = rs485_getc();
			dxl_master_putc(c);
			if(!s_servo_feedback_mask)
				return timeout;
		}
	}
}




void init()
{
	dxl_master_init();
	read_timeouts = 0;
	comm_errors = 0x0;
	valid_servos = 0;
	br_servos = 0;
	memset(servos, 0, sizeof(servos));
	reset_servo_feedback_mask();
	memset(write_buf, 0, sizeof(write_buf));
	memset(br_buf, 0, sizeof(br_buf));
}

void reset_errors()
{
	comm_errors = 0x0;
}

void send_position_cmds()
{
	// Note(sebastian): Only send position commands for servos, where
	// torque_enable is explicitely set. This is necessary, because of a
	// counter-intuitive behaviour of the dynamixel firmware in which
	// a position commands forces the value of the torque_enable register
	// to (1), regardless of the manually specified value.

	uint8_t enabled_servos = 0;
	for(size_t i = 0; i < COUNT_OF(servos); ++i)
		if(servos[i].torque_enable)
			enabled_servos++;
	if(enabled_servos >= valid_servos)
		enabled_servos = valid_servos;

	dxl_master_syncwrite_init(
		write_buf,
		enabled_servos,
		MX_GOAL_POSITION,
		2
	);

	uint8_t cmd_idx = 0;
	for(size_t i = 0; i < COUNT_OF(servos); ++i)
	{
		ServoStatus* s = &servos[i];
		if(!s->torque_enable || s->id == 0)
			continue;
		dxl_master_syncwrite_set(
			write_buf,
			cmd_idx++,
			s->id,
			(uint8_t*)&s->goal_position
		);
	}
	uint8_t size = dxl_master_syncwrite_finalize(write_buf);
	dxl_master_send(write_buf, size);
}

void send_torque_enable()
{
	dxl_master_syncwrite_init(
		write_buf,
		valid_servos,
		MX_TORQUE_ENABLE,
		1
	);
	uint8_t cmd_idx = 0;
	for(size_t i = 0; i < COUNT_OF(servos); ++i)
	{
		ServoStatus* s = &servos[i];
		if(s->id == 0)
			continue;
		dxl_master_syncwrite_set(
			write_buf,
			cmd_idx++,
			s->id,
			(uint8_t*)&s->torque_enable
		);
	}
	uint8_t size = dxl_master_syncwrite_finalize(write_buf);
	dxl_master_send(write_buf, size);
}

void send_max_torque()
{
	dxl_master_syncwrite_init(
		write_buf,
		valid_servos,
		MX_TORQUE_LIMIT,
		2
	);
	uint8_t cmd_idx = 0;
	for(size_t i = 0; i < COUNT_OF(servos); ++i)
	{
		ServoStatus* s = &servos[i];
		if(s->id == 0)
			continue;
		dxl_master_syncwrite_set(
			write_buf,
			cmd_idx++,
			s->id,
			(uint8_t*)&s->max_torque
		);
	}
	uint8_t size = dxl_master_syncwrite_finalize(write_buf);
	dxl_master_send(write_buf, size);
}


uint16_t read_feedback(uint16_t timeout)
{
	dxl_master_bulkread_init(br_buf, br_servos);
	uint8_t cmd_idx = 0;
	for(size_t i = 0; i < COUNT_OF(servos); ++i)
	{
		ServoStatus* s = &servos[i];
		if(s->id == 0)
			continue;
		// HACK (sebastian) : Ignore ID 3 (vacuum) for bulkread
		if(s->id == 3)
			continue;
		dxl_master_bulkread_set(
			br_buf,
			cmd_idx++,
			s->id,
			MX_PRESENT_POSITION,
			DEFAULT_BULKREAD_SIZE
		);
	}
	uint8_t size = dxl_master_bulkread_finalize(br_buf);
	dxl_master_send(br_buf, size);

	return get_feedback(timeout);
}

uint16_t read_eeprom(uint16_t timeout)
{
	dxl_master_bulkread_init(br_buf, valid_servos);
	uint8_t cmd_idx = 0;
	for(size_t i = 0; i < COUNT_OF(servos); ++i)
	{
		ServoStatus* s = &servos[i];
		if(s->id == 0)
			continue;
		dxl_master_bulkread_set(
			br_buf,
			cmd_idx++,
			s->id,
			MX_MAX_TORQUE,
			EEPROM_BULKREAD_SIZE
		);
	}
	uint8_t size = dxl_master_bulkread_finalize(br_buf);
	dxl_master_send(br_buf, size);

	return get_feedback(timeout);
}

}


/// Implemented funtions for the dxl_master lib
void dxl_master_send(const uint8_t* data, uint8_t len)
{
	io::set_RS485_output(true);
	io::reset_RS485_txComplete();

	while(len-- != 0)
		rs485_putc(*(data++));

	while(rs485_txWaiting());
	while(!io::is_RS485_txComplete_set());

	io::set_RS485_output(false);
}


void dxl_master_handleStatus(
	uint8_t id,
	uint8_t error,
	const uint8_t* params,
	uint8_t num_params
)
{
	uint8_t index = dxl::servo_index(id);
	if(index >= COUNT_OF(dxl::servos))
	{
		dxl::comm_errors |= DbgStatusFlags::DXL_BR_INVLD_ID;
		return;
	}
	dxl::s_servo_feedback_mask &= ~(1 << index);

	dxl::ServoStatus* s = &dxl::servos[index];
	s->error = error;
	switch(num_params)
	{
		case dxl::DEFAULT_BULKREAD_SIZE:
		{
			s->position    = params[0] | (params[1] << 8);
			s->speed       = params[2] | (params[3] << 8);
			s->load        = params[4] | (params[5] << 8);
			s->voltage     = params[6];
			s->temperature = params[7];
			break;
		}
		case dxl::EEPROM_BULKREAD_SIZE:
		{
			s->max_torque  = params[0] | (params[1] << 8);
			break;
		}
		default:
		{
			dxl::comm_errors |= DbgStatusFlags::DXL_BR_INVLD_SIZE;
		}
	}
}
///


