// Controller for Dynamaid's new hand
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <util/delay.h>

#include <dxl_master/dxl_master.h>
#include <dxl_master/mx.h>

#include <dxlpro_slave/dxlpro_slave.h>

#include "config.h"
#include "comm.h"
#include "io.h"

static volatile uint16_t g_feedback_positions[NUM_HAND_SERVOS];
static volatile uint8_t g_temperatures[NUM_HAND_SERVOS];
static uint16_t g_servo_torques_temp[NUM_HAND_SERVOS];
static uint16_t g_servo_torques[NUM_HAND_SERVOS];
uint8_t g_servo_torque_write_needed = 0;
volatile uint8_t g_servo_torque_apply = 0;
uint8_t g_temperatureID = 1;

static volatile uint16_t g_goal_positions[NUM_HAND_SERVOS];
static uint8_t g_goal_positions_temp[NUM_HAND_SERVOS];

static uint8_t g_brPacketBuf[DXL_MASTER_BRPACKET_SIZE(NUM_HAND_SERVOS)];
// static uint8_t g_startServoIdx = 0;

static uint8_t g_syncWriteBuf[64];

static uint16_t g_currentADCReadValue = 0;

extern volatile uint8_t dxl_recv_count;

volatile uint8_t g_silence = 0;

void dxlpro_slave_ctrl_set(uint16_t addr, uint8_t value)
{
	if(addr >= 0x40 && addr < 0x40 + NUM_HAND_SERVOS * 2)
	{
		uint8_t idx = (addr - 0x40) / 2;

		if(addr & 1)
			g_goal_positions[idx] = (((uint16_t)value) << 8) | g_goal_positions_temp[idx];
		else
			g_goal_positions_temp[idx] = value;
	}

	if(addr >= 0x80 && addr < 0x80 + NUM_HAND_SERVOS * 2)
	{
		uint8_t idx = (addr - 0x80) / 2;

		if(addr & 1)
			g_servo_torques_temp[idx] = (((uint16_t)value) << 8) | (g_servo_torques_temp[idx] & 0xFF);
		else
			g_servo_torques_temp[idx] = (g_servo_torques_temp[idx] & 0xFF00) | value;

		g_servo_torque_write_needed = 1;
	}

	// Handle reset command
	if(addr == 0x100 && value == 0xAB)
	{
		wdt_enable(WDTO_30MS);
		while(1);
	}

	if(addr == 0x101 && value == 0xAB)
	{
		g_silence = 1;
	}
	if(addr == 0x101 && value == 0xAC)
	{
		g_silence = 0;
	}
}

void dxlpro_slave_ctrl_apply(void)
{
	uint8_t i = 0;

	if(g_servo_torque_write_needed)
	{
		for(i = 0; i < NUM_HAND_SERVOS; ++i)
			g_servo_torques[i] = g_servo_torques_temp[i];

		g_servo_torque_apply = 1;
	}

	g_servo_torque_write_needed = 0;
}

uint8_t dxlpro_slave_ctrl_get(uint16_t addr)
{
	// 	PORTB ^= (1 << 7);
	switch(addr)
	{
		case DXLPRO_SLAVE_REG_MODEL_NUMBER_L:
			return 0xFA;
		case DXLPRO_SLAVE_REG_MODEL_NUMBER_H:
			return 0xFB;
		case DXLPRO_SLAVE_REG_VERSION:
			return 0x01;
	}

	// Range 0x10 - 0x30: Just positions (2 bytes per servo)
	uint16_t pos_end_addr = 0x10 + NUM_HAND_SERVOS*2;
	if(addr >= 0x10 && addr < pos_end_addr)
	{
		uint8_t idx = (addr - 0x10) / 2;
		uint16_t pos = g_feedback_positions[idx];

		if(addr & 1)
		{
			return pos >> 8;
		}
		else
		{
			return pos;
		}
	}

	// Range 0x40 to 0x70: Positions + Temperatures (3 bytes per servo)
	uint16_t pos_and_temp_end_addr = 0x40 + NUM_HAND_SERVOS*3;
	if(addr >= 0x40 && addr < pos_and_temp_end_addr)
	{
		uint8_t idx = (addr - 0x40) / 3;
		uint8_t subIdx = (addr - 0x40) % 3;
		uint16_t pos = g_feedback_positions[idx];

		switch(subIdx)
		{
			case 0: return pos;
			case 1: return pos >> 8;
			case 2: return g_temperatures[idx];
		}
	}

	// 0x100: ADC channel 0
	if(addr == 0x100 || addr == pos_end_addr || addr == pos_and_temp_end_addr)
	{
		g_currentADCReadValue = io_analog();
		return g_currentADCReadValue & 0XFF;
	}
	if(addr == 0x101 || addr == pos_end_addr + 1 || addr == pos_and_temp_end_addr + 1 )
	{
		return g_currentADCReadValue >> 8;
	}

	return 0xFF;
}

void dxl_master_handleStatus(uint8_t id, uint8_t error, const uint8_t* params, uint8_t num_params)
{
	if(id > NUM_HAND_SERVOS || id == 0)
		return;

	if(num_params == 1)
	{
		g_temperatures[id-1] = params[0];
	}
	else if(num_params == 2)
	{
		g_feedback_positions[id-1] = params[0] | (params[1] << 8);
	}
}

void constantSyncWrite(uint8_t addr, uint16_t value, uint8_t len)
{
	uint8_t packetSize;
	uint8_t i;

	dxl_master_syncwrite_init(g_syncWriteBuf, NUM_HAND_SERVOS, addr, len);
	for(i = 0; i < NUM_HAND_SERVOS; ++i)
	{
		dxl_master_syncwrite_set(g_syncWriteBuf, i, i+1, (uint8_t*)&value);
	}

	packetSize = dxl_master_syncwrite_finalize(g_syncWriteBuf);
	dxl_master_send(g_syncWriteBuf, packetSize);
}

int main()
{
	uint8_t i = 0;
	uint8_t packetSize;
	uint8_t brSize;
	uint8_t activeServos;

	io_init();
	comm_init();

	io_setLED(0);

// 	DDRE |= (1 << 1);
//
// 	while(1)
// 	{
// 		PORTE |= (1 << 1);
// 		PORTE &= ~(1 << 1);
// 	}

	sei();

	_delay_ms(1000);

	g_feedback_positions[0] = 0x05;

	for(i = 0; i < NUM_HAND_SERVOS; ++i)
		g_goal_positions[i] = 2048;

	for(i = 0; i < 3; ++i)
	{
		constantSyncWrite(MX_LED, 1, 1);
		io_setLED(1);

		_delay_ms(100);

		constantSyncWrite(MX_LED, 0, 1);
		io_setLED(0);

		_delay_ms(100);
	}

	dxl_master_bulkread_init(g_brPacketBuf, NUM_HAND_SERVOS);
	for(i = 0; i < NUM_HAND_SERVOS; ++i)
	{
		dxl_master_bulkread_set(g_brPacketBuf, i, i+1, MX_PRESENT_POSITION, 2);
	}
	brSize = dxl_master_bulkread_finalize(g_brPacketBuf);

	uint8_t resetCounter = 0;

	while(1)
	{
		while(g_silence);

		if(g_servo_torque_apply || ++resetCounter == 128)
		{
			cli();
			dxl_master_syncwrite_init(g_syncWriteBuf, NUM_HAND_SERVOS, MX_TORQUE_ENABLE, 1);
			for(i = 0; i < NUM_HAND_SERVOS; ++i)
			{
				uint8_t on = (g_servo_torques[i] != 0) ? 1 : 0;

				dxl_master_syncwrite_set(g_syncWriteBuf, i, i+1, &on);
			}
			packetSize = dxl_master_syncwrite_finalize(g_syncWriteBuf);
			sei();

			dxl_master_send(g_syncWriteBuf, packetSize);

			cli();
			dxl_master_syncwrite_init(g_syncWriteBuf, NUM_HAND_SERVOS, MX_TORQUE_LIMIT, 2);
			for(i = 0; i < NUM_HAND_SERVOS; ++i)
			{
				dxl_master_syncwrite_set(g_syncWriteBuf, i, i+1, (uint8_t*)&g_servo_torques[i]);
			}
			packetSize = dxl_master_syncwrite_finalize(g_syncWriteBuf);
			sei();

			dxl_master_send(g_syncWriteBuf, packetSize);

			g_servo_torque_apply = 0;
			resetCounter = 0;
		}

		dxl_master_syncwrite_init(g_syncWriteBuf, NUM_HAND_SERVOS, MX_GOAL_POSITION, 2);
		activeServos = 0;
		for(i = 0; i < NUM_HAND_SERVOS; ++i)
		{
			cli();

			if(g_servo_torques[i] != 0)
			{
				dxl_master_syncwrite_set(g_syncWriteBuf, activeServos, i+1, (uint8_t*)&g_goal_positions[i]);
				activeServos++;
			}

			sei();
		}
		dxl_master_syncwrite_init(g_syncWriteBuf, activeServos, MX_GOAL_POSITION, 2);
		packetSize = dxl_master_syncwrite_finalize(g_syncWriteBuf);
		dxl_master_send(g_syncWriteBuf, packetSize);

		// Send out a bulk read request to all connected servos
		dxl_master_send(g_brPacketBuf, brSize);

		_delay_ms(15);

		// Send out a read request to one of the servos to read temperature
		uint8_t value = 0x24;
		dxl_master_write(g_temperatureID, MX_ALARM_LED, &value, 1);
		dxl_master_read(g_temperatureID, MX_PRESENT_TEMPERATURE, 1);

		if(++g_temperatureID > NUM_HAND_SERVOS)
			g_temperatureID = 1;

		_delay_ms(15);
	}
}
