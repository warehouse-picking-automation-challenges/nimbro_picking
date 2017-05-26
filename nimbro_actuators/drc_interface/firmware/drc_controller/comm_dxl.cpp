// Dynamixel communication
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "comm_dxl.h"

#include "uart_wrapper.h"

#include "io.h"
#include "servo.h"

#include <stdint.h>
#include <string.h>

#include <util/delay.h>

// ROBOTIS: CRC calculation
//@{

uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
	uint16_t i, j;
	uint16_t crc_table[256] = {
		0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
		0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
		0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
		0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
		0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
		0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
		0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
		0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
		0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
		0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
		0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
		0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
		0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
		0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
		0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
		0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
		0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
		0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
		0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
		0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
		0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
		0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
		0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
		0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
		0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
		0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
		0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
		0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
		0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
		0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
		0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
		0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
	};

	for(j = 0; j < data_blk_size; j++)
	{
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
		crc_accum = (crc_accum << 8) ^ crc_table[i];
	}

	return crc_accum;
}

//@}

static const uint8_t MAGIC_SEQUENCE[] = {0xFF, 0xFF, 0xFD};
static uint8_t g_cmdBuf[128];
static uint8_t g_cmdBufIdx = 0;
static uint8_t g_paramBytes = 0;
static uint8_t g_stuffState = 0;

uint8_t g_current_temp_read_id;

static bool g_emergency = false;
static bool g_hardEmergency = false;

static void writeParam(void* src, uint16_t size)
{
	for(uint16_t i = 0; i < size; ++i)
	{
		uint8_t c = ((uint8_t*)src)[i];

		if(c == MAGIC_SEQUENCE[g_stuffState])
			g_stuffState++;
		else
			g_stuffState = 0;

		if(g_stuffState == sizeof(MAGIC_SEQUENCE))
		{
			g_cmdBuf[g_cmdBufIdx++] = 0xFD;
			g_paramBytes++;
			g_stuffState = 0;
		}

		g_cmdBuf[g_cmdBufIdx++] = c;
		g_paramBytes++;
	}
}

struct Header
{
	uint8_t header1;
	uint8_t header2;
	uint8_t header3;

	uint8_t reserved;

	uint8_t id;
	uint16_t length;
	uint8_t instruction;
} __attribute__((packed));

void dxl_startSyncWrite(uint16_t addr, uint16_t data_size)
{
	g_cmdBufIdx = sizeof(Header);
	g_stuffState = 0;
	g_paramBytes = 0;

	writeParam(&addr, 2);
	writeParam(&data_size, 2);
}

void dxl_syncWriteData(uint8_t id, void* data, uint16_t size)
{
	writeParam(&id, 1);
	writeParam(data, size);
}

static void sendCmdBuf()
{
	io_setRS485Output(true);

	UCSR3A |= (1 << TXC3);

	for(uint16_t i = 0; i < g_cmdBufIdx; ++i)
		rs485_putc(g_cmdBuf[i]);

	while(rs485_txWaiting() != 0)
		;

	while(!(UCSR3A & (1 << TXC3)));

	_delay_us(20);
// 	_delay_ms(1);

	io_setRS485Output(false);
}

void dxl_endSyncWrite()
{
	Header* header = (Header*)g_cmdBuf;
	header->header1 = 0xFF;
	header->header2 = 0xFF;
	header->header3 = 0xFD;
	header->reserved = 0;
	header->id = 0xFE;
	header->instruction = 0x83;
	header->length = g_paramBytes + 3;

	uint16_t checksum = update_crc(0, g_cmdBuf, g_cmdBufIdx);
	g_cmdBuf[g_cmdBufIdx++] = checksum & 0xFF;
	g_cmdBuf[g_cmdBufIdx++] = checksum >> 8;

	sendCmdBuf();
}

void dxl_constantSyncWrite(uint16_t addr, uint32_t value, uint16_t size)
{
	dxl_startSyncWrite(addr, size);
	for(uint8_t i = 0; i < g_num_servos; ++i)
	{
		dxl_syncWriteData(g_servos[i].id, &value, size);
	}
	dxl_endSyncWrite();
}

// Bulk read

struct BulkReadRequest
{
	uint8_t id;
	uint16_t addr;
	uint16_t length;
} __attribute__((packed));

enum ParserState
{
	STATE_WAIT_HEADER,
	STATE_ID,
	STATE_LENGTH_LOW,
	STATE_LENGTH_HIGH,
	STATE_INSTRUCTION,
	STATE_ERROR,
	STATE_DATA,
	STATE_CHECKSUM_LOW,
	STATE_CHECKSUM_HIGH
};

const uint8_t STATUS_PACKET_HEADER[] = {0xFF, 0xFF, 0xFD, 0x00};

static void setPacketState(uint8_t id, uint8_t state)
{
	Servo* s = servo_find(id, 0);
	if(!s)
		return;

	s->packet_state = state;
}

static void handlePacket()
{
	uint8_t unstuffed[128];
	uint8_t unstuffedIdx = 0;

	// Step 1: Unstuff
	uint8_t matched = 0;

	for(uint8_t i = 9; i < g_cmdBufIdx; ++i)
	{
		uint8_t c = g_cmdBuf[i];

		if(matched == sizeof(MAGIC_SEQUENCE))
		{
			if(c == 0xFD) // Escape for MAGIC_SEQUENCE
			{
				for(uint8_t i = 0; i < sizeof(MAGIC_SEQUENCE); ++i)
					unstuffed[unstuffedIdx++] = MAGIC_SEQUENCE[i];
				matched = 0;
				continue;
			}
			else
			{
				setPacketState(g_cmdBuf[4], 128);
				return; // Invalid escape
			}
		}

		if(c == MAGIC_SEQUENCE[matched])
			matched++;
		else
		{
			for(uint8_t i = 0; i < matched; ++i)
				unstuffed[unstuffedIdx++] = MAGIC_SEQUENCE[i];
			matched = 0;
			unstuffed[unstuffedIdx++] = c;
		}
	}

	for(uint8_t i = 0; i < matched; ++i)
		unstuffed[unstuffedIdx++] = MAGIC_SEQUENCE[i];

	uint8_t id = g_cmdBuf[4];
	
	if(id == g_ttl_bus_address)
	{
		if(unstuffedIdx == g_num_ttl_servos * 2 + 2)
		{
			for(uint8_t i = 0; i < g_num_ttl_servos; ++i)
			{
				g_ttl_servos[i].current_position = (((uint16_t)unstuffed[2*i+1]) << 8) | unstuffed[2*i];
				g_ttl_servos[i].packet_state = 0;
			}
		}
		else if(unstuffedIdx == g_num_ttl_servos * 3 + 2)
		{
			// Position + temperature
			for(uint8_t i = 0; i < g_num_ttl_servos; ++i)
			{
				g_ttl_servos[i].current_position = (((uint16_t)unstuffed[3*i+1]) << 8) | unstuffed[3*i];
				g_ttl_servos[i].present_temperature = unstuffed[3*i+2];
				g_ttl_servos[i].packet_state = 0;
			}
		}
		g_distance_sensor_value = unstuffed[unstuffedIdx - 2] | unstuffed[unstuffedIdx -1] << 8;
	}
	else // PRO Packets
	{
		if(unstuffedIdx == 12 || unstuffedIdx == 15)
		{
			int32_t* pos = (int32_t*)unstuffed;
			int16_t* torque = (int16_t*)(&unstuffed[10]);

			for(uint8_t i = 0; i < g_num_servos; ++i)
			{
				if(g_servos[i].id == id)
				{
					g_servos[i].current_position = *pos;
					g_servos[i].present_current = *torque;

					// If this is a long packet, we got temperature and voltage as well
					if(unstuffedIdx == 15)
					{
						g_servos[i].present_temperature = unstuffed[14];
						g_servos[i].present_voltage = (unstuffed[12] | (unstuffed[13] << 8)) - SERVO_VOLTAGE_OFFSET;
					}
					// Save the last status byte
					g_servos[i].error = g_cmdBuf[8];

					g_servos[i].timeouts--;
					setPacketState(id, 0);
					break;
				}
			}
		}
		// Torque Enable read (for Configure read)
		else if(unstuffedIdx == 1)
		{
			uint8_t* torque_enable = (uint8_t*) unstuffed;
			for(uint8_t i = 0; i < g_num_servos; ++i)
			{
				if(g_servos[i].id == id)
				{
					if(*torque_enable == 0)
						g_servos[i].max_torque = 0;
					g_servos[i].error = g_cmdBuf[8];
					g_servos[i].timeouts--;
					setPacketState(id, 0);
					break;
				}
			}
		}
		// Goal Torque read (for Configure read)
		else if(unstuffedIdx == 2)
		{
			uint16_t* max_torque = (uint16_t*) unstuffed;
			for(uint8_t i = 0; i < g_num_servos; ++i)
			{
				if(g_servos[i].id == id)
				{
					g_servos[i].max_torque = *max_torque;
					g_servos[i].error = g_cmdBuf[8];
					g_servos[i].timeouts--;
					setPacketState(id, 0);
					break;
				}
			}
		}
		else
		{
			// In case of something strange that cannot have been requested
			setPacketState(id, 32 + unstuffedIdx);
			return;
		}
	}
}

inline void buildBulkreadHeader()
{
	Header* header = (Header*)g_cmdBuf;
	header->header1 = 0xFF;
	header->header2 = 0xFF;
	header->header3 = 0xFD;
	header->reserved = 0;
	header->length = g_paramBytes + 3;
	header->id = 0xFE;
	header->instruction = 0x92;
}

void dxl_sendReadPacket()
{
	g_cmdBufIdx = sizeof(Header);
	g_stuffState = 0;
	g_paramBytes = 0;

	if(g_current_temp_read_id++ >= g_num_servos)
		g_current_temp_read_id = 0;

	for(uint8_t i = 0; i < g_num_servos; ++i)
	{
		BulkReadRequest request;
		request.id = g_servos[g_bulkread_indices[i]].id;
		request.addr = 611;
		// Read more bytes to also get temperature for one servo per read
		request.length = (i == g_current_temp_read_id) ? 15 : 12; // Pos, Vel, Torque and space in between
		writeParam(&request, sizeof(request));

		// Increment timeouts here, decrement it again for those servos we
		// actually got an answer from
		g_servos[i].timeouts++;
	}

	// If DXL1-Servos (in momaro: Hand-Servos) exist, add extra read
	if(g_num_ttl_servos)
	{
		BulkReadRequest request;
		request.id = g_ttl_bus_address;

		if(g_current_temp_read_id == 0)
		{
			request.addr = 0x40; // pos + temperature (3 bytes per servo) + 2bytes distance sensor
			request.length = g_num_ttl_servos * 3 + 2;
		}
		else
		{
			request.addr = 0x10; // positions (2 bytes per servo) + 2 bytes distance sensor
			request.length = g_num_ttl_servos * 2 + 2;
		}

		writeParam(&request, sizeof(request));
	}

	Header* header = (Header*)g_cmdBuf;
	header->header1 = 0xFF;
	header->header2 = 0xFF;
	header->header3 = 0xFD;
	header->reserved = 0;
	header->length = g_paramBytes + 3;
	header->id = 0xFE;
	header->instruction = 0x92;

	// Checksum calculation
	uint16_t crc = update_crc(0, g_cmdBuf, g_cmdBufIdx);

	g_cmdBuf[g_cmdBufIdx++] = crc & 0xFF;
	g_cmdBuf[g_cmdBufIdx++] = crc >> 8;

	rs485_rx_clear();
	sendCmdBuf();
}

void dxl_processReadResponse()
{
	ParserState state = STATE_WAIT_HEADER;
	int state_subIdx = 0;
	int length = 0;
	uint16_t checksum = 0;

	g_cmdBufIdx = 0;

	uint8_t packets = 0;

	uint16_t timeoutTicks = 1200;

	while(1)
	{
		while(rs485_bytesWaiting() == 0)
		{
			_delay_us(10);
			timeoutTicks--;

			if(timeoutTicks == 0)
			{
				g_packet_errors = 0xF0;
				return;
			}
		}

		while(rs485_bytesWaiting() != 0)
		{
			uint8_t c = rs485_getc();

			switch(state)
			{
				case STATE_WAIT_HEADER:
					if(c == STATUS_PACKET_HEADER[state_subIdx])
						state_subIdx++;
					else
						state_subIdx = 0;

					if(state_subIdx == sizeof(STATUS_PACKET_HEADER))
					{
						state_subIdx = 0;
						length = 0;
						memcpy(g_cmdBuf, STATUS_PACKET_HEADER, sizeof(STATUS_PACKET_HEADER));
						g_cmdBufIdx = sizeof(STATUS_PACKET_HEADER);
						state = STATE_ID;
					}
					break;
				case STATE_ID:
					state = STATE_LENGTH_LOW;
					g_cmdBuf[g_cmdBufIdx++] = c;

					setPacketState(g_cmdBuf[4], 7);
					break;
				case STATE_LENGTH_LOW:
					length = c;
					state = STATE_LENGTH_HIGH;
					g_cmdBuf[g_cmdBufIdx++] = c;

					setPacketState(g_cmdBuf[4], 8);
					break;
				case STATE_LENGTH_HIGH:
					length |= c << 8;
					state = STATE_DATA;
					g_cmdBuf[g_cmdBufIdx++] = c;

					setPacketState(g_cmdBuf[4], 9);

					break;
				case STATE_INSTRUCTION:
					if(c == 0x55)
						state = STATE_ERROR;
					else
					{
						state = STATE_WAIT_HEADER;
						state_subIdx = 0;
					}
					g_cmdBuf[g_cmdBufIdx++] = 0x55;
					length--;

					setPacketState(g_cmdBuf[4], 10);

					break;
				case STATE_ERROR:
					g_cmdBuf[g_cmdBufIdx++] = c;
					state = STATE_LENGTH_LOW;
					length--;

					setPacketState(g_cmdBuf[4], 11);

					break;
				case STATE_DATA:
					g_cmdBuf[g_cmdBufIdx++] = c;

					setPacketState(g_cmdBuf[4], 12);

					if(--length == 2)
						state = STATE_CHECKSUM_LOW;
					break;
				case STATE_CHECKSUM_LOW:
					setPacketState(g_cmdBuf[4], 13);

					checksum = c;
					state = STATE_CHECKSUM_HIGH;
					break;
				case STATE_CHECKSUM_HIGH:
				{
					checksum |= c << 8;
					uint16_t calculated = update_crc(0, g_cmdBuf, g_cmdBufIdx);
					if(calculated == checksum)
					{
// 						g_servos[0].current_position = 15;
						setPacketState(g_cmdBuf[4], 15);
						handlePacket();
						packets++;
					}
					else
					{
						setPacketState(g_cmdBuf[4], 16);
					}

					state = STATE_WAIT_HEADER;
					break;
				}
			}
		}

		if(packets == g_num_servos + 1)
		{
			g_packet_errors = 0xF1;
			break;
		}
	}
}

void dxl_initTorqueFromServo()
{
	const uint16_t TORQUE_ENABLE_ADDR = 562;
	const uint16_t MAX_TORQUE_ADDR    = 604;

	// TODO: Do also for ttl servos!!
	//
	//
	// Set max_torque to a bogus value for all servos
	for(uint8_t i = 0; i < g_num_servos; ++i)
		g_servos[i].max_torque = 65535;

	for(uint8_t tries = 0; tries < 20; ++tries)
	{
		// Read Torque Enable
		for(uint8_t i = 0; i < g_num_servos; ++i)
		{
			if(g_servos[i].max_torque != 65535)
				continue;

			g_cmdBufIdx = sizeof(Header);
			g_stuffState = 0;
			g_paramBytes = 0;
			
			BulkReadRequest request;
			request.id = g_servos[i].id;
			request.addr = TORQUE_ENABLE_ADDR;
			request.length = 1;
			writeParam(&request, sizeof(request));
			g_servos[i].timeouts++;

			buildBulkreadHeader();

			uint16_t crc = update_crc(0, g_cmdBuf, g_cmdBufIdx);

			g_cmdBuf[g_cmdBufIdx++] = crc & 0xFF;
			g_cmdBuf[g_cmdBufIdx++] = crc >> 8;

			rs485_rx_clear();
			sendCmdBuf();

			dxl_processReadResponse();
		}
		
		// Read Goal Torque for all servos with enabled torque
		for(uint8_t  i = 0; i < g_num_servos; ++i)
		{
			if(g_servos[i].max_torque != 65535)
				continue;

			g_cmdBufIdx = sizeof(Header);
			g_stuffState = 0;
			g_paramBytes = 0;
			
			BulkReadRequest request;
			request.id = g_servos[i].id;
			request.addr = MAX_TORQUE_ADDR;
			request.length = 2;
			writeParam(&request, sizeof(request));
			g_servos[i].timeouts++;

			buildBulkreadHeader();

			uint16_t crc = update_crc(0, g_cmdBuf, g_cmdBufIdx);

			g_cmdBuf[g_cmdBufIdx++] = crc & 0xFF;
			g_cmdBuf[g_cmdBufIdx++] = crc >> 8;

			rs485_rx_clear();
			sendCmdBuf();

			dxl_processReadResponse();
		}

	}

	// Set goal torque to 0 for all servos that didn't respond in any try
	for(uint8_t i = 0; i < g_num_servos; ++i)
	{
		if(g_servos[i].max_torque != 65535)
			continue;
		g_servos[i].max_torque = 0;
	}
}

void dxl_readActuators()
{
	dxl_sendReadPacket();
	dxl_processReadResponse();
}

void dxl_write_pro_positions()
{
	dxl_startSyncWrite(DXL_REG_GOAL_POSITION, 4);
	for(uint8_t i = 0; i < g_num_servos; ++i)
	{
		if(!(g_servos[i].flags & SERVO_FLAG_VELOCITY_CONTROL))
		{
			dxl_syncWriteData(g_servos[i].id, &g_servos[i].command, 4);
			g_servos[i].commandLastWritten = g_servos[i].command;
			g_servos[i].commandLastWrittenValid = true;
		}
	}
	dxl_endSyncWrite();
}

void dxl_write_pro_velocities()
{
	dxl_startSyncWrite(DXL_REG_GOAL_VELOCITY, 4);
	for(uint8_t i = 0; i < g_num_servos; ++i)
	{
		if(g_servos[i].flags & SERVO_FLAG_VELOCITY_CONTROL)
		{
			dxl_syncWriteData(g_servos[i].id, &g_servos[i].command, 4);
			g_servos[i].commandLastWritten = g_servos[i].command;
			g_servos[i].commandLastWrittenValid = true;
		}
	}
	dxl_endSyncWrite();
}

void dxl_write_ttl_positions()
{
	// sync write because normal write not implemented
	dxl_startSyncWrite(0x40, 2*g_num_ttl_servos);
	uint16_t ttl_goals[MAX_NUM_SERVOS];
	uint8_t i;
	for(i = 0; i < g_num_ttl_servos; ++i)
	{
		ttl_goals[i] = g_ttl_servos[i].command;
		g_ttl_servos[i].commandLastWritten = g_ttl_servos[i].command;
		g_ttl_servos[i].commandLastWrittenValid = true;
	}
	dxl_syncWriteData(g_ttl_bus_address, &ttl_goals, 2*i);
	dxl_endSyncWrite();
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////  EMERGENCY STOP //////////////////////////////////

void dxl_emergencystop_pro_velocity()
{
	// Set velocities so that servo-motion is stopped
	dxl_startSyncWrite(DXL_REG_GOAL_VELOCITY, 4);
	for(int i = 0; i < g_num_servos; ++i)
	{
		int32_t vel_command;
		if(g_servos[i].flags & SERVO_FLAG_VELOCITY_CONTROL)
			vel_command = 0;
		else
			vel_command = 1;
		dxl_syncWriteData(g_servos[i].id, &vel_command, 4);
	}
	dxl_endSyncWrite();
}

void dxl_emergencystop_pro_position_targets()
{
	// Set servo position-targets to last written positions for position-controlled servos

	dxl_startSyncWrite(DXL_REG_GOAL_POSITION, 4);
	for(uint8_t i = 0; i < g_num_servos; ++i)
	{
		if(!g_servos[i].commandLastWrittenValid)
			continue;

		if(!(g_servos[i].flags & SERVO_FLAG_VELOCITY_CONTROL))
			dxl_syncWriteData(g_servos[i].id, &g_servos[i].commandLastWritten, 4);
	}
	dxl_endSyncWrite();
}

void dxl_emergencystop_ttl_position_targets()
{
	// Set hand-servo targets to current positions

	dxl_startSyncWrite(0x40, 2*g_num_ttl_servos);
	uint16_t ttl_goals[MAX_NUM_SERVOS];
	uint8_t i;
	for(i = 0; i < g_num_ttl_servos; ++i)
	{
		// If commandLastWrittenValid is false, commandLastWritten is 0.
		// Just send that as the goal....
		ttl_goals[i] = g_ttl_servos[i].commandLastWritten;
	}
	dxl_syncWriteData(g_ttl_bus_address, &ttl_goals, 2*i);
	dxl_endSyncWrite();
}

void dxl_emergencystop_write_torque()
{
	// Gradually decrease servo power during hard emergency stop

	if(g_cycles_since_estop < 1000)
	{
		dxl_startSyncWrite(DXL_REG_GOAL_TORQUE, 2);
		for(uint8_t i = 0; i < g_num_servos; ++i)
		{
			int32_t torque = (1000 - g_cycles_since_estop) * g_servos[i].max_torque;
			torque /= 1000;
			if(torque < 0)
				torque = 0;

			dxl_syncWriteData(g_servos[i].id, &torque, 2);
		}
		dxl_endSyncWrite();

		dxl_startSyncWrite(0x80, 2*g_num_ttl_servos);
		uint16_t ttl_torques[MAX_NUM_SERVOS];

		for(uint8_t i = 0; i < g_num_ttl_servos; ++i)
		{
			int32_t torque = (1000 - g_cycles_since_estop) * g_ttl_servos[i].max_torque;
			torque /= 1000;
			if(torque < 0)
				torque = 0;

			ttl_torques[i] = torque;
		}
		dxl_syncWriteData(g_ttl_bus_address, ttl_torques, 2*g_num_ttl_servos);
		dxl_endSyncWrite();
	}
	else
	{
		// Switch off torque now.
		dxl_constantSyncWrite(DXL_REG_TORQUE_ENABLE, 0, 1);

		// TTL torque is automatically switched off if goal torque == 0.
		dxl_startSyncWrite(0x80, 2*g_num_ttl_servos);
		uint16_t ttl_torques[MAX_NUM_SERVOS];

		for(uint8_t i = 0; i < g_num_ttl_servos; ++i)
			ttl_torques[i] = 0;

		dxl_syncWriteData(g_ttl_bus_address, ttl_torques, 2*g_num_ttl_servos);
		dxl_endSyncWrite();
	}
}

void dxl_pro_emergencystop_pro_position_reenable()
{
	// Set servo velocity limits to their original values
	dxl_startSyncWrite(DXL_REG_GOAL_VELOCITY, 4);
	for(int i = 0; i < g_num_servos; ++i)
	{
		if(!(g_servos[i].flags & SERVO_FLAG_VELOCITY_CONTROL))
		{
			dxl_syncWriteData(g_servos[i].id, &g_servos[i].max_velocity, 4);
		}
	}
	dxl_endSyncWrite();
}

void dxl_writeActuators()
{
	if(!g_hardEmergency && !io_emergencySwitch_hard() && !io_emergencySwitch_soft())
	{
		if(g_emergency)
			dxl_pro_emergencystop_pro_position_reenable();
		g_emergency = false;

		dxl_write_pro_positions();
		dxl_write_pro_velocities();
		dxl_write_ttl_positions();
	}
	else
	{
		g_emergency = true;

		if(io_emergencySwitch_hard())
		{
			g_hardEmergency = true;

			dxl_emergencystop_write_torque();
		}

		dxl_emergencystop_pro_velocity();
		dxl_emergencystop_ttl_position_targets();
		dxl_emergencystop_pro_position_targets();
	}
	g_cycles_since_last_write = 0;
}

void dxl_rebootServo(uint8_t id)
{
	Header* header = (Header*)g_cmdBuf;
	header->header1 = 0xFF;
	header->header2 = 0xFF;
	header->header3 = 0xFD;
	header->reserved = 0;
	header->id = id;
	header->instruction = 0x08;
	header->length = 3;

	g_cmdBufIdx = sizeof(Header);

	uint16_t checksum = update_crc(0, g_cmdBuf, g_cmdBufIdx);
	g_cmdBuf[g_cmdBufIdx++] = checksum & 0xFF;
	g_cmdBuf[g_cmdBufIdx++] = checksum >> 8;

	sendCmdBuf();
}

bool dxl_isInHardEmergency()
{
	return g_hardEmergency;
}

void dxl_resetHardEmergency()
{
	if(!io_emergencySwitch_hard())
		g_hardEmergency = false;
}
