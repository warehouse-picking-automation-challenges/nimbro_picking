// Dynamixel protocol (1.0) master side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <dxl_master/dxl_master.h>

#include <string.h>

static uint8_t g_txBuffer[64];

static uint8_t g_buffer[64];
static uint8_t g_bufIdx = 0;
static uint8_t g_bufID = 0;
static uint8_t g_bufLength = 0;
static uint8_t g_bufError = 0;
static uint8_t g_checksum = 0;

static enum ParserState
{
	PARSER_STATE_INIT,
	PARSER_STATE_INIT2,
	PARSER_STATE_ID,
	PARSER_STATE_LENGTH,
	PARSER_STATE_ERROR,
	PARSER_STATE_DATA,
	PARSER_STATE_CHECKSUM
} g_parserState = PARSER_STATE_INIT;

enum DXLInstructions
{
	DXL_INSTR_PING       = 0x01,
	DXL_INSTR_READ_DATA  = 0x02,
	DXL_INSTR_WRITE_DATA = 0x03,
	DXL_INSTR_REG_WRITE  = 0x04,
	DXL_INSTR_ACTION     = 0x05,
	DXL_INSTR_RESET      = 0x06,
	DXL_INSTR_SYNC_WRITE = 0x83,
	DXL_INSTR_BULK_READ  = 0x92,
};

void dxl_master_init(void)
{
}

void dxl_master_putc(uint8_t c)
{
	switch(g_parserState)
	{
		case PARSER_STATE_INIT:
			if(c == 0xFF)
				g_parserState = PARSER_STATE_INIT2;
			break;
		case PARSER_STATE_INIT2:
			if(c == 0xFF)
				g_parserState = PARSER_STATE_ID;
			else
				g_parserState = PARSER_STATE_INIT;
			break;
		case PARSER_STATE_ID:
			if(c == 0xFF)
				break;

			g_bufID = c;
			g_parserState = PARSER_STATE_LENGTH;
			g_checksum = c;
			break;
		case PARSER_STATE_LENGTH:
			if(c == 0xFF)
			{
				g_parserState = PARSER_STATE_INIT2;
				break;
			}

			if(c > sizeof(g_buffer) + 2 || c < 2)
			{
				g_parserState = PARSER_STATE_INIT;
				break;
			}

			g_bufLength = c - 2;
			g_checksum += c;
			g_parserState = PARSER_STATE_ERROR;
			break;
		case PARSER_STATE_ERROR:
			if(c == 0xFF)
			{
				g_parserState = PARSER_STATE_INIT2;
				break;
			}

			g_bufError = c;
			g_checksum += c;
			if(g_bufLength)
			{
				g_parserState = PARSER_STATE_DATA;
				g_bufIdx = 0;
			}
			else
				g_parserState = PARSER_STATE_CHECKSUM;
			break;
		case PARSER_STATE_DATA:
			g_buffer[g_bufIdx++] = c;
			g_checksum += c;

			if(g_bufIdx == g_bufLength)
				g_parserState = PARSER_STATE_CHECKSUM;
			break;
		case PARSER_STATE_CHECKSUM:
			if(((uint8_t)~g_checksum) == c)
			{
				dxl_master_handleStatus(g_bufID, g_bufError, g_buffer, g_bufLength);
			}
			g_parserState = PARSER_STATE_INIT;
			break;
	}
}

void dxl_master_reset_parser()
{
	g_parserState = PARSER_STATE_INIT;
}

void dxl_master_ping(uint8_t id)
{
	uint8_t packet[] = {
		0xFF, 0xFF, id, 0x02, DXL_INSTR_PING,
		~(id + 0x02 + DXL_INSTR_PING)
	};

	dxl_master_send(packet, sizeof(packet));
}

void dxl_master_read(uint8_t id, uint8_t addr, uint8_t size)
{
	uint8_t packet[] = {
		0xFF, 0xFF, id, 2 + 2, DXL_INSTR_READ_DATA,
		addr, size,
		~(id + 2 + 2 + DXL_INSTR_READ_DATA + addr + size)
	};

	dxl_master_send(packet, sizeof(packet));
}

void dxl_master_write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t size)
{
	uint8_t i;
	uint8_t checksum = id + size + 3 + DXL_INSTR_WRITE_DATA + addr;

	g_txBuffer[0] = 0xFF;
	g_txBuffer[1] = 0xFF;
	g_txBuffer[2] = id;
	g_txBuffer[3] = size + 3;
	g_txBuffer[4] = DXL_INSTR_WRITE_DATA;
	g_txBuffer[5] = addr;

	for(i = 0; i < size; ++i)
	{
		g_txBuffer[6 + i] = data[i];
		checksum += data[i];
	}

	g_txBuffer[6 + i] = ~checksum;

	dxl_master_send(g_txBuffer, 6 + i + 1);
}

void dxl_master_bulkread_init(uint8_t* packet, uint8_t num_servos)
{
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = 0xFE;
	packet[3] = 3 + 3*num_servos;
	packet[4] = DXL_INSTR_BULK_READ;
	packet[5] = 0;
}

void dxl_master_bulkread_set(uint8_t* packet, uint8_t idx, uint8_t id, uint8_t start, uint8_t len )
{
	uint8_t* base = packet + 6 + 3*idx;
	base[0] = len;
	base[1] = id;
	base[2] = start;
}

uint8_t dxl_master_bulkread_finalize(uint8_t* packet)
{
	uint8_t num_params = packet[3] - 2;
	uint8_t checksum = 0xFE + packet[3] + packet[4];
	uint8_t i;

	for(i = 0; i < num_params; ++i)
		checksum += packet[5 + i];

	packet[5 + num_params] = ~checksum;

	return 5 + num_params + 1;
}

void dxl_master_syncwrite_init(uint8_t* packet, uint8_t num_servos, uint8_t addr, uint8_t len)
{
	packet[0] = 0xFF;
	packet[1] = 0xFF;
	packet[2] = 0xFE;
	packet[3] = 2 + 2 + num_servos * (len + 1);
	packet[4] = DXL_INSTR_SYNC_WRITE;
	packet[5] = addr;
	packet[6] = len;
}

void dxl_master_syncwrite_set(uint8_t* packet, uint8_t idx, uint8_t id, const uint8_t* data)
{
	uint8_t len = packet[6];
	uint8_t* base = packet + 7 + idx * (len+1);

	base[0] = id;
	memcpy(base + 1, data, len);
}

uint8_t dxl_master_syncwrite_finalize(uint8_t* packet)
{
	uint8_t num_params = packet[3] - 2;
	uint8_t checksum = 0xFE + packet[3] + packet[4];
	uint8_t i;

	for(i = 0; i < num_params; ++i)
		checksum += packet[5 + i];

	packet[5 + num_params] = ~checksum;

	return 4 + packet[3];
}
