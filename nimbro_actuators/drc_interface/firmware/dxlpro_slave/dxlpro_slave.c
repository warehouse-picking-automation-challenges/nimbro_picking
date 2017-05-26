// Dynamixel bus (1.0) slave device library
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "dxlpro_slave.h"
#include <string.h>

static void debug(const char* fmt, ...) __attribute__((format (printf, 1, 2)));

#define PACKET_DEBUG 0
#if PACKET_DEBUG
#include <stdio.h>
#include <stdarg.h>

static void debug(const char* fmt, ...)
{
	va_list l;
	va_start(l, fmt);

	vfprintf(stderr, fmt, l);

	va_end(l);
}
#else

static inline void debug(const char* fmt, ...)
{
}

#endif


static const uint16_t crc_table[256] = {
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

static uint16_t update_crc(uint16_t crc_accum, uint8_t data)
{
	uint16_t i;

	i = ((uint16_t)(crc_accum >> 8) ^ data) & 0xFF;
	crc_accum = (crc_accum << 8) ^ crc_table[i];

	return crc_accum;
}


static uint8_t g_buffer[64];
static uint8_t g_bufIdx = 0;
static uint8_t g_bufID = 0;
static uint16_t g_bufLength = 0;
static uint8_t g_bufInstr = 0;
static uint8_t g_id = 0;
static uint16_t g_model = 0;
static uint8_t g_fwVersion = 0;
static uint16_t g_checksum = 0;
static uint16_t g_receivedChecksum = 0;

static uint8_t g_txBuf[64];
static uint8_t g_txBufIdx = 0;
static uint8_t g_txParamBytes = 0;

static uint8_t g_bulkReadResponseAfterID = 0;
static uint16_t g_bulkReadStart = 0;
static uint16_t g_bulkReadLength = 0;

const uint8_t MAGIC_SEQUENCE[] = {0xFF, 0xFF, 0xFD};
uint8_t g_stuffState = 0;

enum ParserState
{
	PARSER_STATE_WAIT_HEADER,
	PARSER_STATE_ID,
	PARSER_STATE_LENGTH_LOW,
	PARSER_STATE_LENGTH_HIGH,
	PARSER_STATE_INSTRUCTION,
	PARSER_STATE_DATA,
	PARSER_STATE_CHECKSUM_LOW,
	PARSER_STATE_CHECKSUM_HIGH
};

static uint8_t g_parserState = PARSER_STATE_WAIT_HEADER;

enum DXLInstructions
{
	DXLPRO_INSTR_PING       = 0x01,
	DXLPRO_INSTR_READ_DATA  = 0x02,
	DXLPRO_INSTR_WRITE_DATA = 0x03,
	DXLPRO_INSTR_REG_WRITE  = 0x04,
	DXLPRO_INSTR_ACTION     = 0x05,
	DXLPRO_INSTR_RESET      = 0x06,
	DXLPRO_STATUS_PACKET    = 0x55,
	DXLPRO_INSTR_SYNC_WRITE = 0x83,
	DXLPRO_INSTR_BULK_READ  = 0x92,
};

void dxlpro_slave_init(uint8_t id, uint16_t model, uint8_t fw_version)
{
	g_id = id;
	g_model = model;
	g_fwVersion = fw_version;
}

extern uint8_t __attribute__((weak)) dxlpro_slave_match_id(uint8_t id);
uint8_t dxlpro_slave_match_id(uint8_t id)
{
	if(id == g_id || id == 0xFE)
		return 1;

	return 0;
}

extern void __attribute__((weak)) dxlpro_slave_ctrl_apply(void);
void dxlpro_slave_ctrl_apply(void)
{
}

extern void __attribute__((weak)) dxlpro_slave_send_begin(void);
void dxlpro_slave_send_begin(void)
{
}

extern void __attribute__((weak)) dxlpro_slave_send_end(void);
void dxlpro_slave_send_end(void)
{
}

static void dxlpro_slave_sendByte(uint8_t c)
{
	if(c == MAGIC_SEQUENCE[g_stuffState])
		g_stuffState++;
	else
		g_stuffState = 0;

	if(g_txBufIdx >= sizeof(g_txBuf) - 4)
		return;

	g_txBuf[g_txBufIdx++] = c;
	g_txParamBytes++;

	if(g_stuffState == sizeof(MAGIC_SEQUENCE))
	{
		g_txBuf[g_txBufIdx++] = 0xFD;
		g_txParamBytes++;
		g_stuffState = 0;
	}
}

void dxlpro_slave_sendStatus(uint8_t error, const uint8_t* data, uint8_t len)
{
	uint8_t i;

	if(len + 20 > sizeof(g_txBuf))
		return;

	g_txBuf[0] = 0xFF;
	g_txBuf[1] = 0xFF;
	g_txBuf[2] = 0xFD;
	g_txBuf[3] = 0x00;
	g_txBuf[4] = g_id;
	// 5, 6: length
	g_txBuf[7] = 0x55;
	g_txBuf[8] = error;

	g_stuffState = 0;
	g_txBufIdx = 9;
	g_txParamBytes = 4;

	for(i = 0; i < len; ++i)
		dxlpro_slave_sendByte(data[i]);

	// Length
	g_txBuf[5] = g_txParamBytes & 0xFF;
	g_txBuf[6] = g_txParamBytes >> 8;

	// Checksum
	uint16_t checksum = 0;

	for(i = 0; i < g_txBufIdx; ++i)
		checksum = update_crc(checksum, g_txBuf[i]);

	g_txBuf[g_txBufIdx++] = checksum & 0xFF;
	g_txBuf[g_txBufIdx++] = checksum >> 8;

	dxlpro_slave_send_begin();

	for(i = 0; i < g_txBufIdx; ++i)
		dxlpro_slave_send(g_txBuf[i]);

	dxlpro_slave_send_end();
}

void dxlpro_send_bulkReadAnswer(void)
{
	uint8_t data[32];
	uint8_t i;

	g_bulkReadResponseAfterID = 0;

	if(g_bulkReadLength > sizeof(data))
		return;

	for(i = 0; i < g_bulkReadLength; ++i)
		data[i] = dxlpro_slave_ctrl_get(g_bulkReadStart + i);

	dxlpro_slave_sendStatus(0, data, g_bulkReadLength);
}

static void handle_packet(void)
{
	uint8_t matched = 0;
	uint8_t unstuffedIdx = 0; // start behind the header
	uint8_t i, j;

	// Check ID
	switch(g_bufInstr)
	{
		case DXLPRO_INSTR_PING:
		case DXLPRO_INSTR_READ_DATA:
		case DXLPRO_INSTR_WRITE_DATA:
			if(!dxlpro_slave_match_id(g_bufID))
				return; // we are not meant

			if(g_bufInstr == DXLPRO_INSTR_PING && g_bufID == 0xFE)
				return; // FIXME: broadcast pings are not supported at this moment

			break;
		case DXLPRO_INSTR_BULK_READ:
		case DXLPRO_INSTR_SYNC_WRITE:
			if(g_bufID != 0xFE)
				return; // bulk/sync always goes to broadcast addr
			break;
		case DXLPRO_STATUS_PACKET:
			// Are we currently waiting for a status packet?
			if(g_bulkReadResponseAfterID != 0)
			{
				if(g_bufID == g_bulkReadResponseAfterID)
					dxlpro_send_bulkReadAnswer();
				else
					return;
			}
			else
				return;
		default:
			return;
	}

	if(g_bufIdx >= 64)
		return;

	// Stable unstuffing
	for(i = 0; i < g_bufIdx; ++i)
	{
		uint8_t c = g_buffer[i];

		if(matched == sizeof(MAGIC_SEQUENCE))
		{
			debug("%02x -> ESCAPE!\n", c);
			if(c == 0xFD) // Escape for MAGIC_SEQUENCE
			{
				for(j = 0; j < sizeof(MAGIC_SEQUENCE); ++j)
				{
					g_buffer[unstuffedIdx++] = MAGIC_SEQUENCE[j];
				}
				matched = 0;
				continue;
			}
			else
				return; // invalid escape
		}

		if(c == MAGIC_SEQUENCE[matched])
		{
			debug("%02x -> matched: %d\n", c, matched);
			matched++;
		}
		else
		{
			for(j = 0; j < matched; ++j)
				g_buffer[unstuffedIdx++] = MAGIC_SEQUENCE[j];
			matched = 0;
			debug("%02x -> matched: %d\n", c, matched);
			g_buffer[unstuffedIdx++] = c;
		}
	}

	for(j = 0; j < matched; ++j)
		g_buffer[unstuffedIdx++] = MAGIC_SEQUENCE[j];

	debug("Unstuffed packet from %d to %d bytes\n", g_bufIdx, unstuffedIdx);

	debug("Handling instruction %d\n", g_bufInstr);

	uint16_t numParams = unstuffedIdx;

	switch(g_bufInstr)
	{
		case DXLPRO_INSTR_PING:
		{
			if(!dxlpro_slave_match_id(g_bufID))
				return;

			uint8_t params[] = {
				g_model & 0xFF,
				g_model >> 8,
				g_fwVersion
			};

			dxlpro_slave_sendStatus(0, params, sizeof(params));
			break;
		}
		case DXLPRO_INSTR_READ_DATA:
		{
			uint8_t idx = 0;
			uint16_t start = g_buffer[0] | ((uint16_t)g_buffer[1] << 8);
			uint16_t len = g_buffer[2] | ((uint16_t)g_buffer[3] << 8);
			uint8_t data[128];

			if(!dxlpro_slave_match_id(g_bufID))
				return;

			if(numParams != 4)
			{
				dxlpro_slave_sendStatus(0x01, 0, 0);
				return;
			}

			if(len > sizeof(g_txBuf) || len > 128)
			{
				dxlpro_slave_sendStatus(0x02, 0, 0);
				return;
			}

			for(; idx != len; ++idx)
			{
				data[idx] = dxlpro_slave_ctrl_get(start + idx);
			}

			dxlpro_slave_sendStatus(0, data, len);
			break;
		}
		case DXLPRO_INSTR_WRITE_DATA:
		{
			uint16_t idx = 0;
			uint16_t start = g_buffer[0] | ((uint16_t)g_buffer[1] << 8);

			if(!dxlpro_slave_match_id(g_bufID))
				return;

			debug("WRITE: %d bytes to 0x%04x\n", numParams - 2, start);
			debug("buffer:");
			for(i = 0; i < numParams; ++i)
				debug(" %02x", g_buffer[i]);
			debug("\n");

			if(numParams < 3)
				return;

			for(; idx != numParams-2; ++idx)
				dxlpro_slave_ctrl_set(start + idx, g_buffer[2 + idx]);

			dxlpro_slave_sendStatus(0, NULL, 0);

			dxlpro_slave_ctrl_apply();

			break;
		}
		case DXLPRO_INSTR_SYNC_WRITE:
		{
			uint8_t idx = 4;
			uint16_t start = g_buffer[0] | ((uint16_t)g_buffer[1] << 8);
			uint16_t len = g_buffer[2] | ((uint16_t)g_buffer[3] << 8);

			if(numParams <= 4)
				return;

			debug("SYNC WRITE %d bytes to addr %d\n", len, start);

			for(; idx < numParams; idx += (len+1))
			{
				uint8_t i;
				uint8_t id = g_buffer[idx];
				if(!dxlpro_slave_match_id(id))
					continue;

				g_bufID = id;
				for(i = 0; i < len; ++i)
					dxlpro_slave_ctrl_set(start + i, g_buffer[idx + 1 + i]);
			}

			dxlpro_slave_ctrl_apply();

			break;
		}
		case DXLPRO_INSTR_BULK_READ:
		{
			uint8_t adressed = 0;
			uint8_t prevID = 0;
			uint8_t i;

			uint16_t start = 0;
			uint16_t len = 0;

			for(i = 0; i < numParams; i += 5)
			{
				if(dxlpro_slave_match_id(g_buffer[i]))
				{
					adressed = 1;
					start = g_buffer[i+1] | (((uint16_t)g_buffer[i+2]) << 8);
					len = g_buffer[i+3] | (((uint16_t)g_buffer[i+4]) << 8);
				}

				if(!adressed)
					prevID = g_buffer[i];
			}

			if(!adressed)
				return;

			debug("BULK READ: I'm answering after ID %d\n", prevID);
			debug("BULK READ: %d bytes from %d\n", len, start);

			g_bulkReadResponseAfterID = prevID;
			g_bulkReadLength = len;
			g_bulkReadStart = start;

			if(prevID == 0)
				dxlpro_send_bulkReadAnswer();

			break;
		}
	}
}

static void dxlpro_slave_handle_byte(uint8_t c)
{
	if(g_parserState != PARSER_STATE_CHECKSUM_LOW && g_parserState != PARSER_STATE_CHECKSUM_HIGH)
	{
		g_checksum = update_crc(g_checksum, c);
	}

	switch(g_parserState)
	{
		case PARSER_STATE_WAIT_HEADER:
			break;
		case PARSER_STATE_ID:
			g_bufID = c;
			g_parserState = PARSER_STATE_LENGTH_LOW;
			break;
		case PARSER_STATE_LENGTH_LOW:
			g_bufLength = c;
			g_parserState = PARSER_STATE_LENGTH_HIGH;
			break;
		case PARSER_STATE_LENGTH_HIGH:
			g_bufLength |= (c << 8);

			if(g_bufLength > sizeof(g_buffer) + 2)
			{
				g_parserState = PARSER_STATE_WAIT_HEADER;
				break;
			}
			g_parserState = PARSER_STATE_INSTRUCTION;
			break;
		case PARSER_STATE_INSTRUCTION:
			g_bufInstr = c;
			if(g_bufLength > 3)
			{
				g_bufIdx = 0;
				g_parserState = PARSER_STATE_DATA;
			}
			else
				g_parserState = PARSER_STATE_CHECKSUM_LOW;
			break;
		case PARSER_STATE_DATA:
			g_buffer[g_bufIdx++] = c;

			if(--g_bufLength == 3)
				g_parserState = PARSER_STATE_CHECKSUM_LOW;
			break;
		case PARSER_STATE_CHECKSUM_LOW:
			g_receivedChecksum = c;
			g_parserState = PARSER_STATE_CHECKSUM_HIGH;
			break;
		case PARSER_STATE_CHECKSUM_HIGH:
			g_parserState = PARSER_STATE_WAIT_HEADER;

			g_receivedChecksum |= (c << 8);

			if(g_receivedChecksum == g_checksum)
			{
				handle_packet();
			}
			break;
	}
}

static const uint8_t PACKET_HEADER[] = {0xFF, 0xFF, 0xFD, 0x00};
static uint8_t g_headerIdx = 0;

void dxlpro_slave_putc(uint8_t c)
{
	uint8_t i;

	//FIXME: What happens if the checksum contains an 0xFF?
	// we will never parse the packet until the next one arrives...

	if(c == PACKET_HEADER[g_headerIdx] && g_parserState != PARSER_STATE_CHECKSUM_LOW && g_parserState != PARSER_STATE_CHECKSUM_HIGH)
	{
		g_headerIdx++;

		if(g_headerIdx == sizeof(PACKET_HEADER))
		{
			g_headerIdx = 0;
			g_bufLength = 0;
			g_bufIdx = 0;
			g_parserState = PARSER_STATE_ID;

			g_checksum = 0;
			for(i = 0; i < sizeof(PACKET_HEADER); ++i)
			{
				g_checksum = update_crc(g_checksum, PACKET_HEADER[i]);
			}
		}
	}
	else
	{
		for(i = 0; i < g_headerIdx; ++i)
			dxlpro_slave_handle_byte(PACKET_HEADER[i]);

		dxlpro_slave_handle_byte(c);
		g_headerIdx = 0;
	}
}
