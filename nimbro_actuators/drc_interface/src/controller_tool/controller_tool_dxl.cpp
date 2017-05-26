// Raw DXL functionality of controller_tool
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "controller_tool_dxl.h"

#include <stdint.h>

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <cstdlib>
#include <vector>

#include <sys/select.h>

#include <readline/readline.h>
#include <readline/history.h>

extern int g_fd; // Declared in controller_tool.cpp

namespace dxl
{

// ROBOTIS: CRC calculation
//@{

static uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
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

const uint8_t MAGIC_SEQUENCE[] = {0xFF, 0xFF, 0xFD};

void sendDXLPacket(uint8_t id, uint8_t instruction, const uint8_t* params, uint16_t paramLength)
{
	static const uint8_t HEADER[] = {0xFF, 0xFF, 0xFD, 0x00};
	uint8_t stuffed[256];

	memcpy(stuffed, HEADER, sizeof(HEADER));

	std::size_t idx = 8;

	int g_stuffState = 0;
	for(uint16_t i = 0; i < paramLength; ++i)
	{
		uint8_t c = params[i];

		if(c == MAGIC_SEQUENCE[g_stuffState])
			g_stuffState++;
		else
			g_stuffState = 0;

		if(g_stuffState == sizeof(MAGIC_SEQUENCE))
		{
			stuffed[idx++] = 0xFD;
			g_stuffState = 0;
		}

		stuffed[idx++] = c;
	}

	int numParams = (idx - 8) + 3;

	stuffed[4] = id;
	stuffed[5] = numParams & 0xFF;
	stuffed[6] = numParams >> 8;
	stuffed[7] = instruction;

	// CRC-16
	uint16_t crc = update_crc(0, stuffed, idx);

	stuffed[idx++] = crc & 0xFF;
	stuffed[idx++] = crc >> 8;

	printf("DXL packet:");
	for(unsigned int i = 0; i < idx; ++i)
		printf(" %02X", stuffed[i]);
	printf("\n");

	if(write(g_fd, stuffed, idx) != (int)idx)
		perror("Could not write()");
}

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

static bool readDXLAnswer(DXLAnswer* answer, unsigned int timeout_ms = 1000)
{
	ParserState state = STATE_WAIT_HEADER;
	int state_subIdx = 0;
	int length = 0;
	uint16_t checksum = 0;

	uint16_t bufIdx = 0;

	uint8_t packets = 0;

	struct timeval timeout;
	timeout.tv_sec = timeout_ms / 1000;
	timeout.tv_usec = (timeout_ms % 1000) * 1000;

	uint8_t buf[1024];

	printf("RX:");
	while(1)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(g_fd, &fds);

		int ret = select(g_fd+1, &fds, 0, 0, &timeout);
		if(ret < 0)
			perror("select()");

		if(ret == 0)
		{
			printf("\n");
			fprintf(stderr, "Timeout\n");
			return false;
		}

		uint8_t readBuf[256];
		ret = read(g_fd, readBuf, sizeof(readBuf));

		if(ret < 0)
			perror("read()");

		if(ret == 0)
		{
			fprintf(stderr, "Device closed\n");
			return false;
		}

		for(std::size_t i = 0; i < (unsigned int)ret; ++i)
		{
			uint8_t c = readBuf[i];
			printf(" %02X", c);

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
						memcpy(buf, STATUS_PACKET_HEADER, sizeof(STATUS_PACKET_HEADER));
						bufIdx = sizeof(STATUS_PACKET_HEADER);
						state = STATE_ID;
					}
					break;
				case STATE_ID:
					state = STATE_LENGTH_LOW;
					buf[bufIdx++] = c;

					break;
				case STATE_LENGTH_LOW:
					length = c;
					state = STATE_LENGTH_HIGH;
					buf[bufIdx++] = c;

					break;
				case STATE_LENGTH_HIGH:
					length |= c << 8;
					state = STATE_DATA;
					buf[bufIdx++] = c;

					break;
				case STATE_INSTRUCTION:
					if(c == 0x55)
						state = STATE_ERROR;
					else
					{
						state = STATE_WAIT_HEADER;
						state_subIdx = 0;
					}
					buf[bufIdx++] = 0x55;
					length--;

					break;
				case STATE_ERROR:
					buf[bufIdx++] = c;
					state = STATE_LENGTH_LOW;
					length--;

					break;
				case STATE_DATA:
					buf[bufIdx++] = c;

					if(--length == 2)
						state = STATE_CHECKSUM_LOW;
					break;
				case STATE_CHECKSUM_LOW:
					checksum = c;
					state = STATE_CHECKSUM_HIGH;
					break;
				case STATE_CHECKSUM_HIGH:
				{
					checksum |= c << 8;
					uint16_t calculated = update_crc(0, buf, bufIdx);
					if(calculated == checksum)
					{
						packets++;
					}
					else

					state = STATE_WAIT_HEADER;
					break;
				}
			}

			if(packets)
				break;
		}

		if(packets)
			break;
	}

	printf("\nGot packet!\n");

	if(answer)
	{
		answer->id = buf[4];
		answer->error = buf[8];

		uint16_t len = buf[5] | (((uint16_t)buf[6]) << 8);
		answer->data.resize(len);

		uint8_t unstuffedIdx = 0;

		// Unstuff
		uint8_t matched = 0;

		for(uint8_t i = 9; i < bufIdx; ++i)
		{
			uint8_t c = buf[i];

			if(matched == sizeof(MAGIC_SEQUENCE))
			{
				if(c == 0xFD) // Escape for MAGIC_SEQUENCE
				{
					for(uint8_t i = 0; i < sizeof(MAGIC_SEQUENCE); ++i)
						answer->data[unstuffedIdx++] = MAGIC_SEQUENCE[i];
					matched = 0;
					continue;
				}
				else
				{
					fprintf(stderr, "Invalid escape: %02X\n", c);
					return false; // Invalid escape
				}
			}

			if(c == MAGIC_SEQUENCE[matched])
				matched++;
			else
			{
				for(uint8_t i = 0; i < matched; ++i)
					answer->data[unstuffedIdx++] = MAGIC_SEQUENCE[i];
				matched = 0;
				answer->data[unstuffedIdx++] = c;
			}
		}

		for(uint8_t i = 0; i < matched; ++i)
			answer->data[unstuffedIdx++] = MAGIC_SEQUENCE[i];

		answer->data.resize(unstuffedIdx);
	}

	return true;
}

static void dxlPing(char* line)
{
	unsigned int id;
	sscanf(line, "ping %u", &id);

	printf("Pinging servo ID %u\n", id);
	sendDXLPacket(id, 0x01, nullptr, 0);

	DXLAnswer answer;
	if(!readDXLAnswer(&answer))
		return;

	printf("Got answer from ID %d: Error byte %d\n", answer.id, answer.error);
}

bool readRegister(uint8_t id, uint16_t addr, uint16_t length, DXLAnswer* answer, unsigned int timeout_ms)
{
	const uint8_t request[] = {
		(uint8_t)(addr & 0xFF),
		(uint8_t)(addr >> 8),
		(uint8_t)(length & 0xFF),
		(uint8_t)(length >> 8)
	};

	sendDXLPacket(id, 0x02, request, sizeof(request));

	if(!readDXLAnswer(answer, timeout_ms))
		return false;

	if(answer->data.size() != length)
		return false;

	return true;
}

bool writeRegister(uint8_t id, uint16_t addr, const uint8_t* data, uint16_t length)
{
	std::vector<uint8_t> request(2 + length);
	request[0] = addr & 0xFF;
	request[1] = addr >> 8;
	memcpy(request.data() + 2, data, length);

	sendDXLPacket(id, 0x03, request.data(), 2 + length);

	DXLAnswer answer;
	if(!readDXLAnswer(&answer))
		return false;

	if(answer.id != id)
		return false;

	if(answer.error & ~0x80)
	{
		fprintf(stderr, "Got error on write: 0x%02X\n", answer.error);
		return false;
	}

	return true;
}

static void dxlRead(char* line)
{
	unsigned int id;
	unsigned int addr;
	unsigned int length;

	if(sscanf(line, "read %u %u %u", &id, &addr, &length) != 3)
	{
		fprintf(stderr, "Usage: read <ID> <address> <length>\n");
		return;
	}

	printf("Sending read request\n");
	const uint8_t request[] = {
		(uint8_t)(addr & 0xFF),
		(uint8_t)(addr >> 8),
		(uint8_t)(length & 0xFF),
		(uint8_t)(length >> 8)
	};

	sendDXLPacket(id, 0x02, request, sizeof(request));

	DXLAnswer answer;
	if(!readDXLAnswer(&answer))
		return;

	printf("Got answer from ID %d: Error byte %d\n", answer.id, answer.error);

	if(answer.data.size() != length)
	{
		fprintf(stderr, "Got invalid answer size %lu\n", answer.data.size());
		return;
	}

	for(unsigned int i = 0; i < length; ++i)
	{
		unsigned int pAddr = addr + i;

		printf(" 0x%04X (%4d): 0x%02X (%3d)\n", pAddr, pAddr, answer.data[i], answer.data[i]);
	}

	if(length <= 4)
	{
		uint32_t value = 0;
		for(unsigned int i = 0; i < length; ++i)
			value |= ((unsigned int)answer.data[i]) << (8*i);

		printf("Aggregated: 0x%X (signed: %10d, unsigned: %10u)\n",
			value, (int32_t)value, value
		);
	}
}

static void dxlBulkRead(char* line)
{
	unsigned int id;
	unsigned int addr;
	unsigned int length;

	if(sscanf(line, "bulk_read %u %u %u", &id, &addr, &length) != 3)
	{
		fprintf(stderr, "Usage: bulk_read <ID> <address> <length>\n");
		return;
	}

	printf("Sending read request\n");
	const uint8_t request[] = {
		(uint8_t)id,
		(uint8_t)(addr & 0xFF),
		(uint8_t)(addr >> 8),
		(uint8_t)(length & 0xFF),
		(uint8_t)(length >> 8)
	};

	sendDXLPacket(id, 0x92, request, sizeof(request));

	DXLAnswer answer;
	if(!readDXLAnswer(&answer))
		return;

	printf("Got answer from ID %d: Error byte %d\n", answer.id, answer.error);

	if(answer.data.size() != length)
	{
		fprintf(stderr, "Got invalid answer size %lu\n", answer.data.size());
		return;
	}

	for(unsigned int i = 0; i < length; ++i)
	{
		unsigned int pAddr = addr + i;

		printf(" 0x%04X (%4d): 0x%02X (%3d)\n", pAddr, pAddr, answer.data[i], answer.data[i]);
	}

	if(length <= 4)
	{
		uint32_t value = 0;
		for(unsigned int i = 0; i < length; ++i)
			value |= ((unsigned int)answer.data[i]) << (8*i);

		printf("Aggregated: 0x%X (signed: %10d, unsigned: %10u)\n",
			value, (int32_t)value, value
		);
	}
}

static void dxlWrite(char* line)
{
	unsigned int id;
	unsigned int addr;
	unsigned int length;
	int value;

	if(sscanf(line, "write %u %u %u %d", &id, &addr, &length, &value) != 4)
	{
		fprintf(stderr, "Usage: write <ID> <address> <length> <value>\n");
		return;
	}

	if(length > 4)
	{
		fprintf(stderr, "Writes to more than 4 bytes are not supported.\n");
		return;
	}

	uint8_t request[2 + 4];
	request[0] = addr & 0xFF;
	request[1] = addr >> 8;

	for(unsigned int i = 0; i < length; ++i)
		request[2+i] = value >> (8*(i));

	sendDXLPacket(id, 0x03, request, 2 + length);

	DXLAnswer answer;
	if(!readDXLAnswer(&answer))
		return;

	printf("Got answer from ID %d: Error byte %d\n", answer.id, answer.error);
}

static void dxlSyncWrite(char* line)
{
	// Note (sebastian): For now allows only sync writes to single servo.
	unsigned int id;
	unsigned int addr;
	unsigned int length;
	int value;

	if(sscanf(line, "sync_write %u %u %u %d", &id, &addr, &length, &value) != 4)
	{
		fprintf(stderr, "Usage: write <ID> <address> <length> <value>\n");
		return;
	}

	if(length > 4)
	{
		fprintf(stderr, "Writes to more than 4 bytes are not supported.\n");
		return;
	}

	uint8_t request[5 + 4];
	request[0] = addr & 0xFF;
	request[1] = addr >> 8;
	request[2] = length & 0xFF;
	request[3] = length >> 8;
	request[4] = id;

	for(unsigned int i = 0; i < length; ++i)
		request[5+i] = value >> (8*(i));

	sendDXLPacket(0xFE, 0x83, request, 5 + length);
}

void shell()
{
	while(1)
	{
		char* line = readline("DXL> ");

		if(!line)
			break;

		if(line[0] == '#' || line[0] == 0)
			continue;

		add_history(line);

		if(strncmp(line, "ping", 4) == 0)
			dxlPing(line);
		else if(strncmp(line, "read", 4) == 0)
			dxlRead(line);
		else if(strncmp(line, "bulk_read", 9) == 0)
			dxlBulkRead(line);
		else if(strncmp(line, "sync_write", 10) == 0)
			dxlSyncWrite(line);
		else if(strncmp(line, "write", 4) == 0)
			dxlWrite(line);
		else if(strcmp(line, "exit") == 0)
			break;
		else
			fprintf(stderr, "Unknown command\n");
	}
}

}

