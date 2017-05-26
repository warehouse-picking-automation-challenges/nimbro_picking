// Commands for communicating with the hand controller
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "controller_tool_hand.h"
#include "controller_tool_dxl.h"

#include "../../firmware/hand_bootloader/interface.h"

#include <stdio.h>
#include <unistd.h>
#include <string.h>

namespace hand
{

static int doFlash(FILE* f)
{
	uint8_t pagebuf[256];

	uint32_t pageIdx = 0;

	while(!feof(f))
	{
		unsigned int retryCount = 0;
		memset(pagebuf, 0xFF, sizeof(pagebuf));

		int bytes = fread(pagebuf, 1, sizeof(pagebuf), f);
		if(bytes == 0)
			continue;
		if(bytes < 0)
		{
			perror("Could not read from input file");
			return 1;
		}

		for(int i = 0; i < 8; ++i)
		{
			const unsigned int packetSize = 256/8;
			uint16_t off = i * packetSize;

			const uint8_t* packetData = pagebuf + off;
			while(!dxl::writeRegister(BOOTLOADER_DXL_ID, BOOTLOADER_REG_PAGEBUFFER + off, packetData, packetSize))
			{
				fprintf(stderr, "Retrying to write packet %d of page %d\n", i, pageIdx);
				usleep(100 * 1000);
			}
		}

		uint32_t pageAddr = pageIdx * 256;

#if 0
		printf("Read back before write\n");
		for(int i = 0; i < 8; ++i)
		{
			const unsigned int packetSize = 256/8;
			uint16_t off = i * packetSize;

			dxl::DXLAnswer answer;
			while(!dxl::readRegister(BOOTLOADER_DXL_ID, BOOTLOADER_REG_PAGEBUFFER + off, packetSize, &answer))
			{
				fprintf(stderr, "Retrying to write packet %d of page %d\n", i, pageIdx);
				usleep(100 * 1000);
			}

			for(unsigned int j = 0; j < packetSize; ++j)
			{
				if(pagebuf[off + j] != answer.data[j])
				{
					fprintf(stderr, "Before-Write mismatch at addr 0x%X: write 0x%02X, got 0x%02X\n", pageAddr + j, pagebuf[off + j], answer.data[j]);
					return 1;
				}
			}
		}
#endif

		uint8_t cmd[] = {
			BOOTLOADER_COMMAND_WRITE_PAGE,
			(uint8_t)(pageAddr),
			(uint8_t)(pageAddr >> 8),
			(uint8_t)(pageAddr >> 16),
			(uint8_t)(pageAddr >> 24)
		};
		while(!dxl::writeRegister(BOOTLOADER_DXL_ID, BOOTLOADER_REG_COMMAND, cmd, sizeof(cmd)))
		{
			if(++retryCount > 100)
				return 1;

			fprintf(stderr, "Retrying to issue write command...\n");
			usleep(100 * 1000);
		}

		usleep(10 * 1000);

		printf("Waiting for write complete...\n");
		dxl::DXLAnswer answer;
		while(!dxl::readRegister(BOOTLOADER_DXL_ID, BOOTLOADER_REG_COMMAND, 1, &answer, 10))
		{
			if(++retryCount > 100)
				return 1;

			usleep(100 * 1000);
		}
		if(answer.data[0] != BOOTLOADER_COMMAND_IDLE)
		{
			fprintf(stderr, "Invalid bootloader return code while writing: %u\n", answer.data[0]);
			return 1;
		}

		cmd[0] = BOOTLOADER_COMMAND_READ_PAGE;
		while(!dxl::writeRegister(BOOTLOADER_DXL_ID, BOOTLOADER_REG_COMMAND, cmd, sizeof(cmd)))
		{
			if(++retryCount > 100)
				return 1;

			fprintf(stderr, "Retrying to issue read command...\n");
			usleep(100 * 1000);
		}

		printf("Waiting for read complete\n");
		while(!dxl::readRegister(BOOTLOADER_DXL_ID, BOOTLOADER_REG_COMMAND, 1, &answer))
		{
			if(++retryCount > 100)
				return 1;

			usleep(100 * 1000);
		}
		if(answer.data[0] != BOOTLOADER_COMMAND_IDLE)
		{
			fprintf(stderr, "Invalid bootloader return code while writing: %u\n", answer.data[0]);
			return 1;
		}

		printf("Read back\n");
		for(int i = 0; i < 8; ++i)
		{
			const unsigned int packetSize = 256/8;
			uint16_t off = i * packetSize;

			dxl::DXLAnswer answer;
			while(!dxl::readRegister(BOOTLOADER_DXL_ID, BOOTLOADER_REG_PAGEBUFFER + off, packetSize, &answer))
			{
				fprintf(stderr, "Retrying to write packet %d of page %d\n", i, pageIdx);
				usleep(100 * 1000);
			}

			for(unsigned int j = 0; j < packetSize; ++j)
			{
				if(pagebuf[off + j] != answer.data[j])
				{
					fprintf(stderr, "After-Write mismatch at addr 0x%X: write 0x%02X, got 0x%02X\n", pageAddr + j, pagebuf[off + j], answer.data[j]);
					return 1;
				}
			}
		}

		printf("Page verified.\n");

		pageIdx++;
	}

	return 0;
}

int flash(const std::string& file)
{
	FILE* f = fopen(file.c_str(), "r");
	if(!f)
	{
		perror("Could not open input file");
		return 1;
	}

	printf("Trying to enter bootloader...\n");

	while(1)
	{
		// Place hand controller in bootloader mode
		uint8_t code = 0xAB;
		dxl::writeRegister(29, 0x100, &code, 1);

		// Check if successful
		dxl::DXLAnswer answer;
		if(dxl::readRegister(BOOTLOADER_DXL_ID, 0, 2, &answer))
		{
			printf("Got model: 0x%02X%02X\n", answer.data[1], answer.data[0]);
			if(answer.data[0] == (BOOTLOADER_MODEL & 0xFF)
				&& answer.data[1] == (BOOTLOADER_MODEL >> 8))
			{
				break;
			}
		}
	}

	printf("Entered.\n");

	int ret = doFlash(f);

	if(ret == 0)
	{
		printf("Exiting bootloader...\n");

		for(int i = 0; i < 10; ++i)
		{
			uint8_t command = BOOTLOADER_COMMAND_EXIT;
			if(dxl::writeRegister(BOOTLOADER_DXL_ID, BOOTLOADER_REG_COMMAND, &command, 1))
				break;
		}
	}

	return ret;
}

}

