// Raw DXL functionality of controller_tool
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CONTROLLER_TOOL_DXL_H
#define CONTROLLER_TOOL_DXL_H

#include <stdint.h>

#include <vector>

// FIXME: Contains copied code from the firmware for DXL packet construction &
//   parsing. Can we refactor this into a library?

namespace dxl
{

struct DXLAnswer
{
	uint8_t id;
	uint8_t error;
	std::vector<uint8_t> data;
};

void shell();

bool readRegister(uint8_t id, uint16_t addr, uint16_t length, DXLAnswer* answer, unsigned int timeout_ms = 1000);
bool writeRegister(uint8_t id, uint16_t addr, const uint8_t* data, uint16_t length);

}

#endif
