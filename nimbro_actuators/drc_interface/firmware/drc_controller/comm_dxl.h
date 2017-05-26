// Dynamixel communication
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COMM_DXL_H
#define COMM_DXL_H

#include <stdint.h>

enum DXLProRegister
{
	DXL_REG_TORQUE_ENABLE = 562,
	DXL_REG_GOAL_POSITION = 596,
	DXL_REG_GOAL_VELOCITY = 600,
	DXL_REG_GOAL_TORQUE = 604,
};

struct DXLPosVelTorque
{
	int32_t position;
	uint32_t velocity;
	uint16_t torque;
} __attribute__((packed));

void dxl_constantSyncWrite(uint16_t addr, uint32_t value, uint16_t size);
void dxl_readActuators();
void dxl_writeActuators();
void dxl_write_ttl_positions();
void dxl_initTorqueFromServo();

void dxl_startSyncWrite(uint16_t addr, uint16_t data_size);
void dxl_syncWriteData(uint8_t id, void* data, uint16_t size);
void dxl_endSyncWrite();

void dxl_rebootServo(uint8_t id);

bool dxl_isInHardEmergency();
void dxl_resetHardEmergency();

#endif
