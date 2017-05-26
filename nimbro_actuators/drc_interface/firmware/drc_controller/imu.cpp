// IMU driver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "imu.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

void imu_init()
{
	// Setup I2C for 100kHz SCL frequency
	TWBR = 72;
	TWSR = 0;

	TWCR = (1 << TWEN) | (1 << TWIE);

	// Kick off an initial START condition
	TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWSTA) | (1 << TWINT);
}

enum GlobState
{
	STATE_INIT,
	STATE_READ
};

static uint8_t g_accData[6];
static uint8_t g_gyroData[6];
static uint8_t g_imuDataBackBuf[6];
static uint8_t g_imuDataValid = 0;

struct InitCmd
{
	uint8_t addr;
	uint8_t reg;
	uint8_t value;
};

InitCmd g_initCmds[] = {
	// Accelerometer setup
	{0x53, 0x31, 0x09},
	{0x53, 0x2D, 0x08},

	// Gyro setup
	{0x68, 0x15, 0x09}, // 100 Hz sample rate
	{0x68, 0x16, 0x1a}, // 98 Hz low pass, 2000°/sec full scale
	{0, 0}
};

static uint8_t g_initIdx = 0;
static uint8_t g_initRegSent = 0;
static uint8_t g_readIdx = 0;

enum Sensor
{
	ACC,
	GYRO
};
Sensor g_currentSensor = ACC;

ISR(TWI_vect)
{
	switch(TW_STATUS)
	{
		case TW_START:
			if(g_initCmds[g_initIdx].reg != 0)
				TWDR = (g_initCmds[g_initIdx].addr << 1) | 0;
			else if(g_currentSensor == ACC)
				TWDR = (0x53 << 1) | 0;
			else
				TWDR = (0x68 << 1) | 0;

			TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
			break;
		case TW_REP_START:
			if(g_currentSensor == ACC)
				TWDR = (0x53 << 1) | 1;
			else
				TWDR = (0x68 << 1) | 1;

			TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
			break;
		case TW_MT_SLA_ACK:
			if(g_initCmds[g_initIdx].reg != 0)
			{
				TWDR = g_initCmds[g_initIdx].reg;
				g_initRegSent = 0;
			}
			else if(g_currentSensor == ACC)
				TWDR = 0x32;
			else
				TWDR = 0x1d;

			TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
			break;
		case TW_MR_SLA_ACK:
			TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
			break;
		case TW_MT_DATA_ACK:
			if(g_initCmds[g_initIdx].reg != 0)
			{
				if(g_initRegSent == 0)
				{
					g_initRegSent = 1;
					TWDR = g_initCmds[g_initIdx].value;
					TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
				}
				else
				{
					TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWSTO) | (1 << TWINT) | (1 << TWSTA);
					g_initIdx++;
				}
			}
			else
			{
				TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWSTA);
			}
			break;
		case TW_MR_DATA_ACK:
		case TW_MR_DATA_NACK:
			g_imuDataBackBuf[g_readIdx] = TWDR;
			g_readIdx++;

			if(g_readIdx == 6)
			{
				PORTJ ^= (1 << 7);
				g_readIdx = 0;

				if(g_currentSensor == ACC)
				{
					for(uint8_t i = 0; i < 6; ++i)
						g_accData[i] = g_imuDataBackBuf[i];
					g_currentSensor = GYRO;
				}
				else
				{
					for(uint8_t i = 0; i < 6; ++i)
						g_gyroData[i] = g_imuDataBackBuf[i];
					g_currentSensor = ACC;
				}

				g_imuDataValid = 1;

				TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWSTO) | (1 << TWSTA);
			}
			else if(g_readIdx == 5)
				TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
			else
				TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
			break;
		case TW_NO_INFO:
			TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
			break;
		default:
			TWCR = (1 << TWEN) | (1 << TWSTO) | (1 << TWSTA) | (1 << TWINT);
			break;
	}
}

void imu_getData(int16_t* accX, int16_t* accY, int16_t* accZ, int16_t* gyroX, int16_t* gyroY, int16_t* gyroZ)
{
	cli();
	*accX = (((uint16_t)g_accData[1]) << 8) | g_accData[0];
	*accY = (((uint16_t)g_accData[3]) << 8) | g_accData[2];
	*accZ = (((uint16_t)g_accData[5]) << 8) | g_accData[4];
	*gyroX = (((uint16_t)g_gyroData[0]) << 8) | g_gyroData[1];
	*gyroY = (((uint16_t)g_gyroData[2]) << 8) | g_gyroData[3];
	*gyroZ = (((uint16_t)g_gyroData[4]) << 8) | g_gyroData[5];
	sei();
}

uint8_t imu_isValid()
{
	return g_imuDataValid;
}
