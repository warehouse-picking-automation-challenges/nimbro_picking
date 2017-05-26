// Dynamixel bus (1.0) slave device library
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DXLPRO_SLAVE_H
#define DXLPRO_SLAVE_H

/**
 * @file
 *
 * @brief Dynamixel bus (2.0) slave device library
 * @author Max Schwarz <max.schwarz@uni-bonn.de>
 *
 * This is a simple implementation of the DXLPRO protocol for the slave side. You
 * need to implement these functions in your application:
 *
 * <ul>
 *   <li>dxlpro_slave_send()</li>
 *   <li>dxlpro_slave_ctrl_get()</li>
 *   <li>dxlpro_slave_ctrl_set()</li>
 * </ul>
 **/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Initialized the library
 *
 * @param id Slave ID of this device
 * @param model Model ID
 * @param fw_version Firmware version
 **/
void dxlpro_slave_init(uint8_t id, uint16_t model, uint8_t fw_version);

/**
 * @brief Data input method
 *
 * Call this method with every byte you receive from the DXLPRO bus.
 **/
void dxlpro_slave_putc(uint8_t c);

uint8_t dxlpro_slave_current_id(void);

enum DXLPROSlaveStandardRegisters
{
	DXLPRO_SLAVE_REG_MODEL_NUMBER_L     = 0x00,
	DXLPRO_SLAVE_REG_MODEL_NUMBER_H     = 0x01,
	DXLPRO_SLAVE_REG_VERSION            = 0x02,
	DXLPRO_SLAVE_REG_ID                 = 0x03,
	DXLPRO_SLAVE_REG_BAUD               = 0x04,
};

//! @name Application hooks
//@{

/**
 * @brief Send data over the bus
 *
 * Implement this method so that it sends @b c over the dynamixel bus.
 **/
void dxlpro_slave_send(uint8_t c);

void dxlpro_slave_send_begin(void);
void dxlpro_slave_send_end(void);

/**
 * @brief Get control table contents
 *
 * Implement this method.
 *
 * @param addr Byte address of the control table location being read
 * @return Control table value at that location
 **/
uint8_t dxlpro_slave_ctrl_get(uint16_t addr);

/**
 * @brief Set control table contents
 *
 * Implement this method.
 *
 * @param addr Byte address of the control table location being set
 * @param data Value
 **/
void dxlpro_slave_ctrl_set(uint16_t addr, uint8_t value);

/**
 * @brief Signal end-of-packet
 *
 * You can implement this method to get notified when a write packet is
 * finished, e.g. to apply the changes.
 **/
void dxlpro_slave_ctrl_apply();

//@}

#ifdef __cplusplus
}
#endif

#endif
