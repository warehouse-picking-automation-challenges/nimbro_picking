// Dynamixel protocol (1.0) master side
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef DXL_MASTER_H
#define DXL_MASTER_H

#include <stdint.h>

/**
 * @file
 *
 * @brief Dynamixel bus (1.0) master library
 * @author Max Schwarz <max.schwarz@uni-bonn.de>
 *
 * This is a simple implementation of the DXL protocol for the master side. You
 * need to implement these functions in your application:
 *
 * <ul>
 *   <li>dxl_master_send()</li>
 *   <li>dxl_master_handleStatus()</li>
 * </ul>
 **/

#ifdef __cplusplus
extern "C"
{
#endif

//! Initialize the library
void dxl_master_init(void);

/**
 * @brief Data input method
 *
 * Call this method with every byte you receive from the DXL bus.
 **/
void dxl_master_putc(uint8_t c);

/**
 * @brief Reset DXL input parser
 *
 * Call this method if you want to reset the state of the internal parser used
 * for reading from the DXL bus.
 **/
void dxl_master_reset_parser();

/**
 * @brief Send a ping packet
 *
 * This method does not wait for the returned status packet. The
 * dxl_master_handleStatus() callback is called when the status packet is
 * received.
 **/
void dxl_master_ping(uint8_t id);

/**
 * @brief Send a write packet
 *
 * Sends a write request to servo @a id. Depending on the RETURN_LEVEL setting
 * of the servo, this might return a status packet! In that case, you have
 * to wait until dxl_master_handleStatus() is called or you are reasonably sure
 * a status packet won't arrive.
 *
 * @param id ID of the slave to address
 * @param addr Start register address
 * @param data Pointer to the data
 * @param size Size of the write
 **/
void dxl_master_write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t size);

/**
 * @brief Send a read packet
 *
 * Sends a read request to servo @a id. This will return a status packet with
 * the requested data. dxl_master_handleStatus() will be called when it is
 * received.
 *
 * @param id ID of the slave to address
 * @param addr Register start address
 * @param size Size of the read
 **/
void dxl_master_read(uint8_t id, uint8_t addr, uint8_t size);

//! @name Bulk reads
//@{

/**
 * @brief Calculate bulk read packet size
 *
 * @param servos Number of servos you want to address in this bulkread packet
 * @return Size of the bulk read packet in bytes
 **/
#define DXL_MASTER_BRPACKET_SIZE(servos) (7 + (3*servos))

/**
 * @brief Initialize bulk read packet
 *
 * @param packet Pointer to the packet buffer
 *      (of size at least DXL_MASTER_BRPACKET_SIZE())
 * @param num_servos Number of servos to address in the packet
 **/
void dxl_master_bulkread_init(uint8_t* packet, uint8_t num_servos);

/**
 * @brief Set part of bulkread packet
 *
 * @param packet Pointer to the packet buffer, initialized with
 *     dxl_master_bulkread_init()
 * @param idx Index into the bulkread packet (0 = first read, 1 = second read,
 *     ...)
 * @param id Servo ID
 * @param start Register start address
 * @param len Size of the read
 **/
void dxl_master_bulkread_set(uint8_t* packet, uint8_t idx, uint8_t id, uint8_t start, uint8_t len);

/**
 * @brief Finalize bulk read packet
 *
 * Sets the checksum of the packet correctly.
 *
 * @return packet length in bytes
 **/
uint8_t dxl_master_bulkread_finalize(uint8_t* packet);

//@}

//@{
void dxl_master_syncwrite_init(uint8_t* packet, uint8_t num_servos, uint8_t addr, uint8_t len);
void dxl_master_syncwrite_set(uint8_t* packet, uint8_t idx, uint8_t id, const uint8_t* data);
uint8_t dxl_master_syncwrite_finalize(uint8_t* packet);
//@}

//! @name API hooks
//@{

/**
 * @brief Send data over the bus
 *
 * Implement this so that it sends the data over the Dynamixel bus.
 *
 * @param data Pointer to the data buffer
 * @param len Size of the write
 **/
void dxl_master_send(const uint8_t* data, uint8_t len);

/**
 * @brief Handle a status packet
 *
 * Implement this so that it handles received status packets correctly.
 *
 * @param id ID of the servo sending the status packet
 * @param error Error byte (see Robotis documentation)
 * @param params Returned results from a read request
 * @param num_params Number of bytes in params
 **/
void dxl_master_handleStatus(uint8_t id, uint8_t error, const uint8_t* params, uint8_t num_params);

//@}

#ifdef __cplusplus
}
#endif

#endif
