// IMU driver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

void imu_init();

void imu_getData(int16_t* acc_x, int16_t* acc_y, int16_t* acc_z,
	int16_t* gyro_x, int16_t* gyro_y, int16_t* gyro_z);

uint8_t imu_isValid();

#endif
