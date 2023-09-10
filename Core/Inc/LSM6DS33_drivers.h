/*
 * LSM6DS33_drivers.h
 *
 *  Created on: Jan 8, 2023
 *      Author: nate
 */

#ifndef LSM6DS33_DRIVERS_H_
#define LSM6DS33_DRIVERS_H_

#include "stm32f4xx_hal.h"
#include <math.h>

// LSM6DS33
#define LSM6DS33_WHOAMI 0x69
#define LSM6DS33_ADDR 0x6A

// Configuration registers
#define CTRL1_XL 0x10
#define CTRL2_G 0x11

// Accelerometer data registers
#define ACC_X_L 0x28
#define ACC_X_H 0x29
#define ACC_Y_L 0x2A
#define ACC_Y_H 0x2B
#define ACC_Z_L 0x2C
#define ACC_Z_H 0x2D

// Gyroscope data registers
#define GYRO_X_L 0x22
#define GYRO_X_H 0x23
#define GYRO_Y_L 0x24
#define GYRO_Y_H 0x25
#define GYRO_Z_L 0x26
#define GYRO_Z_H 0x27

//// STRUCTURES //
typedef struct {
	float acc_data[3];
	float gyro_data[3];
	float orientation[3];

	float dt, alpha;

	I2C_HandleTypeDef *i2c;
	UART_HandleTypeDef *uart;

} LSM6DS33_TypeDef;

//
//// FUNCTION HEADERS //
void LSM6DS33_init();
void LSM6DS33_I2C_acc_raw(LSM6DS33_TypeDef *);
void LSM6DS33_I2C_gyro_raw(LSM6DS33_TypeDef *);
float convert_buf(char*, float);
void updateOrientation(LSM6DS33_TypeDef*);

#endif /* INC_LSM6DS33_DRIVERS_H_ */
