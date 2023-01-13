/*
 * LSM6DS33_drivers.c
 *
 *  Created on: Jan 8, 2023
 *      Author: Nathaniel Mallick
 */

#include "LSM6DS33_drivers.h"

/**
 * @brief Initialization function for the LSM6DS33 IMU
 * */
void LSM6DS33_init(LSM6DS33_TypeDef *dtype)
{
	HAL_StatusTypeDef ret;
	char buf[2];
	// Read WHO AM I register
	ret = HAL_I2C_Mem_Read(dtype->i2c, LSM6DS33_ADDR << 1, 0x0F, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK || buf[0] != LSM6DS33_WHOAMI)
	{
		sprintf((char*)buf, "Error (whoami)\r\n");
		HAL_UART_Transmit(dtype->uart, buf, strlen((char*)buf), HAL_MAX_DELAY);
		exit(0);
	}

	// Setup the accelerometer conf registers
	buf[0] = 0x60;
	ret = HAL_I2C_Mem_Write(dtype->i2c, LSM6DS33_ADDR << 1, 0x10, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK)
	{
		strcpy((char*)buf, "Err in XL configuration\r\n");
		exit(0);
	}

	// Setup the gyroscope conf registers
	buf[0] = 0x62;
	ret = HAL_I2C_Mem_Write(dtype->i2c, LSM6DS33_ADDR << 1, 0x11, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY);

	if (ret != HAL_OK)
	{
		strcpy((char*)buf, "Err in G configuation\r\n");
		exit(0);
	}
}

/**
 * @brief Reads the raw accelerometer data from an LSM6DS33
 * @param Float values containing x,y,z accelerometer data
 * */
void LSM6DS33_I2C_acc_raw(LSM6DS33_TypeDef *dtype)
{
	// HAL status data type
	HAL_StatusTypeDef ret;
	float conv = 0.061/1000;

	// Array to hold 2 bytes of data
	char buf[2];

	// If we successfully read the register on the LSMD6S33
	// 	then we will continue the conversion into an integer/float
	//	if not, then we will keep the old value

	// Read x-axis from LSM6DS33 accelerometer
	ret = HAL_I2C_Mem_Read(dtype->i2c, LSM6DS33_ADDR << 1, ACC_X_L, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
	if (ret == HAL_OK)
		dtype->acc_data[0] = convert_buf(buf, conv);

	// Read y-axis from LSM6DS33 accelerometer
	ret = HAL_I2C_Mem_Read(dtype->i2c, LSM6DS33_ADDR << 1, ACC_Y_L, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
	if (ret == HAL_OK)
		dtype->acc_data[1] = convert_buf(buf, conv);

	// Read z-axis from LSM6DS33 accelerometer
	ret = HAL_I2C_Mem_Read(dtype->i2c, LSM6DS33_ADDR << 1, ACC_Z_L, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
	if (ret == HAL_OK)
		dtype->acc_data[2] = convert_buf(buf, conv);
}

void LSM6DS33_I2C_gyro_raw(LSM6DS33_TypeDef *dtype)
{
	// HAL status data type
	HAL_StatusTypeDef ret;

	// Array to hold 2 bytes of data
	char buf[2];

	// Conversion based on LSM6DS33 angular rate
	// 	sensitivity
	float conv = 4.375/1000;

	// If we successfully read the register on the LSMD6S33 IMU
	// 	then we will continue the conversion into an integer/float
	//	if not, then we will keep the old value

	// Read x-axis from LSM6DS33 gyroscope
	ret = HAL_I2C_Mem_Read(dtype->i2c, LSM6DS33_ADDR << 1, GYRO_X_L, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
	if (ret == HAL_OK)
		dtype->gyro_data[0] = convert_buf(buf, conv);

	// Read y-axis from LSM6DS33 gyroscope
	ret = HAL_I2C_Mem_Read(dtype->i2c, LSM6DS33_ADDR << 1, GYRO_Y_L, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
	if (ret == HAL_OK)
		dtype->gyro_data[1] = convert_buf(buf, conv);

	// Read z-axis from LSM6DS33 gyroscope
	ret = HAL_I2C_Mem_Read(dtype->i2c, LSM6DS33_ADDR << 1, GYRO_Z_L, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY);
	if (ret == HAL_OK)
		dtype->gyro_data[2] = convert_buf(buf, conv);
}

/**
 * @brief Converts a buffer of 2 bytes into one 32 bit floating point number
 * @param Buffer containing 2 bytes to convert
 * @note buf[0] = LSB, buf[1] = MSB
 * */
float convert_buf(char *buf, float conv)
{
	int16_t val;
	float acc;

	val = (buf[1] << 8) | buf[0];

	if ((val & 0x80) == 0)
	{
		val = (~(val - 0x01));
		acc = val * -conv;
	} else
	{
		acc = val*conv;
	}

	return acc;
}



