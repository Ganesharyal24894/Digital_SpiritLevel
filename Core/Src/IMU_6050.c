/*
 * IMU_6050.c
 *
 *  Created on: June 25, 2024
 *      Author: Ganesh
 */

#include "IMU_6050.h"
#include "main.h"



void IMU_Setup(I2C_HandleTypeDef *i2c) {

	uint8_t data = 0x00; // waking up from sleep mode
	HAL_I2C_Mem_Write(i2c, IMU_ADD, IMU_PWRMGMT_REG, 1, &data, 1, 1000);
	data = 0x05; // enabling low pass filter with 10hz cut off frequency
	HAL_I2C_Mem_Write(i2c, IMU_ADD, IMU_FLTR_REG, 1, &data, 1, 1000);
	data = 0x00; // selecting +-2g configuration
	HAL_I2C_Mem_Write(i2c, IMU_ADD, IMU_ACCELCONFIG_REG, 1, &data, 1, 1000);
	data = 0x00; // selecting 250deg/s configuration
	HAL_I2C_Mem_Write(i2c, IMU_ADD, IMU_GYROCONFIG_REG, 1, &data, 1, 1000);
}

void IMU_ReadAll(I2C_HandleTypeDef *i2c, MPU6050_t *IMU) {
	uint8_t Rx_data[14];
	int16_t temp;

	// Read 14 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read(i2c, IMU_ADD, IMU_ACCEL_XOUTH_REG, 1, Rx_data, 14,
	HAL_MAX_DELAY);

	IMU->Accel_X_RAW = (int16_t) (Rx_data[0] << 8 | Rx_data[1]);
	IMU->Accel_Y_RAW = (int16_t) (Rx_data[2] << 8 | Rx_data[3]);
	IMU->Accel_Z_RAW = (int16_t) (Rx_data[4] << 8 | Rx_data[5]);
	temp = (int16_t) (Rx_data[6] << 8 | Rx_data[7]);
	IMU->Gyro_X_RAW = (int16_t) (Rx_data[8] << 8 | Rx_data[9]);
	IMU->Gyro_Y_RAW = (int16_t) (Rx_data[10] << 8 | Rx_data[11]);
	IMU->Gyro_Z_RAW = (int16_t) (Rx_data[12] << 8 | Rx_data[13]);

	IMU->Ax = IMU->Accel_X_RAW / 16384.0; // values in Gs
	IMU->Ay = IMU->Accel_Y_RAW / 16384.0;
	IMU->Az = IMU->Accel_Z_RAW / 16384.0;
	IMU->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
	IMU->Gx = IMU->Gyro_X_RAW / 131.0; // values in degree per second
	IMU->Gy = IMU->Gyro_Y_RAW / 131.0;
	IMU->Gz = IMU->Gyro_Z_RAW / 131.0;
}



