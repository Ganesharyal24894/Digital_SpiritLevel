/*
 * IMU_6050.h
 *
 *  Created on: June 25, 2024
 *      Author: Ganesh
 */

#ifndef INC_IMU_6050_H_
#define INC_IMU_6050_H_

#include "main.h"

#define IMU_ADD 0xD0
#define IMU_FLTR_REG 0x1A
#define IMU_PWRMGMT_REG 0x6B
#define IMU_ACCELCONFIG_REG 0x1C
#define IMU_GYROCONFIG_REG 0x1B
#define IMU_ACCEL_XOUTH_REG 0x3B

//custom data type for MPU_6050
typedef struct {

	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	double Ax;
	double Ay;
	double Az;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	double Gx;
	double Gy;
	double Gz;

	float Temperature;

} MPU6050_t;



void IMU_Setup(I2C_HandleTypeDef *i2c);
void IMU_ReadAll(I2C_HandleTypeDef *i2c, MPU6050_t *IMU);
void IMU_Calibrate(I2C_HandleTypeDef *i2c, MPU6050_t *IMU);

#endif /* INC_IMU_6050_H_ */
