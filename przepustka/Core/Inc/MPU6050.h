/*
 * MPU6050.h
 *
 *  Created on: 30 sie 2021
 *      Author: Mariusz
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU_I2C_TIMEOUT 200


#define SELF_TEST_X		0x0D
#define SELF_TEST_Y		0x0E
#define SELF_TEST_Z		0x0F
#define SELF_TEST_A		0x10
#define SMPLRT_DIV		0x19
#define GYRO_CONFIG 	0x1B
#define ACCEL_CONFIG	0x1C
#define INT_PIN_CFG		0x37
#define	INT_ENABLE		0x38
#define	INT_STATUS		0x3A
#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40
#define TEMP_OUT_H		0x41
#define TEMP_OUT_L		0x42
#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48
#define PWR_MGMT_1		0x6B
#define WHO_AM_I		0X75  // value always 109 decimal (0x68 hex)


typedef struct
{
	I2C_HandleTypeDef 			*mpu_i2c;
	uint8_t						Address;
	int16_t					Accel_raw_X;
	int16_t					Accel_raw_Y;
	int16_t					Accel_raw_Z;
	int16_t					Gyro_raw_X;
	int16_t					Gyro_raw_Y;
	int16_t					Gyro_raw_Z;
	int16_t					Temp;




}MPU6050_t;


void MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *i2c, uint8_t Address);
void MPU6050_Read_Values(MPU6050_t *MPU6050);







#endif /* INC_MPU6050_H_ */
