/*
 * mpu6050.c
 *
 *  Created on: 30 sie 2021
 *      Author: Mariusz
 */

#include "main.h";
#include "MPU6050.h";


void MPU6050_Init(MPU6050_t *mpu, I2C_HandleTypeDef *i2c, uint8_t Address)
{
	mpu->mpu_i2c=i2c;
	mpu->Address=(Address<<1);
	uint8_t Data;
	  // power management register 0X6B we should write all 0's to wake the sensor up
	        Data = 0;
	        HAL_I2C_Mem_Write(mpu->mpu_i2c,mpu->Address, PWR_MGMT_1, 1, &Data, 1, MPU_I2C_TIMEOUT);

	        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	        Data = 0x07;
	        HAL_I2C_Mem_Write(mpu->mpu_i2c,mpu->Address, SMPLRT_DIV, 1, &Data, 1, MPU_I2C_TIMEOUT);

	        // Set accelerometer configuration in ACCEL_CONFIG Register
	        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
	        Data = 0x00;
	        HAL_I2C_Mem_Write(mpu->mpu_i2c,mpu->Address,ACCEL_CONFIG, 1, &Data, 1, MPU_I2C_TIMEOUT);

	        // Set Gyroscopic configuration in GYRO_CONFIG Register
	        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
	        Data = 0x00;
	        HAL_I2C_Mem_Write(mpu->mpu_i2c,mpu->Address,GYRO_CONFIG, 1, &Data, 1, MPU_I2C_TIMEOUT);
}




void MPU6050_Read_Values(MPU6050_t *mpu){


uint8_t Temp[16];
if (mpu->mpu_i2c->State==HAL_I2C_STATE_READY){
	extern uint32_t  time4,time5;
	 	 	 time4=HAL_GetTick()-time5;
	         time5=HAL_GetTick();

if (HAL_I2C_Mem_Read(mpu->mpu_i2c, mpu->Address, 0x3B,1,Temp,14,MPU_I2C_TIMEOUT)==0)
{
mpu->Accel_raw_X=(int16_t)(Temp[0]<<8|Temp[1]);
mpu->Accel_raw_Y=(int16_t)(Temp[2]<<8|Temp[3]);
mpu->Accel_raw_Z=(int16_t)(Temp[4]<<8|Temp[5]);
mpu->Gyro_raw_X=(int16_t)(Temp[8]<<8|Temp[9]);
mpu->Gyro_raw_Y=(int16_t)(Temp[10]<<8|Temp[11]);
mpu->Gyro_raw_Z=(int16_t)(Temp[12]<<8|Temp[13]);
mpu->Temp=(int16_t)(Temp[6]<<8|Temp[7]);
}
else
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
}
}

}
