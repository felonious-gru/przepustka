/*
 * sht21.c
 *
 *  Created on: Aug 18, 2021
 *      Author: Mariusz
 */


#include "main.h";
#include "sht21.h";


uint8_t Read8(SHT21_t *sht, uint8_t Register)
{
	uint8_t Value=0;
	HAL_I2C_Mem_Read(sht->sht21_i2c, sht->Address, Register,1,&Value,1,SHT_I2C_TIMEOUT);
	return Value;
}

//
//
//
//
//


void SHT21_Init(SHT21_t *sht, I2C_HandleTypeDef *i2c, uint8_t Address)
{
	sht->sht21_i2c = i2c;
	sht->Address = Address<<1;
	//sht->Serial = SHT21_Read_Serial(sht);
}

uint64_t SHT21_Read_Serial(SHT21_t *sht)
{
	uint64_t Serial=0;
	uint8_t Temp[8];
	HAL_I2C_Mem_Read(sht->sht21_i2c, sht->Address, 0xFA0F,2,Temp,8,SHT_I2C_TIMEOUT);
	// add serial decoding here
	HAL_I2C_Mem_Read(sht->sht21_i2c, sht->Address, 0xFCC9,1,Temp,6,SHT_I2C_TIMEOUT);
	// add serial decoding here
	return Serial;
}


void SHT21_Write_Command(SHT21_t *sht, uint8_t Command)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,0);
	HAL_I2C_Master_Transmit(sht->sht21_i2c,sht->Address,&Command,1,SHT_I2C_TIMEOUT);
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
}


void SHT21_Measure_T(SHT21_t *sht, uint8_t Hold)
{

 SHT21_Write_Command(sht,Hold?  SHT21_TRIGGER_T_HOLD : SHT21_TRIGGER_T_NOHOLD);
 sht->Last_measurement=SHT_LAST_MEAS_T;
}


 void SHT21_Measure_RH(SHT21_t *sht, uint8_t Hold)
 {
	 SHT21_Write_Command(sht,Hold? SHT21_TRIGGER_RH_HOLD: SHT21_TRIGGER_RH_NOHOLD);
	 sht->Last_measurement=SHT_LAST_MEAS_RH;
 }

 void SHT21_Read_Raw_Value(SHT21_t *sht)
 {
	 uint8_t Value[2]={0};
	 if (HAL_I2C_Master_Receive(sht->sht21_i2c,sht->Address,Value,2,SHT_I2C_TIMEOUT)==0)
		 switch (sht->Last_measurement)
		 {
		 case SHT_LAST_MEAS_T :
		 {
			 sht->temperature_raw=Value[0]<<8|Value[1];
			 break;
		 }
		 case SHT_LAST_MEAS_RH :
		 {
			 sht->rh_raw=Value[0]<<8|Value[1];
			 break;
		 }
		 default: break;
		 }
 }


uint8_t SHT21_Update_calculated_value(SHT21_t *sht);


uint8_t SHT21_Read_Register(SHT21_t *sht)
{	uint8_t Register;
	HAL_I2C_Mem_Read(sht->sht21_i2c,sht->Address,SHT21_READ_REGISTER,1,&Register,1,SHT_I2C_TIMEOUT);
	return Register;
}


void SHT21_Write_Register(SHT21_t *sht, uint8_t Register)
{
	HAL_I2C_Mem_Write(sht->sht21_i2c,sht->Address,SHT21_WRITE_REGISTER,1,&Register,1,SHT_I2C_TIMEOUT);
}


void	SHT21_Set_Resolution(SHT21_t *sht, uint8_t Resolution);
uint8_t SHT21_Check_Resolution(SHT21_t *sht);

void 	SHT21_Enable_Heater(SHT21_t *sht)
{ uint8_t Register;
	Register=SHT21_Read_Register(sht);
	Register|=SHT21_DIAG_HEATER;
	SHT21_Write_Register(sht,Register);
}


void	SHT21_Disable_Heater(SHT21_t *sht)
{
	uint8_t Register;
		Register=SHT21_Read_Register(sht);
		Register&=!(SHT21_DIAG_HEATER);
		SHT21_Write_Register(sht,Register);
}

uint8_t	SHT21_Check_battery(SHT21_t *sht)
{	uint8_t Temp;
	Temp=SHT21_Read_Register(sht);
	Temp=(Temp&SHT21_LOW_BATT)>0 ? 1 : 0;
	return Temp;
}

