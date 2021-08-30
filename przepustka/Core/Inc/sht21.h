/*
 * sht21.h
 *
 *  Created on: Aug 18, 2021
 *      Author: Mariusz
 */

#ifndef INC_SHT21_H_
#define INC_SHT21_H_


//commands
#define SHT_I2C_TIMEOUT	150
#define SHT21_TRIGGER_T_HOLD    0b11100011		//pulls down SCL during measurement
#define SHT21_TRIGGER_RH_HOLD   0b11100101		//pulls down SCL during measurement
#define SHT21_TRIGGER_T_NOHOLD  0b11110011
#define SHT21_TRIGGER_RH_NOHOLD 0b11110101
#define SHT21_WRITE_REGISTER    0b11100110
#define SHT21_READ_REGISTER		0b11100111
#define SHT21_SOFT_RESET		0b11111110

#define SHT_LAST_MEAS_RH		2
#define SHT_LAST_MEAS_T			1

//config register
#define SHT21_DIAG_HEATER 			(1<<2)
#define SHT21_LOW_BATT				(1<<6)
#define SHT21_DISABLE_OTP_RELOAD 	(1<<1)
#define SHT21_RESOLUTION			(1<<7)|(1<<0)
#define	SHT21_RES_RH12_BIT_T_14BIT	0b00000000
#define SHT21_RES_RH8_BIT_T_12BIT	0b00000001
#define SHT21_RES_RH10_BIT_T_13BIT	0b10000000
#define SHT21_RES_RH11_BIT_T_11BIT	0b10000001

typedef struct
{
	I2C_HandleTypeDef 			*sht21_i2c;
	uint8_t						Address;
	uint64_t					Serial;
	uint8_t						Resolution;
	uint8_t						Register;
	uint8_t						Last_measurement; 	//1 - temperature ; 2 - relative humidity
	uint16_t 					temperature_raw;
	uint16_t 					rh_raw;
	float						temperature;
	float						humidity;



}SHT21_t;

void SHT21_Init(SHT21_t *sht, I2C_HandleTypeDef *i2c, uint8_t Address);
void SHT21_Write_Command(SHT21_t *sht, uint8_t Command);
void SHT21_Measure_T(SHT21_t *sht, uint8_t Hold);
void SHT21_Measure_RH(SHT21_t *sht, uint8_t Hold);
void SHT21_Read_Raw_Value(SHT21_t *sht);
uint8_t SHT21_Update_calculated_value(SHT21_t *sht);
uint64_t SHT21_Read_Serial(SHT21_t *sht);
uint8_t SHT21_Read_Register(SHT21_t *sht);
void SHT21_Write_Register(SHT21_t *sht, uint8_t Register);
void	SHT21_Set_Resolution(SHT21_t *sht, uint8_t Resolution);
uint8_t SHT21_Check_Resolution(SHT21_t *sht);
void 	SHT21_Enable_Heater(SHT21_t *sht);
void	SHT21_Disable_Heater(SHT21_t *sht);
uint8_t	SHT21_Check_battery(SHT21_t *sht);

#endif /* INC_SHT21_H_ */
