/*
 * lm35.c
 *
 */
//#include <stm32f10x_rcc.h>
//#include <stm32f10x_gpio.h>
//#include <stm32f10x_i2c.h>
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "lm75.h"


// Read 16-bit LM75 register
#define LM75_ADDRESS (0x48*2)
extern I2C_HandleTypeDef hi2c1;
uint8_t LM75_Init(uint32_t SPI_Clock_Speed)
{
	return 1;
}
uint16_t LM75_ReadReg(uint8_t reg)
{
	uint8_t bb[4];
	uint16_t value;
	bb[0] = reg;
	int ret = HAL_I2C_Master_Transmit(&hi2c1,LM75_ADDRESS,bb,1,1000);
	if ( ret != HAL_OK )
	{
	  		printf("error LM75_ReadReg\n");
	  		return 0;
	}
	HAL_Delay(100);
	ret = HAL_I2C_Master_Receive(&hi2c1,LM75_ADDRESS,bb,2,1000);
	if ( ret != HAL_OK )
	{
		printf("LM75_ReadReg ret = %d\n",ret);
	}
	else
	{
		for(int k=0;k<2;k++)
		{
		//	printf(" x%02X",bb[k]);
		}
		//printf("\n");
	}
	value = bb[0]*256+bb[1];
	return value;
}
// Read value from LM75 configuration register (8 bit)
uint8_t LM75_ReadConf()
{
	uint8_t bb[4];
	uint8_t value;
	bb[0] = LM75_REG_CONF;
	int ret = HAL_I2C_Master_Transmit(&hi2c1,LM75_ADDRESS,bb,1,1000);
	if ( ret != HAL_OK )
	{
	  		printf("error LM75_ReadReg\n");
	  		return 0;
	}
	HAL_Delay(100);
	ret = HAL_I2C_Master_Receive(&hi2c1,LM75_ADDRESS,bb,1,1000);
	if ( ret != HAL_OK )
	{
		printf("LM75_ReadReg ret = %d\n",ret);
	}
	else
	{
		for(int k=0;k<1;k++)
		{
			//printf(" x%02X",bb[k]);
		}
		//printf("\n");
	}
	value = bb[0];
	return value;
}
void LM75_WriteReg(uint8_t reg, uint16_t value)
{
	uint8_t bb[4];
	bb[0] = reg;
	bb[1] = value>>8;
	bb[2] = value&0xff;

	int ret = HAL_I2C_Master_Transmit(&hi2c1,LM75_ADDRESS,bb,3,1000);
	if ( ret != HAL_OK )
	{
		printf("LM75_WriteReg ret = %d\n",ret);
	}
}
// Write value to LM75 configuration register  (8 bit)
void LM75_WriteConf(uint8_t value)
{
	uint8_t bb[4];
	bb[0] = LM75_REG_CONF;
	bb[1] = value;
	int ret = HAL_I2C_Master_Transmit(&hi2c1,LM75_ADDRESS,bb,2,1000);
	if ( ret != HAL_OK )
	{
		printf("LM75_WriteConf ret = %d\n",ret);
	}
}
// Read temperature readings from LM75 in decimal format
// IIIF where:
//   III - integer part
//   F   - fractional part
// e.g. 355 means 35.5C
int16_t LM75_Temperature(void) {
	uint16_t raw;
	int16_t temp;

	raw = LM75_ReadReg(LM75_REG_TEMP) >> 7;
	if (raw & 0x0100) {
		// Negative temperature
		temp = -10 * (((~(uint8_t)(raw & 0xFE) + 1) & 0x7F) >> 1) - (raw & 0x01) * 5;
	} else {
		// Positive temperature
		temp = ((raw & 0xFE) >> 1) * 10 + (raw & 0x01) * 5;
	}

	return temp;
}
// Set LM75 shutdown mode
// newstate:
//    ENABLE = put LM75 into powerdown mode
//    DISABLE = wake up LM75
void LM75_Shutdown(FunctionalState newstate) {
	uint8_t value;

	value = LM75_ReadConf();
	LM75_WriteConf(newstate == ENABLE ? value | 0x01 : value & 0xFE);
}


