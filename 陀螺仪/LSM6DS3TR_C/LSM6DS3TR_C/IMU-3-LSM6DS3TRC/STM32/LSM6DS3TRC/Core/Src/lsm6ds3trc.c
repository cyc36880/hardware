/*
 * @author: rtrobot<admin@rtrobot.org>
 * @website:rtrobot.org
 * @licence: GPL v3
 */

#include "lsm6ds3trc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f1xx_hal.h>

//0:SPI
//1:I2C
uint8_t lsm6ds3trc_mode = 0;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;

/***************************************************************************************************************
LSM6DS3TRC Read Command
****************************************************************************************************************/
void LSM6DS3TRC_ReadCommand(uint8_t reg_addr, uint8_t *rev_data, uint8_t length)
{
	if (lsm6ds3trc_mode == LSM6DS3TRC_MODE_I2C)
	{
		HAL_I2C_Mem_Read(&hi2c1, LSM6DS3TRC_I2CADDR << 0x01, reg_addr, 1, rev_data, length, 100);
	}
	else
	{
		uint8_t reg_spi[1] = {reg_addr | 0x80};
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, reg_spi, 1, 100);
		HAL_SPI_Receive(&hspi1, rev_data, length, 100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
}

/***************************************************************************************************************
LSM6DS3TRC Write Command
****************************************************************************************************************/
void LSM6DS3TRC_WriteCommand(uint8_t reg_addr, uint8_t *send_data, uint16_t length)
{
	if (lsm6ds3trc_mode == LSM6DS3TRC_MODE_I2C)
		HAL_I2C_Mem_Write(&hi2c1, LSM6DS3TRC_I2CADDR << 0x01, reg_addr, 1, send_data, length, 100);
	else
	{
		uint8_t reg_spi[1] = {reg_addr & 0x7f};
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX)
			;
		HAL_SPI_Transmit(&hspi1, reg_spi, 1, 100);
		HAL_SPI_Transmit(&hspi1, send_data, length, 100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	}
}

/***************************************************************************************************************
LSM6DS3TRC Get id
****************************************************************************************************************/
bool LSM6DS3TRC_GetChipID(void)
{
	uint8_t buf = 0;

	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_WHO_AM_I, &buf, 1);
	if (buf == 0x6a)
		return true;
	else
		return false;
}

/***************************************************************************************************************
LSM6DS3TRC reboot and reset register
****************************************************************************************************************/
void LSM6DS3TRC_Reset(void)
{
	uint8_t buf[1] = {0};
	//reboot modules
	buf[0] = 0x80;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	HAL_Delay(15);

	//reset register
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	buf[0] |= 0x01;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	while (buf[0] & 0x01)
		LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Get data status
****************************************************************************************************************/
uint8_t LSM6DS3TRC_Get_Status(void)
{
	uint8_t buf[1] = {0};
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_STATUS_REG, buf, 1);
	return buf[0];
}

/***************************************************************************************************************
LSM6DS3TRC Set Block Data Update
****************************************************************************************************************/
void LSM6DS3TRC_Set_BDU(bool flag)
{
	uint8_t buf[1] = {0};
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL3_C, buf, 1);

	if (flag == true)
	{
		buf[0] |= 0x40;
		LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	}
	else
	{
		buf[0] &= 0xbf;
		LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	}

	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set accelerometer data rate
****************************************************************************************************************/
void LSM6DS3TRC_Set_Accelerometer_Rate(uint8_t rate)
{
	uint8_t buf[1] = {0};
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
	buf[0] |= rate;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set gyroscope data rate
****************************************************************************************************************/
void LSM6DS3TRC_Set_Gyroscope_Rate(uint8_t rate)
{
	uint8_t buf[1] = {0};
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL2_G, buf, 1);
	buf[0] |= rate;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL2_G, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set accelerometer full-scale selection.
****************************************************************************************************************/
void LSM6DS3TRC_Set_Accelerometer_Fullscale(uint8_t value)
{
	uint8_t buf[1] = {0};
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
	buf[0] |= value;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set gyroscope full-scale selection.
****************************************************************************************************************/
void LSM6DS3TRC_Set_Gyroscope_Fullscale(uint8_t value)
{
	uint8_t buf[1] = {0};
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL2_G, buf, 1);
	buf[0] |= value;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL2_G, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set accelerometer analog chain bandwidth.
****************************************************************************************************************/
void LSM6DS3TRC_Set_Accelerometer_Bandwidth(uint8_t BW0XL, uint8_t ODR)
{
	uint8_t buf[1] = {0};
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
	buf[0] |= BW0XL;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);

	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL8_XL, buf, 1);
	buf[0] |= ODR;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL8_XL, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set register 6
****************************************************************************************************************/
void LSM6DS3TRC_Set_Register4(uint8_t reg4)
{
	uint8_t buf[1] = {0};

	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL4_C, buf, 1);
	buf[0] |= reg4;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL4_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set register 5
****************************************************************************************************************/
void LSM6DS3TRC_Set_Register5(uint8_t reg5)
{
	uint8_t buf[1] = {0};

	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL5_C, buf, 1);
	buf[0] |= reg5;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL5_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set register 6
****************************************************************************************************************/
void LSM6DS3TRC_Set_Register6(uint8_t reg6)
{
	uint8_t buf[1] = {0};
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL6_C, buf, 1);
	buf[0] |= reg6;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL6_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set register 7
****************************************************************************************************************/
void LSM6DS3TRC_Set_Register7(uint8_t reg7)
{
	uint8_t buf[1] = {0};

	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_CTRL7_G, buf, 1);
	buf[0] |= reg7;
	LSM6DS3TRC_WriteCommand(LSM6DS3TRC_CTRL7_G, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Get Acceleration Value
****************************************************************************************************************/
void LSM6DS3TRC_Get_Acceleration(uint8_t fsxl, float *acc_float)
{
	uint8_t buf[6];
	int16_t acc[3];
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_OUTX_L_XL, buf, 6);
	acc[0] = buf[1] << 8 | buf[0];
	acc[1] = buf[3] << 8 | buf[2];
	acc[2] = buf[5] << 8 | buf[4];

	switch (fsxl)
	{
	case LSM6DS3TRC_ACC_FSXL_2G:
		acc_float[0] = ((float)acc[0] * 0.061f);
		acc_float[1] = ((float)acc[1] * 0.061f);
		acc_float[2] = ((float)acc[2] * 0.061f);
		break;

	case LSM6DS3TRC_ACC_FSXL_16G:
		acc_float[0] = ((float)acc[0] * 0.488f);
		acc_float[1] = ((float)acc[1] * 0.488f);
		acc_float[2] = ((float)acc[2] * 0.488f);
		break;

	case LSM6DS3TRC_ACC_FSXL_4G:
		acc_float[0] = ((float)acc[0] * 0.122f);
		acc_float[1] = ((float)acc[1] * 0.122f);
		acc_float[2] = ((float)acc[2] * 0.122f);
		break;

	case LSM6DS3TRC_ACC_FSXL_8G:
		acc_float[0] = ((float)acc[0] * 0.244f);
		acc_float[1] = ((float)acc[1] * 0.244f);
		acc_float[2] = ((float)acc[2] * 0.244f);
		break;
	}
}

/***************************************************************************************************************
LSM6DS3TRC Get Gyroscope Value
****************************************************************************************************************/
void LSM6DS3TRC_Get_Gyroscope(uint8_t fsg, float *gry_float)
{
	uint8_t buf[6];
	int16_t gry[3];
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_OUTX_L_G, buf, 6);
	gry[0] = buf[1] << 8 | buf[0];
	gry[1] = buf[3] << 8 | buf[2];
	gry[2] = buf[5] << 8 | buf[4];
	switch (fsg)
	{
	case LSM6DS3TRC_GYR_FSG_245:
		gry_float[0] = ((float)gry[0] * 8.750f);
		gry_float[1] = ((float)gry[1] * 8.750f);
		gry_float[2] = ((float)gry[2] * 8.750f);
		break;
	case LSM6DS3TRC_GYR_FSG_500:
		gry_float[0] = ((float)gry[0] * 17.50f);
		gry_float[1] = ((float)gry[1] * 17.50f);
		gry_float[2] = ((float)gry[2] * 17.50f);
		break;
	case LSM6DS3TRC_GYR_FSG_1000:
		gry_float[0] = ((float)gry[0] * 35.00f);
		gry_float[1] = ((float)gry[1] * 35.00f);
		gry_float[2] = ((float)gry[2] * 35.00f);
		break;
	case LSM6DS3TRC_GYR_FSG_2000:
		gry_float[0] = ((float)gry[0] * 70.00f);
		gry_float[1] = ((float)gry[1] * 70.00f);
		gry_float[2] = ((float)gry[2] * 70.00f);
		break;
	}
}

float LSM6DS3TRC_Get_Temperature(void)
{
	uint8_t buf[2];
	int16_t temp;
	LSM6DS3TRC_ReadCommand(LSM6DS3TRC_OUT_TEMP_L, buf, 2);
	temp = buf[1] << 8 | buf[0];
	return (((float)temp / 256.0) + 25.0);
}

/***************************************************************************************************************
LSM6DS3TRC Init
****************************************************************************************************************/
bool LSM6DS3TRC_Init(uint8_t mode)
{
	lsm6ds3trc_mode = mode;

	if (LSM6DS3TRC_GetChipID() == false)
		return false;

	//reboot and reset register
	LSM6DS3TRC_Reset();

	//enable Block Data Update
	LSM6DS3TRC_Set_BDU(true);

	//Set Data Rate
	LSM6DS3TRC_Set_Accelerometer_Rate(LSM6DS3TRC_ACC_RATE_12HZ5);
	LSM6DS3TRC_Set_Gyroscope_Rate(LSM6DS3TRC_GYR_RATE_12HZ5);

	//Set full-scale selection.
	LSM6DS3TRC_Set_Accelerometer_Fullscale(LSM6DS3TRC_ACC_FSXL_2G);
	LSM6DS3TRC_Set_Gyroscope_Fullscale(LSM6DS3TRC_GYR_FSG_2000);

	//set accelerometer analog chain bandwidth.
	LSM6DS3TRC_Set_Accelerometer_Bandwidth(LSM6DS3TRC_ACC_BW0XL_400HZ, LSM6DS3TRC_ACC_LOW_PASS_ODR_100);

	LSM6DS3TRC_Set_Register7(LSM6DS3TRC_CTRL7_G_HM_MODE_DISABLE | LSM6DS3TRC_CTRL7_G_HPM_260MHZ);
	LSM6DS3TRC_Set_Register6(LSM6DS3TRC_CTRL6_C_FTYPE_1);
	LSM6DS3TRC_Set_Register4(LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE);

	if (LSM6DS3TRC_GetChipID() == false)
		return false;
	return true;
}
