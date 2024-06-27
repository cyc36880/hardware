#include <Wire.h>
#include <SPI.h>
#include "LSM6DS3TRC.h"
//http://rtrobot.org

LSM6DS3TRC::LSM6DS3TRC()
{
}

boolean LSM6DS3TRC::begin()
{
	if (LSM6DS3TRC_MODE == LSM6DS3TRC_MODE_I2C)
	{
		Wire.begin();
		_i2caddr = LSM6DS3TRC_ADDRESS;
		Wire.begin();
	}
	else if (LSM6DS3TRC_MODE == LSM6DS3TRC_MODE_SPI)
	{
		pinMode(Pin_CS, OUTPUT);
		digitalWrite(Pin_CS, HIGH);
		SPI.begin();
	}

	if (GetChipID() == false)
		return false;

	//reboot and reset register
	Reset();

	//enable Block Data Update
	Set_BDU(true);

	//Set Data Rate
	Set_Accelerometer_Rate(LSM6DS3TRC_ACC_RATE_12HZ5);
	Set_Gyroscope_Rate(LSM6DS3TRC_GYR_RATE_12HZ5);

	//Set full-scale selection.
	Set_Accelerometer_Fullscale(LSM6DS3TRC_ACC_FSXL_2G);
	Set_Gyroscope_Fullscale(LSM6DS3TRC_GYR_FSG_2000);

	//set accelerometer analog chain bandwidth.
	Set_Accelerometer_Bandwidth(LSM6DS3TRC_ACC_BW0XL_400HZ, LSM6DS3TRC_ACC_LOW_PASS_ODR_100);

	Set_Register7(LSM6DS3TRC_CTRL7_G_HM_MODE_DISABLE | LSM6DS3TRC_CTRL7_G_HPM_260MHZ);
	Set_Register6(LSM6DS3TRC_CTRL6_C_FTYPE_1);
	Set_Register4(LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE);

	return true;
}

/***************************************************************************************************************
LSM6DS3TRC Get Acceleration Value
****************************************************************************************************************/
void LSM6DS3TRC::Get_Acceleration(uint8_t fsxl, float *acc_float)
{
	uint8_t buf[6];
	int16_t acc[3];
	ReadCommand(LSM6DS3TRC_OUTX_L_XL, buf, 6);
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
void LSM6DS3TRC::Get_Gyroscope(uint8_t fsg, float *gry_float)
{
	uint8_t buf[6];
	int16_t gry[3];
	ReadCommand(LSM6DS3TRC_OUTX_L_G, buf, 6);
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

float LSM6DS3TRC::Get_Temperature(void)
{
	uint8_t buf[2];
	int16_t temp;
	ReadCommand(LSM6DS3TRC_OUT_TEMP_L, buf, 2);
	temp = buf[1] << 8 | buf[0];
	return (((float)temp / 256.0) + 25.0);
}

/***************************************************************************************************************
LSM6DS3TRC Get data status
****************************************************************************************************************/
uint8_t LSM6DS3TRC::Get_Status(void)
{
	uint8_t buf[1] = {0};
	ReadCommand(LSM6DS3TRC_STATUS_REG, buf, 1);
	return buf[0];
}

/***************************************************************************************************************
LSM6DS3TRC reboot and reset register
****************************************************************************************************************/
void LSM6DS3TRC::Reset(void)
{
	uint8_t buf[1] = {0};
	//reboot modules
	buf[0] = 0x80;
	WriteCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	delay(15);

	//reset register
	ReadCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	buf[0] |= 0x01;
	WriteCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	while (buf[0] & 0x01)
		ReadCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set accelerometer data rate
****************************************************************************************************************/
void LSM6DS3TRC::Set_Accelerometer_Rate(uint8_t rate)
{
	uint8_t buf[1] = {0};
	ReadCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
	buf[0] |= rate;
	WriteCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set gyroscope data rate
****************************************************************************************************************/
void LSM6DS3TRC::Set_Gyroscope_Rate(uint8_t rate)
{
	uint8_t buf[1] = {0};
	ReadCommand(LSM6DS3TRC_CTRL2_G, buf, 1);
	buf[0] |= rate;
	WriteCommand(LSM6DS3TRC_CTRL2_G, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set accelerometer full-scale selection.
****************************************************************************************************************/
void LSM6DS3TRC::Set_Accelerometer_Fullscale(uint8_t value)
{
	uint8_t buf[1] = {0};
	ReadCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
	buf[0] |= value;
	WriteCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set gyroscope full-scale selection.
****************************************************************************************************************/
void LSM6DS3TRC::Set_Gyroscope_Fullscale(uint8_t value)
{
	uint8_t buf[1] = {0};
	ReadCommand(LSM6DS3TRC_CTRL2_G, buf, 1);
	buf[0] |= value;
	WriteCommand(LSM6DS3TRC_CTRL2_G, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set Block Data Update
****************************************************************************************************************/
void LSM6DS3TRC::Set_BDU(boolean flag)
{
	uint8_t buf[1] = {0};
	ReadCommand(LSM6DS3TRC_CTRL3_C, buf, 1);

	if (flag == true)
	{
		buf[0] |= 0x40;
		WriteCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	}
	else
	{
		buf[0] &= 0xbf;
		WriteCommand(LSM6DS3TRC_CTRL3_C, buf, 1);
	}
}

/***************************************************************************************************************
LSM6DS3TRC Get id
****************************************************************************************************************/
boolean LSM6DS3TRC::GetChipID(void)
{
	uint8_t buf = 0;

	ReadCommand(LSM6DS3TRC_WHO_AM_I, &buf, 1);

	if (buf == 0x6a)
		return true;
	else
		return false;
}

/***************************************************************************************************************
LSM6DS3TRC Set accelerometer analog chain bandwidth.
****************************************************************************************************************/
void LSM6DS3TRC::Set_Accelerometer_Bandwidth(uint8_t BW0XL, uint8_t ODR)
{
	uint8_t buf[1] = {0};
	ReadCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);
	buf[0] |= BW0XL;
	WriteCommand(LSM6DS3TRC_CTRL1_XL, buf, 1);

	ReadCommand(LSM6DS3TRC_CTRL8_XL, buf, 1);
	buf[0] |= ODR;
	WriteCommand(LSM6DS3TRC_CTRL8_XL, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set register 4
****************************************************************************************************************/
void LSM6DS3TRC::Set_Register4(uint8_t reg4)
{
	uint8_t buf[1] = {0};

	ReadCommand(LSM6DS3TRC_CTRL4_C, buf, 1);
	buf[0] |= reg4;
	WriteCommand(LSM6DS3TRC_CTRL4_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set register 5
****************************************************************************************************************/
void LSM6DS3TRC::Set_Register5(uint8_t reg5)
{
	uint8_t buf[1] = {0};

	ReadCommand(LSM6DS3TRC_CTRL5_C, buf, 1);
	buf[0] |= reg5;
	WriteCommand(LSM6DS3TRC_CTRL5_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set register 6
****************************************************************************************************************/
void LSM6DS3TRC::Set_Register6(uint8_t reg6)
{
	uint8_t buf[1] = {0};
	ReadCommand(LSM6DS3TRC_CTRL6_C, buf, 1);
	buf[0] |= reg6;
	WriteCommand(LSM6DS3TRC_CTRL6_C, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Set register 7
****************************************************************************************************************/
void LSM6DS3TRC::Set_Register7(uint8_t reg7)
{
	uint8_t buf[1] = {0};

	ReadCommand(LSM6DS3TRC_CTRL7_G, buf, 1);
	buf[0] |= reg7;
	WriteCommand(LSM6DS3TRC_CTRL7_G, buf, 1);
}

/***************************************************************************************************************
LSM6DS3TRC Read Command
****************************************************************************************************************/
boolean LSM6DS3TRC::ReadCommand(uint8_t reg_addr, uint8_t *rev_data, uint8_t length)
{
	if (LSM6DS3TRC_MODE == LSM6DS3TRC_MODE_I2C)
	{
		return i2c_ReadCommand(reg_addr, rev_data, length);
	}
	else
		spi_ReadCommand(reg_addr, rev_data, length);
}

/***************************************************************************************************************
LSM6DS3TRC Write Command
****************************************************************************************************************/
void LSM6DS3TRC::WriteCommand(uint8_t reg_addr, uint8_t *send_data, uint16_t length)
{
	if (LSM6DS3TRC_MODE == LSM6DS3TRC_MODE_I2C)
		i2c_WriteCommand(reg_addr, send_data, length);
	else
		spi_WriteCommand(reg_addr, send_data, length);
}

boolean LSM6DS3TRC::i2c_ReadCommand(uint8_t reg_addr, uint8_t *rev_data, uint8_t length)
{
	Wire.beginTransmission(_i2caddr);
	Wire.write(reg_addr);
	Wire.endTransmission(false);
	delay(15);
	while (Wire.requestFrom(_i2caddr, length) != length)
		;
	for (uint8_t i = 0; i < length; i++)
		rev_data[i] = Wire.read();
	return true;
}

void LSM6DS3TRC::i2c_WriteCommand(uint8_t reg, uint8_t *send_data, uint8_t length)
{
	Wire.beginTransmission(_i2caddr);
	Wire.write(reg);
	Wire.write(send_data, length);
	Wire.endTransmission();
}

void LSM6DS3TRC::spi_ReadCommand(uint8_t reg, uint8_t *rev_data, uint8_t length)
{
	SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
	digitalWrite(Pin_CS, LOW);
	SPI.transfer(reg | 0x80); // read, bit 7 high

	for (uint8_t i = 0; i < length; i++)
		rev_data[i] = SPI.transfer(0);

	digitalWrite(Pin_CS, HIGH);
	SPI.endTransaction(); // release the SPI bus
}

void LSM6DS3TRC::spi_WriteCommand(uint8_t reg, uint8_t *send_data, uint8_t length)
{
	SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
	digitalWrite(Pin_CS, LOW);
	SPI.transfer(reg & ~0x80);
	SPI.transfer(send_data, length);
	digitalWrite(Pin_CS, HIGH);
	SPI.endTransaction();
}
