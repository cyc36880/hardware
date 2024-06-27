/*
 * @author: rtrobot<admin@rtrobot.org>
 * @website:rtrobot.org
 * @licence: GPL v3
 */

#include "lsm6ds3trc.h"
#include <stdlib.h>

//0:SPI
//1:I2C
uint8_t lsm6ds3trc_mode = 0;

static inline void cs_select()
{
	asm volatile("nop \n nop \n nop");
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0); // Active low
	asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect()
{
	asm volatile("nop \n nop \n nop");
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
	asm volatile("nop \n nop \n nop");
}

/***************************************************************************************************************
i2c master initialization
****************************************************************************************************************/
void RTrobot_i2c_Init(void)
{
	i2c_init(i2c_default, 400 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void RTrobot_spi_Init(void)
{
	spi_init(spi_default, 100 * 1000);
	gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);
	gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
	gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
	// Make the SPI pins available to picotool
	bi_decl(bi_3pins_with_func(PICO_DEFAULT_SPI_RX_PIN, PICO_DEFAULT_SPI_TX_PIN, PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI));

	// Chip select is active-low, so we'll initialise it to a driven-high state
	gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
	gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
	gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);
	// Make the CS pin available to picotool
	bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "SPI CS"));
}

/***************************************************************************************************************
LSM6DS3TRC Read Command
****************************************************************************************************************/
void LSM6DS3TRC_ReadCommand(uint8_t reg_addr, uint8_t *rev_data, uint8_t length)
{
	if (lsm6ds3trc_mode == LSM6DS3TRC_MODE_I2C)
	{
		i2c_write_blocking(i2c_default, LSM6DS3TRC_I2CADDR, &reg_addr, 1, true);
		i2c_read_blocking(i2c_default, LSM6DS3TRC_I2CADDR, rev_data, length, false);
	}
	else
	{
		reg_addr |= 0x80;
		cs_select();
		spi_write_blocking(spi_default, &reg_addr, 1);
		spi_read_blocking(spi_default, 0, rev_data, length);
		cs_deselect();
	}
}

/***************************************************************************************************************
LSM6DS3TRC Write Command
****************************************************************************************************************/
void LSM6DS3TRC_WriteCommand(uint8_t reg_addr, uint8_t *send_data, uint16_t length)
{
	if (lsm6ds3trc_mode == LSM6DS3TRC_MODE_I2C)
	{
		uint8_t *reg = malloc(length + 2);
		reg[0] = reg_addr;
		for (int i = 0; i < length; i++)
			reg[i + 1] = send_data[i];
		i2c_write_blocking(i2c_default, LSM6DS3TRC_I2CADDR, reg, length + 1, false);
		free(reg);
	}
	else
	{
		uint8_t *reg = malloc(length + 2);
		reg[0] = reg_addr & 0x7f;
		for (int i = 0; i < length; i++)
			reg[i + 1] = send_data[i];
		cs_select();
		spi_write_blocking(spi_default, reg, length + 1);
		cs_deselect();
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
	sleep_ms(15);

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
LSM6DS3TRC Set register 4
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
	if (lsm6ds3trc_mode == LSM6DS3TRC_MODE_I2C)
		RTrobot_i2c_Init();
	else if (lsm6ds3trc_mode == LSM6DS3TRC_MODE_SPI)
		RTrobot_spi_Init();

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
