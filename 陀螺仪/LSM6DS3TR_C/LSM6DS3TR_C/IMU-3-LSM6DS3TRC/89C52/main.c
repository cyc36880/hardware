
#include <stdio.h>
/*
 * main.c:
 * Copyright (c) 2014-2021 Rtrobot. <admin@rtrobot.org>
 *  <http://rtrobot.org>
 ***********************************************************************
 * use keil5.
 */
#include "delay.h"
#include "uart.h"
#include "lsm6ds3trc.h"
#include "i2c.h"
#include "spi.h"

void main(void)
{
    Uart_Init();
        Delay_Ms(1000);
    printf("Start.\r\n");
    //if (LSM6DS3TRC_Init(LSM6DS3TRC_MODE_SPI) == false)
    if (LSM6DS3TRC_Init(LSM6DS3TRC_MODE_I2C) == false)
        printf("LSM6DS3TRC initialize error.\r\n");
    else
        printf("LSM6DS3TRC initialize register finished.\r\n");

    while (1)
    {
		uint8_t status;
		status=LSM6DS3TRC_Get_Status();

		if(status&LSM6DS3TRC_STATUS_ACCELEROMETER)
		{
			float acc[3]={0};
			LSM6DS3TRC_Get_Acceleration(LSM6DS3TRC_ACC_FSXL_2G,acc);
			printf("\r\nacc:X:%2f,\tY:%2f,\tZ:%2f\r\n",acc[0],acc[1],acc[2]);
		}
		if(status&LSM6DS3TRC_STATUS_GYROSCOPE)
		{
			float gyr[3]={0};
			LSM6DS3TRC_Get_Gyroscope(LSM6DS3TRC_GYR_FSG_2000,gyr);
			printf("\r\ngry:X:%4.2f,\tY:%4.2f,\tZ:%4.2f\r\n",gyr[0],gyr[1],gyr[2]);
		}
		if(status&LSM6DS3TRC_STATUS_TEMPERATURE)
		{
			printf("\r\ntemp:%2f\r\n",LSM6DS3TRC_Get_Temperature());
		}

        Delay_Ms(100);
    }
}
