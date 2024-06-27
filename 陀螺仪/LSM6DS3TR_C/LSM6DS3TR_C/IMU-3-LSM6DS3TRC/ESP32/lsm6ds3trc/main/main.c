/*
 * main.c:
 * Copyright (c) 2014-2021 Rtrobot. <admin@rtrobot.org>
 *  <http://rtrobot.org>
 ***********************************************************************
 */

#include <stdio.h>
#include <nvs_flash.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "lsm6ds3trc.h"

void i2c_lsm6ds3trc_task(void *pvParameters)
{
//    if (LSM6DS3TRC_Init(LSM6DS3TRC_MODE_I2C) == false)
	if (LSM6DS3TRC_Init(LSM6DS3TRC_MODE_SPI) == false)
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
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    nvs_flash_init();
    xTaskCreate(i2c_lsm6ds3trc_task, "i2c_tmf8801_task", 4096, NULL, 5, NULL);
}
