/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <bsp.h>
#include <sysctl.h>
#include <syslog.h>
#include <plic.h>
#include "lsm6ds3trc.h"

int main(void)
{
	sysctl_pll_set_freq(SYSCTL_PLL0, 800000000UL);

	msleep(100);
	if (LSM6DS3TRC_Init(LSM6DS3TRC_MODE_I2C) != true)
	{
		printf("LSM6DS3TR initialize error.\r\n");
		while (1);
	}
	else
		printf("LSM6DS3TR initialize register finished.\r\n");

	while (1)
	{
		uint8_t status;
		status=LSM6DS3TRC_Get_Status();
		
		if(status&LSM6DS3TRC_STATUS_ACCELEROMETER)
		{
			float acc[3]={0};
			LSM6DS3TRC_Get_Acceleration(LSM6DS3TRC_ACC_FSXL_2G,acc);
			printf("\r\nacc:\r\nX:%2f,\tY:%2f,\tZ:%2f\r\n",acc[0],acc[1],acc[2]);
		}
		if(status&LSM6DS3TRC_STATUS_GYROSCOPE)
		{
			float gyr[3]={0};
			LSM6DS3TRC_Get_Gyroscope(LSM6DS3TRC_GYR_FSG_2000,gyr);
			printf("\r\ngry:\r\nX:%4.2f,\tY:%4.2f,\tZ:%4.2f\r\n",gyr[0],gyr[1],gyr[2]);
		}
		if(status&LSM6DS3TRC_STATUS_TEMPERATURE)
		{
			printf("\r\ntemp:%2f\r\n",LSM6DS3TRC_Get_Temperature());
		}
		msleep(500);
	}
	return 0;
}


