/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "lsm6ds3trc.h"

int main()
{
    stdio_init_all();

    sleep_ms(10000);
    printf("boot......\r\n");
    if (LSM6DS3TRC_Init(LSM6DS3TRC_MODE_SPI) == false)
        // if (LSM6DS3TRC_Init(LSM6DS3TRC_MODE_I2C) == false)
        printf("LSM6DS3TRC initialize error.\r\n");
    else
        printf("LSM6DS3TRC initialize register finished.\r\n");

    while (true)
    {
        uint8_t status;
        status = LSM6DS3TRC_Get_Status();

        if (status & LSM6DS3TRC_STATUS_ACCELEROMETER)
        {
            float acc[3] = {0};
            LSM6DS3TRC_Get_Acceleration(LSM6DS3TRC_ACC_FSXL_2G, acc);
            printf("\r\nacc:X:%2f,\tY:%2f,\tZ:%2f\r\n", acc[0], acc[1], acc[2]);
        }
        if (status & LSM6DS3TRC_STATUS_GYROSCOPE)
        {
            float gyr[3] = {0};
            LSM6DS3TRC_Get_Gyroscope(LSM6DS3TRC_GYR_FSG_2000, gyr);
            printf("\r\ngry:X:%4.2f,\tY:%4.2f,\tZ:%4.2f\r\n", gyr[0], gyr[1], gyr[2]);
        }
        if (status & LSM6DS3TRC_STATUS_TEMPERATURE)
        {
            printf("\r\ntemp:%2f\r\n", LSM6DS3TRC_Get_Temperature());
        }
        sleep_ms(1000);
    }
    return 0;
}
