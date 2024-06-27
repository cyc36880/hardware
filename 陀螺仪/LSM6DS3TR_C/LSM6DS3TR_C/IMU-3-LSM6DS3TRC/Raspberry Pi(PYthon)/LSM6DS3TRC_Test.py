#!/usr/bin/env python3

# RTrobot LSM6DS3TRC Sensor Test
# http://rtrobot.org
# SPI use spi0


import RTrobot_LSM6DS3TRC
import sys
import time

LSM6DS3TRC_MODE_I2C = 1
LSM6DS3TRC_MODE_SPI = 2

lsm = RTrobot_LSM6DS3TRC.RTrobot_LSM6DS3TRC(mode=LSM6DS3TRC_MODE_SPI, spi_cs=8)
buf = lsm.LSM6DS3TRC_Init()
if buf == False:
    print("LSM6DS3TRC initialize error.")
    while True:
        pass
else:
    print("LSM6DS3TRC initialize register finished.")

while True:
    status = lsm.LSM6DS3TRC_Get_Status()

    if status & lsm.LSM6DS3TRC_STATUS_ACCELEROMETER:
        acc = lsm.LSM6DS3TRC_Get_Acceleration(lsm.LSM6DS3TRC_ACC_FSXL_2G)
        print("acc:", acc)

    if status & lsm.LSM6DS3TRC_STATUS_GYROSCOPE:
        gry = lsm.LSM6DS3TRC_Get_Gyroscope(lsm.LSM6DS3TRC_GYR_FSG_2000)
        print("gry:", gry)

    if status & lsm.LSM6DS3TRC_STATUS_TEMPERATURE:
        temp = lsm.LSM6DS3TRC_Get_Temperature()
        print("TEMPERATURE:", temp)
    time.sleep(0.1)
