#!/usr/bin/env python3

# RTrobot LSM6DS3TRC Test
# http://rtrobot.org
# SPI use spi0

import time
import fcntl
import array
import spidev
import RPi.GPIO as GPIO

I2C_SLAVE = 0x0703
SPI_BUS = 0
SPI_DEVICE = 0


class RTrobot_LSM6DS3TRC:
    LSM6DS3TRC_FUNC_CFG_ACCESS = (0x01)
    LSM6DS3TRC_SENSOR_SYNC_TIME_FRAME = (0x02)
    LSM6DS3TRC_FIFO_CTRL1 = (0x06)
    LSM6DS3TRC_FIFO_CTRL2 = (0x07)
    LSM6DS3TRC_FIFO_CTRL3 = (0x08)
    LSM6DS3TRC_FIFO_CTRL4 = (0x09)
    LSM6DS3TRC_FIFO_CTRL5 = (0x0A)
    LSM6DS3TRC_ORIENT_CFG_G = (0x0B)
    LSM6DS3TRC_INT1_CTRL = (0x0D)
    LSM6DS3TRC_INT2_CTRL = (0x0E)
    LSM6DS3TRC_WHO_AM_I = (0x0F)
    LSM6DS3TRC_CTRL1_XL = (0x10)
    LSM6DS3TRC_CTRL2_G = (0x11)
    LSM6DS3TRC_CTRL3_C = (0x12)
    LSM6DS3TRC_CTRL4_C = (0x13)
    LSM6DS3TRC_CTRL5_C = (0x14)
    LSM6DS3TRC_CTRL6_C = (0x15)
    LSM6DS3TRC_CTRL7_G = (0x16)
    LSM6DS3TRC_CTRL8_XL = (0x17)
    LSM6DS3TRC_CTRL9_XL = (0x18)
    LSM6DS3TRC_CTRL10_C = (0x19)
    LSM6DS3TRC_MASTER_CONFIG = (0x1A)
    LSM6DS3TRC_WAKE_UP_SRC = (0x1B)
    LSM6DS3TRC_TAP_SRC = (0x1C)
    LSM6DS3TRC_D6D_SRC = (0x1D)
    LSM6DS3TRC_STATUS_REG = (0x1E)
    LSM6DS3TRC_OUT_TEMP_L = (0x20)
    LSM6DS3TRC_OUT_TEMP_H = (0x21)
    LSM6DS3TRC_OUTX_L_G = (0x22)
    LSM6DS3TRC_OUTX_H_G = (0x23)
    LSM6DS3TRC_OUTY_L_G = (0x24)
    LSM6DS3TRC_OUTY_H_G = (0x25)
    LSM6DS3TRC_OUTZ_L_G = (0x26)
    LSM6DS3TRC_OUTZ_H_G = (0x27)
    LSM6DS3TRC_OUTX_L_XL = (0x28)
    LSM6DS3TRC_OUTX_H_XL = (0x29)
    LSM6DS3TRC_OUTY_L_XL = (0x2A)
    LSM6DS3TRC_OUTY_H_XL = (0x2B)
    LSM6DS3TRC_OUTZ_L_XL = (0x2C)
    LSM6DS3TRC_OUTZ_H_XL = (0x2D)
    LSM6DS3TRC_SENSORHUB1_REG = (0x2E)
    LSM6DS3TRC_SENSORHUB2_REG = (0x2F)
    LSM6DS3TRC_SENSORHUB3_REG = (0x30)
    LSM6DS3TRC_SENSORHUB4_REG = (0x31)
    LSM6DS3TRC_SENSORHUB5_REG = (0x32)
    LSM6DS3TRC_SENSORHUB6_REG = (0x33)
    LSM6DS3TRC_SENSORHUB7_REG = (0x34)
    LSM6DS3TRC_SENSORHUB8_REG = (0x35)
    LSM6DS3TRC_SENSORHUB9_REG = (0x36)
    LSM6DS3TRC_SENSORHUB10_REG = (0x37)
    LSM6DS3TRC_SENSORHUB11_REG = (0x38)
    LSM6DS3TRC_SENSORHUB12_REG = (0x39)
    LSM6DS3TRC_FIFO_STATUS1 = (0x3A)
    LSM6DS3TRC_FIFO_STATUS2 = (0x3B)
    LSM6DS3TRC_FIFO_STATUS3 = (0x3C)
    LSM6DS3TRC_FIFO_STATUS4 = (0x3D)
    LSM6DS3TRC_FIFO_DATA_OUT_L = (0x3E)
    LSM6DS3TRC_FIFO_DATA_OUT_H = (0x3F)
    LSM6DS3TRC_TIMESTAMP0_REG = (0x40)
    LSM6DS3TRC_TIMESTAMP1_REG = (0x41)
    LSM6DS3TRC_TIMESTAMP2_REG = (0x42)
    LSM6DS3TRC_STEP_TIMESTAMP_L = (0x49)
    LSM6DS3TRC_STEP_TIMESTAMP_H = (0x4A)
    LSM6DS3TRC_STEP_COUNTER_L = (0x4B)
    LSM6DS3TRC_STEP_COUNTER_H = (0x4C)
    LSM6DS3TRC_SENSORHUB13_REG = (0x4D)
    LSM6DS3TRC_SENSORHUB14_REG = (0x4E)
    LSM6DS3TRC_SENSORHUB15_REG = (0x4F)
    LSM6DS3TRC_SENSORHUB16_REG = (0x50)
    LSM6DS3TRC_SENSORHUB17_REG = (0x51)
    LSM6DS3TRC_SENSORHUB18_REG = (0x52)
    LSM6DS3TRC_FUNC_SRC = (0x53)
    LSM6DS3TRC_TAP_CFG = (0x58)
    LSM6DS3TRC_TAP_THS_6D = (0x59)
    LSM6DS3TRC_INT_DUR2 = (0x5A)
    LSM6DS3TRC_WAKE_UP_THS = (0x5B)
    LSM6DS3TRC_WAKE_UP_DUR = (0x5C)
    LSM6DS3TRC_FREE_FALL = (0x5D)
    LSM6DS3TRC_MD1_CFG = (0x5E)
    LSM6DS3TRC_MD2_CFG = (0x5F)
    LSM6DS3TRC_OUT_MAG_RAW_X_L = (0x66)
    LSM6DS3TRC_OUT_MAG_RAW_X_H = (0x67)
    LSM6DS3TRC_OUT_MAG_RAW_Y_L = (0x68)
    LSM6DS3TRC_OUT_MAG_RAW_Y_H = (0x69)
    LSM6DS3TRC_OUT_MAG_RAW_Z_L = (0x6A)
    LSM6DS3TRC_OUT_MAG_RAW_Z_H = (0x6B)
    LSM6DS3TRC_X_OFS_USR = (0x73)
    LSM6DS3TRC_Y_OFS_USR = (0x74)
    LSM6DS3TRC_Z_OFS_USR = (0x75)


# Linear acceleration out data rate
    LSM6DS3TRC_ACC_RATE_0 = (0x00)
    LSM6DS3TRC_ACC_RATE_1HZ6 = (0xB0)
    LSM6DS3TRC_ACC_RATE_12HZ5 = (0x10)
    LSM6DS3TRC_ACC_RATE_26HZ = (0x20)
    LSM6DS3TRC_ACC_RATE_52HZ = (0x30)
    LSM6DS3TRC_ACC_RATE_104HZ = (0x40)
    LSM6DS3TRC_ACC_RATE_208HZ = (0x50)
    LSM6DS3TRC_ACC_RATE_416HZ = (0x60)
    LSM6DS3TRC_ACC_RATE_833HZ = (0x70)
    LSM6DS3TRC_ACC_RATE_1660HZ = (0x80)
    LSM6DS3TRC_ACC_RATE_3330HZ = (0x90)
    LSM6DS3TRC_ACC_RATE_6660HZ = (0xA0)

# Linear gyroscope out data rate
    LSM6DS3TRC_GYR_RATE_0 = (0x00)
    LSM6DS3TRC_GYR_RATE_1HZ6 = (0xB0)
    LSM6DS3TRC_GYR_RATE_12HZ5 = (0x10)
    LSM6DS3TRC_GYR_RATE_26HZ = (0x20)
    LSM6DS3TRC_GYR_RATE_52HZ = (0x30)
    LSM6DS3TRC_GYR_RATE_104HZ = (0x40)
    LSM6DS3TRC_GYR_RATE_208HZ = (0x50)
    LSM6DS3TRC_GYR_RATE_416HZ = (0x60)
    LSM6DS3TRC_GYR_RATE_833HZ = (0x70)
    LSM6DS3TRC_GYR_RATE_1660HZ = (0x80)
    LSM6DS3TRC_GYR_RATE_3330HZ = (0x90)
    LSM6DS3TRC_GYR_RATE_6660HZ = (0xA0)

# Accelerometer full-scale.
    LSM6DS3TRC_ACC_FSXL_2G = (0x00)
    LSM6DS3TRC_ACC_FSXL_16G = (0x04)
    LSM6DS3TRC_ACC_FSXL_4G = (0x08)
    LSM6DS3TRC_ACC_FSXL_8G = (0x0C)

# Gyroscope full-scale.
    LSM6DS3TRC_GYR_FSG_245 = (0x00)
    LSM6DS3TRC_GYR_FSG_500 = (0x04)
    LSM6DS3TRC_GYR_FSG_1000 = (0x08)
    LSM6DS3TRC_GYR_FSG_2000 = (0x0C)

# Accelerometer analog chain bandwidth
    LSM6DS3TRC_ACC_BW0XL_1500HZ = (0x00)
    LSM6DS3TRC_ACC_BW0XL_400HZ = (0x01)

# Accelerometer bandwidth selection
    LSM6DS3TRC_ACC_LOW_PASS_ODR_50 = (0x88)
    LSM6DS3TRC_ACC_LOW_PASS_ODR_100 = (0xA8)
    LSM6DS3TRC_ACC_LOW_PASS_ODR_9 = (0xC8)
    LSM6DS3TRC_ACC_LOW_PASS_ODR_400 = (0xE8)

    LSM6DS3TRC_ACC_HIGH_PASS_ODR_50 = (0x04)
    LSM6DS3TRC_ACC_HIGH_PASS_ODR_100 = (0x24)
    LSM6DS3TRC_ACC_HIGH_PASS_ODR_9 = (0x44)
    LSM6DS3TRC_ACC_HIGH_PASS_ODR_400 = (0x64)


# CTRL4_C register
    LSM6DS3TRC_CTRL4_DEN_XL_EN_DISABLE = (0x00)
    LSM6DS3TRC_CTRL4_DEN_XL_EN_ENABLE = (0x80)
    LSM6DS3TRC_CTRL4_SLEEP_ENABLE = (0x40)
    LSM6DS3TRC_CTRL4_SLEEP_DISABLE = (0x00)
    LSM6DS3TRC_CTRL4_DEN_DRDY_INT1_DISBALE = (0x00)
    LSM6DS3TRC_CTRL4_DEN_DRDY_INT1_ENABLE = (0x20)
    LSM6DS3TRC_CTRL4_DRDY_MASK_DISABLE = (0x00)
    LSM6DS3TRC_CTRL4_DRDY_MASK_ENABLE = (0x08)
    LSM6DS3TRC_CTRL4_I2C_DISABLE = (0x04)
    LSM6DS3TRC_CTRL4_I2C_ENABLE = (0x00)
    LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE = (0x02)
    LSM6DS3TRC_CTRL4_LPF1_SELG_DISABLE = (0x00)

# CTRL6_C register
    LSM6DS3TRC_CTRL6_C_EDGE_TRIGGER = (0x80)
    LSM6DS3TRC_CTRL6_C_LEVEL_TRIGGER = (0x40)
    LSM6DS3TRC_CTRL6_C_LEVEL_LATCHED = (0x60)
    LSM6DS3TRC_CTRL6_C_LEVEL_FIFO = (0xC0)
    LSM6DS3TRC_CTRL6_C_XL_HM_MODE_ENABLE = (0x00)
    LSM6DS3TRC_CTRL6_C_XL_HM_MODE_DISABLE = (0x10)
    LSM6DS3TRC_CTRL6_C_FTYPE_1 = (0x00)
    LSM6DS3TRC_CTRL6_C_FTYPE_2 = (0x01)
    LSM6DS3TRC_CTRL6_C_FTYPE_3 = (0x02)
    LSM6DS3TRC_CTRL6_C_FTYPE_4 = (0x03)

# CTRL7_G register
    LSM6DS3TRC_CTRL7_G_HM_MODE_ENABLE = (0x00)
    LSM6DS3TRC_CTRL7_G_HM_MODE_DISABLE = (0x80)
    LSM6DS3TRC_CTRL7_G_HP_EN_DISABLE = (0x00)
    LSM6DS3TRC_CTRL7_G_HP_EN_ENABLE = (0x40)
    LSM6DS3TRC_CTRL7_G_HPM_16MHZ = (0x00)
    LSM6DS3TRC_CTRL7_G_HPM_65MHZ = (0x10)
    LSM6DS3TRC_CTRL7_G_HPM_260MHZ = (0x20)
    LSM6DS3TRC_CTRL7_G_HPM_1HZ04 = (0x30)
    LSM6DS3TRC_CTRL7_G_ROUNDING_STATUS_DISABLE = (0x04)
    LSM6DS3TRC_CTRL7_G_ROUNDING_STATUS_ENABLE = (0x00)

    LSM6DS3TRC_STATUS_TEMPERATURE = (0x04)
    LSM6DS3TRC_STATUS_GYROSCOPE = (0x02)
    LSM6DS3TRC_STATUS_ACCELEROMETER = (0x01)

    LSM6DS3TRC_MODE_I2C = (1)
    LSM6DS3TRC_MODE_SPI = (2)
    LSM6DS3TRC_MODE = (0)
    LSM6DS3TRC_CSPIN = (0)
    LSM6DS3TRC_ADDRESS = (0x6A)

    def __init__(self, mode=LSM6DS3TRC_MODE_I2C, i2c_no=1, i2c_addr=LSM6DS3TRC_ADDRESS, spi_cs=8):
        self.LSM6DS3TRC_MODE = mode
        if self.LSM6DS3TRC_MODE == self.LSM6DS3TRC_MODE_I2C:
            global LSM6DS3TRC_rb, LSM6DS3TRC_wb
            LSM6DS3TRC_rb = open("/dev/i2c-"+str(i2c_no), "rb", buffering=0)
            LSM6DS3TRC_wb = open("/dev/i2c-"+str(i2c_no), "wb", buffering=0)
            fcntl.ioctl(LSM6DS3TRC_rb, I2C_SLAVE, i2c_addr)
            fcntl.ioctl(LSM6DS3TRC_wb, I2C_SLAVE, i2c_addr)
        if self.LSM6DS3TRC_MODE == self.LSM6DS3TRC_MODE_SPI:
            self.LSM6DS3TRC_CSPIN = spi_cs
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.LSM6DS3TRC_CSPIN, GPIO.OUT)
            GPIO.output(self.LSM6DS3TRC_CSPIN, False)
            global spi
            spi = spidev.SpiDev()
            spi.open(SPI_BUS, SPI_DEVICE)
            spi.max_speed_hz = 100000

    # LSM6DS3TRC Initialization
    def LSM6DS3TRC_Init(self):
        # Read ID
        #buf = [self.LSM6DS3TRC_WHO_AM_I| 0x80]
        buf = [self.LSM6DS3TRC_WHO_AM_I]
        data = self.LSM6DS3TRC_read(buf)
        print(data)
        if data[0] != 0x6a:
            return False

        # reboot and reset register
        self.LSM6DS3TRC_Reset()

        # enable Block Data Update
        self.LSM6DS3TRC_Set_BDU(True)

        # Set Data Rate
        self.LSM6DS3TRC_Set_Accelerometer_Rate(self.LSM6DS3TRC_ACC_RATE_12HZ5)
        self.LSM6DS3TRC_Set_Gyroscope_Rate(self.LSM6DS3TRC_GYR_RATE_12HZ5)

        # Set full-scale selection.
        self.LSM6DS3TRC_Set_Accelerometer_Fullscale(
            self.LSM6DS3TRC_ACC_FSXL_2G)
        self.LSM6DS3TRC_Set_Gyroscope_Fullscale(self.LSM6DS3TRC_GYR_FSG_2000)

        # set accelerometer analog chain bandwidth.
        self.LSM6DS3TRC_Set_Accelerometer_Bandwidth(
            self.LSM6DS3TRC_ACC_BW0XL_400HZ, self.LSM6DS3TRC_ACC_LOW_PASS_ODR_100)

        self.LSM6DS3TRC_Set_Register7(
            self.LSM6DS3TRC_CTRL7_G_HM_MODE_DISABLE | self.LSM6DS3TRC_CTRL7_G_HPM_260MHZ)
        self.LSM6DS3TRC_Set_Register6(self.LSM6DS3TRC_CTRL6_C_FTYPE_1)
        self.LSM6DS3TRC_Set_Register4(self.LSM6DS3TRC_CTRL4_LPF1_SELG_ENABLE)
        return True

    # LSM6DS3TRC Get Temperature Value
    def LSM6DS3TRC_Get_Temperature(self):
        buf = [self.LSM6DS3TRC_OUT_TEMP_L]
        data = self.LSM6DS3TRC_read(buf, 2)
        temp = (data[1] << 8 | data[0])/256.0+25.0
        return temp

    # LSM6DS3TRC Get Gyroscope Value
    def LSM6DS3TRC_Get_Gyroscope(self, fsg):
        buf = [self.LSM6DS3TRC_OUTX_L_G]
        data = self.LSM6DS3TRC_read(buf, 6)
        gry = [data[1] << 8 | data[0], data[3] <<
               8 | data[2], data[5] << 8 | data[4], ]
        gry_float = [0.0, 0.0, 0.0]
        if fsg == self.LSM6DS3TRC_GYR_FSG_245:
            gry_float[0] = gry[0]*8.750
            gry_float[1] = gry[1]*8.750
            gry_float[2] = gry[2]*8.750
        elif fsg == self.LSM6DS3TRC_GYR_FSG_500:
            gry_float[0] = gry[0]*17.50
            gry_float[1] = gry[1]*17.50
            gry_float[2] = gry[2]*17.50
        elif fsg == self.LSM6DS3TRC_GYR_FSG_1000:
            gry_float[0] = gry[0]*35.00
            gry_float[1] = gry[1]*35.00
            gry_float[2] = gry[2]*35.00
        elif fsg == self.LSM6DS3TRC_GYR_FSG_2000:
            gry_float[0] = gry[0]*70.00
            gry_float[1] = gry[1]*70.00
            gry_float[2] = gry[2]*70.00
        return gry_float

    # LSM6DS3TRC Get Acceleration Value
    def LSM6DS3TRC_Get_Acceleration(self, fsxl):
        buf = [self.LSM6DS3TRC_OUTX_L_XL]
        data = self.LSM6DS3TRC_read(buf, 6)
        acc = [data[1] << 8 | data[0], data[3] <<
               8 | data[2], data[5] << 8 | data[4], ]
        acc_float = [0.0, 0.0, 0.0]
        if fsxl == self.LSM6DS3TRC_ACC_FSXL_2G:
            acc_float[0] = acc[0]*0.061
            acc_float[1] = acc[1]*0.061
            acc_float[2] = acc[2]*0.061
        elif fsxl == self.LSM6DS3TRC_ACC_FSXL_16G:
            acc_float[0] = acc[0]*0.488
            acc_float[1] = acc[1]*0.488
            acc_float[2] = acc[2]*0.488
        elif fsxl == self.LSM6DS3TRC_ACC_FSXL_4G:
            acc_float[0] = acc[0]*0.122
            acc_float[1] = acc[1]*0.122
            acc_float[2] = acc[2]*0.122
        elif fsxl == self.LSM6DS3TRC_ACC_FSXL_8G:
            acc_float[0] = acc[0]*0.244
            acc_float[1] = acc[1]*0.244
            acc_float[2] = acc[2]*0.244
        return acc_float

    # LSM6DS3TRC Get data status
    def LSM6DS3TRC_Get_Status(self):
        buf = [self.LSM6DS3TRC_STATUS_REG]
        data = self.LSM6DS3TRC_read(buf)
        return data[0]

    # LSM6DS3TRC Set register 4
    def LSM6DS3TRC_Set_Register4(self, reg4):
        buf = [self.LSM6DS3TRC_CTRL4_C]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL4_C, 0]
        buf2[1] = data[0] | reg4
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set register 5
    def LSM6DS3TRC_Set_Register5(self, reg5):
        buf = [self.LSM6DS3TRC_CTRL5_C]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL5_C, 0]
        buf2[1] = data[0] | reg5
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set register 6
    def LSM6DS3TRC_Set_Register6(self, reg6):
        buf = [self.LSM6DS3TRC_CTRL6_C]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL6_C, 0]
        buf2[1] = data[0] | reg6
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set register 7
    def LSM6DS3TRC_Set_Register7(self, reg7):
        buf = [self.LSM6DS3TRC_CTRL7_G]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL7_G, 0]
        buf2[1] = data[0] | reg7
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set accelerometer analog chain bandwidth.
    def LSM6DS3TRC_Set_Accelerometer_Bandwidth(self, BW0XL, ODR):
        buf = [self.LSM6DS3TRC_CTRL2_G]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL2_G, 0]
        buf2[1] = data[0] | BW0XL
        self.LSM6DS3TRC_write(buf2)
        buf[0] = self.LSM6DS3TRC_CTRL8_XL
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL8_XL, 0]
        buf2[1] = data[0] | ODR
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set gyroscope full-scale selection.
    def LSM6DS3TRC_Set_Gyroscope_Fullscale(self, value):
        buf = [self.LSM6DS3TRC_CTRL2_G]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL2_G, 0]
        buf2[1] = data[0] | value
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set accelerometer full-scale selection.
    def LSM6DS3TRC_Set_Accelerometer_Fullscale(self, value):
        buf = [self.LSM6DS3TRC_CTRL1_XL]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL1_XL, 0]
        buf2[1] = data[0] | value
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set gyroscope data rate
    def LSM6DS3TRC_Set_Gyroscope_Rate(self, rate):
        buf = [self.LSM6DS3TRC_CTRL2_G]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL2_G, 0]
        buf2[1] = data[0] | rate
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set accelerometer data rate
    def LSM6DS3TRC_Set_Accelerometer_Rate(self, rate):
        buf = [self.LSM6DS3TRC_CTRL1_XL]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL1_XL, 0]
        buf2[1] = data[0] | rate
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC Set Block Data Update
    def LSM6DS3TRC_Set_BDU(self, flag):
        buf = [self.LSM6DS3TRC_CTRL3_C]
        data = self.LSM6DS3TRC_read(buf)
        buf2 = [self.LSM6DS3TRC_CTRL3_C, 0]
        if flag is True:
            buf2[1] = data[0] | 0x40
        else:
            buf2[1] = data[0] | 0xbf
        self.LSM6DS3TRC_write(buf2)

    # LSM6DS3TRC reboot and reset register
    def LSM6DS3TRC_Reset(self):
        buf = [self.LSM6DS3TRC_CTRL3_C, 0x80]
        self.LSM6DS3TRC_write(buf)
        time.sleep(0.015)
        # reset register
        buf2 = [self.LSM6DS3TRC_CTRL3_C]
        data = self.LSM6DS3TRC_read(buf2)
        data[0] |= 0x01
        buf[1] = data[0]
        self.LSM6DS3TRC_write(buf)
        while data[0] & 0x01:
            data = self.LSM6DS3TRC_read(buf2)

    def LSM6DS3TRC_read(self, reg, len=1):
        if self.LSM6DS3TRC_MODE == self.LSM6DS3TRC_MODE_I2C:
            buf = self.LSM6DS3TRC_i2c_read(reg, len)
        if self.LSM6DS3TRC_MODE == self.LSM6DS3TRC_MODE_SPI:
            y=0
            for i in reg:
                reg[y] |= 0x80
                y=y+1
            buf = self.LSM6DS3TRC_spi_read(reg, len)
        return buf

    def LSM6DS3TRC_write(self, reg):
        if self.LSM6DS3TRC_MODE == self.LSM6DS3TRC_MODE_I2C:
            self.LSM6DS3TRC_i2c_write(reg)
        if self.LSM6DS3TRC_MODE == self.LSM6DS3TRC_MODE_SPI:
            y=0
            for i in reg:
                reg[y] &= 0x7F
                y=y+1
            self.LSM6DS3TRC_spi_write(reg)

    ######################################i2C######################################

    def LSM6DS3TRC_i2c_read(self, reg, len=1):
        self.LSM6DS3TRC_i2c_write(reg)
        tmp = LSM6DS3TRC_rb.read(len)
        return array.array('B', tmp)

    def LSM6DS3TRC_i2c_write(self, reg):
        buf_binary = bytearray(reg)
        LSM6DS3TRC_wb.write(buf_binary)

    ######################################SPI######################################
    def LSM6DS3TRC_spi_read(self, reg, len=1):
        GPIO.output(self.LSM6DS3TRC_CSPIN, False)
        spi.writebytes(reg)
        data = spi.readbytes(len)
        GPIO.output(self.LSM6DS3TRC_CSPIN, True)
        return data

    def LSM6DS3TRC_spi_write(self, reg):
        GPIO.output(self.LSM6DS3TRC_CSPIN, False)
        buf_binary = bytearray(reg)
        spi.writebytes(buf_binary)
        GPIO.output(self.LSM6DS3TRC_CSPIN, True)
