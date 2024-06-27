#include "control.h"
#include "delay.h"

#include "stdio.h"
#include "math.h"

#include "qmi8658.h"
#include "qmc6308.h"

#include "Device_IIC.h"

#include "myiic2.h"


IIC_DEVICE iic_device = {0};

float acc[3];
float gyro[3];


static void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    IIC2_IO_ENABLE;
        
    // HAL_GPIO_WritePin(DHT11_IO_Init, DHT11_IO_DQ, GPIO_PIN_SET);
    /*Configure GPIO pin : PB9 */
    GPIO_InitStruct.Pin = IIC2_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IIC2_PORT, &GPIO_InitStruct);
}
static void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    IIC2_IO_ENABLE;

    // HAL_GPIO_WritePin(DHT11_IO_Init, DHT11_IO_DQ, GPIO_PIN_SET);

    /*Configure GPIO pins : PBPin PBPin */
    GPIO_InitStruct.Pin = IIC2_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IIC2_PORT, &GPIO_InitStruct);
}

static void SDA_write(uint8_t data)
{
    HAL_GPIO_WritePin(IIC2_PORT,IIC2_SDA_PIN, data ? GPIO_PIN_SET:GPIO_PIN_RESET);
}

static uint8_t SDA_read(void)
{
    return HAL_GPIO_ReadPin(IIC2_PORT, IIC2_SDA_PIN);
}
static void SCL_write(uint8_t data)
{
    HAL_GPIO_WritePin(IIC2_PORT,IIC2_SCL_PIN, data ? GPIO_PIN_SET:GPIO_PIN_RESET);
}



void iic_delay(void)
{
    delay_us(5);
}

void iic_device_init(void)
{
    static _SOFT_IIC_DRIVE IIC_DRIVE = {0};

    IIC_DRIVE._IIC_SDA_input = SDA_IN;
    IIC_DRIVE._IIC_SDA_output = SDA_OUT;
    IIC_DRIVE._IIC_Delay = iic_delay;

    IIC_DRIVE._IIC_SDA_write = SDA_write;
    IIC_DRIVE._IIC_SDA_read = SDA_read;
    IIC_DRIVE._IIC_SCL_write = SCL_write;

    iic_device.device_address = 44;
    iic_device.iic_drive = &IIC_DRIVE;

    IIC_Drive_Add(&iic_device);

    
}


void setup(void)
{
    delay_init(64);

    qmi8658_init();
    qmc6308_init();

    iic_device_init();

    // uint8_t dat = 0xc3;
    // iic_device.Devive_IIC_writeReg(&iic_device, 10, &dat, 1);
    // delay_us(1000);

    // dat = 0x00;
    // iic_device.Devive_IIC_writeReg(&iic_device, 11, &dat, 1);
    // delay_us(1000);
}



void loop(void)
{
    qmi8658_read_xyz(acc, gyro);

    printf("角度 x : %.2f \n", atan2(acc[0], acc[2]) * 180 / 3.1415926);
    printf("角度 y : %.2f\n", atan2(acc[1], acc[2]) * 180 / 3.1415926);

    qmc6308_test(); 

    printf("\n\n\n");

    // uint8_t data[2] = {0};

    // iic_device.Devive_IIC_readReg(&iic_device, 1, &data[0], 1);
    // iic_device.Devive_IIC_readReg(&iic_device, 2, &data[1], 1);


    // printf("data : %d \n", (data[1]<<8) | data[0]);

    delay_ms(200);
}



