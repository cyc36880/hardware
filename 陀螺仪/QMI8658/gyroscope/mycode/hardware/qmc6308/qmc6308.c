#include "qmc6308.h"

#include "stdio.h"
#include "math.h"

#include "myiic2.h"
#include "delay.h"

uint8_t qmc6308_write_dat(uint8_t reg, uint8_t dat)
{
    for( ; ; )
    {
        I2C2_Start();
        I2C2_SendByte((QMI6308_ADDR<<1)|IIC_W);
        if(I2C2_RecvACK())  break;
        I2C2_SendByte(reg);
        if(I2C2_RecvACK())  break;
        I2C2_SendByte(dat);
        if(I2C2_RecvACK())  break;
        I2C2_Stop();
        
        return 0;
        
    }
    return 1;
}

uint8_t qmc6308_read_dat(uint8_t reg, uint8_t *dat)
{
    for( ; ; )
    {
        I2C2_Start();
        I2C2_SendByte((QMI6308_ADDR<<1)|IIC_W);
        if(I2C2_RecvACK())  break;
        I2C2_SendByte(reg);
        if(I2C2_RecvACK())  break;

        I2C2_Start();
        I2C2_SendByte((QMI6308_ADDR<<1)|IIC_R);
        if(I2C2_RecvACK())  break;
        *dat = I2C2_RecvByte(0);
        I2C2_Stop();

        return 0;
    }

    return 1;
}


void qmc6308_init(void)
{
    qmc6308_write_dat(10, 0xc3);
    delay_ms(200);
    qmc6308_write_dat(11, 0x00);
    delay_ms(200);
}


void qmc6308_test(void)
{
    uint8_t dat;
    short xyz[3] = {0};
    uint8_t staflg = 0;

    if(qmc6308_read_dat(9, &staflg) == 1)
    {
        printf("filed..\r\n");
    }

    if(staflg & 0x01)
    {
        if(qmc6308_read_dat(2, &dat) == 1) printf("x filed..\r\n");
        xyz[0] = dat;
        xyz[0] <<= 8;
        qmc6308_read_dat(1, &dat);
        xyz[0] |= dat;
        // xyz[0] = (float)xyz[0] / 32767 * 3000;

        if(qmc6308_read_dat(4, &dat)==1) printf("y filed..\r\n");
        xyz[1] = dat;
        xyz[1] <<= 8;
        qmc6308_read_dat(3, &dat);
        xyz[1] |= dat;
        // xyz[1] = (float)xyz[1] / 32767 * 3000;

        if(qmc6308_read_dat(6, &dat) == 1) printf("z filed..\r\n");
        xyz[2] = dat;
        xyz[2] <<= 8;
        qmc6308_read_dat(5, &dat);
        xyz[2] |= dat;
        // xyz[2] = (float)xyz[2] / 32767 * 3000;

    }
    // printf("x : %d \n", xyz[0]);
    // printf("y : %d \n", xyz[1]);
    // printf("z : %d\n", xyz[2]);
    printf("偏航 angle: %.2f \n", atan2(xyz[0], xyz[1]) * 180 / 3.1415926);
}


