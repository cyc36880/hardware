#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "main.h"

struct _Control
{
    uint8_t mode;
    /**
     * 0 : 无数据
     * 1 : 加速度
     * 2 : 角速度
     * 3 : 角度
     */
    int16_t *gyroData; //陀螺仪数据 3
};

void setup(void);
void loop(void);

extern struct _Control control;

#endif

