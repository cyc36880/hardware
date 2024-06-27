/*************************************************************
 * @LastEditors: 蔡雅超
 * @Date: 2024-05-12 10:10:48
 * @LastEditTime: 2024-06-04 11:06:03
 *************************************************************/
#ifndef _IIC_PORT_H_
#define _IIC_PORT_H_

/**
 * 管理 iic 端口 （x6）
 * 
 * 单个端口，只允许接一个设备
 * 
 * 由于设备包括 输入输出 设备，且各个设备的驱动方式不同，该模块只负责管理端口，不负责设备驱动
 * 
*/

#include "Device_IIC.h"


#define IIC_PORT_NUM 1

enum IIC_PORT_DEV_STA
{
    IIC_PORT_DEV_STA_Have = 0, // 端口有设备
};


typedef struct __IIC_PORT_DEV
{
    uint8_t port_dev_sta;
    IIC_DEVICE dev;  // 设备

}_IIC_PORT_Dev;


typedef struct _PORT_IIC_DEV
{
    uint16_t count_ms; // 端口 自动检测 时使用

    _SOFT_IIC_DRIVE port_iic_drv[IIC_PORT_NUM];
    _IIC_PORT_Dev portDev[IIC_PORT_NUM];
    IIC_DEVICE_LIST port_iic_device_list;

}PORT_IIC_Dev;


void IIC_Port_Init(void);

// 更新端口设备状态
void update_IIC_Port_DevSta(uint8_t ms, uint8_t interval);

// 获取端口设备状态
uint8_t get_IIC_Port_DevSta(uint8_t port);

// 获取端口设备地址
uint8_t get_IIC_Port_DevAddr(uint8_t port);

// 获取端口设备句柄
IIC_DEVICE *get_IIC_Port_DevHandle(uint8_t port);

// 获取端口设备驱动句柄
_SOFT_IIC_DRIVE *get_IIC_Port_DrvHandle(uint8_t port);

/**
 * port 1 - 6
*/


#endif

