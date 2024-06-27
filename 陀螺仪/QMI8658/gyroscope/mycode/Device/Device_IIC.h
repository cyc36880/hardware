/*************************************************************
 * @LastEditors: 蔡雅超
 * @Date: 2024-05-10 14:37:31
 * @LastEditTime: 2024-05-10 17:02:52
 *************************************************************/

#ifndef _DEVICE_IIC_H_
#define _DEVICE_IIC_H_

#include <stdint.h>
#include <stdlib.h>

#define IIC_W 0
#define IIC_R 1 

#define IIC_ACK  0
#define IIC_NACK 1

#define IIC_DEVICE_ONLINE 1

typedef void    (*_IIC_Func_v_v)(void);
typedef void    (*_IIC_Func_v_u8)(uint8_t);
typedef uint8_t (*_IIC_Func_u8_v)(void);
typedef uint8_t (*_IIC_Func_u8_u8)(uint8_t);

// 7位地址长度

typedef struct __SOFT_IIC_DRIVE
{
    // ==== 以下由用户提供
    _IIC_Func_v_v _IIC_Drive_init;

    _IIC_Func_v_v _IIC_SDA_input;
    _IIC_Func_v_v _IIC_SDA_output;
    _IIC_Func_v_v _IIC_Delay;

    _IIC_Func_v_u8 _IIC_SCL_write;
    _IIC_Func_v_u8 _IIC_SDA_write;
    _IIC_Func_u8_v _IIC_SDA_read;
    // =============================
    
    void (*_IIC_start)(struct __SOFT_IIC_DRIVE *);
    void (*_IIC_stop)(struct __SOFT_IIC_DRIVE *);
    void (*_IIC_ack)(struct __SOFT_IIC_DRIVE *);
    void (*_IIC_nack)(struct __SOFT_IIC_DRIVE *);
    uint8_t (*_IIC_wait_ack)(struct __SOFT_IIC_DRIVE *);
    void (*_IIC_send_byte)(struct __SOFT_IIC_DRIVE *, uint8_t);
    uint8_t (*_IIC_read_byte)(struct __SOFT_IIC_DRIVE *, uint8_t);
}_SOFT_IIC_DRIVE;


typedef struct _IIC_DEVICE
{
    // ==== 以下由用户提供
    _SOFT_IIC_DRIVE *iic_drive; // 软件模拟驱动
    uint8_t device_address; // 七位设备地址
    void (*Device_IIC_init)(void); // 初始化
    // =============================

    struct _IIC_DEVICE *next;

    uint8_t (*Device_isOnline)(struct _IIC_DEVICE *iic_device);

    uint8_t (*Device_IIC_writeBytes)(struct _IIC_DEVICE *iic_device, uint8_t *data, uint16_t len);
    uint8_t (*Device_IIC_readBytes)(struct _IIC_DEVICE *iic_device, uint8_t *data, uint16_t len);
    uint8_t (*Devive_IIC_writeReg)(struct _IIC_DEVICE *iic_device, uint8_t reg, uint8_t *data, uint16_t len);
    uint8_t (*Devive_IIC_readReg)(struct _IIC_DEVICE *iic_device, uint8_t reg, uint8_t *data, uint16_t len);
}IIC_DEVICE;


uint8_t IIC_Get_Device_Num(void);
IIC_DEVICE *IIC_Get_Device(uint8_t id);

void IIC_Drive_Add(IIC_DEVICE *iic_device);


#endif


