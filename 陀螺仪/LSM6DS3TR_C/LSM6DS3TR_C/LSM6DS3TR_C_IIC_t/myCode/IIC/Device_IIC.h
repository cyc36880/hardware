/*************************************************************
 * @LastEditors: 蔡雅超
 * @Date: 2024-05-10 14:37:31
 * @LastEditTime: 2024-06-04 19:56:03
 *************************************************************/

#ifndef _DEVICE_IIC_H_
#define _DEVICE_IIC_H_

/**
 * 单个总线上，默认只支持挂载一个设备
 * 对于多个设备的操作，需要自己实现
 * 
*/

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


typedef struct _IIC_DEVICE       IIC_DEVICE;
typedef struct __SOFT_IIC_DRIVE  _SOFT_IIC_DRIVE;


// iic 设备列表
typedef struct _IIC_DEVICE_LIST
{
    IIC_DEVICE *iic_device_header;
    IIC_DEVICE *iic_device_tail;
}IIC_DEVICE_LIST;


struct __SOFT_IIC_DRIVE
{
    // ==== 以下由用户提供
    void (*_IIC_Drive_init)(_SOFT_IIC_DRIVE *); // 端口去初始化

    void (*_IIC_SDA_input)(_SOFT_IIC_DRIVE *);
    void (*_IIC_SDA_output)(_SOFT_IIC_DRIVE *);
    void (*_IIC_Delay)(_SOFT_IIC_DRIVE *);

    void (*_IIC_SCL_write)(_SOFT_IIC_DRIVE *, uint8_t);
    void (*_IIC_SDA_write)(_SOFT_IIC_DRIVE *, uint8_t);
    uint8_t (*_IIC_SDA_read)(_SOFT_IIC_DRIVE *);

    uint16_t delay_time_us; //通信延时

    uint8_t user_data;
    // =============================

    void (*_IIC_start)(struct __SOFT_IIC_DRIVE *);
    void (*_IIC_stop)(struct __SOFT_IIC_DRIVE *);
    void (*_IIC_ack)(struct __SOFT_IIC_DRIVE *);
    void (*_IIC_nack)(struct __SOFT_IIC_DRIVE *);
    uint8_t (*_IIC_wait_ack)(struct __SOFT_IIC_DRIVE *);
    void (*_IIC_send_byte)(struct __SOFT_IIC_DRIVE *, uint8_t);
    uint8_t (*_IIC_read_byte)(struct __SOFT_IIC_DRIVE *, uint8_t);
};


struct _IIC_DEVICE
{
    // ==== 以下由用户提供
    _SOFT_IIC_DRIVE *iic_drive; // 软件模拟驱动
    uint8_t device_address; // 七位设备地址
    void (*Device_IIC_init)(void); // 设备初始化
    // =============================

    struct _IIC_DEVICE *next;
    uint8_t (*scan_DevLowAddr)(struct _IIC_DEVICE *iic_device); // 扫描得到一个低地址设备的addr
    uint8_t (*update_DevLowAddr)(struct _IIC_DEVICE *iic_device); // 更新一个低地址设备的addr
    uint8_t (*update_DevAddrFromDevList)(struct _IIC_DEVICE *iic_device, const uint8_t *addr_list, uint8_t len);// 根据设备列表更新设备地址
    uint8_t (*Device_isOnline)(struct _IIC_DEVICE *iic_device);

    uint8_t (*Device_IIC_writeBytes)(struct _IIC_DEVICE *iic_device, uint8_t *data, uint16_t len);
    uint8_t (*Device_IIC_readBytes)(struct _IIC_DEVICE *iic_device, uint8_t *data, uint16_t len);
    uint8_t (*Devive_IIC_writeReg)(struct _IIC_DEVICE *iic_device, uint8_t reg, uint8_t *data, uint16_t len);
    uint8_t (*Devive_IIC_readReg)(struct _IIC_DEVICE *iic_device, uint8_t reg, uint8_t *data, uint16_t len);
};


uint8_t IIC_Get_DeviceList_Num(IIC_DEVICE_LIST *IIC_Device_List);
IIC_DEVICE *IIC_Get_DeviceInList(IIC_DEVICE_LIST *IIC_Device_List, uint8_t id);

void IIC_Drive_Add(IIC_DEVICE_LIST *IIC_Device_List, IIC_DEVICE *iic_device);


#endif


