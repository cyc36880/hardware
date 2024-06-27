/*************************************************************
 * @LastEditors: 蔡雅超
 * @Date: 2024-05-10 14:37:31
 * @LastEditTime: 2024-05-10 16:49:15
 *************************************************************/
#include "Device_IIC.h"


IIC_DEVICE *IIC_DeviveList_Hraed = NULL;
IIC_DEVICE *IIC_DeviveList_Tail = NULL;


static void _IIC_Start(_SOFT_IIC_DRIVE *iic_drive)
{
    iic_drive->_IIC_SDA_output();
    iic_drive->_IIC_SDA_write(1);
    iic_drive->_IIC_SCL_write(1);
    iic_drive->_IIC_Delay();
    iic_drive->_IIC_SDA_write(0);
    iic_drive->_IIC_Delay();
    iic_drive->_IIC_SCL_write(0);
}

static void _IIC_Stop(_SOFT_IIC_DRIVE *iic_drive)
{
    iic_drive->_IIC_SDA_output();
    iic_drive->_IIC_SDA_write(0);
    iic_drive->_IIC_SCL_write(1);
    iic_drive->_IIC_Delay();
    iic_drive->_IIC_SDA_write(1);
    iic_drive->_IIC_Delay();
}

// 
static void _IIC_Ack(_SOFT_IIC_DRIVE *iic_drive)
{
    iic_drive->_IIC_SCL_write(0);
    iic_drive->_IIC_SDA_output();
    iic_drive->_IIC_SDA_write(0);
    iic_drive->_IIC_Delay();
    iic_drive->_IIC_SCL_write(1);
    iic_drive->_IIC_Delay();
    iic_drive->_IIC_SCL_write(0);
}

static void _IIC_NAck(_SOFT_IIC_DRIVE *iic_drive)
{
    iic_drive->_IIC_SCL_write(0);
    iic_drive->_IIC_SDA_output();
    iic_drive->_IIC_SDA_write(1);
    iic_drive->_IIC_Delay();
    iic_drive->_IIC_SCL_write(1);
    iic_drive->_IIC_Delay();
    iic_drive->_IIC_SCL_write(0);
}

static uint8_t _IIC_WaitAck(_SOFT_IIC_DRIVE *iic_drive)
{
    uint8_t ack;
    iic_drive->_IIC_SDA_input();
    iic_drive->_IIC_SCL_write(1);
    iic_drive->_IIC_Delay();
    ack = iic_drive->_IIC_SDA_read();
    iic_drive->_IIC_SCL_write(0);
    return ack;
}

static void _IIC_SendByte(_SOFT_IIC_DRIVE *iic_drive, uint8_t byte)
{
    iic_drive->_IIC_SDA_output();

    for(uint8_t i=0; i<8; i++)
    {
        iic_drive->_IIC_SDA_write(byte >> 7);
        byte <<= 1;
        iic_drive->_IIC_SCL_write(1);
        iic_drive->_IIC_Delay();
        iic_drive->_IIC_SCL_write(0);
        iic_drive->_IIC_Delay();
    }
}

static uint8_t _IIC_ReadByte(_SOFT_IIC_DRIVE *iic_drive, uint8_t ack)
{
    uint8_t byte = 0;

    iic_drive->_IIC_SDA_write(1);
    iic_drive->_IIC_SDA_input();
    
    for(uint8_t i=0; i<8; i++)
    {
        byte <<= 1;
        iic_drive->_IIC_SCL_write(1);
        iic_drive->_IIC_Delay();
        if(iic_drive->_IIC_SDA_read())
        {
            byte |= 0x01;
        }
        iic_drive->_IIC_SCL_write(0);
        iic_drive->_IIC_Delay();
    }
    if(ack) iic_drive->_IIC_ack(iic_drive);
    else iic_drive->_IIC_nack(iic_drive);

    return byte;
}


// ====================================================================


static uint8_t _IIC_writeBytes(IIC_DEVICE *device, uint8_t *data, uint16_t len)
{
    if(device->iic_drive == NULL) return 1;

    _SOFT_IIC_DRIVE *iic_drive = device->iic_drive;

    for( ; ; )
    {
        iic_drive->_IIC_start(iic_drive);
        iic_drive->_IIC_send_byte(iic_drive, (device->device_address<<1) | IIC_W);
        if(iic_drive->_IIC_wait_ack(iic_drive)) break;

        for(uint8_t i=0; i<len; i++)
        {
            iic_drive->_IIC_send_byte(iic_drive, data[i]);
            if(iic_drive->_IIC_wait_ack(iic_drive)) break;
        }
        iic_drive->_IIC_stop(iic_drive);

        return 0;
    }

    iic_drive->_IIC_stop(iic_drive);

    return 1;
}

static uint8_t _IIC_readBytes(IIC_DEVICE *device, uint8_t *data, uint16_t len)
{
    if(device->iic_drive == NULL) return 1;

    _SOFT_IIC_DRIVE *iic_drive = device->iic_drive;

    for( ; ; )
    {
        iic_drive->_IIC_start(iic_drive);
        iic_drive->_IIC_send_byte(iic_drive, (device->device_address<<1) | IIC_R);
        if(iic_drive->_IIC_wait_ack(iic_drive)) break;

        uint8_t i=0;
        if(len > 1)
        {
            for( ; i<len-1; i++)
            {
                data[i] = iic_drive->_IIC_read_byte(iic_drive, IIC_ACK);
                iic_drive->_IIC_ack(iic_drive);
            }
        }
        data[i] = iic_drive->_IIC_read_byte(iic_drive, IIC_NACK);
        iic_drive->_IIC_nack(iic_drive);
        iic_drive->_IIC_stop(iic_drive);
        return 0;
    }

    iic_drive->_IIC_stop(iic_drive);
    return 1;
}

static uint8_t _IIC_writeReg(IIC_DEVICE *device, uint8_t reg, uint8_t *data, uint16_t len)
{
    if(device->iic_drive == NULL) return 1;

    _SOFT_IIC_DRIVE *iic_drive = device->iic_drive;

    for( ; ; )
    {
        iic_drive->_IIC_start(iic_drive);
        iic_drive->_IIC_send_byte(iic_drive, (device->device_address<<1) | IIC_W);
        if(iic_drive->_IIC_wait_ack(iic_drive)) break;
        
        iic_drive->_IIC_send_byte(iic_drive, reg);
        if(iic_drive->_IIC_wait_ack(iic_drive)) break;
        
        for(uint8_t i=0; i<len; i++)
        {
            iic_drive->_IIC_send_byte(iic_drive, data[i]);
            if(iic_drive->_IIC_wait_ack(iic_drive)) break;
        }
        iic_drive->_IIC_stop(iic_drive);

        return 0;
    }

    iic_drive->_IIC_stop(iic_drive);

    return 1;
}


static uint8_t _IIC_readReg(IIC_DEVICE *device, uint8_t reg, uint8_t *data, uint16_t len)
{
    if(device->iic_drive == NULL) return 1;

    _SOFT_IIC_DRIVE *iic_drive = device->iic_drive;

    for( ; ; )
    {
        iic_drive->_IIC_start(iic_drive);
        iic_drive->_IIC_send_byte(iic_drive, (device->device_address<<1) | IIC_W);
        if(iic_drive->_IIC_wait_ack(iic_drive)) break;
        
        iic_drive->_IIC_send_byte(iic_drive, reg);
        if(iic_drive->_IIC_wait_ack(iic_drive)) break;
        
        iic_drive->_IIC_start(iic_drive);
        iic_drive->_IIC_send_byte(iic_drive, (device->device_address<<1) | IIC_R);
        if(iic_drive->_IIC_wait_ack(iic_drive)) break;
        
        uint8_t i=0;
        if(len > 1)
        {
            for( ; i<len-1; i++)
            {
                data[i] = iic_drive->_IIC_read_byte(iic_drive, IIC_ACK);
                iic_drive->_IIC_ack(iic_drive);
            }
        }
        data[i] = iic_drive->_IIC_read_byte(iic_drive, IIC_NACK);
        iic_drive->_IIC_nack(iic_drive);
        iic_drive->_IIC_stop(iic_drive);
        
        return 0;
    }

    iic_drive->_IIC_stop(iic_drive);

    return 1;
}



/*************************************************************
 * @description: 检查设备是否在线
 * @param {IIC_DEVICE} *iic_device
 * @return {*}
 *************************************************************/
static uint8_t IIC_Device_isOnline(IIC_DEVICE *iic_device)
{
    uint8_t ack = 0;

    if(iic_device->iic_drive == NULL) return !IIC_DEVICE_ONLINE;

    iic_device->iic_drive->_IIC_start(iic_device->iic_drive);
    iic_device->iic_drive->_IIC_send_byte(iic_device->iic_drive, (iic_device->device_address<<1) | IIC_R);
    ack = iic_device->iic_drive->_IIC_wait_ack(iic_device->iic_drive);
    iic_device->iic_drive->_IIC_stop(iic_device->iic_drive);

    return ack==IIC_ACK ? IIC_DEVICE_ONLINE : !IIC_DEVICE_ONLINE;
}

/**
 * [ _SOFT_IIC_DRIVE _IIC_Drive_init = .. ]
 * _SOFT_IIC_DRIVE _IIC_SDA_input = ..
 * 
 *          ......
 * 
 * _SOFT_IIC_DRIVE _IIC_SDA_read = ..
 * 
 * 
 * IIC_Drive_Add (iic_drive)
 * 
*/

void IIC_Drive_Add(IIC_DEVICE *iic_device)
{
    if(IIC_DeviveList_Hraed == NULL) {
        IIC_DeviveList_Hraed = iic_device;
    }
    else {
        IIC_DeviveList_Tail->next = iic_device;
    }
    IIC_DeviveList_Tail = iic_device;

    if(iic_device->iic_drive)
    {
        _SOFT_IIC_DRIVE *iic_drive = iic_device->iic_drive;
        
        if(iic_drive->_IIC_Drive_init) iic_drive->_IIC_Drive_init();

        iic_drive->_IIC_start = _IIC_Start;
        iic_drive->_IIC_stop = _IIC_Stop;
        iic_drive->_IIC_nack = _IIC_NAck;
        iic_drive->_IIC_ack = _IIC_Ack;
        iic_drive->_IIC_wait_ack = _IIC_WaitAck;
        iic_drive->_IIC_send_byte = _IIC_SendByte;
        iic_drive->_IIC_read_byte = _IIC_ReadByte;
    }

    if(iic_device->Device_IIC_init)  {
        iic_device->Device_IIC_init();
    }
    iic_device->Device_IIC_writeBytes =  _IIC_writeBytes;
    iic_device->Device_IIC_readBytes =  _IIC_readBytes;
    iic_device->Devive_IIC_writeReg =  _IIC_writeReg;
    iic_device->Devive_IIC_readReg =  _IIC_readReg;
    iic_device->Device_isOnline = IIC_Device_isOnline;
    iic_device->next = NULL;
}




/*************************************************************
 * @description: 获取iic设备的数量
 * @return {*}
 *************************************************************/
uint8_t IIC_Get_Device_Num(void)
{
    if(IIC_DeviveList_Hraed == NULL) return 0;
    IIC_DEVICE *iic_device = IIC_DeviveList_Hraed;
    uint8_t num = 0;

    for( ; num<255; )
    {
        num++;       
        if(iic_device->next == NULL) break;
        iic_device = iic_device->next;
        
    }
    return num;
}

/*************************************************************
 * @description: 根据插入顺序，获取iic设备
 * @param {uint8_t} id
 * @return {*}
 *************************************************************/
IIC_DEVICE *IIC_Get_Device(uint8_t id)
{
    if(IIC_DeviveList_Hraed == NULL) return NULL;

    IIC_DEVICE *iic_device = IIC_DeviveList_Hraed;
    for(int i=0; i<id; i++)
    {
        if(iic_device->next == NULL) return NULL;
        iic_device = iic_device->next;
    }
    return iic_device;
}



