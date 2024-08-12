/*************************************************************
 * @LastEditors: 蔡雅超
 * @Date: 2024-05-12 09:21:55
 * @LastEditTime: 2024-06-04 15:42:12
 *************************************************************/

#include "Device_IIC.h"




static void _IIC_Start(_SOFT_IIC_DRIVE *iic_drive)
{

    iic_drive->_IIC_SDA_output(iic_drive);
    iic_drive->_IIC_SDA_write(iic_drive, 1);
    iic_drive->_IIC_SCL_write(iic_drive, 1);
    iic_drive->_IIC_Delay(iic_drive);
    iic_drive->_IIC_SDA_write(iic_drive, 0);
    iic_drive->_IIC_Delay(iic_drive);
    iic_drive->_IIC_SCL_write(iic_drive, 0);
}

static void _IIC_Stop(_SOFT_IIC_DRIVE *iic_drive)
{
    iic_drive->_IIC_SDA_output(iic_drive);
    iic_drive->_IIC_SDA_write(iic_drive, 0);
    iic_drive->_IIC_SCL_write(iic_drive, 1);
    iic_drive->_IIC_Delay(iic_drive);
    iic_drive->_IIC_SDA_write(iic_drive, 1);
    iic_drive->_IIC_Delay(iic_drive);
}

// 
static void _IIC_Ack(_SOFT_IIC_DRIVE *iic_drive)
{
    iic_drive->_IIC_SDA_output(iic_drive);

    iic_drive->_IIC_SDA_write(iic_drive, IIC_ACK);
    iic_drive->_IIC_SCL_write(iic_drive, 1);
    iic_drive->_IIC_Delay(iic_drive);
    iic_drive->_IIC_SCL_write(iic_drive, 0);
    iic_drive->_IIC_Delay(iic_drive);

    // iic_drive->_IIC_SCL_write(iic_drive, 0);
    // iic_drive->_IIC_SDA_output(iic_drive);
    // iic_drive->_IIC_SDA_write(iic_drive, 0);
    // iic_drive->_IIC_Delay(iic_drive);
    // iic_drive->_IIC_SCL_write(iic_drive, 1);
    // iic_drive->_IIC_Delay(iic_drive);
    // iic_drive->_IIC_SCL_write(iic_drive, 0);
}

static void _IIC_NAck(_SOFT_IIC_DRIVE *iic_drive)
{
    iic_drive->_IIC_SDA_output(iic_drive);

    iic_drive->_IIC_SDA_write(iic_drive, IIC_NACK);
    iic_drive->_IIC_SCL_write(iic_drive, 1);
    iic_drive->_IIC_Delay(iic_drive);
    iic_drive->_IIC_SCL_write(iic_drive, 0);
    iic_drive->_IIC_Delay(iic_drive);

    // iic_drive->_IIC_SCL_write(iic_drive, 0);
    // iic_drive->_IIC_SDA_output(iic_drive);
    // iic_drive->_IIC_SDA_write(iic_drive, 1);
    // iic_drive->_IIC_Delay(iic_drive);
    // iic_drive->_IIC_SCL_write(iic_drive, 1);
    // iic_drive->_IIC_Delay(iic_drive);
    // iic_drive->_IIC_SCL_write(iic_drive, 0);
}

static uint8_t _IIC_WaitAck(_SOFT_IIC_DRIVE *iic_drive)
{
    uint8_t ack;
    iic_drive->_IIC_SDA_input(iic_drive);
    iic_drive->_IIC_SCL_write(iic_drive, 1);
    iic_drive->_IIC_Delay(iic_drive);
    ack = iic_drive->_IIC_SDA_read(iic_drive);
    iic_drive->_IIC_SCL_write(iic_drive, 0);
    iic_drive->_IIC_Delay(iic_drive);
    return ack;
}

static void _IIC_SendByte(_SOFT_IIC_DRIVE *iic_drive, uint8_t byte)
{
    iic_drive->_IIC_SDA_output(iic_drive);

    for(uint8_t i=0; i<8; i++)
    {
        iic_drive->_IIC_SDA_write(iic_drive, byte >> 7);
        byte <<= 1;
        iic_drive->_IIC_SCL_write(iic_drive, 1);
        iic_drive->_IIC_Delay(iic_drive);
        iic_drive->_IIC_SCL_write(iic_drive, 0);
        iic_drive->_IIC_Delay(iic_drive);
    }
}

static uint8_t _IIC_ReadByte(_SOFT_IIC_DRIVE *iic_drive, uint8_t ack)
{
    uint8_t byte = 0;

    iic_drive->_IIC_SDA_write(iic_drive, 1);
    iic_drive->_IIC_SDA_input(iic_drive);
    
    for(uint8_t i=0; i<8; i++)
    {
        byte <<= 1;
        iic_drive->_IIC_SCL_write(iic_drive, 1);
        iic_drive->_IIC_Delay(iic_drive);
        if(iic_drive->_IIC_SDA_read(iic_drive))
        {
            byte |= 0x01;
        }
        iic_drive->_IIC_SCL_write(iic_drive, 0);
        iic_drive->_IIC_Delay(iic_drive);
    }
    if(ack == IIC_ACK) iic_drive->_IIC_ack(iic_drive);
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
            }
        }
        data[i] = iic_drive->_IIC_read_byte(iic_drive, IIC_NACK);
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
            }
        }
        data[i] = iic_drive->_IIC_read_byte(iic_drive, IIC_NACK);
        iic_drive->_IIC_stop(iic_drive);
        
        return 0;
    }

    iic_drive->_IIC_stop(iic_drive);

    return 1;
}


/*************************************************************
 * @description: 扫描得到一个设备最低地址的设备addr
 * @param {IIC_DEVICE} *iic_device
 * @return {uint8_t} 0xff 无设备，其它设备地址
 *************************************************************/
static uint8_t scan_DevLowAddr(IIC_DEVICE *iic_device)
{
    if(iic_device->iic_drive == NULL) return 0xff;
    
    _SOFT_IIC_DRIVE *iic_drive = iic_device->iic_drive;
    
    uint8_t device_id = 0xff;
    for(uint8_t i=0; i<=0x7f; i++)
    {
        iic_drive->_IIC_start(iic_drive);
        iic_drive->_IIC_send_byte(iic_drive, (i<<1) | IIC_R);
        if(iic_drive->_IIC_wait_ack(iic_drive) == IIC_ACK) 
        {
            iic_drive->_IIC_stop(iic_drive);
            device_id = i;
            break;
        }
        iic_drive->_IIC_stop(iic_drive);
    }
    
    return device_id;
}

/*************************************************************
 * @description: 更新设备最低地址的设备addr
 * @param {IIC_DEVICE} *iic_device
 * @return {*} 0:成功，1:失败
 *************************************************************/
static uint8_t update_DevLowAddr(IIC_DEVICE *iic_device)
{
    uint8_t addr = iic_device->scan_DevLowAddr(iic_device);
    if(addr == 0xff) return 1;
    
    iic_device->device_address = addr;
    
    return 0;
}

/*************************************************************
 * @description: 根据设备地址列表更新设备地址
 * @param {IIC_DEVICE} *iic_device
 * @param {uint8_t} *addr_list 地址列表
 * @param {uint8_t} len 地址列表长度
 * @return {*} 0:成功，1:失败
 *************************************************************/
static uint8_t update_DevAddrFromDevList(IIC_DEVICE *iic_device, const uint8_t *addr_list, uint8_t len)
{
    if(iic_device->iic_drive == NULL) return 1;
    
    _SOFT_IIC_DRIVE *iic_drive = iic_device->iic_drive;

    for(uint8_t i=0; i<len; i++)
    {
        iic_drive->_IIC_start(iic_drive);
        iic_drive->_IIC_send_byte(iic_drive, (addr_list[i]<<1) | IIC_R);
        if(iic_drive->_IIC_wait_ack(iic_drive) == IIC_ACK)
        {
            iic_drive->_IIC_stop(iic_drive);
            iic_device->device_address = addr_list[i];
            return 0;
        }
        iic_drive->_IIC_stop(iic_drive);
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

void IIC_Drive_Add(IIC_DEVICE_LIST *IIC_Device_List, IIC_DEVICE *iic_device)
{
    if(IIC_Device_List->iic_device_header == NULL) {
        IIC_Device_List->iic_device_header = iic_device;
    }
    else {
        IIC_Device_List->iic_device_tail->next = iic_device;
    }
    IIC_Device_List->iic_device_tail = iic_device;

    if(iic_device->iic_drive)
    {
        _SOFT_IIC_DRIVE *iic_drive = iic_device->iic_drive;
        
        if(iic_drive->_IIC_Drive_init) iic_drive->_IIC_Drive_init(iic_drive);

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
    iic_device->scan_DevLowAddr = scan_DevLowAddr;
    iic_device->update_DevLowAddr = update_DevLowAddr;
    iic_device->update_DevAddrFromDevList = update_DevAddrFromDevList;

    iic_device->next = NULL;
}




/*************************************************************
 * @description: 获取iic设备的数量
 * @return {*}
 *************************************************************/
uint8_t IIC_Get_DeviceList_Num(IIC_DEVICE_LIST *IIC_Device_List)
{
    if(IIC_Device_List->iic_device_header == NULL) return 0;
    IIC_DEVICE *iic_device = IIC_Device_List->iic_device_header ;
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
IIC_DEVICE *IIC_Get_DeviceInList(IIC_DEVICE_LIST *IIC_Device_List, uint8_t id)
{
    if(IIC_Device_List->iic_device_header == NULL) return NULL;

    IIC_DEVICE *iic_device = IIC_Device_List->iic_device_header ;
    for(int i=0; i<id; i++)
    {
        if(iic_device->next == NULL) return NULL;
        iic_device = iic_device->next;
    }
    return iic_device;
}



