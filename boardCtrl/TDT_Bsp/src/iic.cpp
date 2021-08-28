/******************************
File name: TDT_BSP\src\iic.cpp
Description: IIC

Author: 彭阳
Version: 1.3.1.191119_alpha
Date: 20.11.11
History:
	——————————————————————————————————————————————————————————————————————————
	20.11.11 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "iic.h"

/**
  * @brief 计算最高位
  * @param [16位数据]
  */
static uint32_t judge_BitSite(uint16_t port)
{
    return port == 0x00 ? 0x00 : judge_BitSite(port >> 1) + 1;
}



/**
  * @brief IIC读取寄存器单字节数据
  * @param [从机地址，寄存器地址]
  */
uint8_t IIC::iicRegReadByte(uint8_t SlaveAddress, uint8_t REG_Address)
{
    unsigned char REG_data;
    if (!iicRegReadData(SlaveAddress, REG_Address, &REG_data, 1))
    {
        return 0;
    }
    return REG_data;
}



/**
  * @brief IIC读取寄存器多字节数据
  * @param [从机地址，寄存器地址，数据存储地址，数据大小]
  */
uint8_t IIC::iicRegReadData(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *ptChar, uint8_t size)
{
    uint8_t I2C_Err = 0;

	//检查传入数据地址和大小
    if (size < 1 || ptChar == NULL)
    {
        return 0;
    }
    //检测总线忙，若总线忙可能处于硬件IIC总线死锁状态，重新初始化复位
    if (iicCheckBusy())
    {
        iicInit();
        return 0;
    }

    I2C_Err |= iicStart();						//发送起始信号
    I2C_Err |= iicWAddr(SlaveAddress);			//发送从机地址+W
    I2C_Err |= iicSendByte(REG_Address);		//发送寄存器地址
    I2C_Err |= iicStart();						//发送起始信号
    I2C_Err |= iicRAddr(SlaveAddress);			//发送从机地址+R
    //循环读取
    while (--size)
    {
        I2C_Err |= iicReceiveDataByte_Ack(ptChar++);	//读取一个字节并发送应答位
    }
    I2C_Err |= iicReceiveDataByte_NoAck(ptChar++);	//最后一次数据不发送应答位
    iicStop();									//发送STOP信号
    
    return !I2C_Err;
}



/**
  * @brief IIC写入寄存器单字节数据
  * @param [从机地址，寄存器地址，需要写入的数据]
  */
uint8_t IIC::iicRegWriteByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)
{
    if (!iicRegWriteData(SlaveAddress, REG_Address, &REG_data, 1))
    {
        return 0;
    }
    return 1;
}



/**
  * @brief IIC写入寄存器多字节数据
  * @param [从机地址，寄存器地址，数据存储地址，数据大小]
  */
uint8_t IIC::iicRegWriteData(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *ptChar, uint8_t size)
{
    uint8_t I2C_Err = 0;
	
	//检查传入数据地址和大小
    if (size < 1 || ptChar == NULL)
    {
        return 0;
    }
   
    I2C_Err |= iicCheckBusy();					//检查总线是否被占用
    I2C_Err |= iicStart();						//发送起始信号
    I2C_Err |= iicWAddr(SlaveAddress);			//发送从机地址+W
    I2C_Err |= iicSendByte(REG_Address);		//发送寄存器地址
	//循环写入
    while (size--)
    {
        I2C_Err |= iicSendByte(*(ptChar++));	//发送一个字节并检查应答位
    }
    iicStop();									//发送停止位

    return !I2C_Err;
}



/**
  * @brief IIC读取单字节数据
  * @param [主机地址]
  */
uint8_t IIC::iicReceiveDataByte(uint8_t SlaveAddress)
{
    unsigned char REG_data;
    if (!iicReceiveData(SlaveAddress, &REG_data, 1))
    {
        return 0;
    }
    return REG_data;
}



/**
  * @brief IIC读取多字节数据
  * @param [从机地址，数据存储地址，数据大小]
  */
uint8_t IIC::iicReceiveData(uint8_t SlaveAddress, uint8_t *ptChar, uint8_t size)
{
    uint8_t I2C_Err = 0;

	//检查传入数据地址和大小
    if (size < 1 || ptChar == NULL)
    {
        return 0;
    }
	
    //检测总线忙，若总线忙可能处于硬件IIC总线死锁状态，重新初始化复位
    if (iicCheckBusy())
    {
        iicInit();
        return 0;
    }
    
    I2C_Err |= iicStart();						//发送起始信号
    I2C_Err |= iicRAddr(SlaveAddress);			//发送从机地址+R
    while (--size)
    {
        I2C_Err |= iicReceiveDataByte_Ack(ptChar++);	//读取一个字节并发送应答位
    }
    I2C_Err |= iicReceiveDataByte_NoAck(ptChar++);	//最后一次数据不发送应答位
    iicStop();									//发送停止位

    return !I2C_Err;
}



/**
  * @brief IIC发送单字节数据
  * @param [从机地址，需写入的数据]
  */
uint8_t IIC::iicSendByte(uint8_t SlaveAddress, uint8_t REG_data)
{
    if (!iicSendData(SlaveAddress, &REG_data, 1))
    {
        return 0;
    }
    return 1;
}



/**
  * @brief IIC发送多字节数据
  * @param [从机地址，数据存储地址，数据大小]
  */
uint8_t IIC::iicSendData(uint8_t SlaveAddress, uint8_t *ptChar, uint8_t size)
{
    uint8_t I2C_Err = 0;
	
	//检查传入数据地址和大小
    if (size < 1 || ptChar == NULL)
    {
        return 0;
    }


    I2C_Err |= iicCheckBusy();					//检查总线是否被占用
    I2C_Err |= iicStart();						//发送起始信号
    I2C_Err |= iicWAddr(SlaveAddress);			//发送从机地址+W

    while (--size)
    {
        I2C_Err |= iicSendByte(*ptChar++);		//发送一个字节并检查应答位
    }
    iicStop();									//发送停止位
    
    return !I2C_Err;
}



/**
  * @brief 硬件iic构造函数
  * @param [SCL端口，SCL引脚，SDA引脚，IIC外设口，*IIC通信频率，*SDA端口，*超时时间]
  */
Hardiic::Hardiic(GPIO_TypeDef *iicSclPort, uint16_t iicSclPin, uint16_t iicSdaPin,
                 I2C_TypeDef *IIC, uint32_t iicFrequency,
                 GPIO_TypeDef *iicSdaPort, uint32_t ulTimeOut_Time)
{
    this->iic = IIC;

    this->iicSclPort = iicSclPort;
    this->iicSclRccAHB1Periph = ((0x01 << (((judge_BitSite((uint32_t) iicSclPort - AHB1PERIPH_BASE)) >> 2))) >> 1);

    if (iicSdaPort == 0)
    {
        this->iicSdaPort = iicSclPort;
        this->iicSdaRccAHB1Periph = iicSclRccAHB1Periph;
    } else
    {
        this->iicSdaPort = iicSdaPort;
        this->iicSdaRccAHB1Periph = ((0x01 << (((judge_BitSite((uint32_t) iicSdaPort - AHB1PERIPH_BASE)) >> 2)))>> 1);
    }

    this->iicSclPin = iicSclPin;
    this->iicSdaPin = iicSdaPin;

    if (iicFrequency > 400000)
    {
        this->iicFrequency = 400000;
    } else
    {
        this->iicFrequency = iicFrequency;
    }

    if (ulTimeOut_Time == 0)
    {
        RCC_ClocksTypeDef rcc_clocks;
        RCC_GetClocksFreq(&rcc_clocks);
        this->ulTimeOut_Time = (rcc_clocks.SYSCLK_Frequency / 10000);
    } else
    {
        this->ulTimeOut_Time = ulTimeOut_Time;
    }

}



/**
  * @brief 硬件iic初始化函数
  * @note  硬件iic也可以使用该函数进行复位并解除总线死锁
  */
void Hardiic::iicInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
	
	//使能时钟
    RCC_AHB1PeriphClockCmd(iicSclRccAHB1Periph | iicSdaRccAHB1Periph, ENABLE);
    RCC_APB1PeriphClockCmd((((uint32_t) iic - APB1PERIPH_BASE) & 0x0F00) << 11, ENABLE);//计算IIC时钟地址

    /* 使用软件强制发出九个时钟脉冲解除IIC总线死锁 */
    GPIO_InitStructure.GPIO_Pin = iicSclPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//配置为推挽可以将时钟线强制拉高
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(iicSclPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = iicSdaPin;
    GPIO_Init(iicSdaPort, &GPIO_InitStructure);

    iicSdaPort->BSRRL = iicSdaPin;

	//发出9个时钟脉冲
    for (uint8_t i = 9; i == 0; i--)
    {
        iicSclPort->BSRRL = iicSclPin;
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        iicSclPort->BSRRH = iicSclPin;
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
    }
	
	//配置外设复用
    GPIO_PinAFConfig(iicSclPort, judge_BitSite(iicSclPin) - 1,((uint8_t) 0x04));/*注意，此处不能合并写成GPIO_PinSource6|GPIO_PinSource7*/
    GPIO_PinAFConfig(iicSdaPort, judge_BitSite(iicSdaPin) - 1,((uint8_t) 0x04));/*((uint8_t)0x04)为GPIO_AF_I2C通用值，为避免误解写成这种形式*/

	//配置IIC引脚
    GPIO_InitStructure.GPIO_Pin = iicSclPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			//正常IIC使用开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(iicSclPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = iicSdaPin;
    GPIO_Init(iicSdaPort, &GPIO_InitStructure);

	//对IIC进行复位
    I2C_SoftwareResetCmd(iic, ENABLE);
    I2C_SoftwareResetCmd(iic, DISABLE);

	//IIC配置
    I2C_DeInit(iic);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_ClockSpeed = iicFrequency;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(iic, &I2C_InitStructure);

	//使能IIC
    I2C_Cmd(iic, ENABLE);
}



/**
  * @brief 硬件iic检查总线忙
  * @note  硬件iic若返回1则可能是总线死锁，读取时发现总线忙则重启iic
  */
uint8_t Hardiic::iicCheckBusy(void)
{
    uint32_t tmr = ulTimeOut_Time;
    while ((--tmr) && I2C_GetFlagStatus(iic, I2C_FLAG_BUSY));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief 硬件iic发出起始信号
  */
uint8_t Hardiic::iicStart(void)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_GenerateSTART(iic, ENABLE);
    while ((--tmr) && (!(I2C_CheckEvent(iic, I2C_EVENT_MASTER_MODE_SELECT))));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief 硬件iic发出停止信号
  */
void Hardiic::iicStop(void)
{
    I2C_GenerateSTOP(iic, ENABLE);
}



/**
  * @brief 硬件iic读取一个字节数据并产生应答位
  * @param [即将读取字节的存储地址]
  */
uint8_t Hardiic::iicReceiveDataByte_Ack(uint8_t *ptChar)
{
    uint32_t tmr = ulTimeOut_Time;
    while ((--tmr) && (!(I2C_CheckEvent(iic, I2C_EVENT_MASTER_BYTE_RECEIVED))));
    *ptChar = I2C_ReceiveData(iic);
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief 硬件iic读取一个字节数据不产生应答位
  * @param [即将读取字节的存储地址]
  */
uint8_t Hardiic::iicReceiveDataByte_NoAck(uint8_t *ptChar)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_AcknowledgeConfig(iic, DISABLE);
    while ((--tmr) && (!(I2C_CheckEvent(iic, I2C_EVENT_MASTER_BYTE_RECEIVED))));
    *ptChar = I2C_ReceiveData(iic);
    I2C_AcknowledgeConfig(iic, ENABLE);
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief 硬件iic发送一个字节数据并检查应答位
  * @param [要发送的数据]
  */
uint8_t Hardiic::iicSendByte(uint8_t SendByte)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_SendData(iic, SendByte);
    while ((--tmr) && (!I2C_CheckEvent(iic, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief 硬件iic发送从机地址并添加写入标志位
  * @param [未左移一位的从机地址]
  */
uint8_t Hardiic::iicWAddr(uint8_t SlaveAddress)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_Send7bitAddress(iic, (SlaveAddress << 1), I2C_Direction_Transmitter); //注意该函数不会移位，要自行左移
    while ((--tmr) && (!I2C_CheckEvent(iic, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief 硬件iic发送从机地址并添加读取标志位
  * @param [未左移一位的从机地址]
  */
uint8_t Hardiic::iicRAddr(uint8_t SlaveAddress)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_Send7bitAddress(iic, (SlaveAddress << 1), I2C_Direction_Receiver);
    while ((--tmr) && (!I2C_CheckEvent(iic, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief 软件iic构造函数
  * @param [SCL端口，SCL引脚，SDA引脚，(占位参数)IIC外设口，*IIC通信频率，*SDA端口，(占位参数)超时时间]
  */
Softiic::Softiic(GPIO_TypeDef *iicSclPort, uint16_t iicSclPin, uint16_t iicSdaPin, I2C_TypeDef *IIC,
                 uint32_t iicFrequency,GPIO_TypeDef *iicSdaPort, uint32_t)
{
    this->iicSclPort = iicSclPort;
    this->iicSclRccAHB1Periph = ((0x01 << (((judge_BitSite((uint32_t) iicSclPort - AHB1PERIPH_BASE)) >> 2))) >> 1);

    if (iicSdaPort == 0)
    {
        this->iicSdaPort = iicSclPort;
        this->iicSdaRccAHB1Periph = iicSclRccAHB1Periph;
    } else
    {
        this->iicSdaPort = iicSdaPort;
        this->iicSdaRccAHB1Periph = ((0x01 << (((judge_BitSite((uint32_t) iicSdaPort - AHB1PERIPH_BASE)) >> 2)))>> 1);
    }

    this->iicSclPin = iicSclPin;
    this->iicSdaPin = iicSdaPin;

    if (iicFrequency > 1100000)
    {
        this->iicFrequency = 0;
    } else if (iicFrequency < 10000)
    {
        this->iicFrequency = 922;
    } else
    {
        this->iicFrequency = (9293320.0f / iicFrequency) - 7.24f;	//此公式根据硬件iic进行计算，保证通信频率相同时通信时间一致
    }
}



/**
  * @brief 软件iic初始化函数
  * @note  软件iic总线不会死锁
  */
void Softiic::iicInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(iicSclRccAHB1Periph | iicSdaRccAHB1Periph, ENABLE);

    GPIO_InitStructure.GPIO_Pin = iicSclPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(iicSclPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = iicSdaPin;
    GPIO_Init(iicSdaPort, &GPIO_InitStructure);
}



/**
  * @brief 内联函数 SCL总线拉高
  */
void Softiic::sclH(void)
{ iicSclPort->BSRRL = iicSclPin; }



/**
  * @brief 内联函数 SCL总线拉低
  */
void Softiic::sclL(void)
{ iicSclPort->BSRRH = iicSclPin; }



/**
  * @brief 内联函数 SDA总线拉高
  */
void Softiic::sdaH(void)
{ iicSdaPort->BSRRL = iicSdaPin; }



/**
  * @brief 内联函数 SDA总线拉低
  */
void Softiic::sdaL(void)
{ iicSdaPort->BSRRH = iicSdaPin; }



/**
  * @brief 内联函数 读取SDA总线电平
  */
uint32_t Softiic::sdaRead(void)
{ return (iicSdaPort->IDR & iicSdaPin); }



/**
  * @brief 等待总线电平跳变，决定软件IIC通信频率
  */
void Softiic::iicSoftDelay(void)
{
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();

    uint16_t i = iicFrequency;
    while (i--)
    {
        __nop();
    }

}



/**
  * @brief 软件iic检查总线忙
  * @note  软件iic若返回1则可能是总线死锁，读取时发现总线忙则重启iic
  */
uint8_t Softiic::iicCheckBusy(void)
{
    sdaH();
    sclH();
    iicSoftDelay();
    if (!sdaRead())
    {
        return 1;
    }else
    {
        return 0;
    }
}



/**
  * @brief 软件iic发出起始信号
  */
uint8_t Softiic::iicStart(void)
{
    sclH();
    iicSoftDelay();
    iicSoftDelay();
    iicSoftDelay();
    sdaL();
    iicSoftDelay();
    if (sdaRead())
    {
        return 1;
    }else
    {
        return 0;
    }
}



/**
  * @brief 软件iic发出停止信号
  */
void Softiic::iicStop(void)
{
    sclL();
    iicSoftDelay();
    sdaL();
    iicSoftDelay();
    sclH();
    iicSoftDelay();
    sdaH();
    iicSoftDelay();
}



/**
  * @brief 软件iic读取一个字节数据并产生应答位
  * @param [即将读取字节的存储地址]
  */
uint8_t Softiic::iicReceiveDataByte_Ack(uint8_t *ptChar)
{
    uint8_t i = 8;
    uint8_t ReceiveByte = 0;
    sdaH();
    while (i--)
    {
        ReceiveByte <<= 1;
        sclL();
        iicSoftDelay();
        sclH();
        iicSoftDelay();
        if (sdaRead())
        {
            ReceiveByte |= 0x01;
        }
    }
    sclL();
    *ptChar = ReceiveByte;
    sclL();
    iicSoftDelay();
    sdaL();
    iicSoftDelay();
    sclH();
    iicSoftDelay();
    sclL();
    iicSoftDelay();
    return 0;
}



/**
  * @brief 软件iic读取一个字节数据不产生应答位
  * @param [即将读取字节的存储地址]
  */
uint8_t Softiic::iicReceiveDataByte_NoAck(uint8_t *ptChar)
{
    uint8_t i = 8;
    uint8_t ReceiveByte = 0;
    sdaH();
    while (i--)
    {
        ReceiveByte <<= 1;
        sclL();
        iicSoftDelay();
        sclH();
        iicSoftDelay();
        if (sdaRead())
        {
            ReceiveByte |= 0x01;
        }
    }
    sclL();
    *ptChar = ReceiveByte;

    sclL();
    iicSoftDelay();
    sdaH();
    iicSoftDelay();
    sclH();
    iicSoftDelay();
    sclL();
    iicSoftDelay();
    return 0;
}



/**
  * @brief 软件iic发送一个字节数据并检查应答位
  * @param [要发送的数据]
  */
uint8_t Softiic::iicSendByte(uint8_t SendByte)
{
    uint8_t i = 8;
    while (i--)
    {
        sclL();
        iicSoftDelay();
        if (SendByte & 0x80)
        {
            sdaH();
        } else
        {
            sdaL();
        }
        SendByte <<= 1;
        iicSoftDelay();
        sclH();
        iicSoftDelay();
    }
    sclL();
    iicSoftDelay();
    sdaH();
    iicSoftDelay();
    sclH();
    iicSoftDelay();
    if (sdaRead())
    {
        sclL();
        iicSoftDelay();
        return 1;
    }
    sclL();
    iicSoftDelay();
    return 0;
}



/**
  * @brief 软件iic发送从机地址并添加写入标志位
  * @param [未左移一位的从机地址]
  */
uint8_t Softiic::iicWAddr(uint8_t SlaveAddress)
{
    return iicSendByte((uint8_t)(((SlaveAddress << 1) & 0xFE) | 0x00));
}



/**
  * @brief 软件iic发送从机地址并添加读取标志位
  * @param [未左移一位的从机地址]
  */
uint8_t Softiic::iicRAddr(uint8_t SlaveAddress)
{
    return iicSendByte((uint8_t)(((SlaveAddress << 1) & 0xFE) | 0x01));
}
