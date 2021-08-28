/******************************
File name: TDT_BSP\inc\iic.h
Description: IIC

Author: 彭阳
Version: 1.3.1.191119_alpha
Date: 20.11.11
History:
	——————————————————————————————————————————————————————————————————————————
	20.11.11 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#ifndef __IIC_H__
#define __IIC_H__

#include "board.h"
#include "stm32f4xx_i2c.h"

/*************************************************
  * @class IIC基类
  * @brief 提供硬件IIC与软件IIC的相同操作
  * @note
*************************************************/
class IIC
{
private:
	
	/*
	基本操作全部为虚函数，通过Softiic与Hardiic进行重载
	返回0为未产生错误，返回1为产生错误
	*/
    
	virtual uint8_t iicCheckBusy(void)=0;					//检查总线忙

    virtual uint8_t iicStart(void)=0;						//产生起始位

    virtual void iicStop(void)=0;										//产生停止位

    virtual uint8_t iicReceiveDataByte_Ack(uint8_t *ptChar)=0;		//读取一个字节并产生应答位

    virtual uint8_t iicReceiveDataByte_NoAck(uint8_t *ptChar)=0;	//读取一个字节不产生应答位

    virtual uint8_t iicSendByte(uint8_t SendByte)=0;		//发送一个字节并检查应答位

    virtual uint8_t iicWAddr(uint8_t SlaveAddress)=0;		//发送从机地址+W标志位

    virtual uint8_t iicRAddr(uint8_t SlaveAddress)=0;		//发送从机地址+R标志位
	
protected:
    /*读写从机寄存器*/

    //多字节读取寄存器
    uint8_t iicRegReadData(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *ptChar, uint8_t size);

    //单字节读取寄存器
    uint8_t iicRegReadByte(uint8_t SlaveAddress, uint8_t REG_Address);

    //多字节写入寄存器
    uint8_t iicRegWriteData(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *ptChar, uint8_t size);

    //单字节写入寄存器
    uint8_t iicRegWriteByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);

    /*接收数据与发送数据*/

    //多字节接收数据
    uint8_t iicReceiveData(uint8_t SlaveAddress, uint8_t *ptChar, uint8_t size);

    //单字节接收数据
    uint8_t iicReceiveDataByte(uint8_t SlaveAddress);

    //多字节发送数据
    uint8_t iicSendData(uint8_t SlaveAddress, uint8_t *ptChar, uint8_t size);

    //单字节发送数据
    uint8_t iicSendByte(uint8_t SlaveAddress, uint8_t REG_data);

public:
	
	//IIC初始化虚函数，通过Softiic与Hardiic进行重载
    virtual void iicInit(void){};	

};

/*************************************************
  * @class 硬件IIC类
  * @brief 包含硬件iic相关配置与基本操作，具体读写由IIC实现
  * @note
*************************************************/
class Hardiic : public IIC
{

private:
    //硬件IIC配置
    I2C_TypeDef	 *iic;					//IIC端口
    GPIO_TypeDef *iicSclPort;			//SCL端口
    GPIO_TypeDef *iicSdaPort;			//SDA端口
    uint16_t 	 iicSclPin;				//SCL引脚
    uint16_t 	 iicSdaPin;				//SDA引脚
    uint32_t 	 iicSclRccAHB1Periph;	//SCL端口时钟
    uint32_t 	 iicSdaRccAHB1Periph;	//SDA端口时钟

    uint32_t 	 ulTimeOut_Time; 			//硬件iic等待超时时间	
    uint32_t 	 iicFrequency;  			//iic通信时钟频率 

	//IIC基本操作，参考IIC类注释
    uint8_t iicCheckBusy(void)override;

     uint8_t iicStart(void) override;

     void iicStop(void) override;

     uint8_t iicReceiveDataByte_Ack(uint8_t *ptChar) override;

     uint8_t iicReceiveDataByte_NoAck(uint8_t *ptChar) override;

     uint8_t iicSendByte(uint8_t SendByte) override;

     uint8_t iicWAddr(uint8_t SlaveAddress) override;

     uint8_t iicRAddr(uint8_t SlaveAddress) override;

public:
	
	//硬件iic构造函数
    /*SCL端口	SCL引脚		SDA引脚		IIC端口		
	IIC通信频率 单位：Hz  默认400kHz，范围400kHz-1Hz	
	*SDA端口 默认为0，即与SCL相同	*硬件IIC等待总线超时时间 默认为0，即系统时钟频率/10000 */ 
    Hardiic(GPIO_TypeDef *iicSclPort, uint16_t iicSclPin, uint16_t iicSdaPin, I2C_TypeDef *IIC,
            uint32_t iicFrequency = 400000,
            GPIO_TypeDef *iicSdaPort = 0, uint32_t ulTimeOut_Time = 0);
	
	//硬件iic初始化，也可用于解除总线死锁
     void iicInit(void) override;
};

/*************************************************
  * @class 软件IIC类
  * @brief 包含软件iic相关配置与基本操作，具体读写由IIC实现
  * @note
*************************************************/
class Softiic : public IIC
{

private:
    //软件IIC配置
    GPIO_TypeDef *iicSclPort;			//SCL端口
    GPIO_TypeDef *iicSdaPort;			//SDA端口
    uint16_t	 iicSclPin;				//SCL引脚
    uint16_t 	 iicSdaPin;				//SDA引脚
    uint32_t 	 iicSclRccAHB1Periph;	//SCL端口时钟
    uint32_t 	 iicSdaRccAHB1Periph;	//SDA端口时钟

	/*当IIC通信频率过高时某些外设可能不支持*/
    uint32_t 	 iicFrequency;				//iic通信频率 初始输入频率经过计算后会换算为延时时间储存在此变量中

    //软件iic总线基本操作
    inline void sclH(void);			//SCL线拉高

    inline void sclL(void);			//SCL线拉低

    inline void sdaH(void);			//SDA线拉高

    inline void sdaL(void);			//SDA线拉低

    inline uint32_t sdaRead(void);		//读取SDA线电平

    void iicSoftDelay(void);			//软件iic延时

    //IIC基本操作，参考IIC类注释
     uint8_t iicCheckBusy(void) override;

     uint8_t iicStart(void) override;

     void iicStop(void) override;

     uint8_t iicReceiveDataByte_Ack(uint8_t *ptChar) override;

     uint8_t iicReceiveDataByte_NoAck(uint8_t *ptChar) override;

     uint8_t iicSendByte(uint8_t SendByte) override;

     uint8_t iicWAddr(uint8_t SlaveAddress) override;

     uint8_t iicRAddr(uint8_t SlaveAddress) override;

public:

    //软件iic构造函数
    /*SCL端口	SCL引脚		SDA引脚		IIC端口（占位变量，用于硬件iic和软件iic随意切换）		
	IIC通信频率 单位：Hz  默认1MHz，范围1.2MHz-10kHz	
	*SDA端口 默认为0，即与SCL相同	*占位变量 */  
    Softiic(GPIO_TypeDef *iicSclPort, uint16_t iicSclPin, uint16_t iicSdaPin, I2C_TypeDef *IIC = 0,
            uint32_t iicFrequency = 1000000,
            GPIO_TypeDef *iicSdaPort = 0, uint32_t= 0);

    // 软件iic初始化
    void iicInit(void) override;
};

#endif
