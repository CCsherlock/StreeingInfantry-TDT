/******************************
File name: TDT_BSP\inc\iic.h
Description: IIC

Author: ����
Version: 1.3.1.191119_alpha
Date: 20.11.11
History:
	����������������������������������������������������������������������������������������������������������������������������������������������������
	20.11.11 �״����
	����������������������������������������������������������������������������������������������������������������������������������������������������
****************************  */
#ifndef __IIC_H__
#define __IIC_H__

#include "board.h"
#include "stm32f4xx_i2c.h"

/*************************************************
  * @class IIC����
  * @brief �ṩӲ��IIC�����IIC����ͬ����
  * @note
*************************************************/
class IIC
{
private:
	
	/*
	��������ȫ��Ϊ�麯����ͨ��Softiic��Hardiic��������
	����0Ϊδ�������󣬷���1Ϊ��������
	*/
    
	virtual uint8_t iicCheckBusy(void)=0;					//�������æ

    virtual uint8_t iicStart(void)=0;						//������ʼλ

    virtual void iicStop(void)=0;										//����ֹͣλ

    virtual uint8_t iicReceiveDataByte_Ack(uint8_t *ptChar)=0;		//��ȡһ���ֽڲ�����Ӧ��λ

    virtual uint8_t iicReceiveDataByte_NoAck(uint8_t *ptChar)=0;	//��ȡһ���ֽڲ�����Ӧ��λ

    virtual uint8_t iicSendByte(uint8_t SendByte)=0;		//����һ���ֽڲ����Ӧ��λ

    virtual uint8_t iicWAddr(uint8_t SlaveAddress)=0;		//���ʹӻ���ַ+W��־λ

    virtual uint8_t iicRAddr(uint8_t SlaveAddress)=0;		//���ʹӻ���ַ+R��־λ
	
protected:
    /*��д�ӻ��Ĵ���*/

    //���ֽڶ�ȡ�Ĵ���
    uint8_t iicRegReadData(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *ptChar, uint8_t size);

    //���ֽڶ�ȡ�Ĵ���
    uint8_t iicRegReadByte(uint8_t SlaveAddress, uint8_t REG_Address);

    //���ֽ�д��Ĵ���
    uint8_t iicRegWriteData(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *ptChar, uint8_t size);

    //���ֽ�д��Ĵ���
    uint8_t iicRegWriteByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);

    /*���������뷢������*/

    //���ֽڽ�������
    uint8_t iicReceiveData(uint8_t SlaveAddress, uint8_t *ptChar, uint8_t size);

    //���ֽڽ�������
    uint8_t iicReceiveDataByte(uint8_t SlaveAddress);

    //���ֽڷ�������
    uint8_t iicSendData(uint8_t SlaveAddress, uint8_t *ptChar, uint8_t size);

    //���ֽڷ�������
    uint8_t iicSendByte(uint8_t SlaveAddress, uint8_t REG_data);

public:
	
	//IIC��ʼ���麯����ͨ��Softiic��Hardiic��������
    virtual void iicInit(void){};	

};

/*************************************************
  * @class Ӳ��IIC��
  * @brief ����Ӳ��iic�����������������������д��IICʵ��
  * @note
*************************************************/
class Hardiic : public IIC
{

private:
    //Ӳ��IIC����
    I2C_TypeDef	 *iic;					//IIC�˿�
    GPIO_TypeDef *iicSclPort;			//SCL�˿�
    GPIO_TypeDef *iicSdaPort;			//SDA�˿�
    uint16_t 	 iicSclPin;				//SCL����
    uint16_t 	 iicSdaPin;				//SDA����
    uint32_t 	 iicSclRccAHB1Periph;	//SCL�˿�ʱ��
    uint32_t 	 iicSdaRccAHB1Periph;	//SDA�˿�ʱ��

    uint32_t 	 ulTimeOut_Time; 			//Ӳ��iic�ȴ���ʱʱ��	
    uint32_t 	 iicFrequency;  			//iicͨ��ʱ��Ƶ�� 

	//IIC�����������ο�IIC��ע��
    uint8_t iicCheckBusy(void)override;

     uint8_t iicStart(void) override;

     void iicStop(void) override;

     uint8_t iicReceiveDataByte_Ack(uint8_t *ptChar) override;

     uint8_t iicReceiveDataByte_NoAck(uint8_t *ptChar) override;

     uint8_t iicSendByte(uint8_t SendByte) override;

     uint8_t iicWAddr(uint8_t SlaveAddress) override;

     uint8_t iicRAddr(uint8_t SlaveAddress) override;

public:
	
	//Ӳ��iic���캯��
    /*SCL�˿�	SCL����		SDA����		IIC�˿�		
	IICͨ��Ƶ�� ��λ��Hz  Ĭ��400kHz����Χ400kHz-1Hz	
	*SDA�˿� Ĭ��Ϊ0������SCL��ͬ	*Ӳ��IIC�ȴ����߳�ʱʱ�� Ĭ��Ϊ0����ϵͳʱ��Ƶ��/10000 */ 
    Hardiic(GPIO_TypeDef *iicSclPort, uint16_t iicSclPin, uint16_t iicSdaPin, I2C_TypeDef *IIC,
            uint32_t iicFrequency = 400000,
            GPIO_TypeDef *iicSdaPort = 0, uint32_t ulTimeOut_Time = 0);
	
	//Ӳ��iic��ʼ����Ҳ�����ڽ����������
     void iicInit(void) override;
};

/*************************************************
  * @class ���IIC��
  * @brief �������iic�����������������������д��IICʵ��
  * @note
*************************************************/
class Softiic : public IIC
{

private:
    //���IIC����
    GPIO_TypeDef *iicSclPort;			//SCL�˿�
    GPIO_TypeDef *iicSdaPort;			//SDA�˿�
    uint16_t	 iicSclPin;				//SCL����
    uint16_t 	 iicSdaPin;				//SDA����
    uint32_t 	 iicSclRccAHB1Periph;	//SCL�˿�ʱ��
    uint32_t 	 iicSdaRccAHB1Periph;	//SDA�˿�ʱ��

	/*��IICͨ��Ƶ�ʹ���ʱĳЩ������ܲ�֧��*/
    uint32_t 	 iicFrequency;				//iicͨ��Ƶ�� ��ʼ����Ƶ�ʾ��������ỻ��Ϊ��ʱʱ�䴢���ڴ˱�����

    //���iic���߻�������
    inline void sclH(void);			//SCL������

    inline void sclL(void);			//SCL������

    inline void sdaH(void);			//SDA������

    inline void sdaL(void);			//SDA������

    inline uint32_t sdaRead(void);		//��ȡSDA�ߵ�ƽ

    void iicSoftDelay(void);			//���iic��ʱ

    //IIC�����������ο�IIC��ע��
     uint8_t iicCheckBusy(void) override;

     uint8_t iicStart(void) override;

     void iicStop(void) override;

     uint8_t iicReceiveDataByte_Ack(uint8_t *ptChar) override;

     uint8_t iicReceiveDataByte_NoAck(uint8_t *ptChar) override;

     uint8_t iicSendByte(uint8_t SendByte) override;

     uint8_t iicWAddr(uint8_t SlaveAddress) override;

     uint8_t iicRAddr(uint8_t SlaveAddress) override;

public:

    //���iic���캯��
    /*SCL�˿�	SCL����		SDA����		IIC�˿ڣ�ռλ����������Ӳ��iic�����iic�����л���		
	IICͨ��Ƶ�� ��λ��Hz  Ĭ��1MHz����Χ1.2MHz-10kHz	
	*SDA�˿� Ĭ��Ϊ0������SCL��ͬ	*ռλ���� */  
    Softiic(GPIO_TypeDef *iicSclPort, uint16_t iicSclPin, uint16_t iicSdaPin, I2C_TypeDef *IIC = 0,
            uint32_t iicFrequency = 1000000,
            GPIO_TypeDef *iicSdaPort = 0, uint32_t= 0);

    // ���iic��ʼ��
    void iicInit(void) override;
};

#endif
