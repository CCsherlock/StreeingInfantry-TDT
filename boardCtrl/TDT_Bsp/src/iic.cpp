/******************************
File name: TDT_BSP\src\iic.cpp
Description: IIC

Author: ����
Version: 1.3.1.191119_alpha
Date: 20.11.11
History:
	����������������������������������������������������������������������������������������������������������������������������������������������������
	20.11.11 �״����
	����������������������������������������������������������������������������������������������������������������������������������������������������
****************************  */
#include "iic.h"

/**
  * @brief �������λ
  * @param [16λ����]
  */
static uint32_t judge_BitSite(uint16_t port)
{
    return port == 0x00 ? 0x00 : judge_BitSite(port >> 1) + 1;
}



/**
  * @brief IIC��ȡ�Ĵ������ֽ�����
  * @param [�ӻ���ַ���Ĵ�����ַ]
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
  * @brief IIC��ȡ�Ĵ������ֽ�����
  * @param [�ӻ���ַ���Ĵ�����ַ�����ݴ洢��ַ�����ݴ�С]
  */
uint8_t IIC::iicRegReadData(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *ptChar, uint8_t size)
{
    uint8_t I2C_Err = 0;

	//��鴫�����ݵ�ַ�ʹ�С
    if (size < 1 || ptChar == NULL)
    {
        return 0;
    }
    //�������æ��������æ���ܴ���Ӳ��IIC��������״̬�����³�ʼ����λ
    if (iicCheckBusy())
    {
        iicInit();
        return 0;
    }

    I2C_Err |= iicStart();						//������ʼ�ź�
    I2C_Err |= iicWAddr(SlaveAddress);			//���ʹӻ���ַ+W
    I2C_Err |= iicSendByte(REG_Address);		//���ͼĴ�����ַ
    I2C_Err |= iicStart();						//������ʼ�ź�
    I2C_Err |= iicRAddr(SlaveAddress);			//���ʹӻ���ַ+R
    //ѭ����ȡ
    while (--size)
    {
        I2C_Err |= iicReceiveDataByte_Ack(ptChar++);	//��ȡһ���ֽڲ�����Ӧ��λ
    }
    I2C_Err |= iicReceiveDataByte_NoAck(ptChar++);	//���һ�����ݲ�����Ӧ��λ
    iicStop();									//����STOP�ź�
    
    return !I2C_Err;
}



/**
  * @brief IICд��Ĵ������ֽ�����
  * @param [�ӻ���ַ���Ĵ�����ַ����Ҫд�������]
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
  * @brief IICд��Ĵ������ֽ�����
  * @param [�ӻ���ַ���Ĵ�����ַ�����ݴ洢��ַ�����ݴ�С]
  */
uint8_t IIC::iicRegWriteData(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *ptChar, uint8_t size)
{
    uint8_t I2C_Err = 0;
	
	//��鴫�����ݵ�ַ�ʹ�С
    if (size < 1 || ptChar == NULL)
    {
        return 0;
    }
   
    I2C_Err |= iicCheckBusy();					//��������Ƿ�ռ��
    I2C_Err |= iicStart();						//������ʼ�ź�
    I2C_Err |= iicWAddr(SlaveAddress);			//���ʹӻ���ַ+W
    I2C_Err |= iicSendByte(REG_Address);		//���ͼĴ�����ַ
	//ѭ��д��
    while (size--)
    {
        I2C_Err |= iicSendByte(*(ptChar++));	//����һ���ֽڲ����Ӧ��λ
    }
    iicStop();									//����ֹͣλ

    return !I2C_Err;
}



/**
  * @brief IIC��ȡ���ֽ�����
  * @param [������ַ]
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
  * @brief IIC��ȡ���ֽ�����
  * @param [�ӻ���ַ�����ݴ洢��ַ�����ݴ�С]
  */
uint8_t IIC::iicReceiveData(uint8_t SlaveAddress, uint8_t *ptChar, uint8_t size)
{
    uint8_t I2C_Err = 0;

	//��鴫�����ݵ�ַ�ʹ�С
    if (size < 1 || ptChar == NULL)
    {
        return 0;
    }
	
    //�������æ��������æ���ܴ���Ӳ��IIC��������״̬�����³�ʼ����λ
    if (iicCheckBusy())
    {
        iicInit();
        return 0;
    }
    
    I2C_Err |= iicStart();						//������ʼ�ź�
    I2C_Err |= iicRAddr(SlaveAddress);			//���ʹӻ���ַ+R
    while (--size)
    {
        I2C_Err |= iicReceiveDataByte_Ack(ptChar++);	//��ȡһ���ֽڲ�����Ӧ��λ
    }
    I2C_Err |= iicReceiveDataByte_NoAck(ptChar++);	//���һ�����ݲ�����Ӧ��λ
    iicStop();									//����ֹͣλ

    return !I2C_Err;
}



/**
  * @brief IIC���͵��ֽ�����
  * @param [�ӻ���ַ����д�������]
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
  * @brief IIC���Ͷ��ֽ�����
  * @param [�ӻ���ַ�����ݴ洢��ַ�����ݴ�С]
  */
uint8_t IIC::iicSendData(uint8_t SlaveAddress, uint8_t *ptChar, uint8_t size)
{
    uint8_t I2C_Err = 0;
	
	//��鴫�����ݵ�ַ�ʹ�С
    if (size < 1 || ptChar == NULL)
    {
        return 0;
    }


    I2C_Err |= iicCheckBusy();					//��������Ƿ�ռ��
    I2C_Err |= iicStart();						//������ʼ�ź�
    I2C_Err |= iicWAddr(SlaveAddress);			//���ʹӻ���ַ+W

    while (--size)
    {
        I2C_Err |= iicSendByte(*ptChar++);		//����һ���ֽڲ����Ӧ��λ
    }
    iicStop();									//����ֹͣλ
    
    return !I2C_Err;
}



/**
  * @brief Ӳ��iic���캯��
  * @param [SCL�˿ڣ�SCL���ţ�SDA���ţ�IIC����ڣ�*IICͨ��Ƶ�ʣ�*SDA�˿ڣ�*��ʱʱ��]
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
  * @brief Ӳ��iic��ʼ������
  * @note  Ӳ��iicҲ����ʹ�øú������и�λ�������������
  */
void Hardiic::iicInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;
	
	//ʹ��ʱ��
    RCC_AHB1PeriphClockCmd(iicSclRccAHB1Periph | iicSdaRccAHB1Periph, ENABLE);
    RCC_APB1PeriphClockCmd((((uint32_t) iic - APB1PERIPH_BASE) & 0x0F00) << 11, ENABLE);//����IICʱ�ӵ�ַ

    /* ʹ�����ǿ�Ʒ����Ÿ�ʱ��������IIC�������� */
    GPIO_InitStructure.GPIO_Pin = iicSclPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//����Ϊ������Խ�ʱ����ǿ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(iicSclPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = iicSdaPin;
    GPIO_Init(iicSdaPort, &GPIO_InitStructure);

    iicSdaPort->BSRRL = iicSdaPin;

	//����9��ʱ������
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
	
	//�������踴��
    GPIO_PinAFConfig(iicSclPort, judge_BitSite(iicSclPin) - 1,((uint8_t) 0x04));/*ע�⣬�˴����ܺϲ�д��GPIO_PinSource6|GPIO_PinSource7*/
    GPIO_PinAFConfig(iicSdaPort, judge_BitSite(iicSdaPin) - 1,((uint8_t) 0x04));/*((uint8_t)0x04)ΪGPIO_AF_I2Cͨ��ֵ��Ϊ�������д��������ʽ*/

	//����IIC����
    GPIO_InitStructure.GPIO_Pin = iicSclPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;			//����IICʹ�ÿ�©���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(iicSclPort, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = iicSdaPin;
    GPIO_Init(iicSdaPort, &GPIO_InitStructure);

	//��IIC���и�λ
    I2C_SoftwareResetCmd(iic, ENABLE);
    I2C_SoftwareResetCmd(iic, DISABLE);

	//IIC����
    I2C_DeInit(iic);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_ClockSpeed = iicFrequency;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(iic, &I2C_InitStructure);

	//ʹ��IIC
    I2C_Cmd(iic, ENABLE);
}



/**
  * @brief Ӳ��iic�������æ
  * @note  Ӳ��iic������1�������������������ȡʱ��������æ������iic
  */
uint8_t Hardiic::iicCheckBusy(void)
{
    uint32_t tmr = ulTimeOut_Time;
    while ((--tmr) && I2C_GetFlagStatus(iic, I2C_FLAG_BUSY));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief Ӳ��iic������ʼ�ź�
  */
uint8_t Hardiic::iicStart(void)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_GenerateSTART(iic, ENABLE);
    while ((--tmr) && (!(I2C_CheckEvent(iic, I2C_EVENT_MASTER_MODE_SELECT))));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief Ӳ��iic����ֹͣ�ź�
  */
void Hardiic::iicStop(void)
{
    I2C_GenerateSTOP(iic, ENABLE);
}



/**
  * @brief Ӳ��iic��ȡһ���ֽ����ݲ�����Ӧ��λ
  * @param [������ȡ�ֽڵĴ洢��ַ]
  */
uint8_t Hardiic::iicReceiveDataByte_Ack(uint8_t *ptChar)
{
    uint32_t tmr = ulTimeOut_Time;
    while ((--tmr) && (!(I2C_CheckEvent(iic, I2C_EVENT_MASTER_BYTE_RECEIVED))));
    *ptChar = I2C_ReceiveData(iic);
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief Ӳ��iic��ȡһ���ֽ����ݲ�����Ӧ��λ
  * @param [������ȡ�ֽڵĴ洢��ַ]
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
  * @brief Ӳ��iic����һ���ֽ����ݲ����Ӧ��λ
  * @param [Ҫ���͵�����]
  */
uint8_t Hardiic::iicSendByte(uint8_t SendByte)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_SendData(iic, SendByte);
    while ((--tmr) && (!I2C_CheckEvent(iic, I2C_EVENT_MASTER_BYTE_TRANSMITTED)));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief Ӳ��iic���ʹӻ���ַ�����д���־λ
  * @param [δ����һλ�Ĵӻ���ַ]
  */
uint8_t Hardiic::iicWAddr(uint8_t SlaveAddress)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_Send7bitAddress(iic, (SlaveAddress << 1), I2C_Direction_Transmitter); //ע��ú���������λ��Ҫ��������
    while ((--tmr) && (!I2C_CheckEvent(iic, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief Ӳ��iic���ʹӻ���ַ����Ӷ�ȡ��־λ
  * @param [δ����һλ�Ĵӻ���ַ]
  */
uint8_t Hardiic::iicRAddr(uint8_t SlaveAddress)
{
    uint32_t tmr = ulTimeOut_Time;
    I2C_Send7bitAddress(iic, (SlaveAddress << 1), I2C_Direction_Receiver);
    while ((--tmr) && (!I2C_CheckEvent(iic, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)));
    return tmr == 0 ? 1 : 0;
}



/**
  * @brief ���iic���캯��
  * @param [SCL�˿ڣ�SCL���ţ�SDA���ţ�(ռλ����)IIC����ڣ�*IICͨ��Ƶ�ʣ�*SDA�˿ڣ�(ռλ����)��ʱʱ��]
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
        this->iicFrequency = (9293320.0f / iicFrequency) - 7.24f;	//�˹�ʽ����Ӳ��iic���м��㣬��֤ͨ��Ƶ����ͬʱͨ��ʱ��һ��
    }
}



/**
  * @brief ���iic��ʼ������
  * @note  ���iic���߲�������
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
  * @brief �������� SCL��������
  */
void Softiic::sclH(void)
{ iicSclPort->BSRRL = iicSclPin; }



/**
  * @brief �������� SCL��������
  */
void Softiic::sclL(void)
{ iicSclPort->BSRRH = iicSclPin; }



/**
  * @brief �������� SDA��������
  */
void Softiic::sdaH(void)
{ iicSdaPort->BSRRL = iicSdaPin; }



/**
  * @brief �������� SDA��������
  */
void Softiic::sdaL(void)
{ iicSdaPort->BSRRH = iicSdaPin; }



/**
  * @brief �������� ��ȡSDA���ߵ�ƽ
  */
uint32_t Softiic::sdaRead(void)
{ return (iicSdaPort->IDR & iicSdaPin); }



/**
  * @brief �ȴ����ߵ�ƽ���䣬�������IICͨ��Ƶ��
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
  * @brief ���iic�������æ
  * @note  ���iic������1�������������������ȡʱ��������æ������iic
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
  * @brief ���iic������ʼ�ź�
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
  * @brief ���iic����ֹͣ�ź�
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
  * @brief ���iic��ȡһ���ֽ����ݲ�����Ӧ��λ
  * @param [������ȡ�ֽڵĴ洢��ַ]
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
  * @brief ���iic��ȡһ���ֽ����ݲ�����Ӧ��λ
  * @param [������ȡ�ֽڵĴ洢��ַ]
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
  * @brief ���iic����һ���ֽ����ݲ����Ӧ��λ
  * @param [Ҫ���͵�����]
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
  * @brief ���iic���ʹӻ���ַ�����д���־λ
  * @param [δ����һλ�Ĵӻ���ַ]
  */
uint8_t Softiic::iicWAddr(uint8_t SlaveAddress)
{
    return iicSendByte((uint8_t)(((SlaveAddress << 1) & 0xFE) | 0x00));
}



/**
  * @brief ���iic���ʹӻ���ַ����Ӷ�ȡ��־λ
  * @param [δ����һλ�Ĵӻ���ַ]
  */
uint8_t Softiic::iicRAddr(uint8_t SlaveAddress)
{
    return iicSendByte((uint8_t)(((SlaveAddress << 1) & 0xFE) | 0x01));
}
