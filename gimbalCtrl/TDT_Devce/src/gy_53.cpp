#include "gy_53.h"
#include "FreeRTOS.h"					//FreeRTOS
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
DMA_InitTypeDef Rx_DMA_InitStructure[3];
u8 tmp_RecvBuff_GY53[3][8];
uint16_t GY_53distence[3] = {2500,2500,2500};
gy_53::gy_53(USART_TypeDef * USART,uint32_t BaudRatePrescaler)
{
	ThisUSART = USART;
	baudRatePrescaler = BaudRatePrescaler;
	if(ThisUSART == USART1)
	{
		usartInitFlag[0] = 1;
	}
	if(ThisUSART == USART3)
	{
		usartInitFlag[1] = 1;
	}
	if(ThisUSART == USART6)
	{
		usartInitFlag[2] = 1;
	}
}
void gy_53::gy_53_Init()
{
	//GPIO端口设置
	USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
	
	if(usartInitFlag[2] == 1)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC |RCC_AHB1Periph_DMA2 , ENABLE); //GPIOA，DMA时钟使能
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);                      //USART6时钟使能

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);  //GPIOA9，USART6，TX
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6); //GPIOA10，USART6，RX

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		USART_DeInit(USART6);
		USART_InitStructure.USART_BaudRate = baudRatePrescaler;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART6, &USART_InitStructure);
		USART_Cmd(USART6, ENABLE);

		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
		USART_DMACmd(USART6, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);

		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		DMA_DeInit(DMA2_Stream1);
		Rx_DMA_InitStructure[2].DMA_Channel = DMA_Channel_5;
		Rx_DMA_InitStructure[2].DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
		Rx_DMA_InitStructure[2].DMA_Memory0BaseAddr = (uint32_t)tmp_RecvBuff_GY53[2];
		Rx_DMA_InitStructure[2].DMA_DIR = DMA_DIR_PeripheralToMemory;
		Rx_DMA_InitStructure[2].DMA_BufferSize = sizeof(tmp_RecvBuff_GY53[2]);
		Rx_DMA_InitStructure[2].DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		Rx_DMA_InitStructure[2].DMA_MemoryInc = DMA_MemoryInc_Enable;
		Rx_DMA_InitStructure[2].DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		Rx_DMA_InitStructure[2].DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		Rx_DMA_InitStructure[2].DMA_Mode = DMA_Mode_Normal;
		Rx_DMA_InitStructure[2].DMA_Priority = DMA_Priority_High;
		Rx_DMA_InitStructure[2].DMA_FIFOMode = DMA_FIFOMode_Disable;
		Rx_DMA_InitStructure[2].DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		Rx_DMA_InitStructure[2].DMA_MemoryBurst = DMA_MemoryBurst_Single;
		Rx_DMA_InitStructure[2].DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream1, &Rx_DMA_InitStructure[2]);
		DMA_Cmd(DMA2_Stream1, ENABLE);
		send_com(0x45);
	}
	if(usartInitFlag[0] == 1)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE); //GPIOA，DMA时钟使能
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);                      //USART1时钟使能

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  //GPIOA9，USART1，TX
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); //GPIOA10，USART1，RX

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		USART_DeInit(USART1);
		USART_InitStructure.USART_BaudRate = baudRatePrescaler;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART1, &USART_InitStructure);
		USART_Cmd(USART1, ENABLE);

		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
		USART_DMACmd(USART1, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);

		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		DMA_DeInit(DMA2_Stream5);
		Rx_DMA_InitStructure[0].DMA_Channel = DMA_Channel_4;
		Rx_DMA_InitStructure[0].DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
		Rx_DMA_InitStructure[0].DMA_Memory0BaseAddr = (uint32_t)tmp_RecvBuff_GY53[0];
		Rx_DMA_InitStructure[0].DMA_DIR = DMA_DIR_PeripheralToMemory;
		Rx_DMA_InitStructure[0].DMA_BufferSize = sizeof(tmp_RecvBuff_GY53[0]);
		Rx_DMA_InitStructure[0].DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		Rx_DMA_InitStructure[0].DMA_MemoryInc = DMA_MemoryInc_Enable;
		Rx_DMA_InitStructure[0].DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		Rx_DMA_InitStructure[0].DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		Rx_DMA_InitStructure[0].DMA_Mode = DMA_Mode_Normal;
		Rx_DMA_InitStructure[0].DMA_Priority = DMA_Priority_High;
		Rx_DMA_InitStructure[0].DMA_FIFOMode = DMA_FIFOMode_Disable;
		Rx_DMA_InitStructure[0].DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		Rx_DMA_InitStructure[0].DMA_MemoryBurst = DMA_MemoryBurst_Single;
		Rx_DMA_InitStructure[0].DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA2_Stream5, &Rx_DMA_InitStructure[0]);
		DMA_Cmd(DMA2_Stream5, ENABLE);
		send_com(0x45);
	}
	if(usartInitFlag[1] == 1)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE); //GPIOA，DMA时钟使能
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);                      //USART1时钟使能

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART1);  //GPIOA9，USART1，TX
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART1); //GPIOA10，USART1，RX

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		USART_DeInit(USART3);
		USART_InitStructure.USART_BaudRate = baudRatePrescaler;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3, &USART_InitStructure);
		USART_Cmd(USART3, ENABLE);

		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
		USART_DMACmd(USART3, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);

		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		DMA_DeInit(DMA1_Stream1);
		Rx_DMA_InitStructure[1].DMA_Channel = DMA_Channel_4;
		Rx_DMA_InitStructure[1].DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
		Rx_DMA_InitStructure[1].DMA_Memory0BaseAddr = (uint32_t)tmp_RecvBuff_GY53[1];
		Rx_DMA_InitStructure[1].DMA_DIR = DMA_DIR_PeripheralToMemory;
		Rx_DMA_InitStructure[1].DMA_BufferSize = sizeof(tmp_RecvBuff_GY53[1]);
		Rx_DMA_InitStructure[1].DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		Rx_DMA_InitStructure[1].DMA_MemoryInc = DMA_MemoryInc_Enable;
		Rx_DMA_InitStructure[1].DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		Rx_DMA_InitStructure[1].DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		Rx_DMA_InitStructure[1].DMA_Mode = DMA_Mode_Normal;
		Rx_DMA_InitStructure[1].DMA_Priority = DMA_Priority_High;
		Rx_DMA_InitStructure[1].DMA_FIFOMode = DMA_FIFOMode_Disable;
		Rx_DMA_InitStructure[1].DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
		Rx_DMA_InitStructure[1].DMA_MemoryBurst = DMA_MemoryBurst_Single;
		Rx_DMA_InitStructure[1].DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_Init(DMA1_Stream1, &Rx_DMA_InitStructure[1]);
		DMA_Cmd(DMA1_Stream1, ENABLE);
		send_com(0x45);
	}
}
#define NORMAL
//#define SETTING
void gy_53::send_com(u8 data)
{
	u8 bytes[3]={0};
#ifdef NORMAL
	bytes[0]=0xa5;
	bytes[1]=data;//功能字节
	bytes[2]=0xEA;
	USART_Send(bytes,3);//发送帧头、功能字节、校验和
#endif
#ifdef SETTING

	if(setting_flag1 == 0&&setting_flag2 == 0&&setting_flag3 == 0)
	{
		bytes[0]=0xa5;
		bytes[1]=0X51;//功能字节
		bytes[2]=0XF6;
		USART_Send(bytes,3);//发送帧头、功能字节、校验和
		setting_flag1 = 1;
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	if(setting_flag1 == 1&&setting_flag2 == 0&&setting_flag3 == 0)
	{
		bytes[0]=0xa5;
		bytes[1]=0X25;//功能字节
		bytes[2]=0XCA;
		USART_Send(bytes,3);//发送帧头、功能字节、校验和
		setting_flag2 = 1;
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	if(setting_flag1 == 1&&setting_flag2 == 1&&setting_flag3 == 0)
	{
		bytes[0]=0xa5;
		bytes[1]=data;//功能字节
		bytes[2]=0xEA;
		USART_Send(bytes,3);//发送帧头、功能字节、校验和
		setting_flag3 = 1;
		setting_ok = 1;
	}
#endif
}
//发送多字节数据
void gy_53::USART_Send_bytes(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USARTx_send_byte(Buffer[i++]);
	}
}
void USART1_IRQHandler(void)
{
    u8 tmp; 
	u8 sum=0;
	static uint8_t i=0;
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) 
    { 
        tmp = USART1->SR; 
        tmp = USART1->DR; 
        DMA_Cmd(DMA2_Stream5,DISABLE); 
	
		
			
		for(sum=0,i=0;i<(tmp_RecvBuff_GY53[0][3]+4);i++)//rgb_data[3]=3
		sum+=tmp_RecvBuff_GY53[0][i];
		if(sum==tmp_RecvBuff_GY53[0][i])//校验和判断
		{
			GY_53distence[0]=tmp_RecvBuff_GY53[0][4]<<8|tmp_RecvBuff_GY53[0][5];
			tmp = GY_53distence[0];
		}
		
        while(DMA_GetCmdStatus(DMA2_Stream5) != DISABLE); 
        DMA_DeInit(DMA2_Stream5); 
        DMA_Init(DMA2_Stream5, &Rx_DMA_InitStructure[0]); 
        DMA_SetCurrDataCounter(DMA2_Stream5, sizeof(tmp_RecvBuff_GY53[0])); 
        DMA_Cmd(DMA2_Stream5,ENABLE); 
    }

}
void USART3_IRQHandler(void)
{
    u8 tmp; 
	u8 sum=0;
	static uint8_t i=0;
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) 
    { 
        tmp = USART3->SR; 
        tmp = USART3->DR; 
        DMA_Cmd(DMA1_Stream1,DISABLE); 
	
		
			
		for(sum=0,i=0;i<(tmp_RecvBuff_GY53[1][3]+4);i++)//rgb_data[3]=3
		sum+=tmp_RecvBuff_GY53[1][i];
		if(sum==tmp_RecvBuff_GY53[1][i])//校验和判断
		{
			GY_53distence[1]=tmp_RecvBuff_GY53[1][4]<<8|tmp_RecvBuff_GY53[1][5];
			tmp = GY_53distence[1];
		}
		
        while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE); 
        DMA_DeInit(DMA1_Stream1); 
        DMA_Init(DMA1_Stream1, &Rx_DMA_InitStructure[1]); 
        DMA_SetCurrDataCounter(DMA1_Stream1, sizeof(tmp_RecvBuff_GY53[1])); 
        DMA_Cmd(DMA1_Stream1,ENABLE); 
    }

}
void USART6_IRQHandler(void)
{
    u8 tmp; 
	u8 sum=0;
	static uint8_t i=0;
    if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) 
    { 
        tmp = USART6->SR; 
        tmp = USART6->DR; 
        DMA_Cmd(DMA2_Stream1,DISABLE); 
	
		
			
		for(sum=0,i=0;i<(tmp_RecvBuff_GY53[2][3]+4);i++)//rgb_data[3]=3
		sum+=tmp_RecvBuff_GY53[2][i];
		if(sum==tmp_RecvBuff_GY53[2][i])//校验和判断
		{
			GY_53distence[2]=tmp_RecvBuff_GY53[2][4]<<8|tmp_RecvBuff_GY53[2][5];
			tmp = GY_53distence[2];
		}
		
        while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE); 
        DMA_DeInit(DMA2_Stream1); 
        DMA_Init(DMA2_Stream1, &Rx_DMA_InitStructure[2]); 
        DMA_SetCurrDataCounter(DMA2_Stream1, sizeof(tmp_RecvBuff_GY53[2])); 
        DMA_Cmd(DMA2_Stream1,ENABLE); 
    }

}
//发送多字节数据+校验和
void gy_53::USART_Send(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];//累加Length-1前的数据
		USARTx_send_byte(Buffer[i++]);
		//while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
	}
}
//发送一个字节数据
//input:byte,待发送的数据
void gy_53::USARTx_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(ThisUSART,USART_FLAG_TC)==RESET);//等待发送完成
	//USART_SendData(USART6,byte);
	USART6->DR=byte;
}