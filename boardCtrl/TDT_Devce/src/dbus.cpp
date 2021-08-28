/******************************
File name: TDT_Device\src\dbus.cpp
Description: 遥控器接收机
function:
	——————————————————————————————————————————————————————————————————————————
	void Dbus_Config(void)
	——————————————————————————————————————————————————————————————————————————
	void USART2_IRQHandler(void)
	——————————————————————————————————————————————————————————————————————————
	static void Handle_data(volatile const uint8_t *sbus_buf, struct _RC *rc_ctrl)
	——————————————————————————————————————————————————————————————————————————
	int Get_Keypress(uint16_t Key)
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加切换为原始数据处理的变量
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次记录
	——————————————————————————————————————————————————————————————————————————
****************************  */
#define __DBUS_DRIVER_GLOBALS
#include "dbus.h"
#include "FreeRTOS.h"					//FreeRTOS使用	 
#include "queue.h"
#include "task.h"
//Chassis_Command Key_Command;
unsigned char sbus_rx_buffer[18];//原始数据



 uint8_t SBUS_rx_buf[2][SBUS_RX_BUF_NUM];//DMA的原始数据
/**
  * @brief 遥控器初始化
  */
void Dbus_Config(void)
{
/* -------------- 初始化时钟资源 ----------------------------*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //PA3  usart3 rx

/* -------------- 配置GPIO ---------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_DeInit(USART2);

	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStructure);

	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

	USART_ClearFlag(USART2, USART_FLAG_IDLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

	USART_Cmd(USART2, ENABLE);

/* -------------- 中断配置 ---------------------------------------*/
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

/* -------------- 配置 DMA -----------------------------------------*/
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Stream5);

	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)SBUS_rx_buf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = SBUS_RX_BUF_NUM;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream5, &DMA_InitStructure);

	DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)SBUS_rx_buf[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
	DMA_Cmd(DMA1_Stream5, DISABLE); //Add a disable
	DMA_Cmd(DMA1_Stream5, ENABLE);
}


				


u8 useOriginHandle=0;//使用原始数据处理方式
/**
  * @brief 遥控器串口接收中断
  */
void USART2_IRQHandler(void)
{
	/*我也不知道干啥用的，就知道好像得定义成变量*/
	BaseType_t xTaskWokenByReceive_Notice;//发送通知用的

	extern QueueHandle_t DBUS_Queue;					//CAN1消息队列句柄
	extern TaskHandle_t DbusTask_Handler;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART2);
    }
    else if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART2);

        if(DMA_GetCurrentMemoryTarget(DMA1_Stream5) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream5, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
            DMA1_Stream5->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
            DMA_Cmd(DMA1_Stream5, ENABLE);
				
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {       
				if(useOriginHandle)
				{
					//原初方法处理遥控器数据--备用
					Handle_data(SBUS_rx_buf[0], &RC);				
				}

				//发送通知，目标为DBUS任务，通知值为原始数据地址，且不保留接受任务的通知值，接受任务的通知值被覆盖
				xTaskNotifyFromISR(DbusTask_Handler,(uint32_t)(&SBUS_rx_buf[0]),eSetValueWithOverwrite,&xTaskWokenByReceive_Notice);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA1_Stream5, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream5);
            DMA_SetCurrDataCounter(DMA1_Stream5, SBUS_RX_BUF_NUM);
            DMA1_Stream5->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
            DMA_Cmd(DMA1_Stream5, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
				if(useOriginHandle)
				{
					//原初方法处理遥控器数据--备用
					Handle_data(SBUS_rx_buf[1], &RC);				
				}

				//发送通知，目标为DBUS任务，通知值为原始数据地址，且不保留接受任务的通知值，接受任务的通知值被覆盖
				xTaskNotifyFromISR(DbusTask_Handler,(uint32_t)(&SBUS_rx_buf[1]),eSetValueWithOverwrite,&xTaskWokenByReceive_Notice);
            }
        }
    }
}

int16_t sbus_decode_buffer[15];
int16_t last_mouse;
int16_t last_last_mouse;
/**
  * @brief 遥控器串口数据原始处理方法
  */
static void Handle_data(volatile const uint8_t *sbus_buf, struct _RC *rc_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }
		rc_ctrl->LastKey.CH[6] = rc_ctrl->Key.CH[6];
		rc_ctrl->LastKey.CH[7] = rc_ctrl->Key.CH[7];
	  
	
		//右摇杆横向  范围+-660
		sbus_decode_buffer[0] = (sbus_buf[0]| (sbus_buf[1] << 8)) & 0x07ff; //!< Channel 0
		rc_ctrl->Key.CH[0] = my_deathzoom(sbus_decode_buffer[0]-1024, 5);
		//右摇杆纵向   范围+-660
		sbus_decode_buffer[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
		rc_ctrl->Key.CH[1] = my_deathzoom(sbus_decode_buffer[1]-1024, 5); 
		//左摇杆横向   范围+-660
		sbus_decode_buffer[2]= ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff; //!< Channel 2
		rc_ctrl->Key.CH[2] = my_deathzoom(sbus_decode_buffer[2]-1024, 5);
		//左摇杆纵向   范围+-660
		sbus_decode_buffer[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
		rc_ctrl->Key.CH[3] = my_deathzoom(sbus_decode_buffer[3]-1024, 5);
		//左边开关  132 上中下
		rc_ctrl->Key.CH[4] = ((sbus_buf[5] >> 4)& 0x000C) >> 2; //!< Switch left
		//右边开关  132 上中下
		rc_ctrl->Key.CH[5] = ((sbus_buf[5] >> 4)& 0x0003); //!< Switch right9 / 9

		/***鼠标X值***/
        rc_ctrl->Key.CH[6] = 	((sbus_buf[6]) | (sbus_buf[7] << 8));//x
		/***鼠标Y值***/
		rc_ctrl->Key.CH[7] = 	-((sbus_buf[8]) | (sbus_buf[9] << 8));//y
		/***鼠标左键***/
		rc_ctrl->Key.CH[8] = sbus_buf[12];//左键
		rc_ctrl->Key.left_jump=rc_ctrl->Key.CH[8];
		
		
		/***鼠标右键***/
		rc_ctrl->Key.CH[9] = sbus_buf[13];//右键
        rc_ctrl->Key.Right_jump=rc_ctrl->Key.CH[9];	
		rc_ctrl->Key.CH[10] = sbus_buf[14] | (sbus_buf[15] << 8);	/***键盘值***/	
		
		RC.Key.D=(RC.Key.CH[10] & KEY_D) ? 1:0;
		RC.Key.A=(RC.Key.CH[10] & KEY_A) ? 1:0;
		RC.Key.S=(RC.Key.CH[10] & KEY_S) ? 1:0;
		RC.Key.W=(RC.Key.CH[10] & KEY_W) ? 1:0;		
		RC.Key.B=(RC.Key.CH[10] & KEY_B) ? 1:0;
		RC.Key.V=(RC.Key.CH[10] & KEY_V) ? 1:0;
		RC.Key.C=(RC.Key.CH[10] & KEY_C) ? 1:0;
		RC.Key.X=(RC.Key.CH[10] & KEY_X) ? 1:0;
		RC.Key.Z=(RC.Key.CH[10] & KEY_Z) ? 1:0;
		RC.Key.G=(RC.Key.CH[10] & KEY_G) ? 1:0;
		RC.Key.F=(RC.Key.CH[10] & KEY_F) ? 1:0;
		RC.Key.R=(RC.Key.CH[10] & KEY_R) ? 1:0;
		RC.Key.E=(RC.Key.CH[10] & KEY_E) ? 1:0;
		RC.Key.Q=(RC.Key.CH[10] & KEY_Q) ? 1:0;
		RC.Key.CTRL=(RC.Key.CH[10] & KEY_CTRL) ? 1:0;
		RC.Key.SHIFT=(RC.Key.CH[10] & KEY_SHIFT) ? 1:0;
}


/**
  * @brief  Be sure if the key has pressed.
  * @param  Key_value
  * @retval True or False
  */
int Get_Keypress(uint16_t Key)
{
    if(RC.Key.CH[10] & Key)
	{
		return 1;
	}
    else
	{
		return 0;
	}
}












