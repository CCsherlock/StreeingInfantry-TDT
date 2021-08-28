/******************************
File name: TDT_Bsp\src\can.cpp
Description: can底层
function:
	——————————————————————————————————————————————————————————————————————————
	
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.16 #合成can1.c和can2.c #修改接收函数，从队列传递改为直接解算
	——————————————————————————————————————————————————————————————————————————
	19.11.12 #首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "can.h"
#include "motor.h"
#include "can_calculate.h"
#include "chassis_task.h"
/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/
CanRxMsg Can1RxMsg;
CanRxMsg Can2RxMsg;


/**
  * @brief can的GPIO和NVIC初始化
  * @param [can口:CAN1/CAN2]
  */
static void canGpioNvicInit(CAN_TypeDef * can_x)
{   
	GPIO_InitTypeDef       CanGpio;//CAN GPIO初始化结构体
    NVIC_InitTypeDef       CanNvic;//CAN 中断结构体

	/*时钟初始化*/
	if(can_x==CAN1)
	{
        #ifdef USE_MAIN_CTRL_2020 //使用USE_MAINCTRL_2020 使用矩形主控（杰哥版）//C/C++选项卡里更改
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//初始化GPIO时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//初始化CNA1时钟
		
		//GPIO复用
		
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);
		/*GPIO初始化*/
		CanGpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
		CanGpio.GPIO_Mode = GPIO_Mode_AF;
		CanGpio.GPIO_OType =GPIO_OType_PP;
		CanGpio.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOB, &CanGpio);
        #endif
        
        #ifdef USE_MAIN_CTRL_2019 //不使用USE_MAINCTRL_2020 默认使用方形主控（敏哥版）//C/C++选项卡里更改
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//初始化GPIO时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//初始化CNA1时钟
		
		//GPIO复用
		
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
		/*GPIO初始化*/
		CanGpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
		CanGpio.GPIO_Mode = GPIO_Mode_AF;
		CanGpio.GPIO_OType =GPIO_OType_PP;
		CanGpio.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init(GPIOA, &CanGpio);
        #endif
		
		/*中断初始化*/
		CanNvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
		CanNvic.NVIC_IRQChannelPreemptionPriority = 3;
		CanNvic.NVIC_IRQChannelSubPriority = 1;
		CanNvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&CanNvic);
		 
		CanNvic.NVIC_IRQChannel = CAN1_TX_IRQn;
		CanNvic.NVIC_IRQChannelPreemptionPriority = 1;
		CanNvic.NVIC_IRQChannelSubPriority = 1;
		CanNvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&CanNvic);   
	}
	else if(can_x==CAN2)	
	{

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 

		CanGpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 ;
		CanGpio.GPIO_Mode = GPIO_Mode_AF;
	//	CanGpio.GPIO_OType = GPIO_OType_PP;
		GPIO_Init(GPIOB, &CanGpio);

		CanNvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
		CanNvic.NVIC_IRQChannelPreemptionPriority = 0;
		CanNvic.NVIC_IRQChannelSubPriority = 0;
		CanNvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&CanNvic);
			
		CanNvic.NVIC_IRQChannel = CAN2_TX_IRQn;
		CanNvic.NVIC_IRQChannelPreemptionPriority = 0;
		CanNvic.NVIC_IRQChannelSubPriority = 0;
		CanNvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&CanNvic);    
	}
}



void canInit(CAN_TypeDef * can_x)
{
	//can的GPIO和NVIC初始化
	canGpioNvicInit(can_x);
	
    CAN_InitTypeDef        Can;//CNA初始化结构体
    CAN_FilterInitTypeDef  CanFilter;//CNA过滤器初始化结构体
	
    /*CAN初始化*/
    CAN_DeInit(can_x);//将外设 CAN 的全部寄存器重设为缺省值
    CAN_StructInit(&Can);//把 Can 结构体中的每一个参数按缺省值填入
	
    Can.CAN_TTCM = DISABLE;// 时间触发通讯模式， ENABLE/DISABLE。
    Can.CAN_ABOM = ENABLE;//  自动离线管理，  ENABLE/DISABLE。
    Can.CAN_AWUM = ENABLE;// 自动唤醒模式， ENABLE/DISABLE。
    Can.CAN_NART = DISABLE;//  非自动重传输模式， ENABLE/DISABLE。
    Can.CAN_RFLM = DISABLE;//  接收FIFO锁定模式  ENABLE/DISABLE。
    Can.CAN_TXFP = ENABLE;// 发送FIFO优先级， ENABLE/DISABLE。
    Can.CAN_Mode = CAN_Mode_Normal;//  CAN硬件工作模式，可选：CAN_Mode_Normal 正常模式； CAN_Mode_Silent 静默模式； CAN_Mode_LoopBack环回模式； CAN_Mode_Silent_LoopBack静默环回模式 。
    Can.CAN_SJW  = CAN_SJW_1tq;// 重新同步跳跃宽度(SJW)，范围CAN_SJW_1tq到CAN_SJW_4tq
    Can.CAN_BS1 = CAN_BS1_9tq;// 时间段 1 的时间单位数目，范围CAN_BS1_1tq到CAN_BS1_16tq
    Can.CAN_BS2 = CAN_BS2_4tq;// 时间段 2 的时间单位数目，范围CAN_BS2_1tq到CAN_BS2_8tq
    Can.CAN_Prescaler = 3;   // 一个时间单位的长度，范围 1 到 1024//42/(1+9+4)/3=1Mbps
    CAN_Init(can_x, &Can);//根据Can的参数初始化CAN1
	/*CAN过滤器配置*/
	if(can_x==CAN1)
	{
		CanFilter.CAN_FilterNumber= 0;//待初始化的过滤器，范围
	}
	else
	{
		CanFilter.CAN_FilterNumber= 14;//待初始化的过滤器，范围
	}
	
	CanFilter.CAN_FilterMode=CAN_FilterMode_IdMask;//过滤器模式：CAN_FilterMode_IdMask 标识符屏蔽位模式 CAN_FilterMode_IdList 标识符列表模式
	CanFilter.CAN_FilterScale=CAN_FilterScale_32bit;//过滤器位宽：CAN_FilterScale_Two16bit 2 个 16 位过滤器 CAN_FilterScale_One32bit 1 个 32 位过滤器 
	CanFilter.CAN_FilterIdHigh=0x0000;//过滤器标识符（32 位位宽时为其高段位，16 位位宽时为第一个）。它的范围是 0x0000 到 0xFFFF
	CanFilter.CAN_FilterIdLow=0x0000;//过滤器标识符（32 位位宽时为其低段位，16 位位宽时为第二个）。它的范围是 0x0000 到 0xFFFF
	CanFilter.CAN_FilterMaskIdHigh=0x0000;//过滤器屏蔽标识符或者过滤器标识符（32 位位宽时为其高段位，16 位位宽时为第一个）。它的范围是 0x0000 到 0xFFFF。 
	CanFilter.CAN_FilterMaskIdLow=0x0000;//过滤器屏蔽标识符或者过滤器标识符（32 位位宽时为其低段位，16 位位 宽时为第二个）。它的范围是 0x0000 到 0xFFFF
	CanFilter.CAN_FilterFIFOAssignment=0;//
	CanFilter.CAN_FilterActivation=ENABLE;//使能或者失能过滤器
	CAN_FilterInit(&CanFilter);//
	//使能或者失能指定的 CAN 中断 
	CAN_ITConfig(can_x,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许,每接收一次进一次中断	
	CAN_ITConfig(can_x,CAN_IT_TME,ENABLE); //

}



/**
  * @brief CAN发送中断
  */
void CAN1_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		//CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);	
	}
}

void CAN2_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{	   
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
	}
}



/**
  * @brief CAN接收中断
  * @note 已经自动输入到对应电机，如果由特殊ID的消息，可以自己再switch
  */
Can Can1;
Can Can2;
void CAN1_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &Can1RxMsg);
		//CAN信息处理
		Can1.Motor_Information_Calculate(Can_1,&Can1RxMsg);
		switch (Can1RxMsg.StdId)
		{
			case 0x135:
			{
				chassis.canRecvStruct.datafloat.data[0] = (float)((int16_t)((Can1RxMsg.Data[0] << 8) | (Can1RxMsg.Data[1])));
				chassis.canRecvStruct.datafloat.data[1] = (float)((int16_t)((Can1RxMsg.Data[2] << 8) | (Can1RxMsg.Data[3])));
				chassis.canRecvStruct.datafloat.data[2] = (float)((int16_t)((Can1RxMsg.Data[4] << 8) | (Can1RxMsg.Data[5])));
				chassis.canRecvStruct.datafloat.data[3] = (float)((int16_t)((Can1RxMsg.Data[6] << 8) | (Can1RxMsg.Data[7])));	
				chassis.canRecvStruct.topCtrlLostcnt = 0;
				chassis.canRecvStruct.topLostFlag = 0;
				break;
			}
			case 0x136:
			{
				chassis.canRecvStruct.ifdeforce = (u8)Can1RxMsg.Data[0];
				chassis.canRecvStruct.ifChassisFollow = (u8)Can1RxMsg.Data[1];
				break;
			}
			default:
				break;			
		}
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
	

}

void CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_Receive(CAN2, CAN_FIFO0, &Can2RxMsg);
		//CAN信息处理
		Can2.Motor_Information_Calculate(Can_2,&Can2RxMsg);
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
	}
}



/**
  * @brief  can1发送数据
  * @param  value：四维数据结构体
  *					id：ID
  */
void canTx(vec4f *value, CAN_TypeDef *can_x, uint32_t id)
{
	CanTxMsg Can1TxMsg;

	Can1TxMsg.IDE = CAN_Id_Standard; //标准帧 CAN_Id_Standard 使用标准标识符 CAN_Id_Extended 使用标准标识符 + 扩展标识符
	Can1TxMsg.RTR = CAN_RTR_Data;	 //数据帧 CAN_RTR_Data 数据帧 CAN_RTR_Remote 远程帧
	Can1TxMsg.DLC = 8;				 //帧长度 范围是 0 到 0x8

	Can1TxMsg.StdId = id; //范围为 0 到 0x7FF

	Can1TxMsg.Data[0] = (u8)((int16_t)value->data[0] >> 8);
	Can1TxMsg.Data[1] = (u8)(value->data[0]);
	Can1TxMsg.Data[2] = (u8)((int16_t)value->data[1] >> 8);
	Can1TxMsg.Data[3] = (u8)(value->data[1]);
	Can1TxMsg.Data[4] = (u8)((int16_t)value->data[2] >> 8);
	Can1TxMsg.Data[5] = (u8)(value->data[2]);
	Can1TxMsg.Data[6] = (u8)((int16_t)value->data[3] >> 8);
	Can1TxMsg.Data[7] = (u8)(value->data[3]);
	u8 mbox = CAN_Transmit(can_x, &Can1TxMsg);
	delayUs(110);
}

void canTx(float *data, CAN_TypeDef *can_x, uint32_t id)
{
	CanTxMsg Can1TxMsg;

	Can1TxMsg.IDE = CAN_Id_Standard; //标准帧 CAN_Id_Standard 使用标准标识符 CAN_Id_Extended 使用标准标识符 + 扩展标识符
	Can1TxMsg.RTR = CAN_RTR_Data;	 //数据帧 CAN_RTR_Data 数据帧 CAN_RTR_Remote 远程帧
	Can1TxMsg.DLC = 8;				 //帧长度 范围是 0 到 0x8

	Can1TxMsg.StdId = id; //范围为 0 到 0x7FF

	Can1TxMsg.Data[0] = (u8)((int16_t)*data >> 8);
	Can1TxMsg.Data[1] = (u8)(*data);
	Can1TxMsg.Data[2] = (u8)((int16_t) * (data + 1) >> 8);
	Can1TxMsg.Data[3] = (u8)(*(data + 1));
	Can1TxMsg.Data[4] = (u8)((int16_t) * (data + 2) >> 8);
	Can1TxMsg.Data[5] = (u8)(*(data + 2));
	Can1TxMsg.Data[6] = (u8)((int16_t) * (data + 3) >> 8);
	Can1TxMsg.Data[7] = (u8)(*(data + 3));
	u8 mbox = CAN_Transmit(can_x, &Can1TxMsg);
	delayUs(110);
}

void canTx(u8 data[8], CAN_TypeDef *can_x, uint32_t id)
{
	CanTxMsg Can1TxMsg;

	Can1TxMsg.IDE = CAN_Id_Standard; //标准帧 CAN_Id_Standard 使用标准标识符 CAN_Id_Extended 使用标准标识符 + 扩展标识符
	Can1TxMsg.RTR = CAN_RTR_Data;	 //数据帧 CAN_RTR_Data 数据帧 CAN_RTR_Remote 远程帧
	Can1TxMsg.DLC = 8;				 //帧长度 范围是 0 到 0x8

	Can1TxMsg.StdId = id; //范围为 0 到 0x7FF

	memcpy(Can1TxMsg.Data, data, 8);

	u8 mbox = CAN_Transmit(can_x, &Can1TxMsg);
	delayUs(110);
}


