/******************************
File name: TDT_Task\src\board.cpp
Description: 滴答定时器的初始化和延时功能，初始化
function:
	——————————————————————————————————————————————————————————————————————————
	void TDT_SysTick_Init(void)
	——————————————————————————————————————————————————————————————————————————
	uint32_t GetSysTime_us(void)
	——————————————————————————————————————————————————————————————————————————
	void DelayUs(uint32_t us)
	——————————————————————————————————————————————————————————————————————————
	void DelayMs(uint32_t ms)
	——————————————————————————————————————————————————————————————————————————
	void DelayUs_Task(u32 nms)
	——————————————————————————————————————————————————————————————————————————
	void TDT_Board_ALL_Init(void)
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次记录
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "board.h"


/**FreeRTOS*START***************/
#include "FreeRTOS.h"					//FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "cycle.h"
#include "iwdg.h"
#include "can.h"
/***宏定义***/
#define TICK_PER_SECOND 1000
#define TICK_US	(1000000/TICK_PER_SECOND)

/***全局变量***/
/*初始化完成标志*/
u8 Init_OK;
//滴答定时器计数变量 ,49天后溢出
volatile uint32_t sysTickUptime=0;
volatile uint32_t sysTickUptimeUs=0;
int* * *a;
/**
  * @brief 初始化滴答定时器
  * @note 如果修改外部晶振,记得修改 HSE_VALUE，PLL_M
  */
void sysTickInit(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t           cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
	cnts = cnts / (8*MPRE);//滴答定时器1/4ms触发一次中断
	//cnts=168000/8;=1ms
	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}




/**
  * @brief 获取us级时间
  * @return us级时间
  * @note 滴答定时器分频在此函数处理好像会损失精度
  */
uint32_t getSysTimeUs(void)
{
	uint32_t ms;
	u32 value;
	ms = sysTickUptime;
	value = (ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD) / MPRE;
	return value;
}



/**
  * @brief US级延时
  * @param us 延时时长，单位US
  * @note 注意在操作系统运行下使用此函数务必进入临界区，否则精度不保证
		并且使用滴答延时会造成极大的开销，时间越长，开销越大，尽量在小几百毫秒以内
  */
void delayUs(uint32_t us)
{
	uint32_t now = getSysTimeUs();
	uint32_t cnt = 0;
	uint32_t ticks = 0;
	uint32_t told = 0;
	uint32_t tnow = 0;
	uint32_t tcnt = 0;
	uint32_t reload = 0;
	reload = SysTick->LOAD;
	ticks = us * 168 / 8;
	told = SysTick->VAL;
	while (1)
	{
		tnow = SysTick->VAL;
		if (tnow != told)
		{
			if (tnow < told)
			{
				tcnt += told - tnow;
			}
			else
			{
				tcnt += reload - tnow + told;
			}
			told = tnow;
			if (tcnt >= ticks)
			{
				break;
			}
		}
		iwdgFeed();
	}
	cnt = getSysTimeUs() - now;
}



/**
  * @brief MS级延时
  * @param ms 延时时长，单位MS
  * @return
  * @note 注意在操作系统运行下使用此函数务必进入临界区，否则精度不保证
  */
void delayMs(uint32_t ms)
{
	delayUs(ms*1000);
}



/**
  * @brief 任务级延时函数
  * @param us延时时长，单位US
  * @note 要延时的us数
  */
void delayUsTask(u32 us)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
	{
		if(us>=1000)						//延时的时间大于OS的最少时间周期
		{
   			vTaskDelay(pdMS_TO_TICKS(us/1000));	 		//FreeRTOS延时
		}
		us%=1000;						//OS已经无法提供这么小的延时了,采用普通方式延时
	}
	delayUs((u32)(us));				//普通方式延时
}



/**
  * @brief 总初始化函数
  * @note 禁止使用延时
  */
void boardALLInit(void)
{
	/* 禁止全局中断*/
	__disable_irq();
	/*CAN1初始化*/
	canInit(CAN1);
	/*CAN2初始化*/
	canInit(CAN2);
	 /*看门狗初始化-喂狗在LED*/
	iwdgInit(4,70);
	/*初始化完成*/
	Init_OK = 1;
	/*  使能全局中断 */
	__enable_irq();
}
/***********End of file*****************  */
