/******************************
File name: TDT_Device\src\led.cpp
Description: LED灯
function:
	——————————————————————————————————————————————————————————————————————————
	void TDT_Led_Init(void)
	——————————————————————————————————————————————————————————————————————————
	void LED_State_Show(u8 state)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.20 肖银河-封装为类，提供给其他具有LED灯性质的外设使用
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "led.h"

/**
  * @brief  LED类构造器
  * @waring 记得设置三个变量的值
  */
Led::Led(uint32_t RCC_AHB1Periph_GPIOx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x, 
	u16 stateOnSpan, u16 stateOffSpan, u16 slowBlinkInterval, 
	u16 fastBlinkInterval,u16 stateResetTime,u8 MAX_PRIORITY_NUM) : 
RCC_AHB1Periph_GPIOx(RCC_AHB1Periph_GPIOx),GPIOx(GPIOx),GPIO_Pin_x(GPIO_Pin_x),
stateOnSpan(stateOnSpan),stateOffSpan(stateOffSpan),slowBlinkInterval(slowBlinkInterval),
fastBlinkInterval(fastBlinkInterval),stateResetTime(stateResetTime),MAX_PRIORITY_NUM(MAX_PRIORITY_NUM),
ErrorCode(new int8_t[MAX_PRIORITY_NUM])
{
	for(int i = 0; i < MAX_PRIORITY_NUM; i++)
		ErrorCode[i] = -1;
}

/**
  * @brief  LED初始化
  * @waring 记得设置三个变量的值
  */
void Led::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOx, &GPIO_InitStructure);
	GPIOx->BSRRL = GPIO_Pin_x;
}

/**
  * @brief  LED状态灯
  * @param  state：状态值
  * @note	可根据状态值闪烁不同的次数
  */
void Led::stateShow(u16 intervalMs, int8_t state=0)
{
	if(state==0) state = getError();
	blinkTimer += intervalMs;
	if (state == -1)
	{
		if (blinkTimer < slowBlinkInterval)
			return;
		blinkTimer = 0;
		toggle();
		return;
	}
	if (state == -2)
	{
		if (blinkTimer < fastBlinkInterval)
			return;
		blinkTimer = 0;
		toggle();
		return;
	}

	if (state == 0)
	{
		state = -1;
	}

	if (state != stateLast) //判断是否值改变
	{
		blinkTimer = 0;
		stateLast = state;
		show(0);
		return;
	}
	
	if(blinkTimer > stateResetTime + stateOffSpan && stateCounter >= state)
	{
		stateCounter = 0;
	}

	if(showState == 0 && blinkTimer > stateOffSpan && stateCounter < state)
	{
		show(1);
		blinkTimer = 0;
		return;
	}
	
	if(showState == 1 && blinkTimer > stateOnSpan && stateCounter < state)
	{
		show(0);
		blinkTimer = 0;
		stateCounter++;
		return;
	}
}
/**
  * @brief  LED显示
  * @param  state：状态值
  * @note	
  */
void Led::show(u8 state)
{
	showState  = state;
	if (state == 0)
	{
		GPIOx->BSRRL = GPIO_Pin_x;
		return;
	}
	GPIOx->BSRRH = GPIO_Pin_x;
}

void Led::setError(int8_t priority, int8_t ES_State)
{
	if(priority < 0)
		priority = MAX_PRIORITY_NUM - priority;
	if(priority < 0)
		return;
	if(priority > MAX_PRIORITY_NUM)
		return;
	if(ES_State != 0)
		ErrorCode[priority] = ES_State;
}

int8_t Led::getError()
{
	//LED只展示位置最先的异常（表现为位于数组的第一个有效数据）
	for(u8 i=0;i<MAX_PRIORITY_NUM;i++)
	{
		if(ErrorCode[i] != -1)
		{
			return ErrorCode[i];
		}
	}
	return -1;
}
