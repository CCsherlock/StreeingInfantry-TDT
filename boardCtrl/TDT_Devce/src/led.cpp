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
Led::Led(uint32_t RCC_AHB1Periph_GPIOx,GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin_x)
{
	this->RCC_AHB1Periph_GPIOx=RCC_AHB1Periph_GPIOx;
	this->GPIOx=GPIOx;
	this->GPIO_Pin_x=GPIO_Pin_x;
}

/**
  * @brief  LED初始化
  * @waring 记得设置三个变量的值
  */
void Led::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOx , ENABLE );
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_x;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOx,&GPIO_InitStructure);
	GPIOx->BSRRL = GPIO_Pin_x;
}

/**
  * @brief  LED状态灯
  * @param  state：状态值
  * @note	可根据状态值闪烁不同的次数
  */
void Led::stateShow(u8 state)
{
	if(state==0)
	{
		state=1;
	}
	static	uint16_t flash_cnt;
	static	u8 flash_flag=0;
	static	uint16_t state_last;
	static	uint16_t state_value;
	static	uint16_t T=300;// 闪烁周期单位ms
	if(state!=state_last)//判断是否值改变
	{
		state_value=T/50+(state-1)*(T/25);
		state_last=state;
		flash_flag=0;
		GPIOx->BSRRL = GPIO_Pin_x;
	}
	if(flash_flag==0)					
	{
		if(state_value%(T/50)==0)
		{
			GPIOx->ODR ^= GPIO_Pin_x;
		}
		if(state_value==0)
		{
			flash_flag=1;
			state_value=T/50+(state-1)*(T/25);
		}
		else
		{
			state_value--;
		}
	}
	if(flash_flag==1)
	{
		flash_cnt++;
	}
	if(flash_cnt==30)//1s根据调用此函数的位置计算	
	{
		flash_flag=0;
		GPIOx->BSRRL = GPIO_Pin_x;
		flash_cnt=0;
	}
}

/**
  * @brief  LED显示
  * @param  state：状态值
  * @note	
  */
void Led::show(u8 state)
{
	if(state==0)
		GPIOx->BSRRL = GPIO_Pin_x;
	else
		GPIOx->BSRRH = GPIO_Pin_x;
}




