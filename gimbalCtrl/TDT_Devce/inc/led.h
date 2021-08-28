/*****************************************************************************
File name: TDT_Device\src\led.h
Description: LED灯
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __LED_H__
#define __LED_H__

#include "board.h"
/***************LED GPIO定义******************/

class Led
{
private:
	uint32_t RCC_AHB1Periph_GPIOx; /*RCC_LED*/
	GPIO_TypeDef *GPIOx;		   /*LED_PORT*/
	uint16_t GPIO_Pin_x;		   /*LED_Pin*/

	int8_t stateLast;
	int8_t stateCounter;

	u8 showState;

	u16 stateOnSpan;
	u16 stateOffSpan;
	u16 stateResetTime;

	u16 slowBlinkInterval;
	u16 fastBlinkInterval;

	u16 blinkTimer;

public:
	Led(uint32_t RCC_AHB1Periph_GPIOx, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x, u16 stateOnSpan = 50, 
		u16 stateOffSpan = 200, u16 slowBlinkInterval = 500, u16 fastBlinkInterval = 100,u16 stateResetTime = 1000,u8 MAX_PRIORITY_NUM = 10);

	void init(void);

	inline void setStateOnSpan(u16 stateOnSpan) { this->stateOnSpan = stateOnSpan; };

	inline void setStateOffSpan(u16 stateOffSpan) { this->stateOffSpan = stateOffSpan; };

	inline void setSlowBlinkInterval(u16 slowBlinkInterval){this->slowBlinkInterval = slowBlinkInterval;};

	inline void setFastBlinkInterval(u16 fastBlinkInterval){this->fastBlinkInterval = fastBlinkInterval;};

	inline void setStateResetTime(u16 stateResetTime){this->stateResetTime = stateResetTime;};
	/**
	 * @brief led闪烁状态控制
	 * 
	 * @param state 正数代表错误状态，-1代表慢闪，-2代表快闪
	 * @param intervalMs 距离上一次多长时间
	 */
	void stateShow(u16 intervalMs, int8_t state);
	void show(u8 state = 1);
	inline u8 getState(){ return (GPIOx->ODR & GPIO_Pin_x); };
	inline void toggle() { GPIOx->ODR ^= GPIO_Pin_x; };
	
	u8 MAX_PRIORITY_NUM;
	int8_t *ErrorCode;//错误码缓存区，最多允许同时存在MAX_PRIORITY_NUM个错误码
/**
  * @brief 错误值 传递
  * @param [优先级(负数则表示后往前数），错误值(0则保持该优先级原有错误码，-1代表慢闪，-2代表快闪)]
  */
	void setError(int8_t priority, int8_t ES_State);

	int8_t getError();
	inline int8_t getError(u8 priority)
	{
		if(priority > MAX_PRIORITY_NUM)
			return 0;
		return ErrorCode[priority];
	}
};



#endif
