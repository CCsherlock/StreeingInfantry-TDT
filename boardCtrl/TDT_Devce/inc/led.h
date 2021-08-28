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



class Led{
private:
		
	uint32_t RCC_AHB1Periph_GPIOx;	/*RCC_LED*/
	GPIO_TypeDef * GPIOx;	/*LED_PORT*/
	uint16_t GPIO_Pin_x;		/*LED_Pin*/
public:

	Led(uint32_t RCC_AHB1Periph_GPIOx,GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin_x);
	void init(void);
	void stateShow(u8 state);
	void show(u8 state=1);

};


	
#endif
