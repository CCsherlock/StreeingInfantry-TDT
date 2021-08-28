/*****************************************************************************
File name: TDT_Bsp\src\timer.h
Description: 定时器
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _TIMER_H
#define _TIMER_H

#include "board.h"
#ifdef __cplusplus
 extern "C" {
#endif	/*__cplusplus*/
	 
extern volatile unsigned long long FreeRTOSRunTimeTicks;
void ConfigureTimeForRunTimeStats(void);
void TIM2_IRQHandler(void);
	 
#ifdef __cplusplus
}
#endif	/*__cplusplus*/

#endif
