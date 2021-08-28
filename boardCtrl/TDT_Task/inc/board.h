/*****************************************************************************
File name: TDT_Task\src\board.h
Description: 滴答定时器的初始化和延时功能，初始化

Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次记录
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

/***************硬件中断分组******************/
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"
//DSP库
#include "arm_math.h"
#include "TDT_USER.h"
	 
#ifdef USE_MAIN_CTRL_2019
#define DEF_MAIN_CTRL_VER
#endif
     
#ifdef USE_MAIN_CTRL_2020
#ifdef DEF_MAIN_CTRL_VER
#error redefine Main Control Version
#endif
#define DEF_MAIN_CTRL_VER
#endif


#ifndef DEF_MAIN_CTRL_VER
#error undef Main Control Version
#endif

#define USE_JUDGEMENT 1

//滴答定时器再分频，MPRE次中断为1ms
#define MPRE 4
//#define OS

#ifndef _Bool
	#define _Bool u8
#endif
enum IsEnable{
	enable=1,disable=0
};

//以下宏定义用于获取枚举个数（通过行数）,需要分别定义以下两个宏，且'|'内的内容为ENUM_NUM(x)中的x，并且枚举中不能含有回车，可放在keil的代码模板Templates中
#define ENUM_START(x) const unsigned char x##_ENUM_START_LINE = __LINE__;enum x
#define ENUM_END(x) const unsigned char x##_ENUM_END_LINE = __LINE__;
//以下宏获取灭据个数时使用，需要与枚举定义处在同一个命名空间下
#define ENUM_NUM(x) (x##_ENUM_END_LINE - x##_ENUM_START_LINE - 3)
//以下宏定义枚举变量时使用
#define ENUM(x) x
//以下宏定义用于重载枚举的++运算符，仅限在任意一个.c(.cpp)中使用
#define USE_OPERATOR_AUTO_INC(x) inline x operator++(x &rs,int){x oldEnum = rs;if ((int)rs < ENUM_NUM(x) - 1)rs = (x)(rs + 1);else rs = (x)0;return oldEnum;}


extern volatile uint32_t sysTickUptime;
extern volatile uint32_t sysTickUptimeUs;

void sysTickInit(void);
uint32_t getSysTimeUs(void);
void delayUs(uint32_t us);
void delayMs(uint32_t ms);
void boardALLInit(void);

#ifdef __cplusplus
extern "C" {
#endif
	
void delayUsTask(u32 us);
	
#ifdef __cplusplus
}
#endif


#endif /* __BOARD_H__ */

