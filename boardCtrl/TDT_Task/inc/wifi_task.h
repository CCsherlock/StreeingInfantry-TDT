/***************************************************************************** 
File name: TDT_Bsp\src\usart.h 
Description: 串口 
Author: 祖传 
Version: 1.1.1.191112_alpha 
Date: 19.11.12 
History:  
	—————————————————————————————————————————————————————————————————————————— 
	19.11.12 首次完成 
	—————————————————————————————————————————————————————————————————————————— 
*****************************************************************************/ 
#ifndef __USART_H__ 
#define __USART_H__ 
#include "board.h" 
 
#pragma pack(1) 
 
typedef struct 
{ 
    u8 frameHeader;//帧头 固定不动
	/*========================↓Replace↓========================*/
	float sysUptime;
    float gimbalTopAngle_Pitch; 
    float gimbalTopAngle_Yaw; 
	float gimbalTopAngle_Roll;
	/*========================↑Replace↑========================*/
	char RESERVED1;//固定不动 
    char RESERVED2;//固定不动 
}ESP8266_SendStruct_t; 
 
 
typedef struct 
{ 
	u8 frameHeader;//帧头 固定不动 
	/*========================↓Replace↓========================*/
    float pitchKp; 
    float pitchKd; 
	float pitchResultMax; 
	float yawKp; 
    float yawKi; 
	float yawIntegralMax; 
	float yawKd; 
	float yawResultMax; 
	float speedKp; 
    float speedKi; 
	float speedIntegralMax; 
	float speedKd; 
	float speedResultMax; 
	float speedSetAmp; 
	float yawSetAmp; 
	float gyroKp; 
	float gyroKi; 
	float deforceAngle;
	/*========================↑Replace↑========================*/
}ESP8266_RequestStruct_t; 
#pragma pack() 
 
 
void TDT_usart_monitor_Init(void); 
#ifdef __cplusplus 
extern "C"{ 
#endif	/*__cplusplus*/ 
void USART6_IRQHandler(void); 
#ifdef __cplusplus 
} 
#endif	/*__cplusplus*/ 
 
 
extern ESP8266_SendStruct_t esp8266_SendStruct; 
 
 
#endif 
