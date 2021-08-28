/*****************************************************************************
File name: TDT_Device\src\dbus.h
Description: 遥控器接收机
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加切换为原始数据处理的变量
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _Dbus_H
#define _Dbus_H

//#include "board.h"

#ifdef __DBUS_DRIVER_GLOBALS
#define __DBUS_DRIVER_EXT
#else
#define __DBUS_DRIVER_EXT extern
#endif

#include "stm32f4xx.h"
#include "TDT_User.h"
#include <stdio.h>
#include "my_math.h"
//#include "error.h"

#define SAFE_STATE
#define NORMAL_STATE
#define TEST_STATE

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

	typedef struct _RC RC_t;

#define KEY_B 0x8000
#define KEY_V 0x4000
#define KEY_C 0x2000
#define KEY_X 0x1000
#define KEY_Z 0x0800
#define KEY_G 0x0400
#define KEY_F 0x0200
#define KEY_R 0x0100
#define KEY_E 0x0080
#define KEY_Q 0x0040
#define KEY_CTRL 0x0020
#define KEY_SHIFT 0x0010
#define KEY_D 0x0008
#define KEY_A 0x0004
#define KEY_S 0x0002
#define KEY_W 0x0001

namespace RCS //RemoteCtrl nameSpace
{
	//拨杆位置
	enum SWPos
	{
		Lost = 0,
		Up = 1,
		Mid = 3,
		Down = 2
	};
	//按键跳变枚举，如Up_Mid指上面拨到中间，值为当前值减上一个位置值
	enum SWTick
	{
		Up_Mid = 2,
		Mid_Up = -2,
		Mid_Down = -1,
		Down_Mid = 1
	};
}

using namespace RCS;
typedef struct
{
	int16_t CH[11];
	SWPos SW1;
	SWPos SW2;
	u8 left_jump;
	u8 Right_jump;
	union
	{
		uint16_t keyValue;
		struct
		{
			//必须按顺序
			uint16_t W : 1;		//0x0001
			uint16_t S : 1;		//0x0002
			uint16_t A : 1;		//0x0004
			uint16_t D : 1;		//0x0008
			uint16_t SHIFT : 1; //0x0010
			uint16_t CTRL : 1;	//0x0020
			uint16_t Q : 1;		//0x0040
			uint16_t E : 1;		//0x0080
			uint16_t R : 1;		//0x0100
			uint16_t F : 1;		//0x0200
			uint16_t G : 1;		//0x0400
			uint16_t Z : 1;		//0x0800
			uint16_t X : 1;		//0x1000
			uint16_t C : 1;		//0x2000
			uint16_t V : 1;		//0x4000
			uint16_t B : 1;		//0x8000
		};
	};
} _Key;

/**
* @struct  _RC
* @brief   定义遥控器结构体
*/
struct _RC
{

	SWPos SWPos;
	_Key Key;		//当前值
	_Key LastKey;	//上次值
	_Key KeyTick;	//跳变值
	_Key KeyPress;	//按下值
	SWTick SW1Tick; //左上角拨杆跳变值
	SWTick SW2Tick; //右上角拨杆跳变值
};

__DBUS_DRIVER_EXT struct _RC RC;

void Dbus_Config(void);
static void Handle_data(volatile const uint8_t *sbus_buf, struct _RC *rc_ctrl);

int Get_Keypress(uint16_t Key);

void Dbus_Key_Zero_Set(void);

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

	void USART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*_Dbus_H*/
