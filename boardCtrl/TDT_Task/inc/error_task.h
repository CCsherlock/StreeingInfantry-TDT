/*****************************************************************************
File name: TDT_Task\src\error_task.h
Description: 异常处理任务
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __ERROR_TASK_H__
#define __ERROR_TASK_H__

#include "board.h"

enum _ErrorCode//错误码
{
	normal = 0,							//状态-正常
	_normal = 1,							//状态-正常
	motorLost,		//电机离线
	dbusLost,		//遥控器离线
	motorOverTemp,	//电机过热
	pidUnLoad,		//pid参数未加载
	imuLost,		//陀螺仪离线
	unLoadMotor,	//存在未加载的电机
	nullHandle		//指向空的句柄
};


//extern _ErrorCode ErrorCode;
void Error(_ErrorCode ES_State , u8 state=1);


#endif
