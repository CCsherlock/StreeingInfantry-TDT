/*****************************************************************************
File name: TDT_Task\src\imu_task.h
Description: 陀螺仪姿态解算任务
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __IMU_TASK_H__
#define __IMU_TASK_H__

#include "board.h"
#include "icm20602.h"

extern Icm20602 GYROTDT;

extern float *visionSendYaw, *visionSendPitch;
#define PITCH_AXIS pitch
#define YAW_AXIS yaw
#endif
