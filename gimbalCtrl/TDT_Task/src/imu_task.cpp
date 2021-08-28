/******************************
File name: TDT_Task\src\imu_task.cpp
Description: 陀螺仪姿态解算任务
function:
	——————————————————————————————————————————————————————————————————————————
	void Imu_Task(void *pvParameters)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "imu_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h"					//FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
/**TDT_Device************************/
#include "cycle.h"
#include "icm20602.h"
#include "flash_var.h"

eulerAngle angleForWatch;
accdata accForWatch;
gyrodata gyroForWatch;

float *visionSendYaw, *visionSendPitch;
Icm20602 GYROTDT(SPI1,100);
void startControlTasks();//等待imu初始化后开启控制任务
/**
  * @brief 陀螺仪任务
  * @note 负责数据读取和解算
  */
void Imu_Task(void *pvParameters)
{
	/*I2C以及icm20602初始化*/
    GYROTDT.init();
	IFlash.link(GYROTDT.gyro.offset, 0);
	IFlash.link(GYROTDT.acc.offset, 1);
	GYROTDT.getOffset();

	GYROTDT.imu_OK = 1;
	startControlTasks();
	Cycle imuCycle;
	//进入循环时的时间
	//视觉发送的值的初始化
	visionSendYaw = &GYROTDT.Angle.YAW_AXIS;
	visionSendPitch = &GYROTDT.Angle.PITCH_AXIS;
	TickType_t PreviousWakeTime=xTaskGetTickCount();
	while(1)
	{
		if(GYROTDT.forceGetOffset)
		{
			GYROTDT.getOffset();
		}
		//绝对延时
		vTaskDelayUntil(&PreviousWakeTime,pdMS_TO_TICKS(2));
	}
}

