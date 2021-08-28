/******************************
File name: TDT_Task\src\start_task.cpp
Description: 开始任务
function:
	——————————————————————————————————————————————————————————————————————————
	void Start_Task(void* pvParameters)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "start_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "judgement.h"
#include "my_task.h"

/*开始任务*/
extern TaskHandle_t StartTask_Handler;

/*呼吸灯任务*/
/*-摘要-*/ #define LED
	/*-优先-*/ #define LED_TASK_PRIO 1
	/*-堆栈-*/ #define LED_STK_SIZE 100
	/*-句柄-*/ TaskHandle_t LedTask_Handler;
/*-声明-*/ extern void Led_Task(void *pvParameters);

/*陀螺仪解算任务*/
/*-摘要-*/ #define IMU
	/*-优先-*/ #define IMU_TASK_PRIO 4
	/*-堆栈-*/ #define IMU_STK_SIZE 300
	/*-句柄-*/ TaskHandle_t ImuTask_Handler;
/*-声明-*/ extern void Imu_Task(void *pvParameters);

/*遥控器数据处理任务*/
/*-摘要-*/ #define DBUS
	/*-优先-*/ #define DBUS_TASK_PRIO 6
	/*-堆栈-*/ #define DBUS_STK_SIZE 100
	/*-句柄-*/ TaskHandle_t DbusTask_Handler;
/*-声明-*/ extern void Dbus_Task(void *pvParameters);

/*底盘任务*/
/*-摘要-*/ #define CHASSIS
	/*-优先-*/ #define CHASSIS_TASK_PRIO 1
	/*-堆栈-*/ #define CHASSIS_STK_SIZE 1000
	/*-句柄-*/ TaskHandle_t ChassisTask_Handler;
/*-声明-*/ extern void Chassis_Task(void *pvParameters);

int8_t creatResult[10] = {-1};

/**
  * @brief 开始任务
  * @note 用于创建任务，完事删了自己
  */
void Start_Task(void *pvParameters)
{
	u8 taskNum = 0;		  //任务总数量
	taskENTER_CRITICAL(); //进入临界区
	//创建其他任务
	creatResult[taskNum++] = mTaskCreate(LED, Led); //LED任务
	creatResult[taskNum++] = mTaskCreate(CHASSIS, Chassis);

	/*初始化*/
	boardALLInit();

	//删除开始任务
	vTaskDelete(StartTask_Handler);
	taskEXIT_CRITICAL(); //退出临界区
}
