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
	/*-优先-*/ #define IMU_TASK_PRIO 7
	/*-堆栈-*/ #define IMU_STK_SIZE 1000
	/*-句柄-*/ TaskHandle_t ImuTask_Handler;
/*-声明-*/ extern void Imu_Task(void *pvParameters);

/*遥控器数据处理任务*/
/*-摘要-*/ #define DBUS
	/*-优先-*/ #define DBUS_TASK_PRIO 7
	/*-堆栈-*/ #define DBUS_STK_SIZE 500
	/*-句柄-*/ TaskHandle_t DbusTask_Handler;
/*-声明-*/ extern void Dbus_Task(void *pvParameters);

/*云台处理任务*/
/*-摘要-*/ #define GIMBAL
	/*-优先-*/ #define GIMBAL_TASK_PRIO 5
	/*-堆栈-*/ #define GIMBAL_STK_SIZE 500
	/*-句柄-*/ TaskHandle_t GimbalTask_Handler;
/*-声明-*/ extern void Gimbal_Task(void *pvParameters);

/*开火处理任务*/
/*-摘要-*/ #define FIRE
	/*-优先-*/ #define FIRE_TASK_PRIO 5
	/*-堆栈-*/ #define FIRE_STK_SIZE 500
	/*-句柄-*/ TaskHandle_t FireTask_Handler;
/*-声明-*/ extern void Fire_Task(void *pvParameters);

/*弹仓盖处理任务*/
/*-摘要-*/ #define AMMO_COVER
	/*-优先-*/ #define AMMO_COVER_TASK_PRIO 5
	/*-堆栈-*/ #define AMMO_COVER_STK_SIZE 500
	/*-句柄-*/ TaskHandle_t AmmoCoverTask_Handler;
/*-声明-*/ extern void AmmoCover_Task(void *pvParameters);

/*状态更新处理任务*/
/*-摘要-*/ #define STATE
	/*-优先-*/ #define STATE_TASK_PRIO 6
	/*-堆栈-*/ #define STATE_STK_SIZE 500
	/*-句柄-*/ TaskHandle_t StateTask_Handler;
/*-声明-*/ extern void State_Task(void *pvParameters);

/*底盘处理任务*/
/*-摘要-*/ #define STEERING_TOP
	/*-优先-*/ #define STEERING_TOP_TASK_PRIO 5
	/*-堆栈-*/ #define STEERING_TOP_STK_SIZE 1000
	/*-句柄-*/ TaskHandle_t SteeringTopTask_Handler;
/*-声明-*/ extern void SteeringTop_Task(void *pvParameters);

/*自动对位处理任务*/
/*-摘要-*/ #define GETGY53
	/*-优先-*/ #define GETGY53_TASK_PRIO 5
	/*-堆栈-*/ #define GETGY53_STK_SIZE 1000
	/*-句柄-*/ TaskHandle_t GetGY53Task_Handler;
/*-声明-*/ extern void GetGY53_Task(void *pvParameters);

int8_t creatResult[10] = {-1};
u8 taskNum = 0; //任务总数量

/**
  * @brief 开始任务
  * @note 用于创建任务，完事删了自己
  */
void Start_Task(void *pvParameters)
{
	taskENTER_CRITICAL(); //进入临界区
	//创建其他任务
	creatResult[taskNum++] = mTaskCreate(LED, Led);			//LED任务
	creatResult[taskNum++] = mTaskCreate(IMU, Imu);			//陀螺仪任务
	creatResult[taskNum++] = mTaskCreate(DBUS, Dbus);		//遥控器解算任务
	creatResult[taskNum++] = mTaskCreate(STATE, State);		//状态更新任务
	creatResult[taskNum++] = mTaskCreate(GETGY53, GetGY53); //LED任务

	/*初始化*/
	boardALLInit();

	//删除开始任务
	vTaskDelete(StartTask_Handler);
	taskEXIT_CRITICAL(); //退出临界区
}

void startControlTasks()
{
	creatResult[taskNum++] = mTaskCreate(GIMBAL, Gimbal);			 //云台任务
	creatResult[taskNum++] = mTaskCreate(FIRE, Fire);				 //开火任务
	creatResult[taskNum++] = mTaskCreate(AMMO_COVER, AmmoCover);	 //弹仓盖任务
	creatResult[taskNum++] = mTaskCreate(STEERING_TOP, SteeringTop); //底盘任务
}
