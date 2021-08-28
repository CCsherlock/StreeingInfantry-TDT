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
#include "mpu6050.h"
#include "TimeMatch.h"
#include "imu.h"
#include "cycle.h"
#include "curve_model.h"
#include "icm20602.h"

#ifdef USE_MAIN_CTRL_2020
Mpu6050 Mpu6050_Top(GPIOC,GPIO_Pin_9,GPIO_Pin_8);
#endif
#ifdef USE_MAIN_CTRL_2019
Mpu6050 Mpu6050_Top(GPIOC,GPIO_Pin_2,GPIO_Pin_1);
#endif
extern tdtusart::timeSimulaneity imuTimeMatch;
//Icm20602 icm20602(SPI1,100);

/**
  * @brief 陀螺仪任务
  * @note 负责数据读取和解算
  */
void Imu_Task(void *pvParameters)
{
	/*I2C以及MPU6050初始化*/
    Mpu6050_Top.init(1000,42);
	  Mpu6050_Top.calOffset_Gyro();

	Cycle imuCycle;
	//进入循环时的时间
	TickType_t PreviousWakeTime=xTaskGetTickCount();
	while(1)
	{
		//绝对延时
		vTaskDelayUntil(&PreviousWakeTime,pdMS_TO_TICKS(2));	
		/*MPU6050读取*/
		Mpu6050_Top.TDT_IMU_update(imuCycle.getCycleT()/2);	
	}
}
 
