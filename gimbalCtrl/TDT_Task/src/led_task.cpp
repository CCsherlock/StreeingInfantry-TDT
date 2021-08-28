/******************************
File name: TDT_Task\src\led_task.cpp
Description: 呼吸灯控制任务
function:
	——————————————————————————————————————————————————————————————————————————
	void Led_Task(void *pvParameters)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "led_task.h"
#include "dbus_task.h"
//#include "gimbal_task.h"
#include "state_task.h"
#include "dbus_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "led.h"
#include "iwdg.h"
#include "vision.h"

//定义板载lED灯对象
Led boardLed = Led(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_14);
Led Laser = Led(RCC_AHB1Periph_GPIOC, GPIOC, GPIO_Pin_8); //21A板

//队列句柄定义
QueueHandle_t LED_Queue; //LED消息队列句柄

/**
  * @brief LED任务函数
  * @note 负责LED的控制和喂狗
  */
float freeHeap = 0;
void Led_Task(void *pvParameters)
{

	/*LED初始化*/
	boardLed.init();
	Laser.init();
	boardLed.show(1);
	Laser.show(1); //灭
	//创建LED接收队列-项目数1，长度1字节
	LED_Queue = xQueueCreate(1, 1);
	u8 ledState = 2;
	u8 visioncnt = 0;
	while (1)
	{
		//趁机喂狗
		iwdgFeed();
		boardLed.stateShow(20, ledState);
		if (visioncnt++ == 20)
		{
			if (visionInfo.visionCnt == 0)
			{
				visionInfo.offlineFlag = 1;
				visionInfo.visionFPS = 0;
			}
			else
			{
				visionInfo.visionFPS = visionInfo.visionCnt;
				visionInfo.visionCnt = 0;
			}
			visioncnt = 0;
		}
		//延时50ms
		vTaskDelay(pdMS_TO_TICKS(25));
		freeHeap = xPortGetFreeHeapSize(); /*打印剩余堆栈大小*/
	}
}
