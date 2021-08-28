/*
 * @Author: your name
 * @Date: 2021-03-03 17:38:56
 * @LastEditTime: 2021-03-06 16:54:58
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Projectd:\RM2021\RTOS_Frame\TDT_Task\src\led_task.cpp
 */
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

/**FreeRTOS*START***************/
#include "FreeRTOS.h"					//FreeRTOS使用	 
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "led.h"
#include "iwdg.h"


//定义板载lED灯对象
Led boardLed = Led(RCC_AHB1Periph_GPIOB , GPIOB , GPIO_Pin_14);

//队列句柄定义
QueueHandle_t	LED_Queue;	//LED消息队列句柄


/**
  * @brief LED任务函数
  * @note 负责LED的控制和喂狗
  */
  float freeHeap=0;
void Led_Task(void *pvParameters)
{
	
    /*LED初始化*/
	boardLed.init();
	boardLed.show(1);
	//创建LED接收队列-项目数1，长度1字节
	LED_Queue = xQueueCreate(1, 1);
	u8 ledState=2;
    u8 visioncnt = 0;
    while(1)
	{
		//趁机喂狗
		iwdgFeed();
		//接收队列消息，不等待
		xQueueReceive(LED_Queue, &ledState, 0);
		//LED状态灯
		boardLed.stateShow(ledState);
		//延时50ms
		vTaskDelay(pdMS_TO_TICKS(25));
		freeHeap=xPortGetFreeHeapSize();			/*打印剩余堆栈大小*/
    }
}





