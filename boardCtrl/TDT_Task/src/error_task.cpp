/******************************
File name: TDT_Task\src\error_task.cpp
Description: 异常处理任务
function:
	——————————————————————————————————————————————————————————————————————————
	void Error_Fun(ErrorCode ES_State)
	——————————————————————————————————————————————————————————————————————————
	void Error_Task(void *pvParameters)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "error_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h"					//FreeRTOS使用	 
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "can.h"
#include "start_task.h"
#include "my_task.h"
#include "error_task.h"


/*电机对象使用状态*/
//_ErrorCode ErrorCode;



//和错误码一一对应
char ErrorChar[10][25]={
	"All OK",	//异常号为0认为正常
	"All OK",	//异常号为1认为正常
	"Motor Lost",
	"Dbus Lost",
	"Over Temp"
	"Pid Un Load",
	"Imu Lost",
	"Unload Motor"
	"Null Handle"
};
u32 errCode=0;//32位错误码，最多支持32个错误位

/*
LED灯ID表：
0：正常
1：正常
2：电机离线
3：遥控器离线
4：电机过热
5：其他错误
*/


/**
  * @brief 错误值 传递
  * @param [错误值,*状态(0:失能,1:使能(默认))]
  */
#define MAX_ERR_NUM 10
u8 ErrorCode[MAX_ERR_NUM];//错误码缓存区，最多允许同时存在十个错误码
void Error(_ErrorCode ES_State , u8 state)
{
	//轮询所有变量，找到空的位置赋值
	for(u8 i=0;i<MAX_ERR_NUM;i++)
	{
		//如果该异常已经记录，直接修改，如果是个新的异常，直接赋值
		if(ErrorCode[i] == ES_State || ErrorCode[i]==0)
		{
			//如果是异常的使能就赋值为异常号，是异常的失能就置零
			ErrorCode[i]=state==1 ? ES_State:0;
			break;
		}
	}
	
	extern QueueHandle_t	LED_Queue;	//LED消息队列句柄
	//LED只展示位置最先的异常（表现为位于数组的第一个有效数据）
	for(u8 i=0;i<MAX_ERR_NUM;i++)
	{
		if(ErrorCode[i]!=0)
		{
			u16 LED_ID=ErrorCode[i];
			//向LED状态队列中填充内容，覆盖旧的内容
			xQueueOverwrite(LED_Queue,&LED_ID);
		}
	}
}




/**
  * @brief 脱力任务
  * @note 脱力时挂起其他任务，恢复脱力任务可以实现脱力
  */
void Stop_Task(void *pvParameters)
{
	float zero[4]={0,0,0,0};
	while(1)
	{
		//等待DBUS任务通知，退出函数时通知值清零，并且读取最新数据
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		/*▲ 零值输出*/
		
		canTx(zero,CAN1,0x200);
		canTx(zero,CAN1,0x1ff);
		canTx(zero,CAN2,0x200);
		canTx(zero,CAN2,0x1ff);
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}
















