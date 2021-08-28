/******************************
  _________               _____       _________ 
 /___  ___/              / ___ \     /___  ___/  
    / /                 / /   \ \       / /     
   / /    _________    / /    / /      / /      
  / /    /___  ___/   / /    / /      / /       
 / /                 / /____/ /      / /        
/_/                 /________/      /_/     


【首次使用请先阅读Readme.md!!】
File name: User/main.cpp
Description: 完成初始化和开始任务的创建，以及打开调度器
function:
	——————————————————————————————————————————————————————————————————————————
	int main(void); //主入口函数
	——————————————————————————————————————————————————————————————————————————
Frame Version: #1.3.3.0113alpha
History: 
	——————————————————————————————————————————————————————————————————————————
	You can get some infomation from "readme.md"
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/

#include "board.h"
#include "cycle.h"
#include "my_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h"					//FreeRTOS使用	 
#include "task.h"
/**FreeRTOS*END***************/
//#include "icm20602.h"

/*开始任务*///用于创建任务、队列、定时器
/*-摘要-*/	#define START
/*-优先-*/	#define START_TASK_PRIO		1
/*-堆栈-*/	#define START_STK_SIZE 		100  
/*-句柄-*/	TaskHandle_t StartTask_Handler;
/*-声明-*/	extern void Start_Task(void *pvParameters);
             
//extern Icm20602 icm20602;

/*通用任务的宏定义在 my_task.h 下*/
int main(void)

{
	/*中断分组*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    /*滴答定时器配置*/
    sysTickInit();
	/*创建开始任务*/
	mTaskCreate(START,Start);
	/*开启任务调度器*/
    vTaskStartScheduler();
}
















