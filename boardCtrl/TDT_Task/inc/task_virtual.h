#ifndef __TASK_VIRTUAL_H
#define __TASK_VIRTUAL_H
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/

class VirtualTask
{
	public:
	//构造函数
	VirtualTask();

	void setTaskHandler(TaskHandle_t taskHandler = NULL);

	//可重写，遥控数据更新时触发
	virtual void remoteCtrlUpdate();

	//脱力
	virtual void deforceCallBack();
	//取消脱力
	virtual void deforceCancelCallBack();
		
	//任务列表
	static VirtualTask **taskList;
	//任务数量
	static int taskNum;
	
private:
	TaskHandle_t taskHandler;

};

#endif
