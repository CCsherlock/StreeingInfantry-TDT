#include "task_virtual.h"
//1、任务、对象都不允许删除
//2、只能在任务里面创建
//3、尽量不要调用关于该任务的挂起、恢复函数（避免多重挂起、恢复）

int VirtualTask::taskNum = 0;
VirtualTask **VirtualTask::taskList = 0;
VirtualTask::VirtualTask():taskHandler(NULL)
{
	if(taskList == 0)
	{
		taskList = (VirtualTask **)malloc(sizeof(VirtualTask *));
	}
	else
	{
		taskList = (VirtualTask **)realloc(taskList, sizeof(VirtualTask *) * taskNum+1);
	}
	
	taskList[taskNum] = this;
	taskNum+=1;
}

void VirtualTask::setTaskHandler(TaskHandle_t taskHandler)
{
	this->taskHandler = taskHandler;
	if (taskHandler == NULL)
	{
		this->taskHandler = xTaskGetCurrentTaskHandle();
	}
}

void VirtualTask::remoteCtrlUpdate()
{

}

void VirtualTask::deforceCallBack()
{
	if (taskHandler != NULL) //避免挂机当前任务
		vTaskSuspend(taskHandler);
}

void VirtualTask::deforceCancelCallBack()
{
	if (taskHandler != NULL) //避免挂机当前任务
		vTaskResume(taskHandler);
}