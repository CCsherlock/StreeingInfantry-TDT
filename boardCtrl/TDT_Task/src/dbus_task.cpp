/******************************
File name: TDT_Task\src\dbus_task.cpp
Description: 遥控器数据解算任务以及命令凑的传达
function:
	——————————————————————————————————————————————————————————————————————————
	void Dbus_Task(void *pvParameters)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "dbus_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
/**TDT_Alg************************/
#include "my_math.h"
#include "dbus.h"
#include "start_task.h"
#include "motor.h"
#include "error_task.h"
#include "mpu6050.h"
#include "task_virtual.h"
#include "KeyProcess.h"

//全局脱力标志位
uint8_t deforceFlag = 1;

//辅助函数声明
static void taskSchedule();
static void dbusHandleData(uint8_t *SBUS_buf);



void rstCount(void);
/**
  * @brief 遥控器数据处理任务
  * @note  任务调度也在这儿
  */
void Dbus_Task(void *pvParameters)
{
	uint8_t *SBUS_buf_task;
	deforceFlag = 1; //若开启任务，则先脱力，防止电机发送值

	u8 dbusOnlineFlag = 0; //dbus离线标志位

	/*DBUS初始化*/
	Dbus_Config();
	while (1)
	{
		//▲ 等待任务通知，退出函数时通知值清零，并且读取最新数据，
		//堵塞时间500ms,没有接收到通知则遥控器离线
		dbusOnlineFlag = xTaskNotifyWait(0xffffffff, 0xffffffff, (uint32_t *)(&SBUS_buf_task), pdMS_TO_TICKS(500));
		/*离线检测*/
		if (dbusOnlineFlag == 0)
		{
			//清除旧数据
			memset(&RC, 0, sizeof(RC));
			//仍然进行按键处理，不过所有键松开
			KeyProcess::keyHandle(0);
			//异常使能
			Error(dbusLost, enable);
			//任然进行任务调度
			taskSchedule();
			continue;
		}
		else
		{
			//异常恢复
			Error(dbusLost, disable);
		}
		/*▲ 数据解算*/
		dbusHandleData(SBUS_buf_task);

		//调用其他任务的遥控器更新函数
		for (int i = 0; i < VirtualTask::taskNum; i++)
		{
			VirtualTask::taskList[i]->remoteCtrlUpdate();
		}
		KeyProcess::keyHandle(RC.Key.keyValue);

		/**▲  执行相关命令进行任务调度**/
		taskSchedule();
	}
}

/**
  * @brief 任务调度
  * @note 通过遥控器指令执行相关任务的挂起和恢复
  * @waring 脱力控制也在这边，注意一下
  */
static void taskSchedule()
{
	//▲ 如果到脱力键，发送通知到紧急停止任务，不带通知值，且不保留接受任务的通知值，接受任务的通知值加一
	if (RC.Key.SW2 == RCS::Down || RC.Key.SW2 == RCS::Lost)
	{
		//调用其他任务的脱力回调函数
		for (int i = 0; i < VirtualTask::taskNum; i++)
		{
			VirtualTask::taskList[i]->deforceCallBack();
		}
		deforceFlag = 1;
		//执行重启计数函数，等待特定的重启操作后将重启
		rstCount();
	}

	//脱力状态解除，恢复挂起的任务，执行一些初始化
	else if (RC.SW2Tick == RCS::Down_Mid)
	{
		//调用其他任务的脱力回调函数
		for (int i = 0; i < VirtualTask::taskNum; i++)
		{
			VirtualTask::taskList[i]->deforceCancelCallBack();
		}
		deforceFlag = 0;
		//清除相关隐患变量
		//轮询所有使能电机
	}
}

/**
  * @brief 遥控器数据处理
  * @note 替代原始的数据处理
  */
static void dbusHandleData(uint8_t *SBUS_buf)
{
	//右摇杆横向  范围+-660
	RC.Key.CH[0] = (SBUS_buf[0] | (SBUS_buf[1] << 8)) & 0x07ff; //!< Channel 0
	RC.Key.CH[0] = my_deathzoom(RC.Key.CH[0] - 1024, 5);
	//右摇杆纵向   范围+-660
	RC.Key.CH[1] = ((SBUS_buf[1] >> 3) | (SBUS_buf[2] << 5)) & 0x07ff; //!< Channel 1
	RC.Key.CH[1] = my_deathzoom(RC.Key.CH[1] - 1024, 5);
	//左摇杆横向   范围+-660
	RC.Key.CH[2] = ((SBUS_buf[2] >> 6) | (SBUS_buf[3] << 2) | (SBUS_buf[4] << 10)) & 0x07ff; //!< Channel 2
	RC.Key.CH[2] = my_deathzoom(RC.Key.CH[2] - 1024, 5);
	//左摇杆纵向   范围+-660
	RC.Key.CH[3] = ((SBUS_buf[4] >> 1) | (SBUS_buf[5] << 7)) & 0x07ff; //!< Channel 3
	RC.Key.CH[3] = my_deathzoom(RC.Key.CH[3] - 1024, 5);
	//左边开关  132 上中下
	RC.Key.CH[4] = ((SBUS_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left
	//右边开关  132 上中下
	RC.Key.CH[5] = ((SBUS_buf[5] >> 4) & 0x0003); //!< Switch right9 / 9

	/***鼠标X值***/
	RC.Key.CH[6] = ((SBUS_buf[6]) | (SBUS_buf[7] << 8)); //x
	/***鼠标Y值***/
	RC.Key.CH[7] = -((SBUS_buf[8]) | (SBUS_buf[9] << 8)); //y
	/***鼠标左键***/
	RC.Key.CH[8] = SBUS_buf[12];
	RC.Key.left_jump = RC.Key.CH[8];
	/***鼠标右键***/
	RC.Key.CH[9] = SBUS_buf[13];
	RC.Key.Right_jump = RC.Key.CH[9];

	/***键盘值***/
	RC.Key.CH[10] = SBUS_buf[14] | (SBUS_buf[15] << 8);

	RC.Key.SW1 = (SWPos)RC.Key.CH[4];
	RC.Key.SW2 = (SWPos)RC.Key.CH[5];

	/*▲ 跳变检测*/
	//拨杆跳变检测
	RC.SW1Tick = (RCS::SWTick)(RC.Key.SW1 - RC.LastKey.SW1);
	RC.SW2Tick = (SWTick)(RC.Key.SW2 - RC.LastKey.SW2);

	//按键跳变检测-异或操作，相同为零，不同为一，那么跳变的位就为1，再赋值给单独的变量
	RC.KeyTick.CH[10] = RC.Key.CH[10] ^ RC.LastKey.CH[10];
	RC.KeyTick.left_jump = RC.Key.left_jump ^ RC.LastKey.left_jump;
	RC.KeyTick.Right_jump = RC.Key.Right_jump ^ RC.LastKey.Right_jump;

	//按下值检测-跳变值并上上一次值的取反-那么跳变且上一次为0的键为1
	RC.KeyPress.CH[10] = RC.KeyTick.CH[10] & (~RC.LastKey.CH[10]);
	RC.KeyPress.left_jump = RC.KeyTick.left_jump & (~RC.LastKey.left_jump);
	RC.KeyPress.Right_jump = RC.KeyTick.Right_jump & (~RC.LastKey.Right_jump);

	/*▲ 根据偏移值，定位按键变量，并赋值*/
	*((uint16_t *)&(RC.Key.keyValue)) = RC.Key.CH[10];
	*((uint16_t *)&(RC.KeyTick.keyValue)) = RC.KeyTick.CH[10];
	*((uint16_t *)&(RC.KeyPress.keyValue)) = RC.KeyPress.CH[10];

	//▲ 记录当前值
	memcpy(&RC.LastKey, &RC.Key, sizeof(RC.Key));
}

void rstCount(void)
{
	static u8 RST_Delay_Cnt = 0;
	static u8 HasRST = 1;
	if (((int)RC.Key.CH[0]) > 600 && ((int)RC.Key.CH[1]) < -600 && ((int)RC.Key.CH[2]) < -600 && ((int)RC.Key.CH[3]) < -600)
	{
		//▼ 自检
		if (RC.Key.SW1 == Mid)
		{
		}
		//▼ 重启
		if (RC.Key.SW1 == Down)
		{
			if (HasRST == 0)
			{
				RST_Delay_Cnt++;
				if (RST_Delay_Cnt > 150) //150*14ms大约为2s
				{
					__set_FAULTMASK(1); //关闭所有中断
					NVIC_SystemReset(); //复位
					while (1)
					{
					} //仅等待复位
				}
			}
		}
	}
	//校准陀螺仪
	else if (((int)RC.Key.CH[0]) < -600 && ((int)RC.Key.CH[1]) < -600 && ((int)RC.Key.CH[2]) > 600 && ((int)RC.Key.CH[3]) < -600)
	{
	}
	else
	{
		RST_Delay_Cnt = 0; //清空计数器
		HasRST = 0;
	}
}
