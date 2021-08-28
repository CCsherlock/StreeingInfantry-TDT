#include "chassis_task.h"
#include "can.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
float motorKp = 3.8f;
float steeringKp1 = 140;
float steeringKi = 10;
float steeringKp2 = 7;
int streeingZero[4] = {284, 339, 7846, 7878 };//舵轮零点
uint8_t deforceFlag; //脱力标志位
Chassis::Chassis()
{

}
/**
 * @brief 底盘电机及PID初始化
 *
 */
void Chassis::motorInit()
{
    motorPid = new PidParam;
	steeringInnerPid = new PidParam;
	steeringOuterPid = new PidParam;
	for (u8 i = 0; i < 4; i++)
	{
		chassisMotor[i] = new Motor(M3508, CAN2, 0x201 + i);
		motorPid->kp = motorKp;
		motorPid->resultMax = chassisMotor[i]->getMotorCurrentLimit();
		chassisMotor[i]->pidInner.paramPtr = motorPid;
		chassisMotor[i]->pidInner.fbValuePtr[0] = &(chassisMotor[i]->canInfo.speed);
	}
	for (u8 i = 0; i < 4; i++)
	{
		steeringMotor[i] = new Motor(GM6020, CAN2, 0x205 + i);

		steeringInnerPid->kp = steeringKp1;
		steeringInnerPid->ki = steeringKi;
		steeringInnerPid->resultMax = steeringMotor[i]->getMotorCurrentLimit();

		steeringOuterPid->kp = steeringKp2;
		steeringOuterPid->resultMax = steeringMotor[i]->getMotorSpeedLimit();

		steeringMotor[i]->pidInner.paramPtr = steeringInnerPid;
		steeringMotor[i]->pidOuter.paramPtr = steeringOuterPid;
		steeringMotor[i]->pidInner.fbValuePtr[0] = &(steeringMotor[i]->canInfo.speed);
		steeringMotor[i]->pidOuter.fbValuePtr[0] = &(motorFdb.data[i]);
		motorEncZore[i] = streeingZero[i];
	}
}
/**
 * @brief 读取当前舵轮角度
 *
 */
void Chassis::AngleRead()
{
	for (u8 i = 0; i < 4; i++)
	{
		motorFdb.data[i] = ((steeringMotor[i]->canInfo.encoder - motorEncZore[i]) / 8191.f) * 360.f; //0--360
		streeingFdb.data[i] = motorFdb.data[i];
	}
}
/**
 * @brief 电机输出
 *
 */
void Chassis::motorOutput()
{
	if(!deforceFlag && !canRecvStruct.topLostFlag)
	{
		for (u8 i = 0; i < 4; i++)
		{
			steeringMotor[i]->ctrlPosition(outputData.angle.data[i]);
			chassisMotor[i]->setPowerOutLimit(&powerKp);
			chassisMotor[i]->ctrlSpeed(outputData.speed_rpm.data[i]);
		}
	}
	else
	{
		for (u8 i = 0; i < 4; i++)
		{
			steeringMotor[i]->ctrlCurrent(0);
			chassisMotor[i]->ctrlCurrent(0);
		}
	}
}
/**
 * @brief 获取当前上板指令速度
 *
 */
void Chassis::getBoradSpeed()
{
	for(u8 i = 0;i<3;i++)
	{
		boardSpeed.data[i] = canRecvStruct.datafloat.data[i];
	}
	powerKp = (float)canRecvStruct.datafloat.data[3]/10000.0f;
}
/**
 * @brief 返回信息给上板
 *
 */
void Chassis::msgSend()
{
	u8 data[8];
	canSendStruct.ifMoving = judgeIfMoving();
	data[0] = canSendStruct.ifMoving;
	data[1] = canSendStruct.readyFlag;
	canTx((u8 *)&data, CAN1, 0x137);
}
/**
 * @brief 判断当前是否在运动
 *
 * @return true 运动
 * @return false 为满足运动判定
 */
bool Chassis::judgeIfMoving()
{
	static int time;
	if(ABS(boardSpeed.data[0])>0||ABS(boardSpeed.data[1])>0)
	{
		float speedtemp;
		for(u8 i = 0; i<4; i++)
		{
			speedtemp += ABS(chassisMotor[i]->canInfo.speed);
		}
		if(speedtemp>10)
		{
			time++;
			if(time>100)
			{
				time = 0;
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else
		{
			time = 0;
			return 0;
		}
	}
	else
	{
		time = 0;
		return 0;
	}
}
void Chassis::chassisRun()
{
	getBoradSpeed();
	msgSend();
	AngleRead();
	steeringWheel_calculate(boardSpeed,streeingFdb);
	motorOutput();
}

Chassis chassis;
void Chassis_Task(void *pvParameters)
{
	chassis.motorInit();
	chassis.canSendStruct.readyFlag = 1;
	while(1)
	{
		chassis.chassisRun();
		chassis.canRecvStruct.topCtrlLostcnt++;
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}