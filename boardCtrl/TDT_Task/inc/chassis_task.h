/*
 * @Author: your name
 * @Date: 2021-04-07 22:13:24
 * @LastEditTime: 2021-04-16 23:20:58
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Projectd:\TDT\TDT-Frame\TDT_Task\inc\chassis_task.h
 */
#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "board.h"
#include "motor.h"
#include "SteeringWheel.h"
#include "pid.h"
struct Chassis : SteeringWheel
{
private:


public:

    Chassis();
    Motor *chassisMotor[4];
	Motor *steeringMotor[4];
    PidParam *motorPid;
	PidParam *steeringInnerPid;
	PidParam *steeringOuterPid;
	int16_t motorEncZore[4];
	vec4f streeingFdb; //舵轮计算的输入反馈参数
	vec4f motorFdb;    //6020计算的输入反馈参数
	vec3f boardSpeed;
	float powerKp;
	
	void getBoradSpeed();//获取底盘速度和功率系数
	void motorInit();
	void AngleRead();
	void motorOutput();
	void msgSend();
	void chassisRun();
	void judgeIfDeforce();
	bool judgeIfMoving();
	struct DeforceJudge
	{
		u8 deforceFlag_last;
		int count;
	}deforceJudge;
	struct CanRecvStruct
	{
		vec4f datafloat;
		float powerLimitKp;
		u8 ifdeforce;
		u8 topLostFlag = 1;
		u8 ifChassisFollow;
		int topCtrlLostcnt = 200;
		float chassisAngle;
	}canRecvStruct;
	struct CanSendStruct
	{
		u8 readyFlag = 0;
		u8 ifMoving = 0;
	}canSendStruct;
	
};
extern struct Chassis chassis;
#endif