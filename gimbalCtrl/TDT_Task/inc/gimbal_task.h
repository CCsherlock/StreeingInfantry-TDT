#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "board.h"
#include "motor.h"
#define GIMBAL_MOTOR_COUNT 2

struct Gimbal
{
public:
	u8 pidUseParamIndex;
	float speedFb[GIMBAL_MOTOR_COUNT];
    float custom[2];
	float gimbalSensitivity[GIMBAL_MOTOR_COUNT][3];
	void operatorCtrl(u8 operatorSensitivity);
	float gimbalPositionSet[GIMBAL_MOTOR_COUNT],gimbalPositionFb[GIMBAL_MOTOR_COUNT];
	u8 lastFIND_ENEMY_OBJ;
	float pitchUpAngle, pitchDownAngle;
	void init();
	void run();
	void loadGimbalParam();
	void valueSetLimit();
	void imu_FbSet();
	void mech_FbSet(u8 speedIsImu);
	void getAngleForChassis();
	float yawAngleForChassis;
	float pitchAngleForChassis;
	
	void valueSwitch(){memcpy(gimbalPositionSet, gimbalPositionFb, sizeof(gimbalPositionFb));};
	void gimbalForwardCheck();
	struct GimbalFowardParam
	{
		float currentCheckAngle[5] = {20.0f,10.0f,0,-10.0f,-20.0f};
		float recordCurrunt;
		float computeCurrunt[5];
		float recordtime;
		float fowardKp[5];
		float outpuKp = 2846.4104;
		u8 setAngleSteep;
		bool checkFinish = 1;
	}gimbalFowardParam;
	float computeForwardFeedback();
};

extern Gimbal gimbal;
extern Motor *gimbalMotor;

#endif
