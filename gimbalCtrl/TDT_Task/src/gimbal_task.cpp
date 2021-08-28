#include "gimbal_task.h"
#include "motor.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "dbus_task.h"
#include "state_task.h"
#include "vision.h"
#include "imu_task.h"
#include "icm20602.h"
#include "filter.h"

extern Icm20602 GYROTDT;

/*********PID参数索引相关*********/
#define PID_GIMBAL_PARAM_COUNT 5
#define PID_OPERATOR_INDEX 0
#define PID_ARMOR_INDEX 1
#define PID_BUFF_INDEX 2
#define PID_BASE_SHOOT_INDEX 3
#define PID_FORWARD_INDEX 4

Kf visionYawAngle, visionPitchAngle;
float yawProcessNiose_Q = 0.05, yawMeasureNoise_R = 30;
float pitchProcessNiose_Q = 0.05, pitchMeasureNoise_R = 50;
u8 kfSwitch = 1;;

Gimbal gimbal;
Motor *gimbalMotor;

PidParam gimbalParamInner[GIMBAL_MOTOR_COUNT][PID_GIMBAL_PARAM_COUNT];
PidParam gimbalParamOuter[GIMBAL_MOTOR_COUNT][PID_GIMBAL_PARAM_COUNT];

/******电机即pid参数索引********/
#define YAW 0
#define PITCH 1

/*************操作手灵敏度相关***************/
#define OPERATOR_NORMAL 0	 //正常
#define OPERATOR_FINE_TUNE 1 //微调
#define OPERATOR_SILENCE 2	 //静默

float PitchStick_Sensitivity = -0.0006f; //灵敏度
float PitchMouse_Sensitivity = -0.003f;

float YawStick_Sensitivity = 0.0008f; //灵敏度
float YawMouse_Sensitivity = 0.0032f;

void Gimbal::init()
{
	gimbalMotor = new Motor[GIMBAL_MOTOR_COUNT]{
		Motor(GM6020, CAN1, 0x206),    //yaw
		Motor(GM6020, CAN2, 0x205)};   //pitch

	GYROTDT.trustAnglePtr[2] = &(gimbalMotor[0].canInfo.totalAngle_f);
	loadGimbalParam();

	gimbalSensitivity[YAW][OPERATOR_NORMAL] = 1;
	gimbalSensitivity[PITCH][OPERATOR_NORMAL] = 1;

	gimbalSensitivity[YAW][OPERATOR_FINE_TUNE] = 0.5;
	gimbalSensitivity[PITCH][OPERATOR_FINE_TUNE] = 0.5;

	gimbalSensitivity[YAW][OPERATOR_SILENCE] = 0;
	gimbalSensitivity[PITCH][OPERATOR_SILENCE] = 0;

	gimbalMotor[YAW].setZeroValue(20); //todo find ZeroAngle//4375 //200
	gimbalMotor[PITCH].setZeroValue(6828);//1292

	for (int i = 0; i < GIMBAL_MOTOR_COUNT; i++)
	{
		gimbalMotor[i].pidInner.setPlanNum(PID_GIMBAL_PARAM_COUNT);
		gimbalMotor[i].pidInner.paramPtr = gimbalParamInner[i];
		gimbalMotor[i].pidOuter.setPlanNum(PID_GIMBAL_PARAM_COUNT);
		gimbalMotor[i].pidOuter.paramPtr = gimbalParamOuter[i];
	}

	for (int i = 0; i < PID_GIMBAL_PARAM_COUNT; i++)
	{
		gimbalMotor[YAW].pidOuter.fbValuePtr[i] = &gimbalPositionFb[YAW];
		gimbalMotor[PITCH].pidOuter.fbValuePtr[i] = &gimbalPositionFb[PITCH];
		gimbalMotor[YAW].pidInner.fbValuePtr[i] = &speedFb[YAW];
		gimbalMotor[PITCH].pidInner.fbValuePtr[i] = &speedFb[PITCH];
	}

	imu_FbSet();
	valueSwitch();
}

#define FIND_ENEMY_OBJ (!vision_RecvStruct.no_Obj && !visionInfo.offlineFlag)
void Gimbal::run()
{
	imu_FbSet();
	if (deforceFlag)
	{
		valueSwitch();
		return;
	}

	if (stateCtrl.stateMode != stateCtrl.lastStateMode)
	{
		memcpy(gimbalPositionSet, gimbalPositionFb, sizeof(gimbalPositionFb));
	}

	stateCtrl.lastStateMode = stateCtrl.stateMode;

	u8 operatorSensitivity = OPERATOR_NORMAL;
	switch (stateCtrl.desireMode)
	{
	case stateCtrl.buffMode:
		operatorSensitivity = OPERATOR_SILENCE;
		pidUseParamIndex = PID_BUFF_INDEX;
		stateCtrl.stateMode = stateCtrl.buffMode;
		if (FIND_ENEMY_OBJ)
		{
			if(kfSwitch)
			{
				gimbalPositionSet[YAW] = visionYawAngle.KalmanFilter(vision_RecvStruct.Yaw,yawProcessNiose_Q, yawMeasureNoise_R,0);
				gimbalPositionSet[PITCH] = visionPitchAngle.KalmanFilter(vision_RecvStruct.Pitch,pitchProcessNiose_Q,pitchMeasureNoise_R,0);
			}
			else
			{
				gimbalPositionSet[YAW] = vision_RecvStruct.Yaw;
				gimbalPositionSet[PITCH] = 	vision_RecvStruct.Pitch;
			}
		}
		break;
	case stateCtrl.baseShootMode:

		stateCtrl.stateMode = stateCtrl.baseShootMode;
		pidUseParamIndex = PID_BASE_SHOOT_INDEX;
		operatorSensitivity = OPERATOR_FINE_TUNE;
		//todo 使用机械角 使用视觉
		break;
	case stateCtrl.armorMode:
		if (FIND_ENEMY_OBJ)
		{
			stateCtrl.stateMode = stateCtrl.armorMode;
			if(kfSwitch)
			{
				gimbalPositionSet[YAW] = visionYawAngle.KalmanFilter(vision_RecvStruct.Yaw,yawProcessNiose_Q, yawMeasureNoise_R,0);
				gimbalPositionSet[PITCH] = visionPitchAngle.KalmanFilter(vision_RecvStruct.Pitch,pitchProcessNiose_Q,pitchMeasureNoise_R,0);
			}
			else
			{
				gimbalPositionSet[YAW] = vision_RecvStruct.Yaw;
				gimbalPositionSet[PITCH] = vision_RecvStruct.Pitch;
			}
			operatorSensitivity = OPERATOR_SILENCE;
			pidUseParamIndex = PID_ARMOR_INDEX;
		}
		else
		{
			pidUseParamIndex = PID_OPERATOR_INDEX;
			operatorSensitivity = OPERATOR_NORMAL;
			stateCtrl.stateMode = stateCtrl.NormalMode;
		}
		break;

	case stateCtrl.autoGetAmmoMode:
		stateCtrl.stateMode = stateCtrl.autoGetAmmoMode;
		pidUseParamIndex = PID_OPERATOR_INDEX;
		operatorSensitivity = OPERATOR_SILENCE;
		break;
	case stateCtrl.NormalMode:
		stateCtrl.stateMode = stateCtrl.NormalMode;
		pidUseParamIndex = PID_OPERATOR_INDEX;
		operatorSensitivity = OPERATOR_NORMAL;
		break;
	case stateCtrl.customMode:
		stateCtrl.stateMode = stateCtrl.customMode;
		pidUseParamIndex = PID_OPERATOR_INDEX;
		operatorSensitivity = OPERATOR_NORMAL;
		break;
	default:
		pidUseParamIndex = PID_OPERATOR_INDEX;
		operatorSensitivity = OPERATOR_NORMAL;
		break;
	}
	if (stateCtrl.stateMode != stateCtrl.lastStateMode)//note 如果在目标丢失，\
		gimbalMode会从装甲板切换成普通模式，设定值等于反馈值
	{
		valueSwitch();
	}

	lastFIND_ENEMY_OBJ = FIND_ENEMY_OBJ;
	stateCtrl.lastStateMode = stateCtrl.stateMode;

	//操作手控制
	operatorCtrl(operatorSensitivity);

	valueSetLimit();
}

void Gimbal::valueSetLimit()
{
	//todo 利用机械角限幅
	gimbalPositionSet[PITCH] = LIMIT(gimbalPositionSet[PITCH], pitchUpAngle, pitchDownAngle);
}

void Gimbal::operatorCtrl(u8 operatorSensitivity)
{
	float angleDelta[GIMBAL_MOTOR_COUNT];
	angleDelta[PITCH] = RC.Key.CH[1] * PitchStick_Sensitivity + RC.Key.CH[7] * PitchMouse_Sensitivity + custom[PITCH];

	angleDelta[YAW] = RC.Key.CH[0] * YawStick_Sensitivity + RC.Key.CH[6] * YawMouse_Sensitivity + custom[YAW];

	for (int i = 0; i < GIMBAL_MOTOR_COUNT; i++)
	{
		gimbalPositionSet[i] += angleDelta[i] * gimbalSensitivity[i][operatorSensitivity];
	}
}
//#define FORWARDCHECK
#define NORMAL
void Gimbal_Task(void *avg)
{
	gimbal.init();
	while (1)
	{
		gimbal.run();
		if (!deforceFlag)
		{
			#ifdef NORMAL
			for (int i = 0; i < GIMBAL_MOTOR_COUNT; i++)
			{
			  gimbalMotor[i].ctrlPosition(gimbal.gimbalPositionSet[i], gimbal.pidUseParamIndex);
			}
			#endif
			#ifdef FORWARDCHECK
			if(!gimbal.gimbalFowardParam.checkFinish)
			{
				gimbal.gimbalForwardCheck();
			}
			else
			{
				  gimbalMotor[YAW].ctrlPosition(gimbal.gimbalPositionSet[YAW], gimbal.pidUseParamIndex);
				  gimbalMotor[PITCH].ctrlPosition(gimbal.gimbalPositionSet[PITCH], gimbal.pidUseParamIndex,1,0,gimbal.computeForwardFeedback());
			}
			#endif
		}

        gimbal.getAngleForChassis();
		vTaskDelay(pdMS_TO_TICKS(2));
	}
}

void Gimbal::loadGimbalParam()
{
	pitchUpAngle = -35;//-39;
	pitchDownAngle = 17;//25;

	gimbalParamInner[YAW][PID_OPERATOR_INDEX].kp = -170;//250;//400;//400;
	gimbalParamInner[YAW][PID_OPERATOR_INDEX].ki = -20;//0;
	gimbalParamInner[YAW][PID_OPERATOR_INDEX].kd = 0;//0.1;//0.1;
	gimbalParamInner[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[YAW][PID_OPERATOR_INDEX].integralErrorMax = 1;

	gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kp = 15;//7;//15;//18;
	gimbalParamOuter[YAW][PID_OPERATOR_INDEX].ki = 0;//1;//50;
	gimbalParamOuter[YAW][PID_OPERATOR_INDEX].kd = 0;
	gimbalParamOuter[YAW][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[YAW][PID_OPERATOR_INDEX].integralErrorMax = 0.5;

	gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kp = -150;//-400;//-300;//-300;
	gimbalParamInner[PITCH][PID_OPERATOR_INDEX].ki = 0;//1000;
	gimbalParamInner[PITCH][PID_OPERATOR_INDEX].kd = 0.1;
	gimbalParamInner[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 0.5;

	gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kp = 9;//8;//6;//6;
	gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].ki = 100;//1.5;
	gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].kd = 0;
	gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[PITCH][PID_OPERATOR_INDEX].integralErrorMax = 0.01;

	gimbalParamInner[YAW][PID_ARMOR_INDEX].kp = 0;//400;
	gimbalParamInner[YAW][PID_ARMOR_INDEX].ki = 0;
	gimbalParamInner[YAW][PID_ARMOR_INDEX].kd = 0;//0.2;
	gimbalParamInner[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[YAW][PID_ARMOR_INDEX].integralErrorMax = 1;

	gimbalParamOuter[YAW][PID_ARMOR_INDEX].kp = 0;//7;
	gimbalParamOuter[YAW][PID_ARMOR_INDEX].ki = 0;//80;
	gimbalParamOuter[YAW][PID_ARMOR_INDEX].kd = 0;
	gimbalParamOuter[YAW][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[YAW][PID_ARMOR_INDEX].integralErrorMax = 0.1;

	gimbalParamInner[PITCH][PID_ARMOR_INDEX].kp = 0;//-300;
	gimbalParamInner[PITCH][PID_ARMOR_INDEX].ki = 0;
	gimbalParamInner[PITCH][PID_ARMOR_INDEX].kd = 0;//2.5;
	gimbalParamInner[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[PITCH][PID_ARMOR_INDEX].integralErrorMax = 10;


	gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kp = 0;//14;
	gimbalParamOuter[PITCH][PID_ARMOR_INDEX].ki = 0;//60;
	gimbalParamOuter[PITCH][PID_ARMOR_INDEX].kd = 0;
	gimbalParamOuter[PITCH][PID_ARMOR_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[PITCH][PID_ARMOR_INDEX].integralErrorMax = 1;

	gimbalParamInner[YAW][PID_BUFF_INDEX].kp = 500;
	gimbalParamInner[YAW][PID_BUFF_INDEX].ki = 200;
	gimbalParamInner[YAW][PID_BUFF_INDEX].kd = 3.5;
	gimbalParamInner[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[YAW][PID_BUFF_INDEX].integralErrorMax = 1;


	gimbalParamOuter[YAW][PID_BUFF_INDEX].kp = 30;
	gimbalParamOuter[YAW][PID_BUFF_INDEX].ki = 50;
	gimbalParamOuter[YAW][PID_BUFF_INDEX].kd = 0;
	gimbalParamOuter[YAW][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[YAW][PID_BUFF_INDEX].integralErrorMax = 0.5;

	gimbalParamInner[PITCH][PID_BUFF_INDEX].kp = -500;
	gimbalParamInner[PITCH][PID_BUFF_INDEX].ki = 50;
	gimbalParamInner[PITCH][PID_BUFF_INDEX].kd = 2.5;
	gimbalParamInner[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[PITCH][PID_BUFF_INDEX].integralErrorMax = 1;


	gimbalParamOuter[PITCH][PID_BUFF_INDEX].kp = 35;
	gimbalParamOuter[PITCH][PID_BUFF_INDEX].ki = 20;
	gimbalParamOuter[PITCH][PID_BUFF_INDEX].kd = 0;
	gimbalParamOuter[PITCH][PID_BUFF_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[PITCH][PID_BUFF_INDEX].integralErrorMax = 0.5;

	gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kp = 300;
	gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
	gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
	gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

	gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kp = 10;
	gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].ki = 0;
	gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].kd = 0;
	gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[YAW][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

	gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kp = -200;
	gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
	gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
	gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

	gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kp = 12;
	gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].ki = 0;
	gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].kd = 0;
	gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[PITCH][PID_BASE_SHOOT_INDEX].integralErrorMax = 100;

	gimbalParamInner[YAW][PID_FORWARD_INDEX].kp = 0;//400;//400;
	gimbalParamInner[YAW][PID_FORWARD_INDEX].ki = 0;//0;
	gimbalParamInner[YAW][PID_FORWARD_INDEX].kd = 0;//0.1;//0.1;
	gimbalParamInner[YAW][PID_FORWARD_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[YAW][PID_FORWARD_INDEX].integralErrorMax = 100;

	gimbalParamOuter[YAW][PID_FORWARD_INDEX].kp = 0;//15;//18;
	gimbalParamOuter[YAW][PID_FORWARD_INDEX].ki = 0;//50;
	gimbalParamOuter[YAW][PID_FORWARD_INDEX].kd = 0;
	gimbalParamOuter[YAW][PID_FORWARD_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[YAW][PID_FORWARD_INDEX].integralErrorMax = 0.5;

	gimbalParamInner[PITCH][PID_FORWARD_INDEX].kp = -300;//-300;//-300;
	gimbalParamInner[PITCH][PID_FORWARD_INDEX].ki = 0;//1000;
	gimbalParamInner[PITCH][PID_FORWARD_INDEX].kd = 0;
	gimbalParamInner[PITCH][PID_FORWARD_INDEX].resultMax = Motor::getMotorCurrentLimit(GM6020);
	gimbalParamInner[PITCH][PID_FORWARD_INDEX].integralErrorMax = 0.5;

	gimbalParamOuter[PITCH][PID_FORWARD_INDEX].kp = 7;//6;//6;
	gimbalParamOuter[PITCH][PID_FORWARD_INDEX].ki = 1.5;
	gimbalParamOuter[PITCH][PID_FORWARD_INDEX].kd = 0;
	gimbalParamOuter[PITCH][PID_FORWARD_INDEX].resultMax = Motor::getMotorSpeedLimit(GM6020) * 6;
	gimbalParamOuter[PITCH][PID_FORWARD_INDEX].integralErrorMax = 100;
}

void Gimbal::imu_FbSet()
{
	gimbalPositionFb[YAW] = *visionSendYaw;
	gimbalPositionFb[PITCH] = *visionSendPitch;
	speedFb[YAW] = GYROTDT.gyro.dps.data[2];
	speedFb[PITCH] = GYROTDT.gyro.dps.data[0];
}

void Gimbal::mech_FbSet(u8 speedIsImu)
{
	gimbalPositionFb[YAW] = gimbalMotor[YAW].canInfo.totalAngle_f;	 
	gimbalPositionFb[PITCH] = gimbalMotor[PITCH].canInfo.totalAngle_f; 
	if (!speedIsImu)
	{
		speedFb[YAW] = gimbalMotor[YAW].canInfo.dps;	 
		speedFb[PITCH] = gimbalMotor[PITCH].canInfo.dps; 
	}
	else
	{
		speedFb[YAW] = GYROTDT.gyro.dps.data[2];
		speedFb[PITCH] = GYROTDT.gyro.dps.data[0];
	}
}

void Gimbal::getAngleForChassis()
{
		   float yawAngleForChassis = gimbalMotor[YAW].canInfo.encoderCalibration * 360 / 8192.0f;
		   float pitchAngleForChassis = gimbalMotor[PITCH].canInfo.encoderCalibration * 360 / 8192.0f;
		   if (yawAngleForChassis > 180)
		   {
		   	yawAngleForChassis -= 360;
		   }
		   else if (yawAngleForChassis < -180)
		   {
		   	yawAngleForChassis += 360;
		   }
		   if (pitchAngleForChassis > 180)
		   {
		   	pitchAngleForChassis -= 360;
		   }
		   else if (pitchAngleForChassis < -180)
		   {
		   	pitchAngleForChassis += 360;
		   }
		   gimbal.yawAngleForChassis = yawAngleForChassis;
		   gimbal.pitchAngleForChassis = -pitchAngleForChassis; //注意：在这里加个负号
}
/**
 * @brief 重力前馈系数测定（debug功能中使用）
 *
 */
void Gimbal::gimbalForwardCheck()
{
	if(ABS(gimbalPositionFb[PITCH] - gimbalFowardParam.currentCheckAngle[gimbalFowardParam.setAngleSteep])>0.2f)
	{
		if(gimbalPositionFb[PITCH]<gimbalFowardParam.currentCheckAngle[gimbalFowardParam.setAngleSteep])
		{
			if(gimbalPositionFb[PITCH] - gimbalFowardParam.currentCheckAngle[gimbalFowardParam.setAngleSteep]<-7.0f)
			{
				gimbalFowardParam.recordCurrunt-=0.8f;
			}
			else
			{
				gimbalFowardParam.recordCurrunt-=0.05f;
			}
		}
		else if(gimbalPositionFb[PITCH]>gimbalFowardParam.currentCheckAngle[gimbalFowardParam.setAngleSteep])
		{
			if(gimbalPositionFb[PITCH] - gimbalFowardParam.currentCheckAngle[gimbalFowardParam.setAngleSteep]>7.0f)
			{
				gimbalFowardParam.recordCurrunt+=0.8f;
			}
			else
			{
				gimbalFowardParam.recordCurrunt+=0.05f;
			}
		}
		gimbalFowardParam.recordtime = 0;
	}
	else
	{
		gimbalFowardParam.recordtime+=0.002;
	}
	if(gimbalFowardParam.recordtime>1.0f)
	{
		gimbalFowardParam.computeCurrunt[gimbalFowardParam.setAngleSteep] = gimbalFowardParam.recordCurrunt;
		if(gimbalFowardParam.setAngleSteep<4)
		{
			gimbalFowardParam.setAngleSteep++;
		}
		else
		{
			for(u8 i = 0;i<5;i++)
			{
				gimbalFowardParam.fowardKp[i] = gimbalFowardParam.computeCurrunt[i]/cos(ABS(gimbalFowardParam.currentCheckAngle[i]*RAD_PER_DEG));
				gimbalFowardParam.outpuKp += 0.2*(gimbalFowardParam.fowardKp[i]);
				gimbalFowardParam.checkFinish = 1;
			}
		}
	}
	gimbalMotor[PITCH].ctrlCurrent(gimbalFowardParam.recordCurrunt);
}
/**
 * @brief 重力前馈计算
 *
 * @return float 当前角度下的前馈值
 */
float Gimbal::computeForwardFeedback()
{
	float result;
	if(gimbalFowardParam.checkFinish == 1)
	{
		result = gimbalFowardParam.outpuKp * cos(ABS(gimbalPositionFb[PITCH]*RAD_PER_DEG));
	}
	else
	{
		result = 0;
	}
	return result;
}