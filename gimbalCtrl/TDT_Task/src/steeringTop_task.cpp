#include "steeringTop_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "fire_task.h"
#include "dbus_task.h"
#include "filter.h"
#include "my_math.h"
#include "icm20602.h"
#include "vision.h"
#include "gimbal_task.h"
#include "can.h"
extern float YawStick_Sensitivity; //灵敏度
extern float YawMouse_Sensitivity;
steeringTop::steeringTop(/* args */)
{
}
void steeringTop::steeringTopInit()
{
    followPid = new PidParam;
    chassisFollow = new Pid(1);
    /*底盘跟随PID*/
    followPid->kp = 7;
    followPid->ki = 20;
    followPid->kd = 0;
    followPid->resultMax = MAX_CHASSIS_VW_SPEED;
    chassisFollow->paramPtr = followPid;
    chassisFollow->fbValuePtr[0] = &gimbalAngle;
}
/**
 * @brief   遥控器控制参数设定
 *
 * @return vec3f
 */
vec3f steeringTop::remoteCtrl()
{
    vec3f speedOut = {0};
    speedOut.data[0] = float(RC.Key.CH[3] / 660.0f) * MAX_CHASSIS_VX_SPEED * remoteSensitivity;
    speedOut.data[1] = float(RC.Key.CH[2] / 660.0f) * MAX_CHASSIS_VY_SPEED * remoteSensitivity;
    return speedOut;
}
/**
 * @brief 键盘控制参数设定
 *
 * @return vec3f
 */
vec3f steeringTop::keyboardCtrl()
{
    vec3f speedOut = {0};
    speedOut.data[0] = (RC.Key.W - RC.Key.S) * MAX_CHASSIS_VX_SPEED * keyboardSensitivity;
    if (RC.Key.CTRL)
        speedOut.data[0] *= 0.3;

    speedOut.data[1] = (RC.Key.D - RC.Key.A) * MAX_CHASSIS_VY_SPEED * keyboardSensitivity;
    if (RC.Key.CTRL)
        speedOut.data[1] *= 0.3;

    return speedOut;
}
/**
 * @brief 底盘跟随参数设定
 *
 * @return vec3f
 */
#define REMOTE_SWITCH 0
vec3f steeringTop::followCtrl()
{
    vec3f speedOut = {0};
#if REMOTE_SWITCH == 1
    if (RC.Key.CH[5] != 2&& (ABS(RC.Key.CH[0])+ABS(RC.Key.CH[1])+ABS(RC.Key.CH[2])+ABS(RC.Key.CH[3]))!=0)
    {
        ifChassisFollow = 1;
    }
    else
    {
        ifChassisFollow = 0;
    }
#endif

    if (ifChassisFollow)
    {
        gimbalAngle = -getGimbalAngle(); // -PI --- PI
		if(ABS(RC.Key.CH[6])<14)
		{
			RC.Key.CH[6] = 0;
		}
        speedOut.data[2] = chassisFollow->Calculate(chassisFollowAngle) - ((float(RC.Key.CH[0] / 660.0f) * remoteSensitivity * 4.0f)) - (float(RC.Key.CH[6] * YawMouse_Sensitivity * 1.0f)) * MAX_CHASSIS_VW_SPEED;
    }
    else
    {
        chassisFollow->Clear();
        speedOut.data[2] = 0;
    }

    return speedOut;
}
/**
 * @brief 小陀螺参数设定
 *
 * @return vec3f
 */
vec3f steeringTop::rotateCtrl()
{
    vec3f speedOut = {0};
    if (rotateFlag)
    {
        if (flexRotate) //变速陀螺
        {
            rotateParam.timeRecord++;
            rotateParam.sinResult = (rotateParam.sinMax) * sinf(rotateParam.sinOmiga * rotateParam.timeRecord) + 1.1f;
        }
        else
        {
            rotateParam.sinResult = 0.7f;
        }
    }
    else
    {
        rotateParam.timeRecord = 0;
        rotateParam.sinResult = 0;
    }

    speedOut.data[2] = MAX_CHASSIS_VW_SPEED * rotateParam.sinResult * (1 + (ABS(keyboardSpeed.data[0]) + ABS(keyboardSpeed.data[1])) * 0.0001) * (-1);

    return speedOut;
}
/**
 * @brief 摇摆参数设定
 *
 * @return vec3f
 */
vec3f steeringTop::swingCtrl()
{
    vec3f speedOut = {0};
    if (swingFlag)
    {
        swingParam.timeRecord++;
        //todo 添加均值参数
        swingParam.sinResult = swingParam.sinMax * sinf(swingParam.sinOmiga * swingParam.timeRecord) + (90.0f / 180.0f * MY_PPPIII);
    }
    else
    {
        swingParam.timeRecord = 0;
        swingParam.sinResult = 0;
    }
    gimbalAngle = getGimbalAngle();
    speedOut.data[2] = chassisFollow->Calculate(swingParam.sinResult);
    return speedOut;
}
/**
 * @brief 外部速度参数设定接口
 *
 * @return vec3f
 */
vec3f steeringTop::customCtrl()
{
    vec3f speedOut = {0};
    if (customFlag)
    {
        speedOut.data[0] = customSpeedIn.data[0];
        speedOut.data[1] = customSpeedIn.data[1];
        speedOut.data[2] = customSpeedIn.data[2];
    }
    else
    {
        memset(&speedOut, 0, sizeof(speedOut));
    }
    return speedOut;
}
/**
 * @brief  底盘云台坐标系转换
 *
 * @param speedIn
 * @return vec3f
 */
vec3f steeringTop::vecterTransform(vec3f speedIn)
{
    vec3f speedOut = speedIn;
    speedIn.data[0] = speedIn.data[0];
    speedIn.data[1] = -1 * speedIn.data[1];
    gimbalAngle = getGimbalAngle();

    //TODO待测试正负号
    speedOut.data[0] = speedIn.data[0] * my_cos(gimbalAngle) - speedIn.data[1] * my_sin(gimbalAngle);
    speedOut.data[1] = -speedIn.data[0] * my_sin(gimbalAngle) - speedIn.data[1] * my_cos(gimbalAngle);
    return speedOut;
}
/**
 * @brief 舵轮云台底盘偏心矫正 ！尚未完成
 *
 * @param AllOutSpeed
 * @return vec3f
 */
vec3f steeringTop::rotateOffsetCompute(vec3f AllOutSpeed)
{
    vec3f outSpeed;
    gimbalAngle = getGimbalAngle();
    rotateOffsetsSpeed.data[0] = -AllOutSpeed.data[2] * rotateOffsetParam.omigaK * sinf(gimbalAngle + (rotateOffsetParam.offsetSita / 57.295779f));
    rotateOffsetsSpeed.data[1] = AllOutSpeed.data[2] * rotateOffsetParam.omigaK * cosf(gimbalAngle + (rotateOffsetParam.offsetSita / 57.295779f));
    rotateOffsetsSpeed.data[2] = 0;
    outSpeed.data[0] = AllOutSpeed.data[0] + rotateOffsetsSpeed.data[0];
    outSpeed.data[1] = AllOutSpeed.data[1] + rotateOffsetsSpeed.data[1];
    outSpeed.data[2] = AllOutSpeed.data[2];
    return outSpeed;
}
/**
 * @brief 功率控制
 *
 */
void steeringTop::powerCtrlOld()
{
    int Num = 0;

    /*获得当前底盘所需参数*/
   powerOfflineCheck();
	/*裁判系统*/
   getJgmtMsg();
	/*功率模块信息*/
   getSuperPowerMsg();
	/*最大功率系数信息*/
   getMaxPower();
	/*判断当前加速模式*/
	judgeShiftMode();
    if (jgmtParam.jgmtOffline == 1)//裁判系统离线
    {
        powerParam.powerLimitKp = powerParam.noUsePowerKp;		//不消耗功率
    }
	else if(powerParam.powerOffline == 1)//功率模块离线
	{                                    //裁判系统及功率模块离线限制
        powerParam.powerLimitKp = powerParam.overFlowKp * LIMIT(((float)(powerParam.RemainPowerBuffer - 30.0f) / 10.0f), powerParam.noUsePowerKp, powerParam.noUsePowerKp+0.2f); //缓冲功率软件限功率
	}
    else
    {
        /*计算限幅系数*/
        if (judgeShift.doubleShift!=0)
        {
            if (powerParam.usingBackup == 0)
            {
				/*加速模式*/
				if(judgeShift.doubleShift == 1)
                powerParam.powerLimitKp = LIMIT(((float)(powerParam.remainPower_P - 50.0f) / 20.0f), powerParam.noUsePowerKpNomal, 1.0f); //百分之52左右 ，用到百分之70开始减速
				/*急速模式*/
				else if(judgeShift.doubleShift == 2)
				powerParam.powerLimitKp = LIMIT(((float)(powerParam.remainPower_P - 50.0f) / 13.0f), powerParam.noUsePowerKpNomal, 1.5f); //百分之52左右 ，用到百分之70开始减速
            }
            else	//使用备用功率
            {
				if(judgeShift.doubleShift == 1)
                powerParam.powerLimitKp = LIMIT(((float)(powerParam.remainPower_P - 30.0f) / 20.0f), powerParam.noUsePowerKpNomal, 1.0f); //百分之30左右 从百分之50开始减速
				/*急速模式*/
				else if(judgeShift.doubleShift == 2)
				powerParam.powerLimitKp = LIMIT(((float)(powerParam.remainPower_P - 30.0f) / 13.0f), powerParam.noUsePowerKpNomal, 1.5f); //百分之30左右 从百分之50开始减速
            }
			powerParam.limitPowerCurrunt = powerParam.remainPower_P;
			if(powerParam.limitPowerCurrunt>60)
			{
				powerParam.limitPowerNext = powerParam.limitPowerCurrunt - 10.0f;
			}
			else
			{
				powerParam.limitPowerNext = powerParam.limitPowerCurrunt - 5.0f;
			}
        }
        else
        {
             powerParam.powerLimitKp = LIMIT((powerParam.remainPower_P - powerParam.limitPowerNext)/10.0f,powerParam.noUsePowerKp,powerParam.noUsePowerKpNomal);
        }
    }
}
u8 steeringTop::judgeShiftMode()
{
	if(RC.Key.SHIFT==0)
	{
		if(judgeShift.judgeShiftFirst == 1)
		{
			judgeShift.timeRecode--;
			judgeShift.judgeShiftNest = 1;
			if(judgeShift.timeRecode<0)
			{
				judgeShift.timeRecode = 25;
				judgeShift.judgeShiftFirst = 0;
				judgeShift.judgeShiftNest = 0;
			}
			judgeShift.doubleShift = 0;
		}
		else
			judgeShift.doubleShift = 0;
		return 0;
	}
	else
	{

		judgeShift.judgeShiftFirst = 1;
		if(judgeShift.timeRecode>0&&judgeShift.judgeShiftNest == 1)
		{
			judgeShift.doubleShift = 2;
			judgeShift.timeRecode = 25;
			return 2;
		}
		else
		{
			judgeShift.doubleShift = 1;
			return 1;
		}
	}

}
/**
 * @brief 输出速度总计算
 *
 */
void steeringTop::speedCompute()
{
    if (customFlag) //当使用自定义速度时，不再进行功率计算等操作
    {
        customSpeedOut = customCtrl();
        followSpeed = followCtrl();
        for (u8 i = 0; i < 3; i++)
        {
            /* code */
            allSpeed.data[i] = customSpeedOut.data[i] + followSpeed.data[i];
        }
    }
    else
    {
		/*从各个入口获取参数*/
        remoteSpeed = remoteCtrl();
        keyboardSpeed = keyboardCtrl();
        followSpeed = followCtrl();
        rotateSpeed = rotateCtrl();
        swingSpeed = swingCtrl();

        for (u8 i = 0; i < 3; i++)
        {
            /* code */
            allSpeed.data[i] = remoteSpeed.data[i] + keyboardSpeed.data[i];
            if (rotateFlag)
                allSpeed.data[i] += rotateSpeed.data[i];
            else if (swingFlag)
                allSpeed.data[i] += swingSpeed.data[i] + followSpeed.data[i];
            else
                allSpeed.data[i] += followSpeed.data[i];
        }
    }
    if (lockChassis) //是否锁底盘
    {
        memset(&allSpeed, 0, sizeof(allSpeed));
    }
    if (allSpeed.data[0] == 0 && allSpeed.data[1] == 0 && allSpeed.data[2] <= 2 && ifChassisFollow != 1 && sendIfmove != 1&&accPama.accelerating!=1&&accPama.decelerating!=1) //当舵轮步兵静止时，输出小速度保持舵想跟随云台
    {
        allSpeed.data[0] = 30;
    }
    if (ifTransform) //是否进行云台坐标转换
    {
        allSpeed = vecterTransform(allSpeed);     //坐标系转换
//        allSpeed = rotateOffsetCompute(allSpeed); //云台偏心测试
        usingAcclerateCurve();                    //使用加速曲线
    }
}
/**
 * @brief 获取底盘Yaw轴的偏差角
 *
 * @return float
 */
float steeringTop::getGimbalAngle()
{
    return (gimbal.yawAngleForChassis) / 360.0f * 6.283186f; //弧度
}
/**
 * @brief 获取当前底盘偏差角度并重新设定底盘跟随角度 （如从 0 -->10)
 *
 */
void steeringTop::changeFollowZero()
{
    chassisFollowAngle = getGimbalAngle();
}
/**
 * @brief 功率离线检测
 *
 */
void steeringTop::powerOfflineCheck()
{
    can1Feedback.SuperPowerOfflineCheck++;
	if(can1Feedback.SuperPowerOfflineCheck>200)
	{
		can1Feedback.SuperPowerOfflineCheck = 200;
	}
    if (can1Feedback.SuperPowerOfflineCheck > 100)
    {
        can1Feedback.SuperPowerOffline = 1;
    }
    else
    {
        can1Feedback.SuperPowerOffline = 0;
    }
}
/**
 * @brief 裁判系统离线检测
 *
 */
void steeringTop::getJgmtMsg()
{
    jgmtParam.jgmtOffline = can1Feedback.jgmtOffline;
    jgmtParam.chassisBuffer = can1Feedback.remainPowerBuffer;
    jgmtParam.bufferLimit = 40 - (can1Feedback.chassisPowerLimit - 45);
}
/**
 * @brief 获取功率模块数据
 *
 */
void steeringTop::getSuperPowerMsg()
{
    powerParam.powerOffline = can1Feedback.SuperPowerOffline;      //离线标志位
    powerParam.remainPower_P = can1Feedback.SuperPowerRemain_P;    //当前剩余百分比
    powerParam.superPower_V = can1Feedback.Boost_V;                //当前电压
    powerParam.RemainPowerBuffer = can1Feedback.remainPowerBuffer; //缓冲能量
}
/**
 * @brief 超功率扣血检测
 *
 */
void steeringTop::powerOverFlowCal()
{
    static u8 checkTime;
    if (checkTime++ > 100)
    {
        powerParam.overLimitCase = 1;
        checkTime = 0;
    }
    if (powerParam.overLimitCase == 1 && can1Feedback.remainPowerBuffer <= 0)
    {
        powerParam.overLimitCase = 0;
        powerParam.overFlowKp -= 0.03f;
        powerParam.overFlowKp = LIMIT(powerParam.overFlowKp, 0.7f, 1.0f);
    }
}
/**
 * @brief 获取当前最大功率并设定无功率消耗输出系数
 *
 */
void steeringTop::getMaxPower()
{
    if (can1Feedback.chassisPowerLimit <= 85)
    {
        powerParam.noUsePowerKp = 0.3;
		powerParam.noUsePowerKpNomal = 0.5;
    }
    else if (can1Feedback.chassisPowerLimit > 85 && can1Feedback.chassisPowerLimit <= 100)
    {
        powerParam.noUsePowerKp = 0.5;
		powerParam.noUsePowerKpNomal = 0.8;
    }
    else
    {
        powerParam.noUsePowerKp = 0.5;
		powerParam.noUsePowerKpNomal = 0.8;
    }
}
/**
 * @brief 加速曲线
 *
 */
void steeringTop::usingAcclerateCurve()
{
	float chassisAngle;
	if(getGimbalAngle() == 0)
	{
		chassisAngle = 0.001f;
	}
	else
	{
		chassisAngle = getGimbalAngle();
	}
    accPama.linnerSpeed = sqrt(my_pow(allSpeed.data[0]) + my_pow(allSpeed.data[1]));
    if ((ABS(accPama.linnerSpeed) - ABS(accPama.linnerSpeedLast)) > 1000) //当加速超过一定阈值时使用曲线
    {
        accPama.accelerating = 1;
		accPama.decelerating = 0;
        accPama.accCnt = 0;
    }
    if ((ABS(accPama.linnerSpeed) - ABS(accPama.linnerSpeedLast)) < -1000) //当减速超过一定阈值时使用曲线
    {
        accPama.decelerating = 1;
		accPama.accelerating = 0;
        accPama.accCnt = 0;
    }
	/*加速曲线*/
    if (accPama.accelerating == 1)
    {
        accPama.accCnt += 0.005;
        accPama.accKp = AcclerateCurve(accPama.accCnt, accPama.accK);
        if (accPama.accKp > 0.999f)
        {
            accPama.accelerating = 0;
        }
    }
    else if(accPama.decelerating != 1)
    {
        accPama.accKp = 1;
        accPama.accCnt = 0;
    }
	/*减速曲线*/
    if (accPama.decelerating == 1)
    {
        accPama.accCnt += 0.005;
        accPama.accKp = DecclerateCurve(accPama.accCnt,20);
        if ( accPama.accKp < 0.01f)
        {
            accPama.decelerating = 0;
        }
    }
    else if(accPama.accelerating != 1)
    {
        accPama.accKp = 1;
        accPama.accCnt = 0;
    }
	/*×系数*/
	if(accPama.accelerating == 1)
	{
		allSpeed.data[0] = allSpeed.data[0] * accPama.accKp;
		allSpeed.data[1] = allSpeed.data[1] * accPama.accKp;
	}
	else if(accPama.decelerating == 1)
	{
		allSpeed.data[0] = accPama.deceleRecodeSpeed[0] * accPama.accKp;
		allSpeed.data[1] = accPama.deceleRecodeSpeed[1] * accPama.accKp;
	}
	if(accPama.decelerating != 1)
	{
		accPama.deceleRecodeSpeed[0] = allSpeed.data[0];
		accPama.deceleRecodeSpeed[1] = allSpeed.data[1];
	}
    accPama.linnerSpeedLast = accPama.linnerSpeed;
}
/**
 * @brief 将上板计算信息发送给下板
 *
 */
void steeringTop::speedSend()
{
	canSendCnt++;
	judgeIfMoving();
    powerCtrlOld();    //功率控制
    speedCompute();    //速度计算
	float chassisAngle;
	chassisAngle = getGimbalAngle();
	jscopSpeed = allSpeed;
	if(canSendCnt%3 == 0)
	{
		/*底盘参数发送给下板*/
		canSendStruct.datafloat.data[0] = allSpeed.data[0];
		canSendStruct.datafloat.data[1] = allSpeed.data[1];
		canSendStruct.datafloat.data[2] = allSpeed.data[2];
		canSendStruct.datafloat.data[3] = powerParam.powerLimitKp * 10000;
		canTx((vec4f *)&canSendStruct.datafloat, CAN1, 0x135);
	}
	else if(canSendCnt%3 == 1)
	{
		canSendStruct.ifdeforce = deforceFlag; //发送给下板脱力标志位
		/*云台给底盘*/
		u8 data1[8];
		data1[0] = canSendStruct.ifdeforce;
		data1[1] = ifChassisFollow;
		canTx((u8 *)&data1, CAN1, 0x136);
	}
	else if(canSendCnt%3 == 2)
	{
		/*云台给通用*/
		canToJug.ifChassisFollow = ifChassisFollow;
		canToJug.doubleShift = 2;//judgeShift.doubleShift;
		canToJug.chassisAngle = (int16_t)(chassisAngle*10000);
		canTx((u8 *)&canToJug, CAN1, 0x166);
	}
	if(canSendCnt>30)
		canSendCnt = 0;
}
/**
 * @brief 自动进补给站
 *
 */
void steeringTop::autoGotoStation()
{
    if (autoIntoStation.startFlag != 1)
    {
        autoIntoStation.runCount = 0;
        autoIntoStation.progress = 1;
        return;
    }
    else
    {
        if (autoIntoStation.runCount > autoIntoStation.progressTime[autoIntoStation.progress])
        {
            autoIntoStation.progress++;
            if (autoIntoStation.progress > STRAIGHT_SECOND)
            {
                autoIntoStation.startFlag = 0;
            }
        }
        if (autoIntoStation.progress_last != autoIntoStation.progress)
        {
            autoIntoStation.runCount = 0;
            autoIntoStation.progress_last = autoIntoStation.progress;
        }
        switch (autoIntoStation.progress)
        {
        case STRAIGHT_FIRST:
            customSpeedIn.data[0] = autoIntoStation.speedStraight;
            customSpeedIn.data[2] = 0;
            break;
        case TURN:
            autoIntoStation.speedTurnLinner = autoIntoStation.turnRadius * autoIntoStation.speedTurn;
            customSpeedIn.data[0] = autoIntoStation.speedTurnLinner;
            customSpeedIn.data[2] = autoIntoStation.speedTurn;
            break;
        case STRAIGHT_SECOND:
            customSpeedIn.data[0] = autoIntoStation.speedStraight;
            customSpeedIn.data[2] = 0;
            break;
        default:
            break;
        }
        autoIntoStation.runCount++;
    }
    autoIntoStation.progress_last = autoIntoStation.progress;
}
/**
 * @brief 判断当前设定值是否属于移动状态
 *
 */
bool steeringTop::judgeIfMoving()
{

	if(RC.Key.W == 0&&RC.Key.S == 0&&RC.Key.A == 0&&RC.Key.D == 0)
	{
		recodetime++;
		if(recodetime>300)
			recodetime = 300;
	}
	else
	{
		recodetime = 0;
	}
	if(recodetime>200)
	{
		sendIfmove = 0;
	}
	else
	{
		sendIfmove = 1;
	}
	return sendIfmove;
}
steeringTop SteeringTop;
void SteeringTop_Task(void *pvParameters)
{
    SteeringTop.steeringTopInit();
    while (1)
    {
        /* code */
        if (!deforceFlag)
        {
            SteeringTop.speedSend();
        }
        else
        {
            memset(&SteeringTop.allSpeed, 0, sizeof(SteeringTop.allSpeed));
            SteeringTop.speedSend(); //脱力时赋值0
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}