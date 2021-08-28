#include "fire_task.h"
#include "dbus_task.h"
#include "motor.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "can.h"
#include "vision.h"
#define FRICTION_CLOSELOOP_INDEX 0
#define FRICTION_OPENLOOP_INDEX 1

#define FRICTION_0mps_INDEX 0
#define FRICTION_15mps_INDEX 1
#define FRICTION_18mps_INDEX 2
#define FRICTION_30mps_INDEX 3

#define FRICTION_A_INDEX 0
#define FRICTION_B_INDEX 1

#define FIRE_INTERVAL_FAST_INDEX 0
#define FIRE_INTERVAL_MID_INDEX 1
#define FIRE_INTERVAL_SLOW_INDEX 2
#define FIRE_INTERVAL_BUFF_INDEX 3
#define LEFT 0
#define RIGHT 1
FireCtrl fireCtrl;
Motor *FeedSprocketMotor[2];
PidParam FeedSprocketPidInner, FeedSprocketPidOuter;

float fireIntervalWatch;

void FireCtrl::fireIntervalSwitch()
{
	if (can1Feedback.max_hp == -1 || can1Feedback.MaxHeat[0] == -1 || unLimitedFired == 1 || can1Feedback.MaxHeat[1] == 0)
	{
		fireInterval = fireIntervalSet[FIRE_INTERVAL_FAST_INDEX];
		return;
	}

	switch (fireMode)
	{
	case SILENCE:
		fireInterval = FLT_MAX;
		break;
	case CLEAN_AMMO:
	case FAST_SHOOT:
		fireInterval = fireIntervalSet[FIRE_INTERVAL_FAST_INDEX];
		break;
	case MID_SHOOT:
		fireInterval = fireIntervalSet[FIRE_INTERVAL_MID_INDEX];
		break;
	case SLOW_SHOOT:
		fireInterval = fireIntervalSet[FIRE_INTERVAL_SLOW_INDEX];
		break;
	case BUFF_SHOOT:
		fireInterval = fireIntervalSet[FIRE_INTERVAL_BUFF_INDEX];
		break;
	case TRIPLE_SHOOT:
		if (fireCtrl.tripleShootCnt == 3)
			fireInterval = fireIntervalSet[FIRE_INTERVAL_SLOW_INDEX] * 4;
		else
			fireInterval = fireIntervalSet[FIRE_INTERVAL_FAST_INDEX];
		break;
	default:
		break;
	}
}

void FireCtrl::frictionSpdSwitch()
{
	if (!startFrictionWheel)
	{
		for (int i = 0; i < FRICTION_COUNT; i++)
		{
			frictionSpdSet[i] = 0;
		}
		return;
	}
	u8 tmpSpeedIndex;
	switch (jgmtBulletSpeedMax)
	{
	case 15:
	default:
		tmpSpeedIndex = FRICTION_15mps_INDEX;
		break;
	case 18:
		tmpSpeedIndex = FRICTION_18mps_INDEX;
		break;
	case 30:
		tmpSpeedIndex = FRICTION_30mps_INDEX;
		break;
	}

	for (int i = 0; i < FRICTION_COUNT; i++)
	{
		frictionSpdSet[i] = frictionSpdExtLimit * frictionSpdArray[forceOpenLoop][tmpSpeedIndex][i];
	}
}

void FireCtrl::fireIntervalLimit()
{
	if (unLimitedFired)
	{
		fireIntervalExtLimit = 1;
		return;
	}

	float heatLast = jgmtHeatMax - finalHeat; //剩余可消耗热量
	fireIntervalExtLimit = heatLast - 20;

	fireIntervalExtLimit = LIMIT(fireIntervalExtLimit, 0.00000001, 1);

	fireInterval /= fireIntervalExtLimit;
}

void FireCtrl::heatCalc()
{
	jgmtHeatCalc();

	sprocketHeatCalc();

	finalHeat = sprocketHeat;
}

void FireCtrl::jgmtHeatCalc()
{
	if (!can1Feedback.jgmtOffline)
		jgmtHeat = can1Feedback.Jgmt_Heat[LEFT];
	else
		jgmtHeat = jgmtHeatMax;
}

void FireCtrl::frictionHeatCalc()
{
	if (thisShootNum.frictionShootNum != lastShootNum.frictionShootNum)
	{
		frictionHeat += 10;
	}
	lastShootNum.frictionShootNum = thisShootNum.frictionShootNum;

	frictionHeat -= fireCtrl.jgmtCoolDown * frictionHeatTimer.getCycleT();

	if (!can2Feedback_LEFT.frictionOffline && !can2Feedback_LEFT.AS5048_offline)
		frictionHeat = jgmtHeatMax;

	frictionHeat = LIMIT(frictionHeat, 0, jgmtHeatMax);
}

void FireCtrl::sprocketHeatCalc()
{
	if (thisShootNum.sprocketShootNum != lastShootNum.sprocketShootNum)
	{
		sprocketHeat += 10;
		fireCtrl.tripleShootCnt++;
		fireCtrl.tripleShootCnt = fireCtrl.tripleShootCnt > 3 ? 1 : fireCtrl.tripleShootCnt;
	}
	lastShootNum.sprocketShootNum = thisShootNum.sprocketShootNum;

	sprocketHeat -= fireCtrl.jgmtCoolDown * sprocketHeatTimer.getCycleT();
	sprocketHeat = LIMIT(sprocketHeat, 0, jgmtHeatMax - 1);
}

void FireCtrl::frictionSpdLimit()
{
	if (jgmtBulletSpeed != 0 && jgmtBulletSpeedMax != 0 && jgmtBulletSpeedMax < jgmtBulletSpeed)
		frictionSpdExtLimit = (jgmtBulletSpeedMax / jgmtBulletSpeed) * frictionSpdExtLimit;

	frictionSpdExtLimit = LIMIT(frictionSpdExtLimit, 0.85f, 1); //防止过度限制
}

void FireCtrl::init()
{
	FeedSprocketMotor[0] = new Motor(M2006, CAN2, 0x202);
	FeedSprocketMotor[1] = new Motor(M2006, CAN2, 0x203);
	loadParam();

	FeedSprocketMotor[0]->pidInner.paramPtr = &FeedSprocketPidInner;
	FeedSprocketMotor[0]->pidOuter.paramPtr = &FeedSprocketPidOuter;
	FeedSprocketMotor[1]->pidInner.paramPtr = &FeedSprocketPidInner;
	FeedSprocketMotor[1]->pidOuter.paramPtr = &FeedSprocketPidOuter;
}

void FireCtrl::run()
{
	u8 sprocket_friction_ready = 1;
	if (deforceFlag)
	{
		if (!lastDeforceFlag)
		{
			deforceStartTime = getSysTimeUs() / 1e6f; //记录开始脱力的时间
		}
		if (timeIntervalFrom_f(deforceStartTime) > 0.5f) //200ms后自动发0
			for (int i = 0; i < FRICTION_COUNT; i++)
				frictionSpdSet[i] = 0;

		lastDeforceFlag = deforceFlag;
		valueSet[0] = valueFb[0];
		valueSet[1] = valueFb[1];
		ammoNowDifferTimes = ammoDifferTimes;
		return;
	}
	lastDeforceFlag = deforceFlag;

	if (!FeedSprocketMotor[0]->canInfo.lostFlag && !FeedSprocketMotor[1]->canInfo.lostFlag)
	{
		if (lastSprocketOfflineFlag[0] && lastSprocketOfflineFlag[1])
			sprocketOfflineTimer = getSysTimeUs() / 1e6f; //记录拨弹电机离线的时间
		if (timeIntervalFrom_f(sprocketOfflineTimer) < 0.3f)
			sprocket_friction_ready = 0;
	}
	lastSprocketOfflineFlag[0] = FeedSprocketMotor[0]->canInfo.lostFlag;
	lastSprocketOfflineFlag[1] = FeedSprocketMotor[1]->canInfo.lostFlag;

	if (!can2Feedback_LEFT.frictionOffline)
	{
		if (lastFrictionOfflineFlag)
			frictionOfflineTimer = getSysTimeUs() / 1e6f; //记录开始脱力的时间
		if (timeIntervalFrom_f(frictionOfflineTimer) < 1.8f)
			sprocket_friction_ready = 0;
	}
	lastFrictionOfflineFlag = can2Feedback_LEFT.frictionOffline;

	if (!startFrictionWheel)
	{
		if (lastStartFrictionWheel)
		{
			stopFrictionWheelTime = getSysTimeUs() / 1e6f; //记录开启摩擦轮的时间
		}
		if (timeIntervalFrom_f(stopFrictionWheelTime) > 0.3f)
			for (int i = 0; i < FRICTION_COUNT; i++)
				frictionSpdSet[i] = 0;

		lastStartFrictionWheel = startFrictionWheel;
		ammoNowDifferTimes = ammoDifferTimes;
		return;
	}
	if (!lastStartFrictionWheel && startFrictionWheel)
	{
		startFrictionWheelTime = getSysTimeUs() / 1e6f;
	}
	lastStartFrictionWheel = startFrictionWheel;

	if (ammoNowDifferTimes < ammoDifferTimes)
	{
		ammoNowDifferTimes++;
		if (ammoNowDifferTimes == ammoDifferTimes)
		{
			fireCnt++;
			if (fireCnt > 99)
			{
				fireCnt = 0;
			}
			if (fireCnt % 2 == 0) //ID ：0
			{
				valueSet[1] += (dirction) * (int32_t)(singleAmmoRota % ammoDifferTimes); //最后一次补上最后的余数
			}
			else //ID ：1
			{
				valueSet[0] -= (dirction) * (int32_t)(singleAmmoRota % ammoDifferTimes); //最后一次补上最后的余数
			}
		}
		else
		{
			if (fireCnt % 2 == 0)
			{
				valueSet[1] += (dirction) * (int32_t)(singleAmmoRota / ammoDifferTimes);
			}
			else
			{
				valueSet[0] -= (dirction) * (int32_t)(singleAmmoRota / ammoDifferTimes);
			}
		}
	}

	fireIntervalSwitch();
	frictionSpdSwitch();

	fireIntervalLimit();
	frictionSpdLimit();

	if (!sprocket_friction_ready)
	{
		valueSet[0] = valueFb[0];
		valueSet[1] = valueFb[1];
		ammoNowDifferTimes = ammoDifferTimes;
		return;
	}

	for (u8 i = 0; i < 2; i++)
	{
		if (ABS(valueSet[i] - valueFb[i]) > 3 * singleAmmoRota)
		{
			if (valueSet[i] > valueFb[i])
			{
				valueSet[i] -= 5 * singleAmmoRota;
			}
			else
			{
				valueSet[i] += 5 * singleAmmoRota;
			}
		}
	}

	if (!fireRequirement()) //开火前置条件不满足
	{
		notFireTimer += notFireTimerCycle.getCycleT();
		if (notFireTimer >= 0.05) //100ms
			sprocketHeat = jgmtHeat;
		return;
	}

	if (!fireSource()) //没人喊开火
	{
		notFireTimer += notFireTimerCycle.getCycleT();
		if (notFireTimer >= 0.05) //100ms
			sprocketHeat = jgmtHeat;
		return;
	}
	notFireTimer = 0;

	ammoNowDifferTimes = 0; //开火，重新计算微分次数

	thisShootNum.sprocketShootNum++; //拨弹轮认为打出弹了，拨弹轮进行热量限制

	lastFireTime = getSysTimeUs() / 1e6f;
}

void Fire_Task(void *avg)
{
	fireCtrl.init();

	while (1)
	{

		if (!can1Feedback.jgmtOffline)
		{
			fireCtrl.jgmtCoolDown = can1Feedback.CoolRate[LEFT];
			fireCtrl.jgmtBulletSpeedMax = can1Feedback.shooterId1_17mmSpeedLimit;
			fireCtrl.jgmtHeatMax = can1Feedback.MaxHeat[LEFT];
		}
		else
		{
			fireCtrl.jgmtCoolDown = 10;
			fireCtrl.jgmtBulletSpeedMax = 15;
			fireCtrl.jgmtHeatMax = 50;
		}

		fireCtrl.heatCalc(); //无论是否脱力，都计算热量

		fireCtrl.valueFb[0] = FeedSprocketMotor[0]->canInfo.totalEncoder;
		fireCtrl.valueFb[1] = FeedSprocketMotor[1]->canInfo.totalEncoder;

		fireCtrl.run();
		if (!deforceFlag)
		{
			FeedSprocketMotor[0]->ctrlPosition(fireCtrl.valueSet[0]);
			FeedSprocketMotor[1]->ctrlPosition(fireCtrl.valueSet[1]);
		}

		fireCtrl.frictionSend();
		vTaskDelay(pdMS_TO_TICKS(5));
		if (can2Feedback_LEFT.frictionOfflineCheck == 0)
		{
			can2Feedback_LEFT.frictionOffline = 1;
		}
		can2Feedback_LEFT.frictionOfflineCheck = 0;
	}
}

u8 FireCtrl::fireRequirement()
{
	if (FeedSprocketMotor[0]->canInfo.lostFlag && FeedSprocketMotor[1]->canInfo.lostFlag) //电机丢失
		return false;

	if (shootErrorFlag) //拨弹轮堵转
		return false;

	if (can2Feedback_LEFT.frictionOffline) //摩擦轮离线
		return false;

	if (!startFrictionWheel || timeIntervalFrom_f(startFrictionWheelTime) < 2.0f)
		return false;

	if (!fireCtrl.forceOpenLoop)
	{
		for (int i = 0; i < FRICTION_COUNT; i++)
			if (frictionSpdSet[i] == 0) //未开启
				return false;
	}

	if (fireCtrl.forceOpenLoop)
		for (int i = 0; i < FRICTION_COUNT; i++)
			if (frictionSpdSet[i] == 0) //未开启
				return false;

	fireIntervalWatch = timeIntervalFrom_f(lastFireTime);
	if (fireInterval > fireIntervalWatch) //冷却时间不够长
		return false;					  //fixme 10-20分钟忽然判定失效

	return true;
}

u8 FireCtrl::fireSource()
{
	switch (fireMode)
	{
	case SILENCE:
		return false;
	case CLEAN_AMMO:
		return true;
	case BUFF_SHOOT:
		if (!vision_RecvStruct.no_Obj && vision_RecvStruct.beat)
			return true;
		break;
	case FAST_SHOOT:
	case MID_SHOOT:
	case SLOW_SHOOT:
	case TRIPLE_SHOOT:
		if (operatorAskFire && !visionAskFire) //相当于只按左键
			return true;
		if (visionAskFire)
		{
			if (operatorAskFire && visionFireAuthority) //或权限
				return true;
			if (!visionInfo.offlineFlag && !vision_RecvStruct.no_Obj && vision_RecvStruct.beat) //与权限
				return true;
		}
		break;
	}
	return false;
}

void FireCtrl::frictionSend()
{
	u8 TxData[8];
	TxData[0] = (u8)(frictionSpdSet[0] >> 8);
	TxData[1] = (u8)(frictionSpdSet[0]);
	TxData[2] = (u8)(frictionSpdSet[1] >> 8);
	TxData[3] = (u8)(frictionSpdSet[1]);
	TxData[7] = forceOpenLoop;

	canTx(TxData, CAN2, 0x100);
	canTx(TxData, CAN2, 0x102);
}

void FireCtrl::loadParam()
{
	FeedSprocketPidInner.kp = 5;
	FeedSprocketPidInner.ki = 0;
	FeedSprocketPidInner.kd = 0;
	FeedSprocketPidInner.resultMax = Motor::getMotorCurrentLimit(M2006);
	FeedSprocketPidInner.integralErrorMax = 100;

	FeedSprocketPidOuter.kp = 0.25;
	FeedSprocketPidOuter.ki = 0;
	FeedSprocketPidOuter.kd = 0;
	FeedSprocketPidOuter.resultMax = Motor::getMotorSpeedLimit(M2006);
	FeedSprocketPidOuter.integralErrorMax = 100;

	frictionSpdExtLimit = 1;
	fireIntervalExtLimit = 1;
	forceOpenLoop = 1;
	can2Feedback_LEFT.frictionOffline = 1;
	visionFireAuthority = 1;

	dirction = -1;
	ammoNumPerRound = 6;
	singleAmmoRota = 8192 * 36 / ammoNumPerRound;

	fireMode = MID_SHOOT;

	fireInterval = fireIntervalSet[FIRE_INTERVAL_MID_INDEX];

	frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 1600;
	frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 1600;

	frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 1800;
	frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 1800;

	frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 2800;
	frictionSpdArray[FRICTION_CLOSELOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 2800;

	frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_A_INDEX] = 4900;
	frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_15mps_INDEX][FRICTION_B_INDEX] = 4900;

	frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_A_INDEX] = 5100;
	frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_18mps_INDEX][FRICTION_B_INDEX] = 5100;

	frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_A_INDEX] = 5200;
	frictionSpdArray[FRICTION_OPENLOOP_INDEX][FRICTION_30mps_INDEX][FRICTION_B_INDEX] = 5200;
}