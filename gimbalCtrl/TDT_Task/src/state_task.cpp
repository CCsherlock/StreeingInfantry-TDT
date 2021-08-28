#include "state_task.h"
#include "ammo_cover_task.h"
#include "gimbal_task.h"
#include "fire_task.h"
#include "vision.h"
#include "dbus_task.h"
#include "steeringTop_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "can.h"
#include "imu_task.h"

extern Can2Feedback can2Feedback_LEFT;
StateCtrl stateCtrl;
struct CanStateStruct canStateStruct;

void StateCtrl::autoGetAmmo()
{
	/*判断是否需要提前退出自动对位*/
	if (ABS(SteeringTop.remoteSpeed.data[0]) > 0 || ABS(SteeringTop.remoteSpeed.data[0]) > 0 || ABS(SteeringTop.keyboardSpeed.data[0]) > 0 || ABS(SteeringTop.keyboardSpeed.data[0]) > 0 || SteeringTop.rotateFlag || SteeringTop.swingFlag)
	{
		autoGetAmmoFromTime = 0;
		ammoCover.setCoverOpen(0);
		stateCtrl.desireMode = stateCtrl.NormalMode;
		stateCtrl.getAmmoIng = 0;
		SteeringTop.customFlag = 0;
		SteeringTop.ifChassisFollow = 1; //切回底盘正常模式
	}
	//todo config timer interval
	if (autoGetAmmoFromTime == 0)
	{
		ammoCover.setCoverOpen(1);
		autoGetAmmoFromTime = getSysTimeUs() / 1e6f;
	}
	if (timeIntervalFrom_f(autoGetAmmoFromTime) < 1.2)
	{
		SteeringTop.customSpeedIn.data[0] = 0;
		SteeringTop.customSpeedIn.data[1] = customeSpeed.data[1];
		return;
	}
	if (timeIntervalFrom_f(autoGetAmmoFromTime) < 2.4)
	{
		SteeringTop.customSpeedIn.data[0] = customeSpeed.data[0];
		SteeringTop.customSpeedIn.data[1] = 0;
		return;
	}
	if (timeIntervalFrom_f(autoGetAmmoFromTime) < 4.5)
	{
		SteeringTop.customSpeedIn.data[0] = -customeSpeed.data[0] * 0.5;
		SteeringTop.customSpeedIn.data[1] = -customeSpeed.data[1] * 0.5;
		return;
	}
	/*走完一遍退出*/
	autoGetAmmoFromTime = 0;
	ammoCover.setCoverOpen(1);
	stateCtrl.getAmmoIng = 0;
	SteeringTop.customFlag = 0;
	SteeringTop.customSpeedIn.data[0] = 0;
	SteeringTop.customSpeedIn.data[1] = 0;
	SteeringTop.customSpeedIn.data[2] = 0;
	SteeringTop.ifTransform = 1;
	stateCtrl.desireMode = stateCtrl.NormalMode;
	SteeringTop.ifChassisFollow = 1; //切回底盘正常模式
}

void enterBuff()
{
	SteeringTop.lockChassis = 1;
	stateCtrl.desireMode = stateCtrl.buffMode;
	fireCtrl.fireMode = fireCtrl.BUFF_SHOOT;
	vision_SendStruct.energyBeatMode = 1;
	visionSendYaw = &GYROTDT.AngleFuseWithTrustAngle.YAW_AXIS;
	visionSendPitch = &GYROTDT.Angle.PITCH_AXIS;
}

void quitBuff()
{
	stateCtrl.desireMode = stateCtrl.NormalMode;
	vision_SendStruct.energyBeatMode = 0;
	fireCtrl.fireMode = fireCtrl.MID_SHOOT;
	SteeringTop.lockChassis = 0;
	visionSendYaw = &GYROTDT.Angle.YAW_AXIS;
	visionSendPitch = &GYROTDT.Angle.PITCH_AXIS;
}

void StateCtrl::stateCanTX()
{
	canStateStruct.forceOpenloop = fireCtrl.forceOpenLoop;
	canStateStruct.frictionOffline = can2Feedback_LEFT.frictionOffline;
	canStateStruct.frictionSpdA = can2Feedback_LEFT.Snail_A_FeedbackSpd_Now;
	canStateStruct.frictionSpdB = can2Feedback_LEFT.Snail_B_FeedbackSpd_Now;
	canStateStruct.buffFlag = stateCtrl.buffModeFlag;
	canStateStruct.ifFllowMode = SteeringTop.ifChassisFollow;
	canStateStruct.unLimitedFired = fireCtrl.unLimitedFired;
	canStateStruct.visionFPS = visionInfo.visionFPS;
	canStateStruct.visionOffline = visionInfo.offlineFlag;
	canStateStruct.PowerPath_Switch = SteeringTop.powerParam.PowerPath_Switch;
	canStateStruct.checkMode = SteeringTop.powerParam.checkMode;
	canStateStruct.ULTSMode = SteeringTop.powerParam.ULTSMode;
	canStateStruct.visionFPS = visionInfo.visionFPS;
	canTx((u8 *)&canStateStruct, CAN1, 0x165);
}
void State_Task(void *pvParameters)
{
	while (1)
	{
		stateCtrl.lastDeforceFlag = deforceFlag;
		stateCtrl.stateCanTX();
		uint32_t tmpGet;
		xTaskNotifyWait(0xffffffff, 0xffffffff, (uint32_t *)(&tmpGet), pdMS_TO_TICKS(14)); //等待dbus通知

		if (!deforceFlag && stateCtrl.lastDeforceFlag)
		{
			ammoCover.setCoverOpen(0);
			stateCtrl.desireMode = stateCtrl.NormalMode;
			fireCtrl.fireMode = fireCtrl.MID_SHOOT;
			fireCtrl.operatorAskFire = 0;
			fireCtrl.visionAskFire = 0;
		}

		u8 tmpOperatorAskFire = 0; //临时变量记录，相当于下方条件只要有一个满足置一
		u8 tmpVisionAskFire = 0;   //临时变量记录，相当于下方条件只要有一个满足置一
		u8 tmpEnableAutoAim = 0;
		u8 tmpStartFrictionWheel = 0;
		u8 tmpSpiningShoot = 0;

		if (stateCtrl.getAmmoIng)
		{
			stateCtrl.autoGetAmmo(); //运行结束后自动退出，云台为补弹模式
		}
		if (RC.Key.SW1 == RCS::SWPos::Up)
		{
			switch (stateCtrl.visionDebugType)
			{
			case VDT_ARMOR:
				stateCtrl.desireMode = stateCtrl.armorMode;
				tmpEnableAutoAim = 1;
				tmpOperatorAskFire = 1;
				tmpVisionAskFire = 1;
				break;
			case VDT_BUFF:
				enterBuff();
				tmpEnableAutoAim = 1;
				tmpOperatorAskFire = 1;
				tmpVisionAskFire = 1;
				break;
			case VDT_BASE_SHOOT:
				stateCtrl.desireMode = stateCtrl.baseShootMode;
				vision_SendStruct.baseShootMode = 1;
				tmpEnableAutoAim = 1;
				tmpOperatorAskFire = 1;
				tmpVisionAskFire = 1;
				break;
			case VDT_AUTO_GET_AMMO:
				if (RC.LastKey.SW1 != RCS::SWPos::Up && stateCtrl.getAmmoIng != 1) //对位期间不可再次启动对位
				{
					stateCtrl.getAmmoIng = 1;
					stateCtrl.desireMode = stateCtrl.autoGetAmmoMode;
					SteeringTop.customFlag = 1;		 //启用自定义模式
					SteeringTop.ifChassisFollow = 0; //取消底盘跟随
					SteeringTop.ifTransform = 1;	 //前进方向为云台指定方向（斜着进补给站，操作手可以调整方向）
				}
				break;
			default:
				break;
			}
		}
		else if (RC.LastKey.SW1 == RCS::SWPos::Up) //从上跳变为其他
		{
			quitBuff();
			SteeringTop.lockChassis = 0;
			stateCtrl.desireMode = stateCtrl.NormalMode;
			tmpEnableAutoAim = 0;
			vision_SendStruct.energyBeatMode = 0;
			vision_SendStruct.baseShootMode = 0;
			stateCtrl.autoGetAmmoFromTime = 0;
		}

		if (RC.Key.SW1 == RCS::SWPos::Down) //清弹不自动开摩擦轮
		{
			fireCtrl.fireMode = fireCtrl.CLEAN_AMMO;
		}
		else if (RC.LastKey.SW1 == RCS::SWPos::Down)
		{
			fireCtrl.fireMode = fireCtrl.MID_SHOOT;
		}

		if (RC.Key.SW2 == RCS::SWPos::Up)
		{
			tmpStartFrictionWheel = 1;
		}

		if (RC.Key.Right_jump && RC.Key.SW1 == RCS::SWPos::Mid)
		{
			stateCtrl.desireMode = stateCtrl.armorMode;
			tmpEnableAutoAim = 1;
			if (timeIntervalFrom_f(stateCtrl.rightClickFromTime) < 0.5f) //0.5秒按下两次
			{
				tmpSpiningShoot = 1;
				stateCtrl.rightClickFromTime = getSysTimeUs() / 1e6f;
			}
			tmpVisionAskFire = 1;
		}
		else if (RC.LastKey.Right_jump && RC.Key.SW1 == RCS::SWPos::Mid)
		{
			vision_SendStruct.SpiningShoot = 0;
			stateCtrl.desireMode = stateCtrl.NormalMode;
			stateCtrl.rightClickFromTime = getSysTimeUs() / 1e6f;
		}

		if (RC.Key.left_jump && RC.Key.SW1 == RCS::SWPos::Mid)
		{
			tmpOperatorAskFire = 1;
		}

		if (((int)RC.Key.CH[0]) < -600 && ((int)RC.Key.CH[1]) > 600 && ((int)RC.Key.CH[2]) > 600 && ((int)RC.Key.CH[3]) > 600)
		{
			//▼ 检录模式
			SteeringTop.powerParam.checkMode = 1; //重启清零
		}

		fireCtrl.operatorAskFire = tmpOperatorAskFire;
		fireCtrl.visionAskFire = tmpVisionAskFire;
		fireCtrl.startFrictionWheel = tmpStartFrictionWheel;
		vision_SendStruct.EnableAutoAim = tmpEnableAutoAim;
		vision_SendStruct.SpiningShoot = tmpSpiningShoot;
	}
}

#include "KeyProcess.h"

KeyProcess ammoCoverToggleOpen(
	KEY_B, [](uint32_t *)
	{ ammoCover.toggleCoverOpen(); },
	NULL, NULL, 1, 1); //组合键

KeyProcess calAmmoCoverOffset(
	KEY_CTRL | KEY_B, [](uint32_t *)
	{ ammoCover.coverOffsetStart(); },
	NULL, NULL); //组合键

KeyProcess buffToggle(
	KEY_R, [](uint32_t *)
	{
		if (RC.Key.SW1 == RCS::SWPos::Mid)
		{
			if (stateCtrl.desireMode == stateCtrl.buffMode)
			{
				stateCtrl.buffModeFlag = 0;
				quitBuff();
			}
			else
			{
				stateCtrl.buffModeFlag = 1;
				enterBuff(); //R是大辐
			}
		}
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift不处理

static float trunCarInterval = 1.0f;
static float lastTurnCarTime = 0;
KeyProcess trunCar(
	KEY_Q | KEY_CTRL, [](uint32_t *)
	{
		if (stateCtrl.desireMode == stateCtrl.NormalMode &&
			timeIntervalFrom_f(lastTurnCarTime) > trunCarInterval)
		{
			gimbal.gimbalPositionSet[0] += 180.0f;
			lastTurnCarTime = getSysTimeUs() / 1e6f;
		}
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift也处理

KeyProcess baseShootToggle(
	KEY_C, [](uint32_t *)
	{
		if (stateCtrl.desireMode == stateCtrl.baseShootMode)
		{
			stateCtrl.desireMode = stateCtrl.NormalMode;
			fireCtrl.fireMode = fireCtrl.MID_SHOOT;
			vision_SendStruct.baseShootMode = 0;
		}
		else
		{
			stateCtrl.desireMode = stateCtrl.baseShootMode;
			fireCtrl.fireMode = fireCtrl.SLOW_SHOOT;
			vision_SendStruct.baseShootMode = 1;
		}
		fireCtrl.unLimitedFired = 0;
	},
	NULL, NULL); //单键，遇到ctrl和shift也处理

KeyProcess unLimitedFire(
	KEY_V, [](uint32_t *)
	{
		fireCtrl.fireMode = fireCtrl.FAST_SHOOT;
		fireCtrl.unLimitedFired = 1;
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift不处理

KeyProcess cancelUnLimitedFire(
	KEY_CTRL | KEY_V, [](uint32_t *)
	{
		fireCtrl.fireMode = fireCtrl.MID_SHOOT;
		fireCtrl.unLimitedFired = 0;
	},
	NULL, NULL); //组合键

KeyProcess fireFreqSwitch(
	KEY_F, [](uint32_t *)
	{
		if (fireCtrl.fireMode == fireCtrl.FAST_SHOOT)
			fireCtrl.fireMode = fireCtrl.MID_SHOOT;
		else if (fireCtrl.fireMode == fireCtrl.MID_SHOOT)
			fireCtrl.fireMode = fireCtrl.TRIPLE_SHOOT;
		else
			fireCtrl.fireMode = fireCtrl.FAST_SHOOT;
		fireCtrl.unLimitedFired = 0;
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift不处理

KeyProcess forceOpenLoopSwitch(
	KEY_CTRL | KEY_F, [](uint32_t *)
	{ fireCtrl.forceOpenLoop = !fireCtrl.forceOpenLoop; },
	NULL, NULL); //单键，遇到ctrl和shift也处理

KeyProcess resetFollow(
	KEY_Z, NULL, [](uint32_t *)
	{ SteeringTop.ifChassisFollow = 0; },
	[](uint32_t *)
	{
		SteeringTop.ifChassisFollow = 1;
		SteeringTop.chassisFollowAngle = 0;
	},
	1, 1); //单键，遇到ctrl和shift不处理

KeyProcess autoGetAmmo(
	KEY_CTRL | KEY_Z, [](uint32_t *)
	{
		if (SteeringTop.ifChassisFollow == 1)
			SteeringTop.ifChassisFollow = 0;
		else if (SteeringTop.ifChassisFollow == 0)
			SteeringTop.ifChassisFollow = 1;
	},
	NULL, NULL); //组合键
KeyProcess releaseBackupPower(
	KEY_X,
	[](uint32_t *)
	{
		if (SteeringTop.powerParam.usingBackup == 1)
			SteeringTop.powerParam.usingBackup = 0;
		else if (SteeringTop.powerParam.usingBackup == 0)
			SteeringTop.powerParam.usingBackup = 1;
	},
	NULL, NULL, 1, 1); //单键，遇到ctrl和shift不处理

KeyProcess cancelReleaseBackupPower(
	KEY_CTRL | KEY_X, [](uint32_t *)
	{
		if (SteeringTop.powerParam.ULTSMode == 1)
			SteeringTop.powerParam.ULTSMode = 0;
		else if (SteeringTop.powerParam.ULTSMode == 0)
			SteeringTop.powerParam.ULTSMode = 1;
	},
	NULL, NULL); //组合键
KeyProcess changeFlexRotate(
	KEY_CTRL | KEY_E, [](uint32_t *)
	{
		if (SteeringTop.flexRotate == 1)
			SteeringTop.flexRotate = 0;
		else if (SteeringTop.flexRotate == 0)
			SteeringTop.flexRotate = 1;
	},
	NULL, NULL); //组合键
KeyProcess rotate(
	KEY_E, NULL,
	[](uint32_t *)
	{
		SteeringTop.rotateFlag = 0;
	},
	[](uint32_t *)
	{ SteeringTop.rotateFlag = 1; }); //单键，遇到ctrl和shift也处理

KeyProcess swing(
	KEY_Q, NULL, [](uint32_t *)
	{ SteeringTop.swingFlag = 0; },
	[](uint32_t *)
	{ SteeringTop.swingFlag = 1; }); //单键，遇到ctrl和shift也处理,Q是摇摆

KeyProcess reset(
	KEY_G | KEY_SHIFT | KEY_CTRL, [](uint32_t *)
	{
		__set_FAULTMASK(1); //关闭所有中断
		NVIC_SystemReset(); //复位
		while (1)
		{
		} //仅等待复位
	},
	NULL, NULL);

KeyProcess exitSpecialMode_W(
	KEY_W, [](uint32_t *)
	{
		if (RC.Key.SW1 == RCS::SWPos::Mid && stateCtrl.desireMode == stateCtrl.buffMode)
		{
			quitBuff();
		}
	},
	NULL, NULL, 0, 0);

KeyProcess exitSpecialMode_D(
	KEY_D, [](uint32_t *)
	{
		if (RC.Key.SW1 == RCS::SWPos::Mid && stateCtrl.desireMode == stateCtrl.buffMode)
		{
			quitBuff();
		}
	},
	NULL, NULL, 0, 0);

KeyProcess exitSpecialMode_A(
	KEY_A, [](uint32_t *)
	{
		if (RC.Key.SW1 == RCS::SWPos::Mid && stateCtrl.desireMode == stateCtrl.buffMode)
		{
			quitBuff();
		}
	},
	NULL, NULL, 0, 0);

KeyProcess exitSpecialMode_S(
	KEY_S, [](uint32_t *)
	{
		if (RC.Key.SW1 == RCS::SWPos::Mid && stateCtrl.desireMode == stateCtrl.buffMode)
		{
			quitBuff();
		}
	},
	NULL, NULL, 0, 0);