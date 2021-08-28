#include "ammo_cover_task.h"
#include "motor.h"
#include "steeringTop_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "dbus_task.h"
Motor ammoCoverMotor(M2006, CAN2, 0x201);
PidParam ammoCoverParamInner = {7.2};
PidParam ammoCoverParamOuter = {0.1};
AmmoCover ammoCover;
#define AMMO_COVER_OFFSET_SPD_MAX 2000
#define AMMO_COVER_OFFSET_CURRENT_MAX 3000

u16 AMMO_COVER_ERROR_THESHOLD = 5;
u8 OFFSET_TIME_MAX = 100;
/**
 * @brief 弹仓盖校准
 *
 * @return u8 校准是否完成
 */
u8 AmmoCover::calOffset()
{
	if (offsetTimes < OFFSET_TIME_MAX)
	{
		ammoCoverParamInner.resultMax = AMMO_COVER_OFFSET_CURRENT_MAX;
		ammoCoverParamOuter.resultMax = AMMO_COVER_OFFSET_SPD_MAX;
		ammoCoverMotor.ctrlSpeed(AMMO_COVER_OFFSET_SPD_MAX);
		if (ammoCoverMotor.canInfo.lostFlag != 1 &&
			ABS(ammoCoverMotor.canInfo.totalEncoder - ammoCoverMotor.canInfo.lastTotalEncoder) < AMMO_COVER_ERROR_THESHOLD)
		{
			coverOffset = ammoCoverMotor.canInfo.totalEncoder;
			offsetTimes++;
		}
		else
		{
			offsetTimes = 0;
		}
		return false;
	}
	offsetTimes = 0;
	return true;
}

void AmmoCover::init()
{
	coverRota = -100000;
	coverOffsetWrong = 1;
	coverOffseting = 1;
	coverOpen = 0;
	ammoCoverMotor.pidInner.paramPtr = &ammoCoverParamInner;
	ammoCoverMotor.pidOuter.paramPtr = &ammoCoverParamOuter;
}

void AmmoCover::run()
{
	if (deforceFlag)
	{
		ammoCoverMotor.pidOuter.Clear();
		ammoCoverMotor.pidInner.Clear();
		offsetTimes = 0;
		coverOffsetStart();
		return;
	}

	if (coverOffseting || coverOffsetWrong)
	{
		if (calOffset())
		{
			coverOffseting = 0;
			coverOffsetWrong = 0;
			coverOpen = 0;
		}
		return;
	}

	if (coverOpen)
	{
		ammoCoverMotor.ctrlPosition(coverOffset + coverRota);
	}
	else
	{
		ammoCoverMotor.ctrlPosition(coverOffset);
	}
}

void AmmoCover_Task(void *pvParameters)
{
	ammoCoverMotor.pidOuter.setPlanNum(1);
	ammoCoverMotor.pidInner.setPlanNum(1);
	ammoCover.init();
	while (1)
	{
		if(SteeringTop.customFlag ==0&&SteeringTop.canRecvStruct.ifMoving&&ammoCover.coverOpen)
		{
			ammoCover.coverOpen = 0;//弹仓盖自动关闭
		}
		ammoCover.run();
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}
