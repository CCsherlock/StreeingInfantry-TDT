#ifndef __STATE_TASK_H
#define __STATE_TASK_H

#include "board.h"
#include "cycle.h"

/***visionDebugType设置***/
#define VDT_ARMOR 0
#define VDT_BUFF 1
#define VDT_BASE_SHOOT 2
#define VDT_AUTO_GET_AMMO 3

class StateCtrl
{
public:
	u8 lastDeforceFlag;
	u8 visionDebugType = VDT_ARMOR;
	float autoGetAmmoFromTime;//从什么时候开始自动对位
	float rightClickFromTime;//从什么时候第一次按下右键
    float holdEtime; //按下E的时间
	void autoGetAmmo();
    void stateCanTX();
	vec3f customeSpeed = {600.0f,600.0f,300.0f};
	bool getAmmoIng = 0;
	u8 buffModeFlag = 0;
	enum
	{
		NormalMode,
		armorMode,
		buffMode,
		baseShootMode,
		autoGetAmmoMode,
		customMode //自定义模式
	}desireMode, //期望的模式
	stateMode, lastStateMode;//实际当前模式(期望模式的条件不满足)
	
};
extern StateCtrl stateCtrl;

struct CanStateStruct
{
	u8 visionOffline:1;
	u8 frictionOffline:1;
	u8 forceOpenloop:1;
	u8 unLimitedFired:1;
	u8 buffFlag:1;
	u8 ifFllowMode:1;
	u8 PowerPath_Switch:1;
	u8 checkMode:1;
	u8 ULTSMode:1;
	u8 customTimeReload:1;
	u8 blockError:1;
	u8 visionFPS;
	u16 frictionSpdA;
	u16 frictionSpdB;
};

extern struct CanStateStruct canStateStruct;
extern struct CanChassisStruct canChassisStruct;
#endif