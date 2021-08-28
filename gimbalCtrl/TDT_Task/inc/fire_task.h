#ifndef __FIRE_TASK_H
#define __FIRE_TASK_H

#include "board.h"
#include "cycle.h"
#include "motor.h"

#define OPENLOOP_OR_CLOSELOOP 2
#define SPEED_COUNT 4
#define FRICTION_COUNT 2

#define FIRE_INTERVAL_COUNT 4

struct FireCtrl
{
public:
	uint16_t frictionSpdArray[OPENLOOP_OR_CLOSELOOP][SPEED_COUNT][FRICTION_COUNT];
	uint16_t frictionSpdSet[FRICTION_COUNT]; //当前摩擦轮设定速度
	uint16_t frictionSpdFb[FRICTION_COUNT];	 //当前摩擦轮设定速度
	u8 forceOpenLoop;						 //是否开闭环
	u8 startFrictionWheel;
	u8 lastStartFrictionWheel;
	float startFrictionWheelTime;
	float stopFrictionWheelTime;

	u8 frictionLost;   //摩擦轮模块离线标志位
	u8 shootErrorFlag; //堵转标志位

	u8 visionFireAuthority; //视觉与左键开火权限0与，1或
	u8 operatorAskFire;		//操作手要求开火
	u8 visionAskFire;		//操作手要求借助视觉开火

	float fireIntervalSet[4] = {0.02f, 0.04f, 0.0625f, 0.5f}; //开火间隔
	float fireInterval;											  //发射间隔
	float lastFireTime;											  //上一次发弹的时间
	int dirction;

	enum
	{
		SILENCE,	//禁止发射
		CLEAN_AMMO, //清弹
		FAST_SHOOT, //高频
		MID_SHOOT,	//正常
		SLOW_SHOOT, //低频
		BUFF_SHOOT,	//打符
		TRIPLE_SHOOT //三连发
	} fireMode;
    u8 tripleShootCnt=0; //三连发计数
	uint32_t ammoBoosterOffset;			 //初始角度
	u8 ammoNumPerRound;					 //每圈拨盘含有多少子弹
	uint32_t singleAmmoRota;			 //每个子弹要旋转的编码器角度
	uint32_t ammoDifferTimes = 4; //子弹微分次数
	uint32_t ammoNowDifferTimes;		 //当前子弹微分次数

	struct
	{
		uint16_t sprocketShootNum; //主控判定的发射数量
		uint16_t frictionShootNum; //摩擦轮判定的发射数量
		uint16_t jgmtShootNum;	   //裁判系统判定的发射数量
	}thisShootNum,lastShootNum;

	int64_t valueSet[2], valueFb[2]; //拨弹轮设定、反馈值

	void fireIntervalSwitch();	//发射间隔切换
	void fireIntervalLimit();	//发射间隔限制
	float fireIntervalExtLimit; //发射间隔限幅系数
	int fireCnt;
	Cycle sprocketHeatTimer;
	void sprocketHeatCalc();	//拨弹盘热量计算
	Cycle frictionHeatTimer;
	void frictionHeatCalc();	//摩擦轮热量计算
	void jgmtHeatCalc();		//裁判系统热量计算
	void heatCalc();			//热量计算
	float sprocketHeat;		//拨弹轮判定的本地热量
	float frictionHeat;		//摩擦轮判定的本地热量
	float jgmtHeat;			//裁判系统热量
	float finalHeat;		//三者结合的最终热量
	int16_t jgmtHeatMax;		//裁判系统热量最大值
	uint16_t jgmtCoolDown;		//裁判系统冷却
	void frictionSpdSwitch();	 //发射速度切换
	void frictionSpdLimit();	 //发射速度限制
	float frictionSpdExtLimit;	 //发射速度限幅系数
	float jgmtBulletSpeed;		 //当前子弹速度
	uint16_t jgmtBulletSpeedMax; //裁判系统射速上限
	float notFireTimer;
	Cycle notFireTimerCycle;

	u8 fireRequirement(); //检查发射条件
	u8 fireSource();	  //检查是视觉/操作手/自检等来源开火

	u8 lastDeforceFlag;
	float deforceStartTime; //脱力开始的时间
	u8 lastFrictionOfflineFlag;
	float frictionOfflineTimer;
	u8 lastSprocketOfflineFlag[2];
	float sprocketOfflineTimer;

	u8 unLimitedFired; //解除热量限制

	void loadParam();
	void init();
	void run();
	
	void frictionSend();
};

extern FireCtrl fireCtrl;
extern Motor *FeedSprocketMotor[2];

#endif
