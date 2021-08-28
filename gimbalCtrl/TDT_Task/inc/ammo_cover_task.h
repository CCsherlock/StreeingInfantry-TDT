#ifndef __AMMO_COVER_TASK_H
#define __AMMO_COVER_TASK_H

#include "board.h"
#include "motor.h"

class AmmoCover
{
	int coverOffset;	 //关闭弹舱盖时的校准位置
	int coverRota;		 //打开弹舱盖的行程
	
	u8 coverOffseting;	 //弹舱盖是否校准中
	u8 coverOffsetWrong; //弹舱盖当前值是否错误

	u8 offsetTimes = 0; //校准次数
	u8 calOffset();

public:
	u8 coverOpen;		 //弹舱盖是否打开
	inline u8 getCoverOpen() { return coverOpen; };
	inline void setCoverOpen(u8 toOpen)
	{
		if (!coverOffseting) //校准中不允许开关
			coverOpen = toOpen;
	};

	inline void toggleCoverOpen()
	{
		if (!coverOffseting) //校准中不允许开关
			coverOpen = !coverOpen;
	};

	inline void coverOffsetStart()
	{
		coverOffseting = 1;
		offsetTimes = 0;
	};
	inline void coverOffsetAbort()
	{
		if (!coverOffsetWrong) //如果当前的校准值不对则不允许停下
			coverOffseting = 0;
	};
	void init();
	void run();
};
extern AmmoCover ammoCover;
extern Motor ammoCoverMotor;

#endif
