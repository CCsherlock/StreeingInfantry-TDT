#ifndef __GET_DISTANCE_TASK_H
#define __GET_DISTANCE_TASK_H
#include "board.h"
#include "gy_53.h"
enum SenserGY
{
	LF = 0,
	LB = 1,
	FF = 2
};
enum JudgeStep
{
	YAW = 0,
	DISY = 1,
	DISX = 2
};
class autoIntoStation
{
	private:

	public:
	u8 autoIntoStationFlag = 0;
	gy_53 *gy53Front;
	gy_53 *gy53Left1;
	gy_53 *gy53Left2;

	autoIntoStation();
	u8 judgeStep;
	void senserInit();
	int distance[3];
	float SideDistance;
	void getDistance();
	void autoGotoStation();
	float threShold[3];
	float judgeSpeed[3];
};
#endif