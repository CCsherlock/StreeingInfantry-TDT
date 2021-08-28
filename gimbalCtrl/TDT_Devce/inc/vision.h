#ifndef _VISION_H
#define _VISION_H
#include "board.h"

#define ANSWER_MODE 0
#pragma pack(1)
struct vision_Send_Struct_t
{
	uint8_t frameHeader; //0xA5
	float gimbalTimeStamp[3];
	uint8_t enemyColor : 1;		//0--Red   1--Blue
	uint8_t energyBeatMode : 1; //0--非打符模式   1--打符模式
	uint8_t baseShootMode : 1;
	uint8_t EnableAutoAim : 1;
	uint8_t SpiningShoot : 1;
	float nominalBulletSpeed;	//标称弹速
	float realBulletSpeed;		//实际弹速

	//todo add 3 int16_t control

	/*↓↓↓↓↓↓↓↓↓↓↓custom data start↓↓↓↓↓↓↓↓↓↓↓*/
	/*↑↑↑↑↑↑↑↑↑↑↑ custom data end ↑↑↑↑↑↑↑↑↑↑↑*/
	uint16_t CRC16CheckSum;
};

struct vision_Recv_Struct_t
{
	uint8_t frameHeader; //0xA5
	double recvTime;
	float Yaw;					   //单位: 度
	float Pitch;				   //单位: 度
	float yawPredictSpd;		   //yaw轴的速度
	uint8_t no_Obj : 1;			   //boolean
	uint8_t beat : 1;			   //boolean
	uint8_t unLimitedFireTime : 6;

	/*↓↓↓↓↓↓↓↓↓↓↓custom data start↓↓↓↓↓↓↓↓↓↓↓*/
	/*↑↑↑↑↑↑↑↑↑↑↑ custom data end ↑↑↑↑↑↑↑↑↑↑↑*/
	uint16_t CRC16CheckSum;
};
#pragma pack()

ENUM_START(objLost_set){
	pitch = 0,
	yaw,
};
ENUM_END(objLost_set)

struct visionInfo_t
{
	u16 visionCnt;
	u16 visionFPS;
	u8 offlineFlag;
	u8 objLost[ENUM_NUM(objLost_set)]; //当目标丢失, 此标志位数组置1, 软件置零, 数组长度自定义, 仅需改此处的声明
};

extern vision_Recv_Struct_t vision_RecvStruct;
extern vision_Send_Struct_t vision_SendStruct;
extern visionInfo_t visionInfo;

void Vision_Init(void);

#ifdef __cplusplus
extern "C"
{
#endif

//	void USART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
