#ifndef _VISION_H
#define _VISION_H
#include "board.h"
#include "imu.h"
#pragma pack(1)
extern Mpu6050 Mpu6050_Top;
struct vision_Send_Struct_t
{
        uint8_t frameHeader; //0xA5
        float yaw;
        float pitch;                //单位: 度
		char gimbalAngleStamp[120];
        uint8_t enemyColor : 1;     //0--Red   1--Blue
        uint8_t energyBeatMode : 1; //0--非打符模式   1--打符模式
        uint8_t baseShootMode:1;//同上 基地吊射模式
        uint8_t EnableAutoAim:1;//开启自瞄，提醒视觉有跳变时优先选择距离枪口最近的装甲板
        uint16_t bulletSpeed;//单位：mm/s
        uint16_t enemyPos[5][2];
        uint16_t selfPos[2];
        uint16_t CRC16CheckSum;
};

struct vision_Recv_Struct_t
{
        uint8_t frameHeader;           //0xA5
        float Yaw;                     //单位: 度
        float Pitch;                   //单位: 度
        uint8_t no_Obj : 1;            //boolean
        uint8_t beat : 1;              //boolean
        uint8_t unLimitedFireTime : 6; //无限火力开火时间，最大64ms
        int16_t followDist;            //动态跟随陀螺的距离
        uint16_t enemyPos[5][2];
		double recvTime;
        uint16_t CRC16CheckSum;
};
#pragma pack()

ENUM_START(objLost_set)
{
    pitch = 0,
    yaw,
};
ENUM_END(objLost_set)

struct visionInfo_t
{
    u16 visionCnt;
    u16 visionFPS;
    u8 offlineFlag;
    u8 objLost[ENUM_NUM(objLost_set)];//当目标丢失, 此标志位数组置1, 软件置零, 数组长度自定义, 仅需改此处的声明
};

extern vision_Recv_Struct_t vision_RecvStruct;
extern vision_Send_Struct_t vision_SendStruct;
extern visionInfo_t visionInfo;

void Vision_Init(void);

#ifdef __cplusplus
 extern "C" {
#endif
	 
void USART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif
