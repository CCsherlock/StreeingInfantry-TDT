/*
 * @Author: your name
 * @Date: 2021-05-09 06:17:01
 * @LastEditTime: 2021-05-10 12:18:02
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Projectd:\TDT\TDT-Infantry\Infantry_II\TDT_Bsp\inc\can.h
 */
/*****************************************************************************
File name: TDT_Bsp\src\can.cpp
Description: can底层
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.16 #合成can1.c和can2.c #修改接收函数，从队列传递改为直接解算
	——————————————————————————————————————————————————————————————————————————
	19.11.12 #首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __CAN1_H__
#define __CAN1_H__

#include "board.h"

void canInit(CAN_TypeDef *can_x);
void canTx(vec4f* value,CAN_TypeDef * can_x,uint32_t id);
void canTx(float *data, CAN_TypeDef *can_x, uint32_t id);
void canTx(u8 data[8], CAN_TypeDef *can_x, uint32_t id);

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

	void CAN1_TX_IRQHandler(void);
	void CAN2_TX_IRQHandler(void);
	void CAN1_RX0_IRQHandler(void);
	void CAN2_RX0_IRQHandler(void);
#ifdef __cplusplus
}
#endif /*__cplusplus*/

struct Can1Feedback
{ /*裁判系统数据*/
	int8_t game_progress;
	int8_t hurt_armor_id;
	int8_t hurt_type;
	int8_t robot_level;
	int8_t buff_type;
	float Jgmt_RealPower; //擦破系统实时功率值
	float remainPowerBuffer;
	uint16_t chassisPowerLimit;
	uint16_t shooterId1_17mmSpeedLimit;
	uint16_t shooterId2_17mmSpeedLimit;
	float SuperPowerRemain_P;
	float SuperPower_RealPower;
	float Boost_V;
	float SuperPowerReady;
	float Jgmt_OutSpd;
	float lastJgmtOutSpd[2];
	float Jgmt_Heat[2];
	u16 CoolRate[2];
	int16_t MaxHeat[2];
	int16_t remain_hp;
	int16_t max_hp;

	u8 EnemyColor;

	u8 SuperPowerOffline;
	u8 SuperPowerOfflineCheck;
	u8 jgmtOffline;
	u8 jgmtOfflineCheck;
};

struct Can2Feedback
{
	u16 Snail_A_FeedbackSpd_Now; //摩擦轮B实际转速
	u16 Snail_B_FeedbackSpd_Now; //摩擦轮A实际转速
	u8 AS5048_offline;			//todo 磁编码器离线
	u8 ready_to_fire;			//摩擦轮允许开火
	u8 frictionOffline;			//摩擦轮模块离线
	u8 frictionOfflineCheck;	//摩擦轮模块离线
};

extern Can1Feedback can1Feedback;
extern Can2Feedback can2Feedback_LEFT;
extern Can2Feedback can2Feedback_RIGHT;
#endif
