/******************************
File name: TDT_Alg\src\can_calculation.cpp
Description: PID算法
function:
	——————————————————————————————————————————————————————————————————————————
	void Can::Motor_Offset(CAN_TypeDef* CANx, CanRxMsg* _CanRxMsg)
	——————————————————————————————————————————————————————————————————————————
	void  Can::Motor_Information_Calculate(CAN_TypeDef* CANx, CanRxMsg* _CanRxMsg)
	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.18 肖银河-增加对云台电机和底盘电机的区分，增加电机离线检测
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "can_calculate.h"
#include "motor.h"



/**
  * @brief DJI电机上电初始化
  * @param 【CAN1/CAN2，Can原始数据地址】
  */
void Can::Motor_Offset(u8 can_x, CanRxMsg* _CanRxMsg)
{
	//获取数据输出地址
	Info=&(motorList[can_x][_CanRxMsg->StdId-0x201].motorPoint->canInfo);
	/*获取电机类型*/
	MotorType motorType=motorList[can_x][_CanRxMsg->StdId-0x201].motorPoint->getType();

	//非云台电机才认为上电点为零点
	if( motorType != GM6020 && motorType != GM3510)
	{
		Info->offsetEncoder= (int16_t)(_CanRxMsg->Data[0] << 8 | _CanRxMsg->Data[1]);	
	}
	//初始角度
	Info->encoder 	= 	(int16_t)((_CanRxMsg->Data[0]<<8)|(_CanRxMsg->Data[1]));//机械角度
	Info->lastEncoder=Info->encoder;//解决上电圈数不对问题
	Info->totalRound=0;//电机角度归零时用
	Info->totalAngle=0;
	Info->totalEncoder_SI=0;
	Info->totalEncoder=0;
}



/**
  * @brief DJI电机can返回信息计算
  */
void Can::Motor_Information_Calculate(u8 can_x, CanRxMsg* _CanRxMsg)
{
	//不是大疆标准电机，直接退出
	if(IsDJIMotorCheck(*_CanRxMsg) == 0)
	{
		return;
	}
	//如果该can口的电机没有使能，则报error并退出
	if(motorList[can_x][_CanRxMsg->StdId-0x201].enableFlag==0)
	{
	//	Error(unLoadMotor);//错误码：存在未加载的电机
		return;
	}
	if(&(motorList[can_x][_CanRxMsg->StdId-0x201].motorPoint->canInfo) != 0)
	{
		//获取数据输出地址
		Info=&(motorList[can_x][_CanRxMsg->StdId-0x201].motorPoint->canInfo);
	}
	
	//初始化校准
	if( ++Info->msgCnt < 3 )
	{
		Motor_Offset(can_x,_CanRxMsg);
		return;
	}
	

	Info->msgCnt=100;
	
	/*获取电机转速及机械角度*/
	MotorType motorType=motorList[can_x][_CanRxMsg->StdId-0x201].motorPoint->getType();

	//云台电机
	if( motorType == GM3510)
	{
		Info->encoder 	= 	(int16_t)((_CanRxMsg->Data[0]<<8)|(_CanRxMsg->Data[1]));//机械角度
		Info->trueCurrent	=	(int16_t)((_CanRxMsg->Data[2]<<8)|(_CanRxMsg->Data[3]));//实际电流		
		Info->temperature	=	(int16_t)(_CanRxMsg->Data[6]);//温度
	}
	//底盘电机
	else if( motorType == GM6020 ||motorType==M2006 || motorType==M3508 || motorType==M3510)
	{
		Info->encoder 	= 	(int16_t)((_CanRxMsg->Data[0]<<8)|(_CanRxMsg->Data[1]));//机械角度
		Info->speed   	= 	(int16_t)((_CanRxMsg->Data[2]<<8)|(_CanRxMsg->Data[3]));//速度
		Info->trueCurrent = (int16_t)((_CanRxMsg->Data[4] << 8) | (_CanRxMsg->Data[5])); //实际电流
		Info->temperature	=	(int16_t)(_CanRxMsg->Data[6]);//温度	
	}

	/*计算校准后的机械角度*/
	Info->encoderCalibration = Info->encoder - Info->offsetEncoder;
	if (Info->encoderCalibration > 4096)
	{
		Info->encoderCalibration -= 8192;
	}
	else if (Info->encoderCalibration < -4096)
	{
		Info->encoderCalibration += 8192;
	}


	/*计算电机旋转总圈数*/
	if (Info->encoderCalibration - Info->lastEncoderCalibration > 4096)
	{
		Info->totalRound--;
	}
	else if (Info->encoderCalibration - Info->lastEncoderCalibration < -4096)
	{
		Info->totalRound++;
	}
	/*计算电机旋转总机械角度值*/
	Info->totalEncoder = (Info->totalRound * 8192 + Info->encoderCalibration);
	/*计算电机旋转总角度*/
	Info->totalAngle=Info->totalEncoder*0.0439453125f;//(强制转换成整型)
	Info->totalAngle_f = Info->totalEncoder * 0.0439453125f;//(浮点型)
		
	
	/*计算云台电机旋转速度=机械角度值之差与RPM换算*/
	if(  motorType == GM3510)
	{
		Info->speed = Info->totalEncoder - Info->lastTotalEncoder;
	}
	
	if( motorType == GM6020 ||motorType==M2006 || motorType==M3508 || motorType==M3510)
	{
		Info->dps = Info->speed * 6.0f;//(浮点型)
	}
	
	
	Info->totalEncoder_SI += Info->speed;
   //速度积分和位置反馈做互补滤波！！！
	/*记录此次机械角度*/
	Info->lastEncoder=Info->encoder;
	Info->lastTotalEncoder=Info->totalEncoder;
	Info->lastEncoderCalibration = Info->encoderCalibration;

	/*电机离线检测部分*/
	Info->lostCnt = 0;//清空计数器
	Info->lostFlag = 0;//电机在线
	
	//将can信息共享
	memcpy(&motorList[can_x][_CanRxMsg->StdId-0x201].motorPoint->canInfo,Info,sizeof(CanInfo));
}


/**
  * @brief 大疆标准电机ID检查
  * @return 1:是，0：不是
  */
uint8_t Can::IsDJIMotorCheck(CanRxMsg _CanRxMsg)
{
	if(_CanRxMsg.StdId>0x200 && _CanRxMsg.StdId<0x209)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}



