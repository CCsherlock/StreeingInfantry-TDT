/*****************************************************************************
File name: TDT_Device\inc\motor.h
Author: 郑俊元
Version: 1.3.1.191119_alpha
Date: 19.10.15
History: 
	——————————————————————————————————————————————————————————————————————————
	参考Readme.md
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "board.h"
#include "pid.h"
#include "can_calculate.h"
#include <stdint.h>

/*can口*/
enum Can_x
{
	Can_1 = 0, Can_2 = 1		//can口
};


//电机类型-d带G的为云台电机
enum MotorType
{
	M2006,M3508,M3510,GM3510,GM6020
};



/*************************************************
  * @class Motor电机
  * @brief 电机类，集成速度位置环的调用
  * @note
*************************************************/
class Motor
{
public:
	Pid pidInner;
	Pid pidOuter;

	CanInfo canInfo;

	/*▲ 初始化方法*/

	/**
	 * @brief Construct a new Motor object
	 */
	Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID);

	/**
	 * @brief 对电机进行初始化,包括CAN使能
	 */
	void motorInit(void);

	/*▲ 电机控制方法*/

	/**
	 * @brief 控制电流值，包括温度限幅、功率限幅、最大值限幅
	 * @param  sendFlag         是否自动调用发送函数
	 */
	float ctrlCurrent(float current, u8 sendFlag = 1);

	/**
	 * @brief 控制速度，pid计算后自动调用ctrlCurrent
	 * @param  sendFlag         是否自动调用发送函数
	 * @return float 电流值
	 */
	float ctrlSpeed(float speed, int8_t planIndex = 0, u8 sendFlag = 1);

	/**
	 * @brief 控制位置，pid计算后自动调用ctrlSpeed
	 * @param  sendFlag         是否自动调用发送函数
	 * @return float 电流值
	 */
	float ctrlPosition(double position, int8_t planIndex = 0, u8 sendFlag = 1);

	/**
	 * @brief 电机零点矫正，只对方案0适用
	 * @param  reSetSpeed       每次调用此函数，外环设定值的增量
	 * @param  maxErr           外环最大偏差
	 * @param  outLimit         输出限幅，若默认则为电流输出限幅的1/3
	 * @return u8 返回是否已经堵转，务必判断此标志位，堵转立刻退出
	 */
	u8 ctrlMotorOffset(float reSetSpeed,float maxErr,float outLimit=0);
	
/*▲ 电机设置方法*/

	/// @brief 设置电机机械零点值
	void setZeroValue(uint16_t offset);

	/// @brief 设置功率限幅指针
	void setPowerOutLimit(float* Kp);
	
	typedef void (*OutFunction)( float ,Motor* );
	
	//非大疆电机的发送函数设置[结果输出,电机编号(用于提供多个电机共用一个发送函数时区分不同电机用)]
	void setOutFunction(OutFunction);

	/// @brief 设置是否使能该电机
	void setMotorState(FunctionalState state);

	/*▲ 私有变量访问接口*/

	/// @brief 电机类型
	MotorType getType(void);

	/// @brief can口
	Can_x getCan_x(void);

	/// @brief 电机can_id
	uint32_t getStd_Id(void);

	/// @brief 获取电机是否使能
	uint8_t getEnableMotor(void);

	/// @brief 该对象对应电机的电流最大值
	uint16_t getMotorCurrentLimit(void);

	/// @brief 指定电机类型的电流最大值
	static uint16_t getMotorCurrentLimit(MotorType motorType);

	/// @brief 该对象对应电机的速度最大值
	uint16_t getMotorSpeedLimit(void);

	/// @brief 指定电机类型的速度最大值
	static uint16_t getMotorSpeedLimit(MotorType motorType);

	/// @brief 该对象对应电机的温度最大值
	uint16_t getMotorMaxTemp(void);

	/// @brief 指定电机类型的温度最大值
	static uint16_t getMotorMaxTemp(MotorType motorType);
	
private:
	/*
	结构体-电机基本信息-初始化时填充
	只允许构造器赋值
	电机常规参数
	*/
	struct _motorInfo{
		MotorType type;				//电机类型
		Can_x can_x;				//挂载CAN总线[CAN_1 / CAN_2]
		uint32_t std_ID;			//电机CAN总线反馈StdID
	}motorInfo;
	//电机额外辅助数据
	struct{
		u8 pidOuterCnt;//2			//外环计次-用于外环
		u8 posOffSetFlag;			//位置校零标志位
		double offSetPos;			//位置校零设定值变量
		float overHeatKp;		//电机过热保护系数，作用于内环输出
		float* powerOutKp;			//功率控制系数，作用于内环输出 , 功率输出限幅系数
		//以下变量初始化函数中进行赋默认值
		float currentLimit;			//电流限制
		float speedLimit;			//速度限制
		float offSetLimit;			//位置校零输出限幅系数
		float tempLimit;			//温度限制
		float criticalTemp;		//临界温度
		float maxOverTemp;		//最大超出温度
		u8 isDjiMotorFlag;			//否是DJI电机的标志位
		float errorLog[10];
	}otherInfo;
	//功能使能位
	struct{
		u8 overTempProtect;	//过热保护
		u8 canSendMsg;		//数据输出
		u8 speedToMech;		//机械角与速度积分合成
		u8 powerLimit;		//使用功率系数powerKp
	}enableFlag;

	/// @brief 电机电流发送值
	void motorPowerOut(float canResult);

	/// @brief 检查电机是否离线
	static void offlineCheck();

	/// @brief 对温度进行限幅计算
	void overHeatProtect(int16_t temp);

	/// @brief 是否为大疆电机
	u8 isDJIMotor(CAN_TypeDef* _CANx,uint32_t _Std_ID);
	static u8 DJI_MotorFlag[2][8];//用来检查std_id是否重复
	u8 canBeEnable; //为1才可初始化电机	为1条件：DJI电机: std_id：0x201-0x208，std_id不重复，
					//						非DJI电机: std_id不在0x201-0x20B
	u8 enableMotor; //电机使能标志位

	//非大疆电机的发送函数指针[结果输出,电机对象地址(用于提供多个电机共用一个发送函数时区分不同电机用)]
	void (*otherMotorFunction)(float,Motor*);

	static float canBuff[2][8]; //缓冲区

	/// @brief 定时器回调函数，can发送
	static void sendCanMsg_Callback(void* xTimer);
};


//结构体：电机列表，用于通过CAN口和ID检索（轮询）所有电机
typedef struct {
	Motor* motorPoint;	//电机对象指针
	u8 enableFlag;		//电机使能标志位
	u8 sendEnableFlag;	//电机发送使能标志位
	u16 updateOfflineCnt;	//电机更新离线计数器
	u8 updateOfflineFlag;   //电机更新离线标志位
} MotorList;

/*电机对象标记*/
extern MotorList motorList[2][8];



//最小线性二乘法拟合线性方程
class Linear
{
private:
	uint16_t getValueCnt;
	double l[4];
	
public:
	float kA;
	float kB;
	Linear()
	{
		getValueCnt=1;
		memset(l,0,sizeof(l));
	}
	void clear(void)
	{
		getValueCnt=1;
		memset(l,0,sizeof(l));
	}
	void linearRegression(float valueX,float valueY,float valueNum,float *_aResult=0,float *_bResult=0,u8 resetFlag=0)
	{
		if(resetFlag==1)
		{
			Linear();
		}
		if(++getValueCnt<valueNum)
		{
			l[0]+=valueX;
			l[1]+=valueY;
			l[2]+=valueX*valueY;
			l[3]+=valueX*valueX;
		}
		else 
		{
			kA=(valueNum*l[2]-l[0]*l[1])/(valueNum*l[3]-l[0]*l[0]);
			kB=l[1]/valueNum-kA*l[0]/valueNum;
			if(_aResult!=0 && _bResult!=0)
			{
				*_aResult=kA;
				*_bResult=kB;
			}
		}
	}
};


void testMotor(void);

#endif
