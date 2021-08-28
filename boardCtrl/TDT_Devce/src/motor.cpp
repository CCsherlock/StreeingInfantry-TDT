/******************************
File name: TDT_Device\src\can_task.cpp
Description: 电机数据的处理
Class:
	——————————————————————————————————————————————————————————————————————————
	Motor
	——————————————————————————————————————————————————————————————————————————
Author: 郑竣元
Version: 1.3.1.191119_alpha
Date: 19.10.15
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.24 肖银河-改写数据合帧的方法，修复数据合帧时的BUG
	——————————————————————————————————————————————————————————————————————————
	19.11.19 肖银河-总体框架更改，将三大主体部分重新分配完成-核心功能已测试
	——————————————————————————————————————————————————————————————————————————
	19.11.15 肖银河-将方法的具体实现放到cpp文件
	——————————————————————————————————————————————————————————————————————————
	19.11.12 肖银河-将C_Motor分为Pid feedBack Motor三部分，并完善代码规范
	——————————————————————————————————————————————————————————————————————————
	19.10.15 郑竣元-首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************/
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/
#include "motor.h"
#include "can.h"
#include "dbus.h"
MotorList motorList[2][8];
//顺序对应枚举变量MotorType里的电机类型排列-M2006,M3508,M3510,GM3510,GM6020
//电机电流输出限制表
uint16_t motorCurrentLimitList[5] = {10000, 16384, 32760, 29000, 30000};
//电机速度输出限制表-取决于各类电机的最高转速
uint16_t motorSpeedLimitList[5] = {9500, 9158, 9600, 1200, 320};
//电机绕组最高允许温度列表
uint16_t motorMaxTempList[5] = {0, 125, 0, 100, 125};

/*电机类大疆电机标志位初始化*/
u8 Motor::DJI_MotorFlag[2][8] = {0};
float Motor::canBuff[2][8] = {0};

static void *sendCanMsg_Handle = 0;
static u8 timerInit_Flag = 0;

extern uint8_t deforceFlag; //脱力标志位

/**
 * @brief Construct a new Motor:: Motor object
 * @param  motorType        电机类型: 用以区别不同类型的电机
 * @param  offsetEncoder    零点值：用以设置机械零点
 * @param  enableFlag       使能位：是否在构造时初始化
 * @param  CANx             CAN口
 * @param  std_ID           CAN标准标识符
 */
Motor::Motor(MotorType motorType, CAN_TypeDef *_Canx, uint32_t _Std_ID) : pidInner(Pid(1)), pidOuter(Pid(1))
{
	//电机基本信息填充
	motorInfo.type = motorType;
	motorInfo.std_ID = _Std_ID;
	motorInfo.can_x = _Canx == CAN1 ? Can_1 : Can_2;
	canInfo.offsetEncoder = 0;

	//电机额外信息填充
	otherInfo.currentLimit = motorCurrentLimitList[motorType]; //输出限幅（内环输出限幅）
	otherInfo.speedLimit = motorSpeedLimitList[motorType];	 //速度限幅（外环输出限幅）
	otherInfo.tempLimit = motorMaxTempList[motorType];		   //绕组最高允许温度
	otherInfo.offSetLimit = otherInfo.currentLimit / 3;		   //默认位置校零时以三分之一输出开始
	otherInfo.criticalTemp = 0.7f * otherInfo.tempLimit;	   //过热保护临界温度
	otherInfo.maxOverTemp = 0.8f * otherInfo.tempLimit;		   //过热保护截止温度，高于此温度电机输出为0

	//对于有最高绕组温度的电机开启过热保护
	if (motorMaxTempList[motorType] != 0)
	{
		this->enableFlag.overTempProtect = 1; //过热保护使能
	}

	//pid默认反馈值填充
	pidInner.fbValuePtr[0] = &canInfo.speed;
	pidOuter.fbValuePtr[0] = &canInfo.totalEncoder;

	//进行初始化
	motorInit();
}

/**
 * @brief 对电机进行初始化,包括CAN使能
 */
void Motor::motorInit(void)
{
	//检测重复定义
	if (isDJIMotor(motorInfo.can_x == Can_1 ? CAN1 : CAN2, motorInfo.std_ID)) //是大疆电机
	{
		otherInfo.isDjiMotorFlag = 1;
		if (DJI_MotorFlag[motorInfo.can_x][motorInfo.std_ID - 0x201] == 0 || (DJI_MotorFlag[motorInfo.can_x][motorInfo.std_ID - 0x201] == 1 && motorList[motorInfo.can_x][motorInfo.std_ID - 0x201].motorPoint == this)) //此前该id未定义
		{
			canBeEnable = 1; //能够被初始化

			if (timerInit_Flag == 0) //发送定时器未开启
			{
				timerInit_Flag = 1;
				//创建定时器, 1ms触发一次，自动重装载，定时器回调函数：Motor::sendCanMsg_Callback
				sendCanMsg_Handle = xTimerCreate((const char *)"sendCanMsg_Timer", pdMS_TO_TICKS(2), (UBaseType_t)pdTRUE, (void *)1, (TimerCallbackFunction_t) Motor::sendCanMsg_Callback);
				//开启定时器
				xTimerStart(sendCanMsg_Handle, pdMS_TO_TICKS(2));
			}
		}
		else
		{
			canBeEnable = 0; //无法被使能
		}
	}
	else
	{
		otherInfo.isDjiMotorFlag = 0;
		canBeEnable = 1;
	}

	//电机功能使能位
	enableFlag.canSendMsg = canBeEnable; //发送使能
	enableMotor = canBeEnable;

	if(otherInfo.isDjiMotorFlag)//大疆电机
	{
		DJI_MotorFlag[motorInfo.can_x][motorInfo.std_ID - 0x201] = canBeEnable;
		//更新电机列表
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].motorPoint = this;
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].enableFlag = canBeEnable;
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].sendEnableFlag = canBeEnable;
	}
}

/**
 * @brief 控制电流值，包括温度限幅、功率限幅、最大值限幅
 * @param  current          电流值
 * @param  sendFlag         是否自动调用发送函数
 * @return float 经过温度限幅、功率限幅、最大值限幅的电流值
 */
float Motor::ctrlCurrent(float current, u8 sendFlag)
{
	//输出限幅
	if (this->otherInfo.isDjiMotorFlag)
	{
		current = LIMIT(current, -motorCurrentLimitList[motorInfo.type], motorCurrentLimitList[motorInfo.type]);
	}
	//过热保护
	if (enableFlag.overTempProtect == 1)
	{
		this->overHeatProtect(canInfo.temperature);
		current *= otherInfo.overHeatKp;
	}
	// 功率输出限幅系数
	if (enableFlag.powerLimit == 1)
	{
		current *= *otherInfo.powerOutKp;
	}

	//如果can发送使能且电机在线
	if (sendFlag && enableFlag.canSendMsg)
	{
		//非大疆电机调用自定义发送函数
		if (this->otherInfo.isDjiMotorFlag == 0)
		{
			//参数检查
			if (this->otherMotorFunction != 0)
			{
				otherMotorFunction(current, this);
			}
		}
		else if (otherInfo.isDjiMotorFlag && canInfo.lostFlag || deforceFlag) //电机未丢失并且未脱力
		{
			/*结果发送到缓存区*/
			motorPowerOut(0); //电机丢失或脱力 发0
			return 0;
		}
		else
		{
			motorPowerOut(current); //
		}
	}
	return current;
}

/**
 * @brief 控制速度，pid计算后自动调用ctrlCurrent
 * @param  speed            速度设定值
 * @param  planIndex        方案号
 * @param  sendFlag         是否自动调用发送函数
 * @return float 经过内环pid控制，以及温度限幅、功率限幅、最大值限幅的电流值
 */
float Motor::ctrlSpeed(float speed, int8_t planIndex, u8 sendFlag)
{
	/*电流输出*/
	//DJI电机丢失发0
	if ((otherInfo.isDjiMotorFlag && canInfo.lostFlag))
	{
		pidInner.Clear();
		return ctrlCurrent(0, sendFlag);
	}
	/*PID计算，默认输出到对象的result变量*/
	pidInner.Calculate(speed, planIndex);
	return ctrlCurrent(pidInner.result, sendFlag);
}

/**
 * @brief 控制位置，pid计算后自动调用ctrlSpeed
 * @param  position         位置设定值
 * @param  planIndex        方案号
 * @param  sendFlag         是否自动调用发送函数
 * @return float 经过双环pid控制，以及温度限幅、功率限幅、最大值限幅的电流值
 */
float Motor::ctrlPosition(double position, int8_t planIndex, u8 sendFlag)
{
	/*内环*/
	//todo PositiveFeedback
	//DJI电机丢失发0
	if ((otherInfo.isDjiMotorFlag && canInfo.lostFlag))
	{
		pidOuter.Clear();
		return ctrlSpeed(0, 0, sendFlag);
	}
	
	/*外环*/
	if (++otherInfo.pidOuterCnt >= 2)
	{
		otherInfo.pidOuterCnt = 0;
		/*PID计算，默认输出到对象的result变量*/
		pidOuter.Calculate(position, planIndex);
	}
	return ctrlSpeed(pidOuter.result, planIndex, sendFlag);
}

/**
 * @brief 电机电流发送值
 * @param  canResult        电流值
 */
void Motor::motorPowerOut(float canResult)
{
	//根据can口和id填充缓冲区并清空计数器和标志位
	canBuff[motorInfo.can_x][motorInfo.std_ID - 0x201] = canResult;				//填充缓冲区
	motorList[motorInfo.can_x][motorInfo.std_ID - 0x201].updateOfflineCnt = 0;  //清空计数器
	motorList[motorInfo.can_x][motorInfo.std_ID - 0x201].updateOfflineFlag = 0; //清空标志位
}

/**
  * @brief 非大疆电机的发送函数设置
  * @param [结果输出,电机编号]
  * @note 电机编号用于提供多个电机共用一个发送函数时区分不同电机用
  */

void Motor::setOutFunction(OutFunction outFun)
{
	otherMotorFunction = outFun;
}

/**
 * @brief 
 * @param  _CANx            CAN口
 * @param  _Std_ID          can_id
 * @return u8 是否为大疆电机
 */
u8 Motor::isDJIMotor(CAN_TypeDef *_CANx, uint32_t _Std_ID)
{
	if (_CANx != CAN1 && _CANx != CAN2)
	{
		return 0;
	}
	else
	{
		if (_Std_ID < 0x201 || _Std_ID > 0x208)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
}

/**
 * @brief 电机零点矫正，只对方案0适用
 * @param  reSetSpeed       每次调用此函数，外环设定值的增量
 * @param  maxErr           外环最大偏差
 * @param  outLimit         输出限幅，若默认则为电流输出限幅的1/3
 * @return u8 返回是否已经堵转，务必判断此标志位，堵转立刻退出
 */
#define Change_Value(a, b) \
	a ^= b;                \
	b ^= a, a ^= b; //交换两个变量的值，仅限于整形相同变量类型
u8 Motor::ctrlMotorOffset(float reSetSpeed, float maxErr, float outLimit)
{
	if (outLimit != 0)
	{
		//更新输出限幅
		this->otherInfo.offSetLimit = outLimit;
	}

	/*校零开始*/
	if (otherInfo.posOffSetFlag == 0) //获取初始角
	{
		//更改PID输出限幅为校零模式
		pidOuter.paramPtr[0].resultMax = otherInfo.offSetLimit;
		//获取外环信息
		otherInfo.offSetPos = *pidOuter.fbValuePtr[0];
		otherInfo.posOffSetFlag = 1;
	}
	else if (otherInfo.posOffSetFlag == 1) //开始反转
	{
		otherInfo.offSetPos += reSetSpeed;
	}

	/*校零结束*/
	if (pidOuter.Is_LockTurn(maxErr) == 1) //当堵转时
	{
		int16_t offsetEncoderBuf = canInfo.offsetEncoder;
		memset(&canInfo, 0, sizeof(CanInfo));
		canInfo.offsetEncoder = offsetEncoderBuf;

		pidOuter.Clear();
		pidOuter.Clear();
		otherInfo.posOffSetFlag = 0;
		//恢复PID输出限幅为正常模式
		pidOuter.paramPtr[0].resultMax = otherInfo.currentLimit;
		return 1;
	}
	else
	{
		ctrlPosition(otherInfo.offSetPos);
	}
	return 0;
}

/**
 * @brief 对温度进行限幅计算
 * @param  temp             当前温度
 */
void Motor::overHeatProtect(int16_t temp)
{
	otherInfo.overHeatKp = (float)(1.0f - ((temp - otherInfo.criticalTemp) / (otherInfo.maxOverTemp - otherInfo.criticalTemp)));
	otherInfo.overHeatKp = LIMIT(otherInfo.overHeatKp, 0, 1);
}

/********************/

/**
 * @brief Set the Zero Value object
 * @param  offset           电机机械零点值
 */
void Motor::setZeroValue(uint16_t offset)
{
	canInfo.offsetEncoder = offset;

	canInfo.encoderCalibration = canInfo.encoder - canInfo.offsetEncoder;
	if (canInfo.encoderCalibration > 4096)
	{
		canInfo.encoderCalibration -= 8192;
	}
	else if (canInfo.encoderCalibration < -4096)
	{
		canInfo.encoderCalibration += 8192;
	}
	canInfo.encoderCalibration = canInfo.encoder - canInfo.offsetEncoder;
	if (canInfo.encoderCalibration > 4096)
	{
		canInfo.encoderCalibration -= 8192;
	}
	else if (canInfo.encoderCalibration < -4096)
	{
		canInfo.encoderCalibration += 8192;
	}
}

/**
 * @brief Set the Power Out Limit object
 * @param  Kp               功率限幅指针
 */
void Motor::setPowerOutLimit(float *Kp)
{
	enableFlag.powerLimit = 1;
	otherInfo.powerOutKp = Kp;
}

/**
 * @brief Set the Motor State object
 * @param  state            是否使能该电机
 */
void Motor::setMotorState(FunctionalState state)
{
	if (state == ENABLE)
	{
		motorInit();
	}
	else
	{
		enableMotor = 0;
		motorList[(u8)motorInfo.can_x][motorInfo.std_ID - 0x201].enableFlag = 0;
	}
}

/**
 * @brief Get the Type object
 * @return MotorType 电机类型
 */
MotorType Motor::getType(void)
{
	return motorInfo.type;
}
/**
 * @brief Get the Can x object
 * @return Can_x can口
 */
Can_x Motor::getCan_x(void)
{
	return motorInfo.can_x;
}
/**
 * @brief Get the Std Id object
 * @return uint32_t 电机can_id
 */
uint32_t Motor::getStd_Id(void)
{
	return motorInfo.std_ID;
}

/**
 * @brief Get the Enable Motor object
 * @return uint8_t 获取电机是否使能
 */
uint8_t Motor::getEnableMotor(void)
{
	return enableMotor;
}

/**
 * @brief Get the Motor Current Limit object
 * @return uint16_t 该对象对应电机的电流最大值
 */
uint16_t Motor::getMotorCurrentLimit(void)
{
	return motorCurrentLimitList[motorInfo.type];
}
/**
 * @brief Get the Motor Current Limit object
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的电流最大值
 */
uint16_t Motor::getMotorCurrentLimit(MotorType motorType)
{
	return motorCurrentLimitList[motorType];
}

/**
 * @brief Get the Motor Speed Limit object
 * @return uint16_t 该对象对应电机的速度最大值
 */
uint16_t Motor::getMotorSpeedLimit(void)
{
	return motorSpeedLimitList[motorInfo.type];
}
/**
 * @brief Get the Motor Speed Limit object
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的速度最大值
 */
uint16_t Motor::getMotorSpeedLimit(MotorType motorType)
{
	return motorSpeedLimitList[motorType];
}

/**
 * @brief Get the Motor Max Temp object
 * @return uint16_t 该对象对应电机的温度最大值
 */
uint16_t Motor::getMotorMaxTemp(void)
{
	return motorMaxTempList[motorInfo.type];
}
/**
 * @brief Get the Motor Max Temp object
 * @param  motorType        指定电机类型
 * @return uint16_t 指定电机类型的温度最大值
 */
uint16_t Motor::getMotorMaxTemp(MotorType motorType)
{
	return motorMaxTempList[motorType];
}

/**
 * @brief 定时器回调函数，can发送
 * @param  xTimer           哪个定时器
 */
void Motor::sendCanMsg_Callback(TimerHandle_t xTimer)
{
	if (deforceFlag)
	{
		float canBuff[8] = {0};
		//can1 or can2
		for (int can_i = 0; can_i < 2; can_i++)
		{
			//0~3 OR 4~7
			for (int idIndex = 0; idIndex < 2; idIndex++)
			{
				canTx(canBuff, can_i == 0 ? CAN1 : CAN2, idIndex == 0 ? 0x200 : 0x1ff);
			}
		}
		return;
	}
	//can1 or can2
	for (int can_i = 0; can_i < 2; can_i++)
	{
		//0~3 OR 4~7
		for (int idIndex = 0; idIndex < 2; idIndex++)
		{
			bool containMotor = false;
			//遍历4个电机
			for (int idOffset = 0; idOffset < 4; idOffset++)
			{
				//未定义
				if(motorList[can_i][idIndex * 4 + idOffset].motorPoint  == 0)
					continue;
				if(motorList[can_i][idIndex * 4 + idOffset].motorPoint->enableMotor == 0)
					continue;
				
				containMotor = true;
				/*电机离线检测部分*/
				if(motorList[can_i][idIndex * 4 + idOffset].motorPoint->canInfo.lostCnt > 200)//两百个周期都没有接收到
				{
					motorList[can_i][idIndex * 4 + idOffset].motorPoint->canInfo.lostFlag = 1;//离线
					canBuff[can_i][idIndex * 4 + idOffset] = 0; //自动脱力该电机
					continue;
				}
				motorList[can_i][idIndex * 4 + idOffset].motorPoint->canInfo.lostCnt++;
				
				//超过100ms没有刷新计数器，认为没有调用控制/发送函数，自动脱力该电机
				if (motorList[can_i][idIndex * 4 + idOffset].updateOfflineCnt > 100) //100ms
				{
					motorList[can_i][idIndex * 4 + idOffset].updateOfflineFlag = 1;
					canBuff[can_i][idIndex * 4 + idOffset] = 0; //自动脱力该电机
					continue;
				}
				motorList[can_i][idIndex * 4 + idOffset].updateOfflineCnt++;
			}
			if (!containMotor) //不包含电机，跳过
				continue;
			
			canTx(&canBuff[can_i][idIndex * 4], can_i == 0 ? CAN1 : CAN2, idIndex == 0 ? 0x200 : 0x1ff);
		}
	}
}

/************Motor类-结束***********  */
#if 0
#define PLAN_NUM 2
PidParam innerPidPara[PLAN_NUM], outerPidPara[PLAN_NUM];
Motor motorTest(GM6020, CAN1, 0x205);

float setPos=0;
u8 start=0;

void testMotor(void)
{
	if(start==0)
	{

		innerPidPara[0].resultMax = motorTest.getMotorCurrentLimit();
		innerPidPara[1].resultMax = motorTest.getMotorCurrentLimit();

		outerPidPara[0].resultMax = motorTest.getMotorSpeedLimit();
		outerPidPara[1].resultMax = motorTest.getMotorSpeedLimit();

		motorTest.pidInner.setPlanNum(PLAN_NUM);
		motorTest.pidOuter.setPlanNum(PLAN_NUM);

		motorTest.pidInner.paramPtr = innerPidPara;
		motorTest.pidOuter.paramPtr = outerPidPara;

		//方案号0默认填充反馈值
		// motorTest.pidInner.fbValuePtr[0] = &motorTest.canInfo.speed;
		// motorTest.pidOuter.fbValuePtr[0] = &motorTest.canInfo.totalEncoder;

		motorTest.pidInner.fbValuePtr[1] = &motorTest.canInfo.speed;
		motorTest.pidOuter.fbValuePtr[1] = &motorTest.canInfo.totalEncoder;

		motorTest.motorInit();

		start=1;
	}
	motorTest.ctrlPosition(setPos);

}
#endif
