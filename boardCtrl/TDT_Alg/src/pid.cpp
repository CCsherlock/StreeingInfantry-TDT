/******************************
File name: TDT_Alg\src\pid.cpp
Description: PID算法
function:
	——————————————————————————————————————————————————————————————————————————

	——————————————————————————————————————————————————————————————————————————
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History:
	——————————————————————————————————————————————————————————————————————————
	19.11.20 肖银河
	——————————————————————————————————————————————————————————————————————————
	19.11.19 肖银河-增加微分时间的处理
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "pid.h"

//运算符重载 根据=后面的类型执行对应的函数
void _FbValuePtr::operator=(uint8_t *fbValuePtr)
{
	dataType = type_u8;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(uint16_t *fbValuePtr)
{
	dataType = type_u16;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(uint32_t *fbValuePtr)
{
	dataType = type_u32;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(uint64_t *fbValuePtr)
{
	dataType = type_u64;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(int8_t *fbValuePtr)
{
	dataType = type_i8;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(int16_t *fbValuePtr)
{
	dataType = type_i16;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(int32_t *fbValuePtr)
{
	dataType = type_i32;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(int64_t *fbValuePtr)
{
	dataType = type_i64;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(float *fbValuePtr)
{
	dataType = type_float;
	this->fbValuePtr = fbValuePtr;
}
void _FbValuePtr::operator=(double *fbValuePtr)
{
	dataType = type_double;
	this->fbValuePtr = fbValuePtr;
}

//正负号或者对fbValue的值进行放大缩小
void _FbValuePtr::operator*=(double directOrAmp)
{
	this->directOrAmp *= directOrAmp;
}

//运算符重载 根据dataType确定fbValuePtr对应的函数
double _FbValuePtr::operator*()
{
	if (!(*this))
	{
		return 0;
	}
	switch (dataType)
	{
	case type_u8:
		return directOrAmp * ((*((uint8_t *)fbValuePtr)));
	case type_u16:
		return directOrAmp * (*((uint16_t *)fbValuePtr));
	case type_u32:
		return directOrAmp * (*((uint32_t *)fbValuePtr));
	case type_u64:
		return directOrAmp * (*((uint64_t *)fbValuePtr));
	case type_i8:
		return directOrAmp * (*((int8_t *)fbValuePtr));
	case type_i16:
		return directOrAmp * (*((int16_t *)fbValuePtr));
	case type_i32:
		return directOrAmp * (*((int32_t *)fbValuePtr));
	case type_i64:
		return directOrAmp * (*((int64_t *)fbValuePtr));
	case type_float:
		return directOrAmp * (*((float *)fbValuePtr));
	case type_double:
		return directOrAmp * (*((double *)fbValuePtr));
	}
}

//操作符重载 判断指针是否为空
bool _FbValuePtr::operator!()
{
	return fbValuePtr == 0;
}

//构造器重载 根据参数的类型执行对应的函数
_FbValuePtr::_FbValuePtr() : fbValuePtr(0), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(int8_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_u8), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(int16_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_u16), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(int32_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_u32), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(int64_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_u64), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(uint8_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_i8), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(uint16_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_i16), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(uint32_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_i32), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(uint64_t *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_i64), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(float *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_float), directOrAmp(1) {}
_FbValuePtr::_FbValuePtr(double *fbValuePtr) : fbValuePtr(fbValuePtr), dataType(type_double), directOrAmp(1) {}

/**
  * @brief 构造器
  */
Pid::Pid() : planNum(0), lastPlanIndex(-1), paramPtr(0), fbValuePtr(0)
{
}

/**
  * @brief Pid类构造器，初始化传入参数
  * @param [pid参数指针]
  * @note 自定义Pid时可以直接传入参数
  */
Pid::Pid(int planNum) : planNum(planNum),
						lastPlanIndex(-1),
						paramPtr(0),fbValuePtr(0)
{
	if(planNum > 0)
		fbValuePtr = new _FbValuePtr[planNum];
}

/**
  * @brief 装载默认PID参数,数据载入new指针
  * @param [&PID数据地址]
  */
void Pid::setPlanNum(int planNum)
{
	if(this->planNum == planNum)
		return;
	if(this->planNum == 0)
	{
		fbValuePtr = new _FbValuePtr[planNum];
		this->planNum = planNum;
		return;
	}
	if(planNum == 0)
	{
		delete fbValuePtr;
		planNum = 0;
		return;
	}

	auto tmp = new _FbValuePtr[planNum];
	int num = planNum > this->planNum?this->planNum:planNum;

	memcpy(tmp,fbValuePtr,num * sizeof(_FbValuePtr));

	delete fbValuePtr;
	fbValuePtr = tmp;
	this->planNum = planNum;
}


int Pid::getPlanNum()
{
	return planNum;
}

/**
  * @brief 清零具有累加性质的PID变量
  */
void Pid::Clear()
{
	lastError = 0;
	integralError = 0;
	deltaError = 0;
	result = 0;
	pidCycle.getCycleT();
}

/**
  * @brief 检查PID参数是否加载
  * @return [bool]参数是否加载
  * @note 实际上只检查了输出限幅和积分限幅
  */
bool Pid::LoadCheck(int planIndex)
{
	if (planIndex > planNum || fbValuePtr == 0 || paramPtr == 0 || !fbValuePtr[planIndex])
		return false;
	if (paramPtr[planIndex].resultMax == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

/**
  * @brief PID计算函数
  * @param
  * @return [bool]参数是否加载
  * @note 实际上只检查了输出限幅和积分限幅
  */
float Pid::Calculate(double _setValue, double _fbValue, int planIndex, float *_result, float _T)
{
	if(!LoadCheck(planIndex))
		return 0;
	if (planIndex != lastPlanIndex)//方案号更改时
	{
		nowPlanIndex = planIndex;
		nowParamPtr = paramPtr+planIndex;

		lastPlanIndex = planIndex;
		Clear();
		return 0;
	}


	//默认结果地址
	if (_result == 0)
	{
		_result = &(this->result);
	}
	/* 设定值 */
	setValue = _setValue;
	/* 反馈值 */
	fbValue = _fbValue;
	/* 偏差 = 设定值 - 反馈值 */
	error = setValue - fbValue;

	//默认周期时间
	if (_T == 0)
	{
		_T = pidCycle.getCycleT();
		//如果间隔时间过长, 不计算积分微分
		if (_T > 0.1f) //100ms
		{
			Clear();
			result = paramPtr[planIndex].kp * error;
			if (this->paramPtr[planIndex].positiveFBFlag == 1)
			{
				result *= -1;
			}
			/* 总的输出不能超出电机给定值的范围 */
			result = LIMIT(result, -paramPtr[planIndex].resultMax, paramPtr[planIndex].resultMax);
			//输出
			*_result = result;
			return result;
		}
	}

	/* 偏差进行积分 */
	integralError += error * _T;
	if (_T != 0)
	{
			/* 偏差增量 */
			deltaError = (error - lastError) / _T;
			/* 记录本次误差 */
			lastError = error;
	}
	if(paramPtr[planIndex].ki == 0)
	{
		integralError = 0;//清积分
	}
	/* 总的输出 = 比例项的输出 + 积分项的输出 + 微分项的输出 */
	result = paramPtr[planIndex].kp * error + paramPtr[planIndex].ki * integralError + paramPtr[planIndex].kd * deltaError;

	if (this->paramPtr[planIndex].positiveFBFlag == 1)
	{
		result *= -1;
	}
	lastDeltaError = deltaError;
	/* 总的输出不能超出电机给定值的范围 */
	result = LIMIT(result, -paramPtr[planIndex].resultMax, paramPtr[planIndex].resultMax);
	//输出
	*_result = result;
	return result;
}

float Pid::Calculate(double _setValue, int planIndex, float *result, float T_)
{
	if(!LoadCheck(planIndex))
		return 0;
	return Calculate(_setValue, *fbValuePtr[planIndex], planIndex, result, T_);
}

double Pid::getFbValue(int planIndex)
{
	if (planIndex >= planNum || planIndex < 0)
		return 0;
	return *fbValuePtr[planIndex];
}

//电机堵转检测【最大误差】
u8 Pid::Is_LockTurn(float maxErr)
{
	if (ABS(error) > maxErr)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
