/*****************************************************************************
File name: TDT_Alg\src\imu.h
Description: 陀螺仪姿态解算算法
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19/11.19
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.19 首次记录
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _IMU_H
#define _IMU_H

#include "board.h"

#define USE_AHRS_LIB      0   //使能AHRS航资参考系统
#define COMPARE_SELF_LIB  0	   //若使能则将AHRS输出到另一个数组上(AHRS_data.nowAngle)
#define AUTO_CALIBRATION  1    //使能动态自动校准
#define GYRO_Auto_Calibration_Times 1000  //每多少次循环进行一次自动校正
#define FILTER_NUM   10		//陀螺仪滤波参数


#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RAD )

#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.324841f
#endif

#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329f
#endif

#define ITEMS  6

namespace imu{

    const uint8_t A_X = 0;
    const uint8_t A_Y = 1;
    const uint8_t A_Z = 2;
    const uint8_t G_X = 3;
    const uint8_t G_Y = 4;
    const uint8_t G_Z = 5;

    const uint8_t x = 0;
    const uint8_t y = 1;
    const uint8_t z = 2;
}

typedef struct _accdata
{
    vec3int16 origin;  //原始值
    vec3f offset_max;  //零偏值最大值
    vec3f offset_min;  //零偏值最小值	
    vec3f offset;      //零偏值 
    vec3f calibration; //校准值
    vec3f filter;      //滑动平均滤波值
	vec3f accValue;	   //加速度值，单位：m/s²
} accdata;

typedef struct _gyrodata
{
    vec3int16 origin;  //原始值
    vec3f offset_max;  //零偏值最大值
    vec3f offset_min;  //零偏值最小值	
    vec3f offset;      //零偏值 
    vec3f calibration; //校准值
    vec3f filter;      //滑动平均滤波值
    vec3f dps;         //度每秒 
    vec3f radps;       //弧度每秒
    vec3f var;         //方差
} gyrodata;

typedef struct _eulerAngle
{
    float pitch;
    float roll;
    float yaw;
} eulerAngle;

typedef struct{
    int16_t roundYaw;
    int16_t roundPitch;
    int16_t roundRoll;
} angleRounde;

typedef struct 
{
    float q0;
    float q1;
    float q2;
    float q3;
}insQuat;

typedef struct {
    float exInt;
    float eyInt;
    float ezInt;
}eInt;

class ImuCalc
{
private:
    angleRounde round;
    insQuat quaternion;
    eInt err; // scaled integral error
    void TDT_accFilter(void);
    void TDT_gyroFilter(void);
public:
    eulerAngle LastAngle;
    eulerAngle nowAngle;
    eulerAngle Angle;

	accdata acc;
    gyrodata gyro;

	float gyroCalibration[3][GYRO_Auto_Calibration_Times];

#if USE_AHRS_LIB	//使用航资参考系统
	struct
	{
		float mag[3];		//磁力计,并未用到，仅定义
		float ins_quat[4];	//四元数
		#if COMPARE_SELF_LIB
			eulerAngle nowAngle;
		#endif
	}AHRS_data;
#endif

    int16_t FILT_BUF[ITEMS][FILTER_NUM] = {0};
	
	//使用滤波
	bool accUseFilter;
	bool gyroUseFilter;

    ImuCalc();
	
	//过圈处理，获得总角度Angle
	void ImuCrossRoundHandle();

	//默认仅做了单位转换
	virtual void gyroAccUpdate();
	
	//比例因子：转换单位
	float accValueFector;
	float gyroDpsFector;

	//自动校准
	void gyroAutoCalibration();

    void calOffset_Gyro(void);//陀螺仪零偏矫正
    void calOffset_Acc(void);//加速度零偏矫正

    void TDT_IMU_update(float half_T);
	void delayMs(u32 ms);
};

#endif
