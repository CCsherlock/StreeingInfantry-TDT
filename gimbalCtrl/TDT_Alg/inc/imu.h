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
#define ORIGIN_DATA_COUNT 1000


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
    vec3f dynamicSum;  //校准时求和计算
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
} angleRound;

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

struct ImuCalc
{
private:
    angleRound round;
    insQuat quaternion;
    eInt err; // scaled integral error
    void TDT_accFilter(void);
    void TDT_gyroFilter(void);
public:
    eulerAngle LastAngle;
    eulerAngle nowAngle;
    eulerAngle Angle;

    eulerAngle AngleNoZero;

    eulerAngle AngleFuseWithTrustAngle;
    float trustAngleErr[3] = {0};
    float trustAngleInt[3] = {0};
    float *trustAnglePtr[3] = {0};

    eulerAngle AngleFuseWithTrustDps;
    float trustDpsErr[3] = {0};
    float trustDpsInt[3] = {0};
    float *trustDpsPtr[3] = {0};

    accdata acc;
    gyrodata gyro;

	u8 imu_OK;//imu初始校准完成

	struct
	{
		float mag[3];		//磁力计,并未用到，仅定义
		float ins_quat[4];	//四元数
		eulerAngle LastAngle;
		eulerAngle nowAngle;
		eulerAngle Angle;
	}AHRS_data;

    int16_t FILT_BUF[ITEMS][FILTER_NUM] = {0};
	
	//使用滤波
	bool accUseFilter;
	bool gyroUseFilter;

    ImuCalc();

    void initalAngle(eulerAngle initial);

    //过圈处理，获得总角度Angle
    void ImuCrossRoundHandle(eulerAngle &LastAngle, eulerAngle &nowAngle, eulerAngle &Angle);

    //默认仅做了单位转换
	virtual void gyroAccUpdate();
	
	//比例因子：转换单位
	float accValueFector;
	float gyroDpsFector;

	//自动校准
	bool gyroAutoCalibration();
	void gyroManualCalibration();
	struct ManualParam
	{
		int imuOriginDataIndex;
		float imuOriginDataMax[3];
		float imuOriginDataMin[3];
		float imuOriginDataAvarage[3];
		float ORIGIN_DATA_ERROR_MAX = 15.0f;
		u8 Gyro_Offset = 1;
	}manualParam;

    void getOffset(void);	//获取零偏矫正数据
	u8 forceGetOffset;		//强制进行零偏矫正

    void calOffset_Gyro(void);//陀螺仪零偏矫正
    void calOffset_Acc(void);//加速度零偏矫正
	u8 accOffsetOk = 0;

	//返回读取陀螺仪的时间
    uint64_t TDT_IMU_update(float half_T, bool useComplementaryFilter = true);
	void delayMs(u32 ms);
	void getAccOffset();
	struct AccOffsetParam
	{
		u8 ifmoving;
		vec3f accNow,accLast,accVar;
		float accVarThreshold = 1000;
        float gravityThreshold = 1000;
        int count;
		vec3f offsetValue;
		float offsetValueSum[6];
        u8 calibarationStep;
        bool completeFlag = 1;
    }accOffsetParam;
};

#endif
