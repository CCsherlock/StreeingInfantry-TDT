/******************************
File name: TDT_Alg\src\imu.cpp
Description: 陀螺仪姿态解算算法
function:
	——————————————————————————————————————————————————————————————————————————
	void TDT_IMUBotupdate(float half_T, vec3f* gyro, vec3f* acc)
	——————————————————————————————————————————————————————————————————————————
	void TDT_IMUTopupdate(float half_T, vec3f* gyro, vec3f* acc)
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19/11.19
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.19 首次记录
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "imu.h"
#include "my_math.h"
#include "mpu6050.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "TimeMatch.h"

using namespace imu;
extern tdtusart::timeSimulaneity imuTimeMatch;

#define PITCH_AXIS roll
#define YAW_AXIS yaw

//二阶互补滤波系数，规律：基本时间常数tau得到基本系数a，Kp=2*a，Ki=a^2;
#define Kp 0.6f                           // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.1f                           // integral gain governs rate of convergence of gyroscope biases

#if USE_AHRS_LIB && COMPARE_SELF_LIB
//J-scope数据测试
eulerAngle angleFromLib;
eulerAngle angleFromSelf;
#endif
ImuCalc::ImuCalc():round({0}),quaternion({1,0,0,0}),err({0}),
accValueFector(1),gyroDpsFector(1)
{
	memset(&Angle,0,sizeof(Angle));
	memset(&LastAngle,0,sizeof(LastAngle));
	memset(&nowAngle,0,sizeof(nowAngle));
}


void ImuCalc::calOffset_Acc()
{
    uint16_t cnt_g = 1000;
    int32_t tempgx = 0, tempgy = 0, tempgz = 0;

    acc.offset_max.data[x] = -32768;
    acc.offset_max.data[y] = -32768;
    acc.offset_max.data[z] = -32768;
    acc.offset_min.data[x] = 32767;
    acc.offset_min.data[y] = 32767;
    acc.offset_min.data[z] = 32767;

    while (cnt_g--)
    {
        delayMs(2);
        gyroAccUpdate();
        if (acc.origin.data[x] > acc.offset_max.data[x])
            acc.offset_max.data[x] = acc.origin.data[x];
        if (acc.origin.data[y] > acc.offset_max.data[y])
            acc.offset_max.data[y] = acc.origin.data[y];
        if (acc.origin.data[z] > acc.offset_max.data[z])
            acc.offset_max.data[z] = acc.origin.data[z];

        if (acc.origin.data[x] < acc.offset_min.data[x])
            acc.offset_min.data[x] = acc.origin.data[x];
        if (acc.origin.data[y] < acc.offset_min.data[y])
            acc.offset_min.data[y] = acc.origin.data[y];
        if (acc.origin.data[z] < acc.offset_min.data[z])
            acc.offset_min.data[z] = acc.origin.data[z];

        tempgx += acc.origin.data[x];
        tempgy += acc.origin.data[y];
        tempgz += acc.origin.data[z];
    }

    //1000次数据有一个异常,重新校准
    if (acc.offset_max.data[x] - acc.offset_min.data[x] > 200 ||
        acc.offset_max.data[y] - acc.offset_min.data[y] > 200 ||
        acc.offset_max.data[z] - acc.offset_min.data[z] > 200)
        calOffset_Acc();
    else
    {
        acc.offset.data[x] = (float) (tempgx) / 1000;//-28.50//
        acc.offset.data[y] = (float) (tempgy) / 1000;//23.465//
        acc.offset.data[z] = (float) (tempgz) / 1000;//-13.90/
    }
//	acc.offset.data[x] =275.552002;  //495.138;
//	acc.offset.data[y] =105.733002;   //10.645005;
//	acc.offset.data[z] =0;
}

//陀螺仪零偏矫正
void ImuCalc::calOffset_Gyro(void)
{
    static u8 calibrationTimes = 0;
    while (1)
    {
        /*读取1000次陀螺仪数值*/
        for (int i = 0; i < GYRO_Auto_Calibration_Times; i++)
        {
			gyroAccUpdate();
            gyroCalibration[0][i] = gyro.origin.data[0];
            gyroCalibration[1][i] = gyro.origin.data[1];
            gyroCalibration[2][i] = gyro.origin.data[2];
            delayMs(1);
        }

        /*方差的计算*/
        arm_var_f32(gyroCalibration[0], GYRO_Auto_Calibration_Times, &gyro.var.data[0]);
        arm_var_f32(gyroCalibration[1], GYRO_Auto_Calibration_Times, &gyro.var.data[1]);
        arm_var_f32(gyroCalibration[2], GYRO_Auto_Calibration_Times, &gyro.var.data[2]);

        calibrationTimes++;//已进行的自动校正次数

        /*若自动校正无法通过，则使用默认零偏校正值*/
        if (calibrationTimes > 10)
        {
            gyro.offset.data[x] = -85.0f;//-85.3320007;
            gyro.offset.data[y] = -18.0f;//-16.4330006;
            gyro.offset.data[z] = -2.5f;//-2.45499992;
            break;
        }

        /*方差小于一定值，更新零漂校正值*/
        if (gyro.var.data[0] < 50 && gyro.var.data[1] < 50 && gyro.var.data[2] < 50)
        {
            /*计算平均值，作为偏移量*/
            arm_mean_f32(gyroCalibration[0], GYRO_Auto_Calibration_Times, &gyro.offset.data[0]);
            arm_mean_f32(gyroCalibration[1], GYRO_Auto_Calibration_Times, &gyro.offset.data[1]);
            arm_mean_f32(gyroCalibration[2], GYRO_Auto_Calibration_Times, &gyro.offset.data[2]);
            break;
        }
    }
	
#if USE_AHRS_LIB

	//AHRS航资参考系统初始化
	AHRS_init(AHRS_data.ins_quat, acc.calibration.data, AHRS_data.mag);
	//航资参考系统获取角度，具体声明参考ahrs_lib.h
    get_angle(AHRS_data.ins_quat, &Angle.pitch, &Angle.yaw, &Angle.roll);
	//单位换算
	Angle.pitch *= RAD_TO_ANGLE;
	Angle.yaw *= RAD_TO_ANGLE;
	Angle.roll *= RAD_TO_ANGLE;
		
	nowAngle.pitch = Angle.pitch;
	nowAngle.yaw = Angle.yaw;
	nowAngle.roll = Angle.roll;
#endif
}

void ImuCalc::TDT_accFilter()
{
    u8 i;
    int32_t FILT_TMP[ITEMS] = {0};

    for (i = FILTER_NUM - 1; i >= 1; i--)
    {
        FILT_BUF[A_X][i] = FILT_BUF[A_X][i - 1];
        FILT_BUF[A_Y][i] = FILT_BUF[A_Y][i - 1];
        FILT_BUF[A_Z][i] = FILT_BUF[A_Z][i - 1];
    }

    FILT_BUF[A_X][0] = acc.calibration.data[x];
    FILT_BUF[A_Y][0] = acc.calibration.data[y];
    FILT_BUF[A_Z][0] = acc.calibration.data[z];

    for (i = 0; i < FILTER_NUM; i++)
    {
        FILT_TMP[A_X] += FILT_BUF[A_X][i];
        FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
        FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
    }
	
    acc.filter.data[x] = (float) (FILT_TMP[A_X]) / (float) FILTER_NUM;
    acc.filter.data[y] = (float) (FILT_TMP[A_Y]) / (float) FILTER_NUM;
    acc.filter.data[z] = (float) (FILT_TMP[A_Z]) / (float) FILTER_NUM;
		
	acc.accValue.data[x] = acc.filter.data[x] * accValueFector;
	acc.accValue.data[y] = acc.filter.data[y] * accValueFector;
	acc.accValue.data[z] = acc.filter.data[z] * accValueFector;
}

void ImuCalc::TDT_gyroFilter()
{
    u8 i;
    int32_t FILT_TMP[ITEMS] = {0};


    for (i = FILTER_NUM - 1; i >= 1; i--)
    {
        FILT_BUF[G_X][i] = FILT_BUF[G_X][i - 1];
        FILT_BUF[G_Y][i] = FILT_BUF[G_Y][i - 1];
        FILT_BUF[G_Z][i] = FILT_BUF[G_Z][i - 1];
    }

    FILT_BUF[G_X][0] = gyro.calibration.data[x];
    FILT_BUF[G_Y][0] = gyro.calibration.data[y];
    FILT_BUF[G_Z][0] = gyro.calibration.data[z];

    for (i = 0; i < FILTER_NUM; i++)
    {
        FILT_TMP[G_X] += FILT_BUF[G_X][i];
        FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
        FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
    }

    gyro.filter.data[x] = (float) (FILT_TMP[G_X]) / (float) FILTER_NUM;
    gyro.filter.data[y] = (float) (FILT_TMP[G_Y]) / (float) FILTER_NUM;
    gyro.filter.data[z] = (float) (FILT_TMP[G_Z]) / (float) FILTER_NUM;
	
    gyro.dps.data[x] = gyro.filter.data[x] * gyroDpsFector;
    gyro.dps.data[y] = gyro.filter.data[y] * gyroDpsFector;
    gyro.dps.data[z] = gyro.filter.data[z] * gyroDpsFector;

    gyro.radps.data[x] = gyro.dps.data[x] * ANGLE_TO_RAD;
    gyro.radps.data[y] = gyro.dps.data[y] * ANGLE_TO_RAD;
    gyro.radps.data[z] = gyro.dps.data[z] * ANGLE_TO_RAD;
}

void ImuCalc::gyroAutoCalibration()
{
	static uint16_t runningTimes;

    //记录原始数据
    gyroCalibration[0][runningTimes] = gyro.origin.data[0];
    gyroCalibration[1][runningTimes] = gyro.origin.data[1];
    gyroCalibration[2][runningTimes] = gyro.origin.data[2];

    runningTimes++;
    //每循环GYRO_Auto_Calibration_Times次执行校准函数
    if (runningTimes >= GYRO_Auto_Calibration_Times)
    {
        /*方差计算*/
        arm_var_f32(gyroCalibration[0], GYRO_Auto_Calibration_Times, &gyro.var.data[0]);
        arm_var_f32(gyroCalibration[1], GYRO_Auto_Calibration_Times, &gyro.var.data[1]);
        arm_var_f32(gyroCalibration[2], GYRO_Auto_Calibration_Times, &gyro.var.data[2]);

        /*方差小于一定值，更新零漂校正值*/
        if (gyro.var.data[0] < 50 && gyro.var.data[1] < 50 && gyro.var.data[2] < 50)
        {
            arm_mean_f32(gyroCalibration[0], GYRO_Auto_Calibration_Times, &gyro.offset.data[0]);
            arm_mean_f32(gyroCalibration[1], GYRO_Auto_Calibration_Times, &gyro.offset.data[1]);
            arm_mean_f32(gyroCalibration[2], GYRO_Auto_Calibration_Times, &gyro.offset.data[2]);
        }
        runningTimes = 0;
    }
}

void ImuCalc::TDT_IMU_update(float half_T)
{
	gyroAccUpdate();
	uint32_t tmpTime = getSysTimeUs();
	ImuCalc::gyroAccUpdate();	//进行单位换算

	#if AUTO_CALIBRATION == 1
	gyroAutoCalibration();
	#endif
	float vx, vy, vz;//(r系到b系的第三列)

	float norm;
	float ex, ey, ez;

	float gx = gyro.radps.data[x];
	float gy = gyro.radps.data[y];
	float gz = gyro.radps.data[z];
	float ax = acc.accValue.data[x];
	float ay = acc.accValue.data[y];
	float az = acc.accValue.data[z];
	
	if(accUseFilter)
	{
		TDT_accFilter();
		ax = acc.accValue.data[x];
		ay = acc.accValue.data[y];
		az = acc.accValue.data[z];
	}
	if(gyroUseFilter)
	{
		TDT_gyroFilter();
		gx = gyro.radps.data[x];
		gy = gyro.radps.data[y];
		gz = gyro.radps.data[z];
	}
	
	
#if USE_AHRS_LIB

	float gyroLibIn[3] = {gx,gy,gz};
	float accLibIn[3] = {ax,ay,az};
	float angleLibOut[3] = {0};
	//航资参考系统数据更新，具体声明参考ahrs_lib.h
	AHRS_update(AHRS_data.ins_quat, 0.001f*2, gyroLibIn, accLibIn, AHRS_data.mag);
	//记录上一次的值
	memcpy(&LastAngle,&nowAngle,sizeof(Angle));
	//航资参考系统获取角度，具体声明参考ahrs_lib.h
    get_angle(AHRS_data.ins_quat, angleLibOut, angleLibOut+1, angleLibOut+2);
	//单位换算
	#if COMPARE_SELF_LIB
	AHRS_data.nowAngle.roll = angleLibOut[2] * RAD_TO_ANGLE;
	AHRS_data.nowAngle.pitch = angleLibOut[1] * RAD_TO_ANGLE;
	AHRS_data.nowAngle.yaw = angleLibOut[0] * RAD_TO_ANGLE;
	#else
	nowAngle.roll = angleLibOut[2] * RAD_TO_ANGLE;
	nowAngle.pitch = angleLibOut[1] * RAD_TO_ANGLE;
	nowAngle.yaw = angleLibOut[0] * RAD_TO_ANGLE;
	//过圈处理
	ImuCrossRoundHandle();
	#endif
#endif
	//使用库并且比较
#if !USE_AHRS_LIB || (USE_AHRS_LIB && COMPARE_SELF_LIB)
	//acc数据归一化
	norm = my_sqrt(ax*ax + ay*ay + az*az);       

	if(norm)
	{
		//acc数据归一化
		norm = my_sqrt(ax*ax + ay*ay + az*az);       
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;

		// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
		vx = 2 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2); //四元素中xyz的表示
		vy = 2*(quaternion.q0*quaternion.q1 + quaternion.q2*quaternion.q3);
		vz = 1 - 2*(quaternion.q1*quaternion.q1 + quaternion.q2*quaternion.q2);

		// error is sum of cross product between reference direction of fields and direction measured by sensors
		ex = (ay*vz - az*vy) ;                         					 //向量外积在相减得到差分就是误差
		ey = (az*vx - ax*vz) ;
		ez = (ax*vy - ay*vx) ;

		err.exInt = err.exInt + ex *Ki *2 *0.001f;								  //对误差进行积分
		err.eyInt = err.eyInt + ey *Ki *2 *0.001f;
		err.ezInt = err.ezInt + ez *Ki *2 *0.001f;

		// 积分限幅
		err.exInt = LIMIT(err.exInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
		err.eyInt = LIMIT(err.eyInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
		err.ezInt = LIMIT(err.ezInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );

		// adjusted gyroscope measurements
		gx = gx + Kp *(ex + err.exInt);					   						
		gy = gy + Kp *(ey + err.eyInt);				   							
		gz = gz + Kp *(ez + err.ezInt);					   					  							

		// integrate quaternion rate and normalise						   //四元素的微分方程
		insQuat tmp_q = {0};
		tmp_q.q0 = quaternion.q0 + (-quaternion.q1*gx - quaternion.q2*gy - quaternion.q3*gz) *0.001f;
		tmp_q.q1 = quaternion.q1 + ( quaternion.q0*gx + quaternion.q2*gz - quaternion.q3*gy) *0.001f;
		tmp_q.q2 = quaternion.q2 + ( quaternion.q0*gy - quaternion.q1*gz + quaternion.q3*gx) *0.001f;
		tmp_q.q3 = quaternion.q3 + ( quaternion.q0*gz + quaternion.q1*gy - quaternion.q2*gx) *0.001f;
		
		quaternion.q0 = tmp_q.q0;
		quaternion.q1 = tmp_q.q1;
		quaternion.q2 = tmp_q.q2;
        quaternion.q3 = tmp_q.q3;

		// normalise quaternion
		norm = my_sqrt(quaternion.q0*quaternion.q0 + quaternion.q1*quaternion.q1 + quaternion.q2*quaternion.q2 + quaternion.q3*quaternion.q3);
		quaternion.q0 = quaternion.q0 / norm;
		quaternion.q1 = quaternion.q1 / norm;
		quaternion.q2 = quaternion.q2 / norm;
		quaternion.q3 = quaternion.q3 / norm;
		
		memcpy(&LastAngle,&nowAngle,sizeof(Angle));

		nowAngle.yaw = fast_atan2(2*quaternion.q1*quaternion.q2+2*quaternion.q0*quaternion.q3, -2*quaternion.q2*quaternion.q2-2*quaternion.q3*quaternion.q3+1) *57.3f;
		nowAngle.roll = fast_atan2(2*quaternion.q2*quaternion.q3 + 2*quaternion.q0*quaternion.q1, -2*quaternion.q1*quaternion.q1 - 2*quaternion.q2*quaternion.q2 + 1) *57.3f;
		nowAngle.pitch = asin(-2*quaternion.q1*quaternion.q3 + 2*quaternion.q0*quaternion.q2) *57.3f; 

		ImuCrossRoundHandle();
	}
#endif
	
#if USE_AHRS_LIB && COMPARE_SELF_LIB
	angleFromLib = AHRS_data.nowAngle;
	angleFromSelf = nowAngle;
#endif

	imuTimeMatch.top()<<(vec2f({Angle.PITCH_AXIS,Angle.YAW_AXIS}));
}

void ImuCalc::ImuCrossRoundHandle()
{
	//过圈
	if(nowAngle.yaw-LastAngle.yaw > 180)
	{
		round.roundYaw--;
	}
	else if(nowAngle.yaw-LastAngle.yaw < -180)
	{
		round.roundYaw++;
	}
	
	if(nowAngle.roll-LastAngle.roll > 180)
	{
		round.roundRoll--;
	}
	else if(nowAngle.roll-LastAngle.roll < -180)
	{
		round.roundRoll++;
	}
	
	if(nowAngle.pitch-LastAngle.pitch> 180)
	{
		round.roundPitch--;
	}
	else if(nowAngle.pitch-LastAngle.pitch< -180)
	{
		round.roundPitch++;
	}
	Angle.yaw = round.roundYaw*360+nowAngle.yaw;
	Angle.pitch = round.roundPitch*360+nowAngle.pitch;
	Angle.roll =round.roundRoll*360+nowAngle.roll;
//	Angle.readTime = gyro.readTime;
}

void ImuCalc::gyroAccUpdate()
{
	//减去校准值
	acc.calibration.data[x] = acc.origin.data[x] - acc.offset.data[x];
	acc.calibration.data[y] = acc.origin.data[y] - acc.offset.data[y];
	acc.calibration.data[z] = acc.origin.data[z] - acc.offset.data[z];
	
	acc.accValue.data[x] = acc.calibration.data[x] * accValueFector;
	acc.accValue.data[y] = acc.calibration.data[y] * accValueFector;
	acc.accValue.data[z] = acc.calibration.data[z] * accValueFector;
	
	
	//减去校准值
	gyro.calibration.data[x] = gyro.origin.data[x] - gyro.offset.data[x];
	gyro.calibration.data[y] = gyro.origin.data[y] - gyro.offset.data[y];
	gyro.calibration.data[z] = gyro.origin.data[z] - gyro.offset.data[z];

	gyro.dps.data[x] = gyro.calibration.data[x] * gyroDpsFector;
	gyro.dps.data[y] = gyro.calibration.data[y] * gyroDpsFector;
	gyro.dps.data[z] = gyro.calibration.data[z] * gyroDpsFector;

	gyro.radps.data[x] = gyro.dps.data[x] * ANGLE_TO_RAD;
	gyro.radps.data[y] = gyro.dps.data[y] * ANGLE_TO_RAD;
	gyro.radps.data[z] = gyro.dps.data[z] * ANGLE_TO_RAD;
}

void ImuCalc::delayMs(u32 ms)
{
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//FreeRTOS系统已经运行
    {
        vTaskDelay(pdMS_TO_TICKS(ms));	
    }
	else
	{
		::delayMs(ms);
	}
}