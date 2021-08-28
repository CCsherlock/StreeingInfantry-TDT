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
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "flash_var.h"

using namespace imu;

#define PITCH_AXIS pitch
#define YAW_AXIS yaw

//二阶互补滤波系数，规律：基本时间常数tau得到基本系数a，Kp=2*a，Ki=a^2;
#define Kp -0.6f // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki -0.1f // integral gain governs rate of convergence of gyroscope biases

float trustAngleKp[3] = {0, 0, 0.6};
float trustAngleKi[3] = {0, 0, 0.09};
float trustAngleIntegralMax[3] = {0, 0, 1};

float trustDpsKp[3] = {0};
float trustDpsKi[3] = {0};
float trustDpsIntegralMax[3] = {0};

ImuCalc::ImuCalc() : round({0}), quaternion({1, 0, 0, 0}), err({0}),
					 accValueFector(1), gyroDpsFector(1)
{
	memset(&Angle, 0, sizeof(Angle));
	memset(&LastAngle, 0, sizeof(LastAngle));
	memset(&nowAngle, 0, sizeof(nowAngle));
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
	if (acc.offset_max.data[x] - acc.offset_min.data[x] > 50 ||
		acc.offset_max.data[y] - acc.offset_min.data[y] > 50 ||
		acc.offset_max.data[z] - acc.offset_min.data[z] > 50)
		calOffset_Acc();
	else
	{
		accOffsetOk = 1;
		acc.offset.data[x] = (float)(tempgx) / 1000;
		acc.offset.data[y] = (float)(tempgy) / 1000;
		acc.offset.data[z] = (float)(tempgz) / 1000;
	}
}

//陀螺仪零偏矫正
void ImuCalc::getOffset(void)
{
	if (!forceGetOffset && IFlash.read() == 0) //成功读取并且不需要强行矫正
	{
		gyroAccUpdate();
		ImuCalc::gyroAccUpdate(); //进行单位换算
		eulerAngle initial;		  //初始欧拉角
		initial.yaw = 0;
		initial.pitch = atan2(-acc.accValue.data[0], acc.accValue.data[2]) * RAD_TO_ANGLE;
		initial.roll = atan2(acc.accValue.data[1], acc.accValue.data[2]) * RAD_TO_ANGLE;

		initalAngle(initial);

		return;
	}

	//读取失败或需要强行矫正时，只校准陀螺仪，不校准加速度，并存入flash
	while (!gyroAutoCalibration())
	{
		void iwdgFeed(void);
		iwdgFeed();
		gyroAccUpdate();
		delayMs(2);
	}

	IFlash.save();

	__set_FAULTMASK(1); //关闭所有中断
	NVIC_SystemReset(); //复位
	while (1)
	{
	} //仅等待复位
}

void ImuCalc::initalAngle(eulerAngle initial)
{
	//三角函数运算较为耗时，现行运算存入缓存
	float dataBuf[2][3] = {cosf(initial.yaw / (2 * RAD_TO_ANGLE)), cosf(initial.pitch / (2 * RAD_TO_ANGLE)), cosf(initial.roll / (2 * RAD_TO_ANGLE)),
						   sinf(initial.yaw / (2 * RAD_TO_ANGLE)), sinf(initial.pitch / (2 * RAD_TO_ANGLE)), sinf(initial.roll / (2 * RAD_TO_ANGLE))};

	//更新四元数
	quaternion.q0 = dataBuf[0][0] * dataBuf[0][1] * dataBuf[0][2] + dataBuf[1][0] * dataBuf[1][1] * dataBuf[1][2];
	quaternion.q1 = dataBuf[0][0] * dataBuf[0][1] * dataBuf[1][2] - dataBuf[1][0] * dataBuf[1][1] * dataBuf[0][2];
	quaternion.q2 = dataBuf[0][0] * dataBuf[1][1] * dataBuf[0][2] + dataBuf[1][0] * dataBuf[0][1] * dataBuf[1][2];
	quaternion.q3 = dataBuf[1][0] * dataBuf[0][1] * dataBuf[0][2] - dataBuf[0][0] * dataBuf[1][1] * dataBuf[1][2];

	//规范化最新四元数（归一化）
	float norm = sqrt(quaternion.q0 * quaternion.q0 + quaternion.q1 * quaternion.q1 +
					  quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3);
	quaternion.q0 = quaternion.q0 / norm;
	quaternion.q1 = quaternion.q1 / norm;
	quaternion.q2 = quaternion.q2 / norm;
	quaternion.q3 = quaternion.q3 / norm;

	//直接更新欧拉角
	Angle.yaw = atan2(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3, -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) * RAD_TO_ANGLE;
	Angle.roll = atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1, -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * RAD_TO_ANGLE;
	Angle.pitch = asin(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * RAD_TO_ANGLE;

	round.roundYaw = 0;

	nowAngle.pitch = Angle.pitch;
	nowAngle.yaw = Angle.yaw;
	nowAngle.roll = Angle.roll;
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

	acc.filter.data[x] = (float)(FILT_TMP[A_X]) / (float)FILTER_NUM;
	acc.filter.data[y] = (float)(FILT_TMP[A_Y]) / (float)FILTER_NUM;
	acc.filter.data[z] = (float)(FILT_TMP[A_Z]) / (float)FILTER_NUM;

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

	gyro.filter.data[x] = (float)(FILT_TMP[G_X]) / (float)FILTER_NUM;
	gyro.filter.data[y] = (float)(FILT_TMP[G_Y]) / (float)FILTER_NUM;
	gyro.filter.data[z] = (float)(FILT_TMP[G_Z]) / (float)FILTER_NUM;

	gyro.dps.data[x] = gyro.filter.data[x] * gyroDpsFector;
	gyro.dps.data[y] = gyro.filter.data[y] * gyroDpsFector;
	gyro.dps.data[z] = gyro.filter.data[z] * gyroDpsFector;

	gyro.radps.data[x] = gyro.dps.data[x] * ANGLE_TO_RAD;
	gyro.radps.data[y] = gyro.dps.data[y] * ANGLE_TO_RAD;
	gyro.radps.data[z] = gyro.dps.data[z] * ANGLE_TO_RAD;
}

bool ImuCalc::gyroAutoCalibration()
{
	static uint16_t runningTimes;
	if (runningTimes == 0)
	{
		gyro.dynamicSum.data[x] = 0;
		gyro.dynamicSum.data[y] = 0;
		gyro.dynamicSum.data[z] = 0;
		gyro.offset_max.data[x] = -32768;
		gyro.offset_max.data[y] = -32768;
		gyro.offset_max.data[z] = -32768;
		gyro.offset_min.data[x] = 32767;
		gyro.offset_min.data[y] = 32767;
		gyro.offset_min.data[z] = 32767;
	}

	if (gyro.origin.data[x] > gyro.offset_max.data[x])
		gyro.offset_max.data[x] = gyro.origin.data[x];
	if (gyro.origin.data[y] > gyro.offset_max.data[y])
		gyro.offset_max.data[y] = gyro.origin.data[y];
	if (gyro.origin.data[z] > gyro.offset_max.data[z])
		gyro.offset_max.data[z] = gyro.origin.data[z];

	if (gyro.origin.data[x] < gyro.offset_min.data[x])
		gyro.offset_min.data[x] = gyro.origin.data[x];
	if (gyro.origin.data[y] < gyro.offset_min.data[y])
		gyro.offset_min.data[y] = gyro.origin.data[y];
	if (gyro.origin.data[z] < gyro.offset_min.data[z])
		gyro.offset_min.data[z] = gyro.origin.data[z];

	gyro.dynamicSum.data[x] += gyro.origin.data[x];
	gyro.dynamicSum.data[y] += gyro.origin.data[y];
	gyro.dynamicSum.data[z] += gyro.origin.data[z];

	runningTimes++;

	if (gyro.offset_max.data[x] - gyro.offset_min.data[x] > 30 ||
		gyro.offset_max.data[y] - gyro.offset_min.data[y] > 30 ||
		gyro.offset_max.data[z] - gyro.offset_min.data[z] > 30)
	{
		runningTimes = 0;
	}

	if (runningTimes >= GYRO_Auto_Calibration_Times)
	{
		gyro.offset.data[x] = (float)(gyro.dynamicSum.data[x]) / runningTimes;
		gyro.offset.data[y] = (float)(gyro.dynamicSum.data[y]) / runningTimes;
		gyro.offset.data[z] = (float)(gyro.dynamicSum.data[z]) / runningTimes;
		runningTimes = 0;
		return true;
	}
	return false;
}

uint64_t ImuCalc::TDT_IMU_update(float half_T, bool useComplementaryFilter)
{
	gyroAccUpdate();
	uint64_t readImuTime = getSysTimeUs();

	float norm;
	float ex, ey, ez;

	if (accUseFilter)
	{
		TDT_accFilter();
	}
	if (gyroUseFilter)
	{
		TDT_gyroFilter();
	}

	ImuCalc::gyroAccUpdate(); //进行单位换算

	float gx = gyro.radps.data[x];
	float gy = gyro.radps.data[y];
	float gz = gyro.radps.data[z];
	float ax = acc.accValue.data[x];
	float ay = acc.accValue.data[y];
	float az = acc.accValue.data[z];

	AngleNoZero.pitch += gy * 2 * half_T * 57.295780f;
	AngleNoZero.roll += gx * 2 * half_T * 57.295780f;
	AngleNoZero.yaw += gz * 2 * half_T * 57.295780f;

	AngleFuseWithTrustAngle.pitch += gy * 2 * half_T * 57.295780f;
	AngleFuseWithTrustAngle.roll += gx * 2 * half_T * 57.295780f;
	AngleFuseWithTrustAngle.yaw += gz * 2 * half_T * 57.295780f;

	for (int i = 0; i < 3; i++)
	{
		if (!trustAnglePtr[i])
			continue;

		trustAngleErr[i] = *trustAnglePtr[i] - ((vec3f *)&AngleFuseWithTrustAngle)->data[i];
		trustAngleInt[i] += trustAngleErr[i] * 2 * half_T;
		trustAngleInt[i] = LIMIT(trustAngleInt[i], -trustAngleIntegralMax[i], trustAngleIntegralMax[i]);
		((vec3f *)&AngleFuseWithTrustAngle)->data[i] += trustAngleErr[i] * trustAngleKp[i] + trustAngleInt[i] * trustAngleKi[i];
	}

	//acc数据归一化
	norm = my_sqrt(ax * ax + ay * ay + az * az);

	if (norm)
	{
		//acc数据归一化
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;

		if (useComplementaryFilter)
		{
			float vx, vy, vz; //(r系到b系的第三列)
			// estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
			vx = 2 * (quaternion.q1 * quaternion.q3 - quaternion.q0 * quaternion.q2); //四元素中xyz的表示
			vy = 2 * (quaternion.q0 * quaternion.q1 + quaternion.q2 * quaternion.q3);
			vz = 1 - 2 * (quaternion.q1 * quaternion.q1 + quaternion.q2 * quaternion.q2);

			// error is sum of cross product between reference direction of fields and direction measured by sensors
			ex = (ay * vz - az * vy); //向量外积在相减得到差分就是误差
			ey = (az * vx - ax * vz);
			ez = (ax * vy - ay * vx);

			err.exInt += ex * Ki * 2 * half_T; //对误差进行积分
			err.eyInt += ey * Ki * 2 * half_T;
			err.ezInt += ez * Ki * 2 * half_T;

			// 积分限幅
			err.exInt = LIMIT(err.exInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
			err.eyInt = LIMIT(err.eyInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);
			err.ezInt = LIMIT(err.ezInt, -IMU_INTEGRAL_LIM, IMU_INTEGRAL_LIM);

			// adjusted gyroscope measurements
			gx = gx + Kp * (ex + err.exInt);
			gy = gy + Kp * (ey + err.eyInt);
			gz = gz + Kp * (ez + err.ezInt);
		}

		// integrate quaternion rate and normalise						   //四元素的微分方程
		insQuat tmp_q = {0};
		tmp_q.q0 = quaternion.q0 + (-quaternion.q1 * gx - quaternion.q2 * gy - quaternion.q3 * gz) * half_T;
		tmp_q.q1 = quaternion.q1 + (quaternion.q0 * gx + quaternion.q2 * gz - quaternion.q3 * gy) * half_T;
		tmp_q.q2 = quaternion.q2 + (quaternion.q0 * gy - quaternion.q1 * gz + quaternion.q3 * gx) * half_T;
		tmp_q.q3 = quaternion.q3 + (quaternion.q0 * gz + quaternion.q1 * gy - quaternion.q2 * gx) * half_T;

		quaternion.q0 = tmp_q.q0;
		quaternion.q1 = tmp_q.q1;
		quaternion.q2 = tmp_q.q2;
		quaternion.q3 = tmp_q.q3;

		// normalise quaternion
		norm = my_sqrt(quaternion.q0 * quaternion.q0 + quaternion.q1 * quaternion.q1 + quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3);
		quaternion.q0 = quaternion.q0 / norm;
		quaternion.q1 = quaternion.q1 / norm;
		quaternion.q2 = quaternion.q2 / norm;
		quaternion.q3 = quaternion.q3 / norm;

		memcpy(&LastAngle, &nowAngle, sizeof(Angle));

		nowAngle.yaw = fast_atan2(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3, -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) * 57.295780f;
		nowAngle.roll = fast_atan2(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1, -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * 57.295780f;
		nowAngle.pitch = asin(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * 57.295780f;

		ImuCrossRoundHandle(LastAngle, nowAngle, Angle);
	}

	return readImuTime;
}

void ImuCalc::ImuCrossRoundHandle(eulerAngle &LastAngle, eulerAngle &nowAngle, eulerAngle &Angle)
{
	//过圈
	if (nowAngle.yaw - LastAngle.yaw > 180)
	{
		round.roundYaw--;
	}
	else if (nowAngle.yaw - LastAngle.yaw < -180)
	{
		round.roundYaw++;
	}

	Angle.yaw = round.roundYaw * 360 + nowAngle.yaw;
	Angle.pitch = nowAngle.roll;
	Angle.roll = nowAngle.pitch;
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
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) //FreeRTOS系统已经运行
	{
		vTaskDelay(pdMS_TO_TICKS(ms));
	}
	else
	{
		::delayMs(ms);
	}
}
void ImuCalc::getAccOffset()
{
	if (accOffsetParam.completeFlag)
	{
		return;
	}
	gyroAccUpdate();
	for (u8 i = 0; i < 3; i++)
	{
		accOffsetParam.accNow.data[i] = acc.origin.data[i];
		accOffsetParam.accVar.data[i] = sqrt(my_pow((accOffsetParam.accNow.data[i] - accOffsetParam.accLast.data[i]) / 0.002f));
	}
	if (accOffsetParam.accVar.data[0] < accOffsetParam.accVarThreshold && accOffsetParam.accVar.data[1] < accOffsetParam.accVarThreshold && accOffsetParam.accVar.data[2] < accOffsetParam.accVarThreshold)
	{
		accOffsetParam.ifmoving = 0;
	}
	else
	{
		accOffsetParam.ifmoving = 1;
	}
	if (!accOffsetParam.ifmoving)
	{
		if (accOffsetParam.calibarationStep < 6)
		{
			u8 step = accOffsetParam.calibarationStep;
			if (accOffsetParam.count < 1000)
			{
				accOffsetParam.offsetValueSum[step] += acc.origin.data[(int)(step / 2)];
				accOffsetParam.count++;
			}
			else
			{
				accOffsetParam.count = 0;
				accOffsetParam.calibarationStep++;
			}
		}
	}
	else
	{
		accOffsetParam.offsetValueSum[accOffsetParam.calibarationStep] = 0;
		accOffsetParam.count = 0;
	}
	if (accOffsetParam.calibarationStep == 6)
	{
		accOffsetParam.offsetValue.data[0] = (accOffsetParam.offsetValueSum[0] + accOffsetParam.offsetValueSum[1]) / 2000.0f;
		accOffsetParam.offsetValue.data[1] = (accOffsetParam.offsetValueSum[2] + accOffsetParam.offsetValueSum[3]) / 2000.0f;
		accOffsetParam.offsetValue.data[2] = (accOffsetParam.offsetValueSum[4] + accOffsetParam.offsetValueSum[5]) / 2000.0f;
		accOffsetParam.completeFlag = 1;
		IFlash.link(accOffsetParam.offsetValue, 2);
		IFlash.save();
	}
	memcpy(accOffsetParam.accLast.data, accOffsetParam.accNow.data, sizeof(accOffsetParam.accNow.data));
}