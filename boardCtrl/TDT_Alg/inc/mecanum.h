/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of�
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.� See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <board.h>
#ifndef __MECANUM_H__
#define __MECANUM_H__
#include "filter.h"
#ifdef MECANUM_H_GLOBAL
#define MECANUM_H_EXTERN
#else
#define MECANUM_H_EXTERN extern
#endif

/************************ chassis parameter ****************************/
/* the radius of wheel(mm) */
#define RADIUS 76
/* the perimeter of wheel(mm) */
#define PERIMETER 478
/* wheel track distance(mm) */
#define WHEELTRACK 350
/* wheelbase distance(mm) */
#define WHEELBASE 380
#define WHEELNUMBER 4
/* gimbal is relative to chassis center x axis offset(mm) */
#define ROTATE_X_OFFSET 0
/* gimbal is relative to chassis center y axis offset(mm) */
#define ROTATE_Y_OFFSET 0

/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define MOTOR_DECELE_RATIO_3508 (1.0f / 19.0f)
#define MOTOR_DECELE_RATIO_2006 (1.0f / 36.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM 18000//4000mm/s //8500 //8347rpm = 3500mm/s
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED 4000 //8000rpm
#define MAX_CHASSIS_VY_SPEED 4000
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VW_SPEED 800 //5000rpm

#define MOTOR_ENCODER_ACCURACY 8192.0f
#define WHEELNUMBER 4


class Mecanum
{
private:
	struct mecanum_structure
	{
		float wheel_perimeter; /* the perimeter(mm) of wheel */
		float wheeltrack;	   /* wheel track distance(mm) */
		float wheelbase;	   /* wheelbase distance(mm) */
		float rotate_x_offset; /* rotate offset(mm) relative to the x-axis of the chassis center */
		float rotate_y_offset; /* rotate offset(mm) relative to the y-axis of the chassis center */
	};

	struct mecanum_position
	{
		float v_x_mm;
		float v_y_mm;
		float rate_deg;
		float position_x_mm;
		float position_y_mm;
		float angle_deg;
	};

public:
	float wheelSetError[4];
	float norm;
	float wheelSpeedKp[4];
	Lpf2p lpf2p_fdpk;
	float K_CU; //可信度（原拟为正态分布，由于采样问题搁置）（现用余弦函数代替）
	struct mecanum_speed
	{
		float vx; // +forward	-back
		float vy; // +left	-right
		float vw; // +anticlockwise	-clockwise
	};

	struct mecanum_gyro
	{
		float yaw_gyro_angle;
		float yaw_gyro_rate;
	};

	struct mecanum
	{
		struct mecanum_structure param;
		struct mecanum_speed speed;
		struct mecanum_position position;
		struct mecanum_gyro gyro;
		float wheel_rpm[4];
	} mec;

	struct mecanum_motor_fdb
	{
		float total_ecd;
		float speed_rpm;
	};
	struct mecanum_motor_fdb mec_fdb[4];

	/**
		  * @brief Mecanum 构造器
		  * @param 给轮组赋值默认参数，mecanum.h修改宏定义
		  * @note
		  */
	Mecanum(); //构造器

	/**
		  * @brief mecanum 解算.F:forword; B:backword; L:left; R:right
		  * @param input : ccx=+vx(mm/s)  ccy=+vy(mm/s)  ccw=+vw(deg/s) x方向速度 y方向速度 旋转速度
		  *        output: every wheel speed(rpm) 各麦轮轮速
		  * @note  1=FR 2=FL 3=BL 4=BR  轮组命名为逆时针顺序 轮组安装为O型安装
		  */
	void mecanum_calculate(vec3f speedDef, struct mecanum_motor_fdb wheel_fdb[]);

	/**
		  * @brief get mecanum chassis odometry, using global yaw gyro angle.(gyro 需要在世界绝对坐标系下底盘偏航角)
		  * @param
		  * @note
		  */
	void mecanum_position_measure(struct mecanum_motor_fdb wheel_fdb[]); //根据主动轮编码器计算里程，未测试！

	/**
		  * @brief 获取单个麦轮对应的速度.F:forword; B:backword; L:left; R:right
		  * @param input : wheelIndex 麦轮编号
		  *        output: 单麦轮轮速
		  * @note  1=FR 2=FL 3=BL 4=BR  轮组命名为逆时针顺序 轮组安装为O型安装
		  */
	float get_single_mecanum_wheel(uint8_t wheelIndex)
	{
		if (wheelIndex < 0 || wheelIndex > 4)
		{
			return 0;
		}
		return mec.wheel_rpm[wheelIndex];
	}
};

#endif // __MECANUM_H__
