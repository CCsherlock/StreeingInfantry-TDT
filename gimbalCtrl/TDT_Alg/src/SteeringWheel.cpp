#include "SteeringWheel.h"
#include "my_math.h"
#include "math.h"
#define RADIAN_TO_DEGREE 57.3f
#define DEGREE_TO_RADIAN 0.017452f
SteeringWheel::SteeringWheel()
{
	steeringPara.wheelbase = WHEELBASE;	  //L
	steeringPara.wheeltrack = WHEELTRACK; //D
	steeringPara.rotate_x_offset = ROTATE_X_OFFSET;
	steeringPara.rotate_y_offset = ROTATE_Y_OFFSET;
	steeringPara.wheel_perimeter = PERIMETER;
	steeringPara.W_Cdistance = my_sqrt(my_pow(WHEELBASE) + my_pow(WHEELTRACK));
}
void SteeringWheel::steeringWheel_calculate(vec3f speedDef, vec4f wheelFeedback)
{
	steeringSpeed.vx = speedDef.data[0];
	steeringSpeed.vy = speedDef.data[1];
	steeringSpeed.vw = speedDef.data[2];
	wheels.RF.angleFdb = wheelFeedback.data[0];
	wheels.LF.angleFdb = wheelFeedback.data[1];
	wheels.LB.angleFdb = wheelFeedback.data[2];
	wheels.RB.angleFdb = wheelFeedback.data[3];
	if (steeringSpeed.vx == 0 && steeringSpeed.vy == 0 && steeringSpeed.vw == 0) //当三个速度向量均为0时则不进行运算
	{
		wheels.LF.speed_rpm = 0; //当不进行计算时，所有的轮速均为0 轮子角度保持上一帧的数据不变
		wheels.LB.speed_rpm = 0;
		wheels.RF.speed_rpm = 0;
		wheels.RB.speed_rpm = 0;
		wheels.LF.speed = 0;
		wheels.LB.speed = 0;
		wheels.RF.speed = 0;
		wheels.RB.speed = 0;
	}
	else
	{
		LIMIT(steeringSpeed.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED); //mm/s
		LIMIT(steeringSpeed.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED); //mm/s
		LIMIT(steeringSpeed.vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED); //deg/s
		//当存在机器人平移速度时
		if (steeringSpeed.vy != 0 || steeringSpeed.vx != 0)
		{
			basePara.alpha = atan2(steeringSpeed.vy, steeringSpeed.vx); //计算车体偏航角   取值范围（-pi -- pi）
		}

		basePara.sinAlpha = sinf(basePara.alpha); //（-1 --  1）
		basePara.cosAlpha = cosf(basePara.alpha); //（-1 --  1）
		basePara.speedV = my_sqrt(my_pow(steeringSpeed.vy) + my_pow(steeringSpeed.vx));

		//LF:  1
		wheels.LF.speed = my_sqrt(my_pow(steeringPara.W_Cdistance) * my_pow(steeringSpeed.vw) / 2 + my_pow(basePara.speedV) + (steeringPara.wheelbase * basePara.sinAlpha + steeringPara.wheeltrack * basePara.cosAlpha) * steeringSpeed.vw * basePara.speedV);
		wheels.LF.angle = (PI / 2) - atan2((2 * basePara.speedV * basePara.cosAlpha + steeringPara.wheeltrack * steeringSpeed.vw), (2 * basePara.speedV * basePara.sinAlpha + steeringPara.wheelbase * steeringSpeed.vw));
		if (wheels.LF.angle < 0)
		{
			wheels.LF.angle += 2 * PI;
		}
		wheels.LF.angle = wheels.LF.angle * RADIAN_TO_DEGREE; //(0 -- 360)

		//LB:   2
		wheels.LB.speed = my_sqrt((my_pow(steeringPara.W_Cdistance) * my_pow(steeringSpeed.vw)) / 2 + my_pow(basePara.speedV) + (-steeringPara.wheelbase * basePara.sinAlpha + steeringPara.wheeltrack * basePara.cosAlpha) * steeringSpeed.vw * basePara.speedV);
		wheels.LB.angle = (PI / 2) - atan2((2 * basePara.speedV * basePara.cosAlpha + steeringPara.wheeltrack * steeringSpeed.vw), -(-2 * basePara.speedV * basePara.sinAlpha + steeringPara.wheelbase * steeringSpeed.vw));
		if (wheels.LB.angle < 0)
		{
			wheels.LB.angle += 2 * PI;
		}
		wheels.LB.angle = wheels.LB.angle * RADIAN_TO_DEGREE; //(0 -- 360)

		//RF:   0
		wheels.RF.speed = my_sqrt((my_pow(steeringPara.W_Cdistance) * my_pow(steeringSpeed.vw)) / 2 + my_pow(basePara.speedV) + (steeringPara.wheelbase * basePara.sinAlpha - steeringPara.wheeltrack * basePara.cosAlpha) * steeringSpeed.vw * basePara.speedV);
		wheels.RF.angle = (PI / 2) - atan2((2 * basePara.speedV * basePara.cosAlpha - steeringPara.wheeltrack * steeringSpeed.vw), (2 * basePara.speedV * basePara.sinAlpha + steeringPara.wheelbase * steeringSpeed.vw));
		if (wheels.RF.angle < 0)
		{
			wheels.RF.angle += 2 * PI;
		}
		wheels.RF.angle = wheels.RF.angle * RADIAN_TO_DEGREE; //(0 -- 360)

		//RB:   3
		wheels.RB.speed = my_sqrt((my_pow(steeringPara.W_Cdistance) * my_pow(steeringSpeed.vw)) / 2 + my_pow(basePara.speedV) + (-steeringPara.wheelbase * basePara.sinAlpha - steeringPara.wheeltrack * basePara.cosAlpha) * steeringSpeed.vw * basePara.speedV);
		wheels.RB.angle = (PI / 2) - atan2((2 * basePara.speedV * basePara.cosAlpha - steeringPara.wheeltrack * steeringSpeed.vw), -(-2 * basePara.speedV * basePara.sinAlpha + steeringPara.wheelbase * steeringSpeed.vw));
		if (wheels.RB.angle < 0)
		{
			wheels.RB.angle += 2 * PI;
		}
		wheels.RB.angle = wheels.RB.angle * RADIAN_TO_DEGREE; //(0 -- 360)
	}
	/*设定值转化为当前角度圈数下的设定值*/ //-180---180°
	if (ABS(wheels.RF.angle - wheels.RF.angleFdb) > 180.f)
	{
		if (wheels.RF.angle > wheels.RF.angleFdb)
		{
			wheels.RF.angle -= 360.f;
		}
		else if (wheels.RF.angle < wheels.RF.angleFdb)
		{
			wheels.RF.angle += 360.f;
		}
	}
	/*设定值转化为当前角度圈数下的设定值*/ //-180---180°
	if (ABS(wheels.LF.angle - wheels.LF.angleFdb) > 180.f)
	{
		if (wheels.LF.angle > wheels.LF.angleFdb)
		{
			wheels.LF.angle -= 360.f;
		}
		else if (wheels.LF.angle < wheels.LF.angleFdb)
		{
			wheels.LF.angle += 360.f;
		}
	}
	/*设定值转化为当前角度圈数下的设定值*/ //-180---180°
	if (ABS(wheels.LB.angle - wheels.LB.angleFdb) > 180.f)
	{
		if (wheels.LB.angle > wheels.LB.angleFdb)
		{
			wheels.LB.angle -= 360.f;
		}
		else if (wheels.LB.angle < wheels.LB.angleFdb)
		{
			wheels.LB.angle += 360.f;
		}
	}
	/*设定值转化为当前角度圈数下的设定值*/ //-180---180°
	if (ABS(wheels.RB.angle - wheels.RB.angleFdb) > 180.f)
	{
		if (wheels.RB.angle > wheels.RB.angleFdb)
		{
			wheels.RB.angle -= 360.f;
		}
		else if (wheels.RB.angle < wheels.RB.angleFdb)
		{
			wheels.RB.angle += 360.f;
		}
	}
	/*输出轮速*/
	wheels.LF.speed_rpm = -wheels.LF.speed / steeringPara.wheel_perimeter * 60; //* MOTOR_DECELE_RATIO_3508; //mm/s --->n/min 转子转速
	wheels.LB.speed_rpm = -wheels.LB.speed / steeringPara.wheel_perimeter * 60 ;//* MOTOR_DECELE_RATIO_3508;
	wheels.RF.speed_rpm = wheels.RF.speed / steeringPara.wheel_perimeter * 60 ;//* MOTOR_DECELE_RATIO_3508;
	wheels.RB.speed_rpm = wheels.RB.speed / steeringPara.wheel_perimeter * 60 ;//* MOTOR_DECELE_RATIO_3508;
	/*输出结构体赋值*/
	outputData.angle.data[0] = wheels.RF.angle; //-180---180°
	outputData.angle.data[1] = wheels.LF.angle;
	outputData.angle.data[2] = wheels.LB.angle;
	outputData.angle.data[3] = wheels.RB.angle;
	outputData.speed_rpm.data[0] = wheels.RF.speed_rpm;
	outputData.speed_rpm.data[1] = wheels.LF.speed_rpm;
	outputData.speed_rpm.data[2] = wheels.LB.speed_rpm;
	outputData.speed_rpm.data[3] = wheels.RB.speed_rpm;
}