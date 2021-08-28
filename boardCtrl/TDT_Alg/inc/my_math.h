/*****************************************************************************
File name: TDT_Alg\src\my_math.h
Description: 快速数学计算
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.16 肖银河-改写my_deathzoom函数-解决遥控器最大值会小于660问题
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#include "stm32f4xx.h"




#define ABS(x) ( (x)>0?(x):-(x) )
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define GETDIRECT(x) ( (x)>0?(1):-(1) )


#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256
#define MY_PPPIII       3.14159f
#define MY_PPPIII_HALF  1.570796f

float fast_atan2(float yy, float xx);
float my_pow(float a);
float my_sqrt(float number);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float xx,float zoom);
float To_180_degrees(float xx);
float Math_Max(float a,float b,float c,float d,float e,float f);
float rpmToDps(float rpm);
float dpsToRpm(float dps);
float rpmToRadps(float rpm);
float RadpsToRpm(float radps);
float Ramp_function_sin(double Input, double min, double max);
float Ramp_function_cos(double Input, double min, double max);

inline float Ramp_function(double Input,double point_down,double point_zero){return LIMIT((Input-point_zero)/(point_down-point_zero),0.0f,1.0f);}

#endif













