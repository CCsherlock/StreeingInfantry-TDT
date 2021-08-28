#include "stdint.h"
#ifndef __TDT_INFANTRY__
#define __TDT_INFANTRY__

#define Run_Mode 1
#define Abandon !Run_Mode		/**不调用的部分不编译**/


//动作参数宏定义
#define	ON		1
#define	OFF		0




/**
* @struct  _vec2f
* @brief 二维float向量结构体
*/
typedef struct _vec2f
{
	float data[2];
}vec2f;
/**
* @struct  _vec2int16
* @brief 二维int16向量结构体
*/
typedef struct _vec2int16
{
	 short data[2];
}vec2int16;





/**
* @struct  _vec3f
* @brief 三维float向量结构体
*/
typedef struct _vec3f
{
	float data[3];
}vec3f;

/**
* @struct  _vec3int16
* @brief 三维int16向量结构体
*/
typedef struct _vec3int16
{
	 short data[3];
}vec3int16;



/**
* @struct  _vec4f
* @brief 四维float向量结构体
*/
typedef struct _vec4f
{
	float data[4];
}vec4f;
/**
* @struct  _vec4int16
* @brief 四维int16向量结构体
*/
typedef struct _vec4int16
{
	 short data[4];
}vec4int16;





/**
* @struct  _vec6f
* @brief 六维float向量结构体
*/
typedef struct _vec6f
{
	float data[6];
}vec6f;
/**
* @struct  _vec6int16
* @brief 六维int16向量结构体
*/
typedef struct _vec6int16
{
	 short data[6];
}vec6int16;




#endif
