/*****************************************************************************
File name: TDT_Alg\src\filter.cpp
Description: 滤波器算法，集成卡尔曼，低通
function:
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.19 肖银河-将滤波函数改写为类
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _FILTER_H
#define _FILTER_H

//#include "board.h"
#include "stm32f4xx.h"


//低通
class Lpf2p{
private:
	 float           _cutoff_freq1; 
	 float           _a11;
	 float           _a21;
	 float           _b01;
	 float           _b11;
	 float           _b21;
	 float           _delay_element_11;        // buffered sample -1
	 float           _delay_element_21;        // buffered sample -2
public:
	void SetCutoffFreq(float sample_freq, float cutoff_freq);
	float Apply(float sample);
	
};


//卡尔曼
class Kf{

private:
	double x_last;
	double p_last;
public:
	double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R,u8 kind);

};


#endif
