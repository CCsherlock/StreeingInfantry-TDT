/******************************
File name: TDT_Alg\src\filter.cpp
Description: 滤波器算法，集成卡尔曼，低通
function:
	——————————————————————————————————————————————————————————————————————————
	void Lpf2p::SetCutoffFreq(float sample_freq, float cutoff_freq)
	——————————————————————————————————————————————————————————————————————————
	float Lpf2p::Apply(float sample)
	——————————————————————————————————————————————————————————————————————————
	double Kf::KalmanFilter(const double ResrcData,double ProcessNoise_Q,double MeasureNoise_R,u8 kind)
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.19 肖银河-将滤波函数改写为类
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "filter.h"
#include "math.h"


#define M_PI_F 3.1415926f

#define KALMAN_Q		0.03
#define KALMAN_R        10.0000


/**
  * @brief 设置低通滤波截止频率
  */
void Lpf2p::SetCutoffFreq(float sample_freq, float cutoff_freq)
{
	float fr =0;  
	float ohm =0;
	float c =0;

	fr= sample_freq/cutoff_freq;
	ohm=tanf(M_PI_F/fr);
	c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

	_cutoff_freq1 = cutoff_freq;
	
	if (_cutoff_freq1 > 0.0f) 
	{
		_b01 = ohm*ohm/c;
		_b11 = 2.0f*_b01;
		_b21 = _b01;
		_a11 = 2.0f*(ohm*ohm-1.0f)/c;
		_a21 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
	}
}

/**
  * @brief 获取输出
  */
float Lpf2p::Apply(float sample)
{
	float delay_element_0 = 0, output=0;
	if (_cutoff_freq1 <= 0.0f) 
	{
		// no filtering
		return sample;
	}
	else
	{
		delay_element_0 = sample - _delay_element_11 * _a11 - _delay_element_21 * _a21;
		// do the filtering
		if (isnan(delay_element_0) || isinf(delay_element_0)) {
				// don't allow bad values to propogate via the filter
				delay_element_0 = sample;
		}
		output = delay_element_0 * _b01 + _delay_element_11 * _b11 + _delay_element_21 * _b21;

		_delay_element_21 = _delay_element_11;
		_delay_element_11 = delay_element_0;

		// return the value.  Should be no need to check limits
		return output;
	}
}


/**
  * @brief 卡尔曼滤波函数
  * @other 	Q：过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R：测量噪声，R增大，动态响应变慢，收敛稳定性变好
  */
double Kf::KalmanFilter(const double ResrcData,double ProcessNoise_Q,double MeasureNoise_R,u8 kind)
{
   double R = MeasureNoise_R;
   double Q = ProcessNoise_Q;
   double x_mid;
   double x_now;
   double p_mid ;
   double p_now;
   double kg;
   x_mid=x_last;
   p_mid=p_last+Q;
   kg=p_mid/(p_mid+R);
   x_now=x_mid+kg*(ResrcData-x_mid);

   p_now=(1-kg)*p_mid;
   p_last = p_now;
   x_last = x_now;
   return x_now;
}





