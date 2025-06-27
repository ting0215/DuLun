/*
 * Math_Function.h
 *
 *  Created on: 2024年4月17日
 *      Author: 黄春华
 */

#ifndef CODE_MATH_FUNCTION_H_
#define CODE_MATH_FUNCTION_H_

#include "zf_common_headfile.h"

typedef struct
{
    float               X;                                                  // 状态估计值
    float               P;                                                  // 状态估计的不确定性
    float               F;                                                  // 状态转移模型系数
    float               Q;                                                  // 过程噪声协方差
    float               H;                                                  // 观测模型系数
    float               R;                                                  // 观测噪声协方差
}Once_Dimensional_Kalman_Parameter;

float Constrain_Data(float amt, float low, float high);
float Data_Mean_Filter(float new_data,float *data_record,int data_num);
float Calc_Slope(float *x, float* y,int st,int ed,int n,double Slope_max, double Slope_min);
float Low_Complementary_Filter(double new_data, double old_dat, double dt, double alpha);
double calculateStandardDeviation(int16 arr[], int ed, double mean);
float convertTo180Range(float gyroAngle);
double customMod(double dividend, double divisor);
float convertTo360(float angle) ;

//float KalmanFilter(KalmanFilter *kf, float angle);
//void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float initial_value);
//void KalmanFilter_Update(KalmanFilter *kf, float measurement);
void once_dimensional_kalman_init(Once_Dimensional_Kalman_Parameter* kf, float x, float p, float f, float q, float h, float r);
void once_dimensional_kalman_update(Once_Dimensional_Kalman_Parameter* kf, float Z);


#endif /* CODE_MATH_FUNCTION_H_ */
