/*
 * Mahony.h
 *
 *  Created on: 2024��4��17��
 *      Author: �ƴ���
 */

#ifndef CODE_MAHONY_H_
#define CODE_MAHONY_H_

#include "zf_common_headfile.h"

#define Gyro_Gr 0.01745           //�Ƕ���ת��Ϊ������ (180/3.1415)

#define   KalmanFliter_count    100000
#define   IMU963RA_KALMANFILTER_COEFFICIENT     1.005f

#define Kp 3.0f//4.5f
#define Ki 0.02f//0.03f
#define halfT 0.005f                 //�������ڵ�һ�룬��λs

extern float Z_COMP_Slope;
static float invSqrt(float x);       //���ټ��� 1/Sqrt(x)
void Prepare_Data(float data[6]);
void Imu_Update(float gx,float gy,float gz,float ax,float ay,float az);
void Angle_slope(double time, float angle);

void Imu_Update963(float ax,float ay,float az,float gx,float gy,float gz);
void Prepare_Data963(float data[6]);

void drift_get(void);

#endif /* CODE_MAHONY_H_ */
