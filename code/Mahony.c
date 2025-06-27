/*
 * Mahony.c
 *
 *  Created on: 2024��4��17��
 *      Author: �ƴ���
 */

#include <Mahony.h>

float Z_COMP_Slope = 0;
float out_z;
extern Once_Dimensional_Kalman_Parameter imu660ra_KalmanFliter_gyro_Z;
extern float Bias;
//KalmanFilter kf;
/*************************************************************************
*  �������ƣ�static float invSqrt(float x)
*  ����˵�������ټ��� 1/Sqrt(x)
*  ����˵����
  * @param    x   �� ��Ҫ���п�����ֵ
*  �������أ�float��y
*  �޸�ʱ�䣺2023��1��11��
*  �޸����ߣ�����
*  ��    ע��float a;  invSqrt(float a);
*************************************************************************/
static float invSqrt(float x)       //���ټ��� 1/Sqrt(x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*************************************************************************
*  �������ƣ�void Prepare_Data(float data[6])
*  ����˵������ȡimu660ra��ԭʼ���ݲ�ת����ʵ�����ݼ��ٶȺͽ��ٶ�
*  ����˵����
  * @param    data[6]   �� data[0]~data[6] �ֱ�����ʵ�ʵ�ax��ay��az��gx��gy��gz
*  �������أ���
*  �޸�ʱ�䣺2023��1��11��
*  �޸����ߣ�����
*  ��    ע��float data[6];  Prepare_Data(float data[6]);
*************************************************************************/
void Prepare_Data(float data[6])
{
    imu660ra_get_acc();               //��ȡ���ٶȼ�ԭʼֵ
    imu660ra_get_gyro();              //��ȡ���ٶ�ԭʼֵ

    //�����ٶ�ԭʼADֵת��Ϊm/s^2
    data[0] = imu660ra_acc_transition(imu660ra_acc_x);  //��λΪ g(m/s^2)
    data[1] = imu660ra_acc_transition(imu660ra_acc_y);  //��λΪ g(m/s^2)
    data[2] = imu660ra_acc_transition(imu660ra_acc_z);  //��λΪ g(m/s^2)

    //��������ADֵת��Ϊ ����/s
    data[3] = imu660ra_gyro_transition(imu660ra_gyro_x)*Gyro_Gr;   // ��λΪ ����/s
    data[4] = imu660ra_gyro_transition(imu660ra_gyro_y)*Gyro_Gr;   // ��λΪ ����/s
    data[5] = imu660ra_gyro_transition(imu660ra_gyro_z)*Gyro_Gr;   // ��λΪ ����/s
}
//void Prepare_Data963(float data[6])
//{
//    imu963ra_get_acc();               //��ȡ���ٶȼ�ԭʼֵ
//    imu963ra_get_gyro();              //��ȡ���ٶ�ԭʼֵ
//
//    //�����ٶ�ԭʼADֵת��Ϊm/s^2
//    data[0] = imu963ra_acc_transition(imu963ra_acc_x);  //��λΪ g(m/s^2)
//    data[1] = imu963ra_acc_transition(imu963ra_acc_y);  //��λΪ g(m/s^2)
//    data[2] = imu963ra_acc_transition(imu963ra_acc_z);  //��λΪ g(m/s^2)
//
//    //��������ADֵת��Ϊ ����/s
//    data[3] = imu963ra_gyro_transition(imu963ra_gyro_x)*Gyro_Gr;   // ��λΪ ����/s
//    data[4] = imu963ra_gyro_transition(imu963ra_gyro_y)*Gyro_Gr;   // ��λΪ ����/s
//    data[5] = imu963ra_gyro_transition(imu963ra_gyro_z)*Gyro_Gr;   // ��λΪ ����/s
//}
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //��Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;      //������������ۼƻ���
/*************************************************************************
*  �������ƣ�void Imu_Update(float ax,float ay,float az,float gx,float gy,float gz)
*  ����˵��:��Ԫ������+Mohoney�����˲�,�����Pitch��Yaw��Roll�Ƕ�
*  ����˵����
  * @param    ax��ay��az   ʵ�ʵļ��ٶ�ֵ
  * @param    gx��gy��gz   ʵ�ʵĽ��ٶ�ֵ
*  �������أ���
*  �޸�ʱ�䣺2023��1��11��
*  �޸����ߣ�����
*  ��    ע��float data[6];  Imu_Update(Actual_data[0],Actual_data[1],Actual_data[2],Actual_data[3],Actual_data[4],Actual_data[5]);
*************************************************************************/
void Imu_Update(float ax,float ay,float az,float gx,float gy,float gz)
{
    float vx,vy,vz;                         //ʵ���������ٶ�
    float ex,ey,ez;                         //�����������
    float norm;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(ax*ay*az == 0)
        return;

    //���ٶȼƲ�������������(��������ϵ),���ٶ����ݵĹ�һ��
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //��Ԫ���Ƴ���ʵ����������(��������ϵ),DCM������ת
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //������,�ڻ�������ϵ������������õ���������
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    //���������Ϊ���ٶ�,��������PI���㣬�������ٶ�
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //���ٶȲ���
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    //������Ԫ��
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    //��λ����Ԫ��
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    //��Ԫ������ŷ����
    Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f;
    Pitch = -asin(2.f * (q1q3 - q0q2))* 57.3f;
    Roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;

    once_dimensional_kalman_update(&imu660ra_KalmanFliter_gyro_Z, imu660ra_gyro_z);
    out_z = imu660ra_KalmanFliter_gyro_Z.X;

    Yaw_B1 += imu660ra_gyro_transition(out_z) * 0.01 * IMU963RA_KALMANFILTER_COEFFICIENT - Bias;

    //Yaw_B1 += (imu660ra_gyro_transition(imu660ra_gyro_z) * 0.01 - Bias);

    //Yaw_B2 = KalmanFilter(kf, Yaw_B1);
    //Yaw_B1 -= 0.0005;      //0.0008
    //Yaw_B = Data_Mean_Filter(Yaw, Yaw_Data_Mean_Filter, 10);
//    if(Start_Car_Para == 2) Yaw = Yaw_B;                                // ����ģʽ
//    else
//    {
//        //Yaw = Yaw_B - Z_COMP_Slope * Running_Time_Yaw;                  // ��������ģʽ
//        //Yaw_Tur = convertTo360(Yaw);
//
//    }

    Yaw_Tur = convertTo360(Yaw_B1);

}

void drift_get(void)
{
    int i;
    double KalmanFliter_sum;
    for(i=0;i<1000;i++)
    {
        imu660ra_get_gyro();
        system_delay_ms(10);
        KalmanFliter_sum += imu660ra_gyro_z;
    }
    Bias = KalmanFliter_sum / 1000;
}

