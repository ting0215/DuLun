/*
 * Mahony.c
 *
 *  Created on: 2024年4月17日
 *      Author: 黄春华
 */

#include <Mahony.h>

float Z_COMP_Slope = 0;
float out_z;
extern Once_Dimensional_Kalman_Parameter imu660ra_KalmanFliter_gyro_Z;
extern float Bias;
//KalmanFilter kf;
/*************************************************************************
*  函数名称：static float invSqrt(float x)
*  功能说明：快速计算 1/Sqrt(x)
*  参数说明：
  * @param    x   ： 将要进行开方的值
*  函数返回：float型y
*  修改时间：2023年1月11日
*  修改作者：安健
*  备    注：float a;  invSqrt(float a);
*************************************************************************/
static float invSqrt(float x)       //快速计算 1/Sqrt(x)
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
*  函数名称：void Prepare_Data(float data[6])
*  功能说明：获取imu660ra的原始数据并转换成实际数据加速度和角速度
*  参数说明：
  * @param    data[6]   ： data[0]~data[6] 分别存放了实际的ax，ay，az，gx，gy，gz
*  函数返回：无
*  修改时间：2023年1月11日
*  修改作者：安健
*  备    注：float data[6];  Prepare_Data(float data[6]);
*************************************************************************/
void Prepare_Data(float data[6])
{
    imu660ra_get_acc();               //获取加速度计原始值
    imu660ra_get_gyro();              //获取角速度原始值

    //将加速度原始AD值转换为m/s^2
    data[0] = imu660ra_acc_transition(imu660ra_acc_x);  //单位为 g(m/s^2)
    data[1] = imu660ra_acc_transition(imu660ra_acc_y);  //单位为 g(m/s^2)
    data[2] = imu660ra_acc_transition(imu660ra_acc_z);  //单位为 g(m/s^2)

    //将陀螺仪AD值转换为 弧度/s
    data[3] = imu660ra_gyro_transition(imu660ra_gyro_x)*Gyro_Gr;   // 单位为 弧度/s
    data[4] = imu660ra_gyro_transition(imu660ra_gyro_y)*Gyro_Gr;   // 单位为 弧度/s
    data[5] = imu660ra_gyro_transition(imu660ra_gyro_z)*Gyro_Gr;   // 单位为 弧度/s
}
//void Prepare_Data963(float data[6])
//{
//    imu963ra_get_acc();               //获取加速度计原始值
//    imu963ra_get_gyro();              //获取角速度原始值
//
//    //将加速度原始AD值转换为m/s^2
//    data[0] = imu963ra_acc_transition(imu963ra_acc_x);  //单位为 g(m/s^2)
//    data[1] = imu963ra_acc_transition(imu963ra_acc_y);  //单位为 g(m/s^2)
//    data[2] = imu963ra_acc_transition(imu963ra_acc_z);  //单位为 g(m/s^2)
//
//    //将陀螺仪AD值转换为 弧度/s
//    data[3] = imu963ra_gyro_transition(imu963ra_gyro_x)*Gyro_Gr;   // 单位为 弧度/s
//    data[4] = imu963ra_gyro_transition(imu963ra_gyro_y)*Gyro_Gr;   // 单位为 弧度/s
//    data[5] = imu963ra_gyro_transition(imu963ra_gyro_z)*Gyro_Gr;   // 单位为 弧度/s
//}
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //四元数
float exInt = 0, eyInt = 0, ezInt = 0;      //叉积计算误差的累计积分
/*************************************************************************
*  函数名称：void Imu_Update(float ax,float ay,float az,float gx,float gy,float gz)
*  功能说明:四元数解算+Mohoney互补滤波,解算出Pitch，Yaw，Roll角度
*  参数说明：
  * @param    ax，ay，az   实际的加速度值
  * @param    gx，gy，gz   实际的角速度值
*  函数返回：无
*  修改时间：2023年1月11日
*  修改作者：安健
*  备    注：float data[6];  Imu_Update(Actual_data[0],Actual_data[1],Actual_data[2],Actual_data[3],Actual_data[4],Actual_data[5]);
*************************************************************************/
void Imu_Update(float ax,float ay,float az,float gx,float gy,float gz)
{
    float vx,vy,vz;                         //实际重力加速度
    float ex,ey,ez;                         //叉积计算的误差
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

    //加速度计测量的重力方向(机体坐标系),加速度数据的归一化
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //四元数推出的实际重力方向(机体坐标系),DCM矩阵旋转
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //叉积误差,在机体坐标系下做向量叉积得到补偿数据
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    //叉积误差积分为角速度,对误差进行PI计算，补偿角速度
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //角速度补偿
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    //更新四元数
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    //单位化四元数
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    //四元数反解欧拉角
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
//    if(Start_Car_Para == 2) Yaw = Yaw_B;                                // 矫正模式
//    else
//    {
//        //Yaw = Yaw_B - Z_COMP_Slope * Running_Time_Yaw;                  // 正常补偿模式
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

