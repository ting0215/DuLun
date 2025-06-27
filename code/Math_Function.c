/*
 * Math_Function.c
 *
 *  Created on: 2024年4月17日
 *      Author: 黄春华
 */
#include<Math_Function.h>
/**数据的均值滤波
 *  功能说明：进行数据的平均滤波
 *  参数说明：
 *  @param new_data：输入新采样到的数据
 *  @param *data_record 存放数据的数组
 *  @param data_num       存放数据的数量
 *  函数返回：@sum/data_num;    返回滤波后的数据
 *  修改时间：2023年5月5日 安健
 *  备    注：
 *************************************************************************/
float Data_Mean_Filter(float new_data,float *data_record,int data_num)
{
    float sum = 0.2f;
    float test_data;
    test_data = new_data;
    int i;
    for(i = data_num - 1; i > 0 ; i--)       //将现有数据后移一位
    {
        data_record[i] = data_record[i-1];
        sum += 1 / 10 * data_record[i-1];
    }
    data_record[0] = new_data;              //第一位是新的数据
    sum += new_data;
    test_data = sum/data_num;
    return sum/data_num;                    //返回均值
}


/**数据的低通互补滤波
 * @see:  Low_Complementary_Filter(double new_data, double old_dat, double dt, double alpha)
 * @param new_data 当前时刻的值
 * @param old_dat  上一时刻的值
 * @param dt 采样时间间隔
 * @param alpha 互补滤波系数
 * @return data 滤波后的数据
 * @author: 安健
 * @date:   2023.5.5
 */
float Low_Complementary_Filter(float new_data, float old_dat, float dt, float alpha)
{
    float data;
    data = alpha * old_dat + (1 - alpha) * new_data;
    return data;
}

// 将角度转换为-180到180的范围
float convertTo180Range(float gyroAngle)
{
    gyroAngle = fmodf(gyroAngle, 360.0f); // 使用模运算将角度限制在0到360度之间

    if (gyroAngle > 180.0f) {
        gyroAngle -= 360.0f; // 将大于180度的角度减去360度，使其范围在-180到180之间
    }

    return gyroAngle;
}

double customMod(double dividend, double divisor) {
    double quotient = floor(dividend / divisor);
    double remainder = dividend - (divisor * quotient);
    return remainder;
}

float convertTo360(float angle)
{
    angle = customMod(angle,360);  // 取模运算，限制角度在0-359之间
    if (angle < 0) {
        angle = angle + 360;  // 处理负角度，转换为等效的正角度
    }
    return angle;
}


//卡尔曼滤波


//#define dt 0.01  // Time step
//#define Q_angle 1  // Process noise
//#define R_angle 1  // Measurement noise
//
//float angle = 0;  // Initial angle estimate
//float bias = 0;   // Initial gyro bias estimate
//float P[2][2] = {{0, 0}, {0, 0}};  // Error covariance matrix
//
//float KalmanFilter(float newAngle, float newRate)
//{
//    // Predict
//    angle += (newRate - bias) * dt;
//    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
//    P[0][1] -= dt * P[1][1];
//    P[1][0] -= dt * P[1][1];
//    P[1][1] += Q_angle * dt;
//
//    // Update
//    float S = P[0][0] + R_angle;
//    float K[2];  // Kalman gain
//    K[0] = P[0][0] / S;
//    K[1] = P[1][0] / S;
//    float y = newAngle - angle;
//    angle += K[0] * y;
//    bias += K[1] * y;
//    float P00_temp = P[0][0];
//    float P01_temp = P[0][1];
//
//    P[0][0] -= K[0] * P00_temp;
//    P[0][1] -= K[0] * P01_temp;
//    P[1][0] -= K[1] * P00_temp;
//    P[1][1] -= K[1] * P01_temp;
//
//    return angle;
//}


//// 初始化卡尔曼滤波器
//void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float initial_value) {
//    kf->Q = Q;
//    kf->R = R;
//    kf->x = initial_value;
//    kf->P = 1.0;
//    kf->K = 0.0;
//}
//
//// 卡尔曼滤波器更新
//void KalmanFilter_Update(KalmanFilter *kf, float measurement) {
//    // 预测步骤
//    kf->P = kf->P + kf->Q;
//
//    // 更新步骤
//    kf->K = kf->P / (kf->P + kf->R);
//    kf->x = kf->x + kf->K * (measurement - kf->x);
//    kf->P = (1 - kf->K) * kf->P;
//}
//
//float KalmanFilter(KalmanFilter *kf, float angle)
//{
//
//    KalmanFilter_Init(&kf, 0.1, 0.1, 0.0); // 初始化过程噪声协方差Q，测量噪声协方差R，初始值
//
//    KalmanFilter_Update(&kf, angle);
//
//    return kf.x;
//}

/**封装函数**/
// 初始化一维卡尔曼滤波
Once_Dimensional_Kalman_Parameter imu660ra_KalmanFliter_gyro_Z;

void once_dimensional_kalman_init(Once_Dimensional_Kalman_Parameter* kf, float x, float p, float f, float q, float h, float r)
{
    kf->X = x;                                                                  // 初始状态估计值
    kf->P = p;                                                                  // 初始状态估计的不确定性
    kf->F = f;                                                                  // 状态转移模型系数
    kf->Q = q;                                                                  // 过程噪声协方差
    kf->H = h;                                                                  // 观测模型系数
    kf->R = r;                                                                  // 观测噪声协方差
}

// 一维卡尔曼滤波更新
void once_dimensional_kalman_update(Once_Dimensional_Kalman_Parameter* kf, float Z)
{
    // 预测步骤
    float X_predict = kf->F * kf->X;
    float P_predict = kf->F * kf->P * kf->F + kf->Q;

    // 更新步骤
    float Y = Z - kf->H * X_predict;                                            // 计算预测误差
    float S = kf->H * P_predict * kf->H + kf->R;                                // 计算预测误差协方差
    float K = P_predict * kf->H / S;                                            // 计算卡尔曼增益
    kf->X = X_predict + K * Y;                                                  // 更新状态估计值
    kf->P = (1 - K * kf->H) * P_predict;                                        // 更新状态估计的不确定性
}
