/*
 * Math_Function.c
 *
 *  Created on: 2024��4��17��
 *      Author: �ƴ���
 */
#include<Math_Function.h>
/**���ݵľ�ֵ�˲�
 *  ����˵�����������ݵ�ƽ���˲�
 *  ����˵����
 *  @param new_data�������²�����������
 *  @param *data_record ������ݵ�����
 *  @param data_num       ������ݵ�����
 *  �������أ�@sum/data_num;    �����˲��������
 *  �޸�ʱ�䣺2023��5��5�� ����
 *  ��    ע��
 *************************************************************************/
float Data_Mean_Filter(float new_data,float *data_record,int data_num)
{
    float sum = 0.2f;
    float test_data;
    test_data = new_data;
    int i;
    for(i = data_num - 1; i > 0 ; i--)       //���������ݺ���һλ
    {
        data_record[i] = data_record[i-1];
        sum += 1 / 10 * data_record[i-1];
    }
    data_record[0] = new_data;              //��һλ���µ�����
    sum += new_data;
    test_data = sum/data_num;
    return sum/data_num;                    //���ؾ�ֵ
}


/**���ݵĵ�ͨ�����˲�
 * @see:  Low_Complementary_Filter(double new_data, double old_dat, double dt, double alpha)
 * @param new_data ��ǰʱ�̵�ֵ
 * @param old_dat  ��һʱ�̵�ֵ
 * @param dt ����ʱ����
 * @param alpha �����˲�ϵ��
 * @return data �˲��������
 * @author: ����
 * @date:   2023.5.5
 */
float Low_Complementary_Filter(float new_data, float old_dat, float dt, float alpha)
{
    float data;
    data = alpha * old_dat + (1 - alpha) * new_data;
    return data;
}

// ���Ƕ�ת��Ϊ-180��180�ķ�Χ
float convertTo180Range(float gyroAngle)
{
    gyroAngle = fmodf(gyroAngle, 360.0f); // ʹ��ģ���㽫�Ƕ�������0��360��֮��

    if (gyroAngle > 180.0f) {
        gyroAngle -= 360.0f; // ������180�ȵĽǶȼ�ȥ360�ȣ�ʹ�䷶Χ��-180��180֮��
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
    angle = customMod(angle,360);  // ȡģ���㣬���ƽǶ���0-359֮��
    if (angle < 0) {
        angle = angle + 360;  // �����Ƕȣ�ת��Ϊ��Ч�����Ƕ�
    }
    return angle;
}


//�������˲�


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


//// ��ʼ���������˲���
//void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float initial_value) {
//    kf->Q = Q;
//    kf->R = R;
//    kf->x = initial_value;
//    kf->P = 1.0;
//    kf->K = 0.0;
//}
//
//// �������˲�������
//void KalmanFilter_Update(KalmanFilter *kf, float measurement) {
//    // Ԥ�ⲽ��
//    kf->P = kf->P + kf->Q;
//
//    // ���²���
//    kf->K = kf->P / (kf->P + kf->R);
//    kf->x = kf->x + kf->K * (measurement - kf->x);
//    kf->P = (1 - kf->K) * kf->P;
//}
//
//float KalmanFilter(KalmanFilter *kf, float angle)
//{
//
//    KalmanFilter_Init(&kf, 0.1, 0.1, 0.0); // ��ʼ����������Э����Q����������Э����R����ʼֵ
//
//    KalmanFilter_Update(&kf, angle);
//
//    return kf.x;
//}

/**��װ����**/
// ��ʼ��һά�������˲�
Once_Dimensional_Kalman_Parameter imu660ra_KalmanFliter_gyro_Z;

void once_dimensional_kalman_init(Once_Dimensional_Kalman_Parameter* kf, float x, float p, float f, float q, float h, float r)
{
    kf->X = x;                                                                  // ��ʼ״̬����ֵ
    kf->P = p;                                                                  // ��ʼ״̬���ƵĲ�ȷ����
    kf->F = f;                                                                  // ״̬ת��ģ��ϵ��
    kf->Q = q;                                                                  // ��������Э����
    kf->H = h;                                                                  // �۲�ģ��ϵ��
    kf->R = r;                                                                  // �۲�����Э����
}

// һά�������˲�����
void once_dimensional_kalman_update(Once_Dimensional_Kalman_Parameter* kf, float Z)
{
    // Ԥ�ⲽ��
    float X_predict = kf->F * kf->X;
    float P_predict = kf->F * kf->P * kf->F + kf->Q;

    // ���²���
    float Y = Z - kf->H * X_predict;                                            // ����Ԥ�����
    float S = kf->H * P_predict * kf->H + kf->R;                                // ����Ԥ�����Э����
    float K = P_predict * kf->H / S;                                            // ���㿨��������
    kf->X = X_predict + K * Y;                                                  // ����״̬����ֵ
    kf->P = (1 - K * kf->H) * P_predict;                                        // ����״̬���ƵĲ�ȷ����
}
