/*
 * PID.h
 *
 *  Created on: 2024��4��16��
 *      Author: �ƴ���
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

typedef struct
{
    float                actual;     //ʵ��ֵ
    float                target;     //Ŀ��ֵ

    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //�����޷�

    float                out_p;      //KP���
    float                out_i;      //KI���
    float                out_d;      //KD���
    float                out;        //pid���
    float                omax;       //����޷�����
    float                omin;       //����޷�����

    float                integrator; //< ����ֵ
    float                last_error; //< �ϴ����
    float                last_derivative;//< �ϴ���������ϴ����֮��
    unsigned long        last_t;     //< �ϴ�ʱ��
}pid_param_t;

float Constrain_Data(float amt, float low, float high);

void  PidInit(pid_param_t * pid);

void integrator_Init(pid_param_t * pid);

float PidLocCtrl(pid_param_t * pid);

float PidIncCtrl(pid_param_t * pid);

#endif /* CODE_PID_H_ */

