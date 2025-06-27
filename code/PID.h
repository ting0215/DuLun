/*
 * PID.h
 *
 *  Created on: 2024年4月16日
 *      Author: 黄春华
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

typedef struct
{
    float                actual;     //实际值
    float                target;     //目标值

    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //积分限幅

    float                out_p;      //KP输出
    float                out_i;      //KI输出
    float                out_d;      //KD输出
    float                out;        //pid输出
    float                omax;       //输出限幅上限
    float                omin;       //输出限幅下限

    float                integrator; //< 积分值
    float                last_error; //< 上次误差
    float                last_derivative;//< 上次误差与上上次误差之差
    unsigned long        last_t;     //< 上次时间
}pid_param_t;

float Constrain_Data(float amt, float low, float high);

void  PidInit(pid_param_t * pid);

void integrator_Init(pid_param_t * pid);

float PidLocCtrl(pid_param_t * pid);

float PidIncCtrl(pid_param_t * pid);

#endif /* CODE_PID_H_ */

