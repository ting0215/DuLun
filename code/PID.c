/*
 * PID.c
 *
 *  Created on: 2024年4月16日
 *      Author: 黄春华
 */

#include "PID.h"



/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：限幅函数
 *  参数说明：
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float Constrain_Data(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid参数初始化函数
void PidInit(pid_param_t * pid)
{
    pid->actual    = 0;
    pid->target    = 0;
    pid->kp        = 0;
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->omax      = 0;
    pid->omin      = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

void integrator_Init(pid_param_t * pid)
{
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}
/*************************************************************************
 *  函数名称：float PidLocCtrl(pid_param_t * pid)
 *  功能说明：pid位置式控制器输出
 *  参数说明：
  * @param    pid     pid参数
 *  函数返回：PID输出结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidLocCtrl(pid_param_t * pid)
{
    /* 误差计算 */
    float error=0;
    error=pid->target-pid->actual;
    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
    pid->integrator=Constrain_Data(pid->integrator, -pid->imax, pid->imax);

    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    /* 输出值限幅 */
    pid->out=Constrain_Data(pid->out, pid->omin, pid->omax);

    return pid->out;
}
/*************************************************************************
 *  函数名称：float PidIncCtrl(pid_param_t * pid)
 *  功能说明：pid增量式控制器输出
 *  参数说明：
  * @param    pid     pid参数
 *  函数返回：PID输出结果   注意输出结果已经包涵了上次结果
 *  修改时间：2020年1月14日 安健
 *  备    注：
 *************************************************************************/
float PidIncCtrl(pid_param_t * pid)
{
    float error=pid->target-pid->actual;

    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    /* 输出值限幅 */
    pid->out=Constrain_Data(pid->out, pid->omin, pid->omax);

    return pid->out;
}




