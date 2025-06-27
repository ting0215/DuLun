/*
 * PID.c
 *
 *  Created on: 2024��4��16��
 *      Author: �ƴ���
 */

#include "PID.h"



/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵�����޷�����
 *  ����˵����
  * @param    amt   �� ����
  * @param    low   �� ���ֵ
  * @param    high  �� ���ֵ
 *  �������أ���
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float Constrain_Data(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid������ʼ������
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
 *  �������ƣ�float PidLocCtrl(pid_param_t * pid)
 *  ����˵����pidλ��ʽ���������
 *  ����˵����
  * @param    pid     pid����
 *  �������أ�PID������
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float PidLocCtrl(pid_param_t * pid)
{
    /* ������ */
    float error=0;
    error=pid->target-pid->actual;
    /* �ۻ���� */
    pid->integrator += error;

    /* ����޷� */
    pid->integrator=Constrain_Data(pid->integrator, -pid->imax, pid->imax);

    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    /* ���ֵ�޷� */
    pid->out=Constrain_Data(pid->out, pid->omin, pid->omax);

    return pid->out;
}
/*************************************************************************
 *  �������ƣ�float PidIncCtrl(pid_param_t * pid)
 *  ����˵����pid����ʽ���������
 *  ����˵����
  * @param    pid     pid����
 *  �������أ�PID������   ע���������Ѿ��������ϴν��
 *  �޸�ʱ�䣺2020��1��14�� ����
 *  ��    ע��
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

    /* ���ֵ�޷� */
    pid->out=Constrain_Data(pid->out, pid->omin, pid->omax);

    return pid->out;
}




