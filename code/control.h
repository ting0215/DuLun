 /*
 * control.h
 *
 *  Created on: 2024年4月17日
 *      Author: 黄春华
 */
#include "zf_common_headfile.h"
#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

void Brush_Motor_Init(void);
void Brush_Motor_Run(int Puty, int stop);
void Motor_Pid_Init(void);
float Motor_Speed_Get(void);
void Roll_Y_Aspect(float angle);     // 前后平衡的角速度环
void Roll_Y_speed(float angle_velocity, int *Out);
void  Motor_Speed_Aspect(float speed);                                  // 速度环
void Brushless_Motor_Init(void);
void Brushless_Motor_Run(int A_Puty,int B_Puty, int stop);
void Momentum_Pid_Init(void);
void Wheel_Speed_Get(void);
void Roll_Speed_X_Aspect(int actual_g,int *Motor_PWM1,int *Motor_PWM2);
void Roll_X_Aspect(void);
void Roll_X_Speed_Aspect(void);
void Steering_Aspect(float Turn, float target);
void Steering_Speed_Aspect(float speed1,float speed2);
void Set_Dynamic_Zero(float line_speed, int MdValue, float* X_angle, float* Y_angle, float Alpha1, float Alpha2);
void integrator_to_zero(void);
void Steering_PID_Init(void);

extern float Z_angle_out;
void Z_angle(float angle_Z_aim,float angle_Z);
extern float angle_aim ;
void Z_speed(float speed1,float speed2);
extern float Z_speed_out;

#endif /* CODE_CONTROL_H_ */
