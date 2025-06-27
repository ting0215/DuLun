/*
 * guandao.h
 *
 *  Created on: 2024年4月21日
 *      Author: 黄春华
 */
#include "zf_common_headfile.h"
#ifndef CODE_GUANDAO_H_
#define CODE_GUANDAO_H_

#define circle_angle 360

float Distance_Get(int encoder_count, float angle);
float Angle_workout(float nx,float ny,float sx,float sy);
float Distance_workout(float nx,float ny,float sx,float sy);
float Distance_Get_x(int encoder_count, float angle);
float Distance_Get_y(int encoder_count, float angle);

extern float point_x[10],point_y[10];
extern int point_pointer;
extern float point_car_x,point_car_y;
extern float Distance_aim;
extern float angle_error;

//extern float angle_x_aim;

void location_get(void);
void point_aim_get(void);
void point_location_save(void);
void search_point_fixed(void);
void search_point_logic(void);
void V_decision(float d_aim,float angle_z_aim);

#endif /* CODE_GUANDAO_H_ */
