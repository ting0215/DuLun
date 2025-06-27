/*
 * guandao.c
 *
 *  Created on: 2024��4��21��
 *      Author: �ƴ���
 */
/*------------------------------------ͷ�ļ�������-----------------------*/
#include <guandao.h>

/*------------------------------------����������-------------------------*/
float Distance = 0;   // �������ܾ���
extern pid_param_t Steering;
extern int numpath;
extern float save_x[100],save_y[100];
extern float  Mechanical_zero_x;
extern float  Mechanical_zero_y;

/*-------------------------------------����������------------------------*/

/*************************************************************************
 *  �������ƣ�float Distance_Get()
 *  ����˵�������ڱ������Ĳ���ƫ���ǵĹ��Ե���λ�ƽ���
 *  ����˵����@encoder_count:��������õ�ʵ���ٶ�  @angle:ƫ����(Yaw)
 *  �������أ�
 *  �޸�ʱ�䣺2023��4��17�� �ƴ���
 *  ��    ע��Motor_Speed_Get();
 *************************************************************************/
float Distance_Get(int encoder_count, float angle)
{
    float number = (float)encoder_count/512*8.2*3.1415;
    Distance += number;
    return Distance;
}

/*
 * ����֮���ƫ�ǣ���+y�ᣨС����λʱ��ͷ����Ϊ��������ʱ�루0~360��
 */
float Angle_workout(float nx, float ny, float sx, float sy)
{
    float Delta_X;
    float Delta_Y;
    float yaw;

    Delta_X = -nx + sx;
    Delta_Y = -ny + sy;

    // atan2 ���صĽǶȷ�ΧΪ -180 �� 180 �ȣ�������Ҫ����ת��Ϊ 0 �� 360 ��//???
    yaw = atan2(Delta_X, Delta_Y) * 180 / PI;  // ������ʱ�뷽��ĽǶȣ���y��Ϊ0�ȣ�

//    // ���Ƕȵ���Ϊ 0 �� 360 ��֮��
//    if (yaw < 0)
//    {
//        yaw += 360;
//    }
//
//    // ��ʱ�뷽������ĽǶ�
//    yaw = 360 - yaw;

    return yaw;
}
/*
 * ����֮��ľ������
 */
float Distance_workout(float nx,float ny,float sx,float sy)
{
    float Delta_X;
    float Delta_Y;
    float Distance;

    Delta_X = fabsf(nx - sx);
    Delta_Y = fabsf(ny - sy);
    Distance = (float)sqrt(Delta_X*Delta_X + Delta_Y*Delta_Y);

    return Distance;
}

float point_x[10]={{0}},point_y[10]={{0}};
int point_pointer = 0;//Ŀ�����������ָ��
float point_car_x = 0,point_car_y = 0;
float Distance_aim = 0;
float angle_error = 0;

/*************************************************************************
 *  �������ƣ�float Distance_Get_x()
 *  ����˵������ȡ��ǰX������
 *  ����˵����@encoder_count:��������õ�ʵ���ٶ�  @angle:ƫ����(Yaw)
 *  �������أ�
 *  �޸�ʱ�䣺2023��4��17�� �ƴ���
 *  ��    ע��Motor_Speed_Get(); ����Ƕ�ΪYaw_Tur 0-360 ��ʱ��
 *************************************************************************/
float Distance_Get_x(int encoder_count, float angle)
{
    float number = (float)encoder_count / 4096 * 8.2 * 3.1415;
    if (angle >= 0 && angle < 180)
    {
        X_Distance += - number * sinf(angle * 0.01745);
    }
    else if (angle >= 180 && angle <= 360)
    {
        angle -= 360;
        X_Distance += -number * sinf(angle * 0.01745);
    }

    return X_Distance;
}


float Distance_Get_y(int encoder_count, float angle)
{
    float number = (float)encoder_count/4096*8.2*3.1415 ;
    if ((angle >=0  && angle < 90) || (angle > 270 && angle <= 360))
    {
        Y_Distance += number * cosf(angle * Gyro_Gr);
    }
     if(angle >= 90 && angle <= 270)
     {
         angle = angle - 360;
         angle =  - angle;
         Y_Distance += number * cosf(angle * Gyro_Gr);
     }
     return Y_Distance;
}


//��ȡ����������
void location_get()
{
    point_car_x=Distance_Get_x(C_Speed, Yaw_Tur);
    point_car_y=Distance_Get_y(C_Speed, Yaw_Tur);
}

//Ŀ�����ǶȻ�ȡ
void point_aim_get()
{
    Distance_aim=Distance_workout(point_x[point_pointer],point_y[point_pointer],point_car_x,point_car_y);
    angle_error=Angle_workout(point_x[point_pointer],point_y[point_pointer],point_car_x,point_car_y)+convertTo360(Yaw_B1)/*+180*/;
    if(angle_error>=180)
    {
        angle_error=angle_error-360;
    }
//    if((angle_error>=-370) && (angle_error<=-180))
//    {
//        angle_error=360+angle_error;
//    }
    //tft180_show_float (60, 60,Angle_workout(point_x[point_pointer],point_y[point_pointer],point_car_x,point_car_y), 4,4);
    //tft180_show_float (60, 80,Yaw_B1, 4,4);
}
uint8_t SGSD = 0;
//�������
int point_save=0;
void point_location_save()
{
    //int guandao_open=0;
    if(KEY_detect(KEY_2))point_save=1;//���ģʽ��־λ
    else point_save=0;
    while(point_save==1)
    {
        point_pointer=0;
    while(point_save==1)
    {
        system_delay_ms(20);
        if(KEY_detect(KEY_2))
        {
            if(point_pointer < 10)
            {
               point_x[point_pointer]=point_car_x;
               point_y[point_pointer]=point_car_y;
               point_pointer++;
               system_delay_ms(500);//ʷ(��һ��Ϊ����)
            }
        }
        if(KEY_detect(KEY_1))
        {
            point_car_x=0;
            point_car_y=0;
            X_Distance=0;
            Y_Distance=0;
            point_save=0;
            break;
        }
    }
    break;
    }
    if(KEY_detect(KEY_1))//����ʱ��������
    {
        point_car_x=0;
        point_car_y=0;
        X_Distance=0;
        Y_Distance=0;
        point_pointer=0;
        Stop_Car=0;
    }
}

//Ѳ�� �̶���
void search_point_fixed()
{
    if(point_save==0)
    {
        //point_aim_get();
        if(Distance_aim<1.0)point_pointer+=1;
        if(point_pointer>=10)point_pointer=0;
    }

}

//Ѳ�� �߼��жϵ�
void search_point_logic()
{
        float point_data[10][3];//���� �Ƕ� ���

        for(int i=0;i<10;i++)//����ÿ����ľ���Ƕ�
        {
            point_data[i][0]=Distance_workout(point_x[i],point_y[i],point_car_x,point_car_y);
            point_data[i][1]=Angle_workout(point_x[i],point_y[i],point_car_x,point_car_y);
            point_data[i][2]=(float)i;
        }
        float temp1 = 0;
        float temp2 = 0;
        float temp3 = 0;
        for(int a=0;a<10;a++)//��������������
        {
            for(int b=1;b<10-a;b++)
            {
                if(point_data[b-1][0]>point_data[b][0])
                {
                    temp1 = point_data[b-1][0];
                    point_data[b-1][0] = point_data[b][0];
                    point_data[b][0] = temp1;

                    temp2 = point_data[b-1][1];
                    point_data[b-1][1] = point_data[b][1];
                    point_data[b][1] = temp2;

                    temp3 = point_data[b-1][2];
                    point_data[b-1][2]=point_data[b][2];
                    point_data[b][2]=temp3;
                }
            }
        }

        int aim_pointer1=0;
        int aim_pointer2=0;
        for(int j=0;j<10;j++)//ֻѡȡһ����Χ�ڵĵ�
        {
            if (point_data[j][0]<0.5)aim_pointer1=j;
            if (point_data[j][0]<2.0)aim_pointer2=j;
        }

        float temp4 = 0;
        float temp5 = 0;
        float temp6 = 0;
        for(int a=aim_pointer1; a < aim_pointer2; a++)//���Ƕ���������
        {
            for(int b=aim_pointer1+1;b<aim_pointer2-a;b++)
            {
                if(point_data[b-1][1]>point_data[b][1])
                {
                    temp4=point_data[b-1][1];
                    point_data[b-1][1]=point_data[b][1];
                    point_data[b][1]=temp4;

                    temp5=point_data[b-1][0];
                    point_data[b-1][0]=point_data[b][0];
                    point_data[b][0]=temp5;

                    temp6=point_data[b-1][2];
                    point_data[b-1][2]=point_data[b][2];
                    point_data[b][2]=temp6;
                }
            }
        }
        point_pointer=point_data[aim_pointer1][2];
        point_aim_get();
}

void V_decision(float d_aim,float angle_z_aim)//CT
{
    float r_decision=0;//ת��뾶

    r_decision = (d_aim/2)/sinf(abs(angle_z_aim)*3.1415/180);

    Speed_Straght=-9;//-(8+3*log10f(5*d_aim+1));

    //X_Dynamic_Zero =0.01*angle_z_aim;//��ʽ����
    X_Dynamic_Zero =atanf( 0.015 * (Speed_Straght * Speed_Straght)/ r_decision );//���Ե���atanf
    //X_Dynamic_Zero =roundf(X_Dynamic_Zero,4);
    //tft180_show_float (0, 0,X_Dynamic_Zero/*X_Distance*/, 4,4);
    //X_Dynamic_Zeroֵ����
}
