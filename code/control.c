/*
 * control.c
 *
 *  Created on: 2024��4��17��
 *      Author: �ƴ���
 */


/*-----------------------------ͷ�ļ�������----------------------------------*/
#include <control.h>

/*------------------------------�궨����------------------------------------*/
//���Ƶ��
#define MOTOR_FREQUENCY    10000
//��ˢ���PWM����
#define Brush_MotorN       ATOM0_CH1_P21_3                           // ��ˢ���PWM���������
#define Brush_MotorP       ATOM0_CH0_P21_2                           // ��ˢ���PWM���������

//��ˢ���PWM����
#define Brushless_MotorA       ATOM0_CH5_P02_5        //PWM�����  ����
#define Brushless_MtorDirA     P02_4                  //��������
#define Brushless_MtorStopA    P11_2                  //ɲ������

#define Brushless_MotorB       ATOM0_CH7_P02_7        //PWM�����  ǰ��
#define Brushless_MtorDirB     P02_6                  //��������
#define Brushless_MtorStopB    P11_3                  //ɲ������

//��е���Ľ��ٶ�ֵ
#define Mechanical_zero -17// -0.015

/*-----------------------------����������-----------------------------------*/
float  Mechanical_zero_x = 0.15;              // x�ᶯ̬��е���ĽǶ�ֵ  ֱ����ROLL��-2.25 2.0     //-2.15
float  Mechanical_zero_x_basic = 0.15;         // x���ֶ���е���ĽǶ�ֵ -2.3                    //-2.15
float Mechanical_zero_y = -1.98;       // y�ᶯ̬��е���ĽǶ�ֵ-4.5    �н���PITCH��ƽ��ʱ��-0.8    //-1.4 //-2.0 //-1.95
float Mechanical_zero_y_basic = -1.98; // y���ֶ���е���ĽǶ�ֵ-4.5                             //-1.4 //-2.0 //-1.95

extern float Speed_Straght; // ֱ���ٶ�

float angle_aim =0;

pid_param_t Motor_Speed; // ǰ��ƽ��y���ٶ��⻷
pid_param_t Roll_Y;      // y��Ƕ��ڻ�
pid_param_t Roll_Y_Speed;
pid_param_t Wheel_Speed;                    // �������ٶ��⻷
pid_param_t Roll_X;                         // �����ֽǶ��ڻ�
pid_param_t Roll_Speed_X;                   // �����ֽ��ٶ��ڻ�
pid_param_t Steering;                       // ����
pid_param_t Steering_Speed;                 // �����ٶȻ�

short int Y_Balance_Count=0;                // ��ʱ��־λ��1,5,25�ֱ���ƴ���PID�ĵ�������2ms,10ms,50ms


/*------------------------------����������-----------------------------*/

/*************************************************************************
 *  �������ƣ�void Brush_Motor_Init()
 *  ����˵������ˢֱ�������ʼ��
 *  ����˵����
 *  �������أ�
 *  �޸�ʱ�䣺2023��2��10�� ����
 *  �޸����ߣ�����
 *  ��    ע����Ҫ��ʼ��PWM�ͷ��������
 *************************************************************************/
void Brush_Motor_Init(void)
{
    pwm_init(Brush_MotorP, MOTOR_FREQUENCY, 0); // ��ˢ���C��ת��PWM����ڳ�ʼ��
    pwm_init(Brush_MotorN, MOTOR_FREQUENCY, 0); // ��ˢ���C��ת��PWM����ڳ�ʼ��
    Motor_Pid_Init();
}
/*************************************************************************
 *  �������ƣ�void Brush_Motor_Runnode-red
 *  )
 *  ����˵������ˢ���PWM��ֵ����
 *  ����˵����@Puty:pwmֵ
 *  �������أ�
 *  �޸�ʱ�䣺2023��2��10��
 *  �޸����ߣ�����
 *  ��    ע��
 *************************************************************************/
void Brush_Motor_Run(int Puty, int stop)
{
    if (Puty > 3000)
        Puty = 3000;
    if (Puty < -3000)
        Puty = -3000;
    if (stop) // ͣ��������־
    {
        pwm_set_duty(Brush_MotorP, 0);
        pwm_set_duty(Brush_MotorN, 0);
    }
    else
    {
        if (Puty >= 0)
        {
            pwm_set_duty(Brush_MotorP, Puty+250);
            pwm_set_duty(Brush_MotorN, 0);
        }
        else
        {
            Puty = -Puty;
            pwm_set_duty(Brush_MotorP, 0);
            pwm_set_duty(Brush_MotorN, Puty+250);
        }
    }
}
/*************************************************************************
 *  �������ƣ�void Motor_Pid_Init()
 *  ����˵����ǰ��ƽ��y���2��PID�ṹ���ʼ��--�ǶȻ����ٶȻ�
 *  ����˵����
 *  �������أ���
 *  �޸�ʱ�䣺2023��2��10�� ����
 *  ��    ע��Motor_Pid_Init();
 *************************************************************************/
void Motor_Pid_Init(void)
{
    PidInit(&Roll_Y_Speed);
    PidInit(&Motor_Speed);
    PidInit(&Roll_Y);
    //���ٶȻ�PI�趨
    Roll_Y_Speed.kp = -1.2;    //-1.5 -2.6ƽ��    //-1.0
    Roll_Y_Speed.ki = -2.5;     //-15 2.5        //-2.5
    Roll_Y_Speed.kd = 0;
    // �ٶȻ���PD�趨     //�� ��Ҫ��
    Motor_Speed.kp = 0.9;//1.5;      // -0.02
    Motor_Speed.ki = 0.15;//0.3;          // 0
    Motor_Speed.kd = 0.25;//0.5;
    // �ǶȻ���PD�趨     //�� ��Ҫ��
    Roll_Y.kp = 460;                       //420//440
    Roll_Y.ki = 0.450;      //              //0.3//0.45
    Roll_Y.kd = 2.00;                       //2.0//2.10
}
/*************************************************************************
 *  �������ƣ�void Motor_Speed_Get()
 *  ����˵������ȡֱ����ˢc������ٶ�
 *  ����˵����
 *  �������أ�����ƽ���˲�����ٶ�ֵ
 *  �޸�ʱ�䣺2023��2��10�� ����
 *  ��    ע��Motor_Speed_Get();
 *************************************************************************/
//�����˲�����
float Err_LowOut_last_Y,Encoder_S_Y;
float a_Y=0.7;
float Motor_Speed_Get(void)
{
    /*������ʵ���ٶȻ���*/
    float Speed = 0;
    // ˳ʱ��Ϊ-����ʱ��Ϊ+�����������ţ�
    Speed = encoder_get_count(TIM4_ENCODER) * 1.0;
    encoder_clear_count(TIM4_ENCODER);
    X_Distance = Distance_Get_x(Speed,Yaw_Tur);
    Y_Distance = Distance_Get_y(Speed,Yaw_Tur);
    //Distance_Get(Speed, Yaw);
    Speed = Speed * 50 * 2 * 3.14 / 9500;
    //Speed = Speed*2.514453125;//Speed/512*8.2*3.14/0.02;    ʵ���ٶ�ֵ cm/s
    // �ٶȵ�ƽ���˲�
    //    Speed=Data_Mean_Filter(Speed,speed_Record,SPEED_RECORD_NUM);

    //CT
    float Err_Y,Err_LowOut_Y;
        //����ƫ��ֵ
        Err_Y=Speed;
        //��ͨ�˲�
        Err_LowOut_Y=(1-a_Y)*Err_Y+a_Y*Err_LowOut_last_Y;
        Err_LowOut_last_Y=Err_LowOut_Y;
        //����
        Encoder_S_Y+=Err_LowOut_Y;
        //�����޷�
        Encoder_S_Y=Encoder_S_Y>200?200:(Encoder_S_Y<(-200)?(-200):Encoder_S_Y);
    //CT

    return Err_LowOut_Y;
}

/*************************************************************************
 *  �������ƣ�void Roll_Y_Speed()
 *  ����˵��: ǰ��ƽ��y��Ľ��ٶȻ�����
 *  ����˵����@angular_velocity ʵ�ʽ��ٶ�
 *  �������أ���
 *  �޸�ʱ�䣺2024��5��10�� �ƴ���
 *  ��    ע��Roll_Y_Aspect();
 *************************************************************************/
////�����˲�����CT
//float Err_LowOut_last_y,Encoder_S_y;
//float a_y=0.7;
void Roll_Y_speed(float angle_velocity, int *Out)
{
//    //CT//��ȷ��Ч��
//    float Err_y,Err_LowOut_y;
//        //����ƫ��ֵ
//        Err_y=angle_velocity;
//        //��ͨ�˲�
//        Err_LowOut_y=(1-a_y)*Err_y+a_y*Err_LowOut_last_y;
//        Err_LowOut_last_y=Err_LowOut_y;
//        //����
//        Encoder_S_y+=Err_LowOut_y;
//        //�����޷�
//        Encoder_S_y=Encoder_S_y>200?200:(Encoder_S_y<(-200)?(-200):Encoder_S_y);
//        angle_velocity=Err_LowOut_y;
//    //CT

    Roll_Y_Speed.actual = angle_velocity;//
    Roll_Y_Speed.target = Roll_Y.out ;//- 6;

    Roll_Y_Speed.omax = 5000;
    Roll_Y_Speed.omin = -5000;

    //Roll_Y_Speed.imax=2000;

    PidLocCtrl(&Roll_Y_Speed);

    *Out = Roll_Y_Speed.out;
}

/*************************************************************************
 *  �������ƣ�void Roll_Y_Aspect()
 *  ����˵��: ǰ��ƽ��y��ĽǶȻ�����
 *  ����˵����@angle ʵ�ʽǶ�
 *         @angular_velocity ʵ�ʽ��ٶ�
 *  �������أ���
 *  �޸�ʱ�䣺2023��2��10�� ����
 *  ��    ע��Roll_Y_Aspect();
 *************************************************************************/
void Roll_Y_Aspect(float angle)
{
    Roll_Y.actual = angle;
    Roll_Y.target = Mechanical_zero_y + Motor_Speed.out;

        // �ٶȻ�������޷�
    Roll_Y.omax =3000;
    Roll_Y.omin = -3000;

        // λ��ʽPIDӦ��
    PidLocCtrl(&Roll_Y);
    //*Out = (int)AngleControlOut;
}
//void Roll_Y_Aspect(float angle, float angular_velocity, int *Out)
//{
//    float Bias_Angle;
//    float AngleControlOut;
//
//    Bias_Angle = (Mechanical_zero_y + Motor_Speed.out) - angle; // Mechanical_zero_y��е���ĽǶ�ֵ��Motor_Speed.out���빹�ɴ���PID
//
//    // PD�������õ����
//    AngleControlOut = Bias_Angle * Roll_Y.kp + (angular_velocity - 6) * Roll_Y.kd; // 14��Ư
//
//    //    if(AngleControlOut>=0) AngleControlOut=AngleControlOut+150;                               // 150������С
//    //    else                   AngleControlOut=AngleControlOut-150;
//    // �ǶȻ�������޷�
//    AngleControlOut = Constrain_Data(AngleControlOut, -3000.0, 3000.0);
//
//    Roll_Y.out = AngleControlOut;
//    *Out = (int)AngleControlOut;
//}
/*************************************************************************
 *  �������ƣ�void Motor_Speed_Aspect(float Speed)
 *  ����˵�����ٶȻ�����
 *  ����˵����@Speed :�ٶ�ʵ��ֵ
 *  �������أ������ٶȻ��� �Ƕ� ���
 *  �޸�ʱ�䣺2023��2��10�� ����
 *  ��    ע��ÿ20msִ��һ��ִ��һ�ε���
 *************************************************************************/
void Motor_Speed_Aspect(float Speed)
{
    // �ٶȻ������㣬Ŀ���ٶ�Ϊ0
    Motor_Speed.actual = Speed;
    Motor_Speed.target = Speed_Straght;//Speed_Straght;

    // �ٶȻ�������޷�
    Motor_Speed.omax = 10;
    Motor_Speed.omin = -10;

    // λ��ʽPIDӦ��
    PidLocCtrl(&Motor_Speed);
}
/*************************************************************************
*  �������ƣ�void Brushless_Motor_Init()
*  ����˵������ˢ�����ʼ��
*  ����˵����
*  �������أ�
*  �޸�ʱ�䣺2023��1��14��
*  �޸����ߣ�����
*  ��    ע����Ҫ��ʼ��PWM�ͱ������ڣ�ɲ���ߣ����������
*************************************************************************/
void Brushless_Motor_Init(void)
{
    pwm_init(Brushless_MotorB, 10000, MOTOR_FREQUENCY);                 // ��ˢ��� B PWM������ų�ʼ��,��ռ�ձ�ͣת
    gpio_init(Brushless_MtorDirB, GPO, GPIO_HIGH, GPO_PUSH_PULL);       // B�����߳�ʼ����1Ϊ˳ʱ��
    gpio_init(Brushless_MtorStopB, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // ɲ���߳�ʼ�������0ɲ��

    pwm_init(Brushless_MotorA, 10000, MOTOR_FREQUENCY);                 // ��ˢ��� A PWM������ų�ʼ��,��ռ�ձ�ͣת
    gpio_init(Brushless_MtorDirA, GPO, GPIO_HIGH, GPO_PUSH_PULL);       // A�����߳�ʼ����1Ϊ˳ʱ��
    gpio_init(Brushless_MtorStopA, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // ɲ���߳�ʼ�������0ɲ��

    Momentum_Pid_Init();
}
/*************************************************************************
*  �������ƣ�void Brushless_Motor_Run()
*  ����˵������ˢ���PWM��ֵ����
*  ����˵����
*  �������أ�
*  �޸�ʱ�䣺2023��1��15��
*  �޸����ߣ�����
*  ��    ע����Ҫ��ʼ��PWM��������Ҫ�ı䷽���ߵĸߵ͵�ƽ
*************************************************************************/
void Brushless_Motor_Run(int A_Puty,int B_Puty, int stop)
{
    if(A_Puty < 240 && A_Puty > 0)   A_Puty = 240;
    if(A_Puty > -240 && A_Puty < 0)   A_Puty = -240;
    if(B_Puty < 240 && B_Puty > 0)   B_Puty = 240;
    if(B_Puty > -240 && B_Puty < 0)   B_Puty = -240;

    if(A_Puty > 10000)   A_Puty = 10000;
    if(A_Puty < -10000)   A_Puty = -10000;
    if(B_Puty > 10000)   B_Puty = 10000;
    if(B_Puty < -10000)   B_Puty = -10000;
    if(A_Puty<0)
    {
        gpio_set_level(Brushless_MtorDirA,0);// A������ʱ��
        A_Puty=0-A_Puty;
    }
    else
    {
        gpio_set_level(Brushless_MtorDirA,1);// A����˳ʱ��
    }
    if(B_Puty<0)
    {
        gpio_set_level(Brushless_MtorDirB,1);// B������ʱ��
        B_Puty=0-B_Puty;
    }
    else
    {
        gpio_set_level(Brushless_MtorDirB,0);// B����˳ʱ��
    }
    if(Stop_Car)
    {
        pwm_set_duty(Brushless_MotorB, 0);    // ��ˢ��� B PWM���0
        pwm_set_duty(Brushless_MotorA, 0);    // ��ˢ��� A PWM���0
        gpio_set_level(Brushless_MtorStopA,0);// Aɲ��
        gpio_set_level(Brushless_MtorStopB,0);// Bɲ��
    }
    else
    {
        pwm_set_duty(Brushless_MotorB, B_Puty);// ��ˢ��� B PWM���
        pwm_set_duty(Brushless_MotorA, A_Puty);// ��ˢ��� A PWM���
        gpio_set_level(Brushless_MtorStopA,1); // �ɿ�ɲ��
        gpio_set_level(Brushless_MtorStopB,1); // �ɿ�ɲ��
    }
}
/*************************************************************************
 *  �������ƣ�void Momentum_Pid_Init()
 *  ����˵���������ֵ��ĸ�PID�ṹ���ʼ��
 *  ����˵����
 *  �������أ���
 *  �޸�ʱ�䣺2023��1��9�� ����
 *  ��    ע��All_Pid_Init();
 *************************************************************************/
void Momentum_Pid_Init(void)
{
    PidInit(&Roll_Speed_X);
    PidInit(&Roll_X);
    PidInit(&Wheel_Speed);
    PidInit(&Steering);
    PidInit(&Steering_Speed);

    //ֱ�����ٶȻ�(PI)
    Roll_Speed_X.kp = -2.80;//-3.5  -6  -5    //-2.8
    Roll_Speed_X.ki = -1.3;// -1 -1.2 -0.5   //-1.3
    Roll_Speed_X.kd = -0.0;//0

    //ֱ���ǶȻ�(PD)
    Roll_X.kp = 150;//139 250    160      //150
    Roll_X.ki = 5.0;//4  2      2.5       //5
    Roll_X.kd = 20;//      0              //20

    //ֱ���ٶȻ�
    Wheel_Speed.kp= -0.018;  //0.0005     //0.018
    Wheel_Speed.ki= 0;   //0.0005      0
    Wheel_Speed.kd= 0;

    //ת��ǶȻ�
    Steering.kp       = 30;     //30
    Steering.ki       = 0;
    Steering.kd       = 0;    //���Ƕ�ת��Ĳ����趨

    //ת���ٶȻ�
    Steering_Speed.kp = 0;
    Steering_Speed.ki = 0;
    Steering_Speed.kd = 0;
}
/*************************************************************************
 *  �������ƣ�void Wheel_Speed_Get()
 *  ����˵������ȡ�����ֵ��ٶ�
 *  ����˵����
 *  �������أ���
 *  �޸�ʱ�䣺2023��1��9�� ����
 *  ��    ע��All_Pid_Init();
 *************************************************************************/
void Wheel_Speed_Get(void)
{
    /*������ʵ���ٶȻ���*/
    short int Front_Speed=0;
    short int Back_Speed=0;
    // ˳ʱ��Ϊ-����ʱ��Ϊ+�����������ţ�
    Front_Speed=encoder_get_count(TIM5_ENCODER);    //����B
    Back_Speed=encoder_get_count(TIM2_ENCODER);     //����A
}
/*************************************************************************
 *  �������ƣ�int Roll_Speed_X_Aspect()
 *  ����˵�������ٶȻ�����
 *  ����˵����
 *  @param:actual_g ʵ�ʽ��ٶ�ֵ
 *  �������أ����ؽ��ٶȻ���PWM���
 *  �޸�ʱ�䣺2023��1��14�� ����
 *  ��    ע��ÿ2msִ��һ��ִ��һ�ε���
 *************************************************************************/
void Roll_Speed_X_Aspect(int actual_g,int *Motor_PWM1,int *Motor_PWM2)
{
    // ���ٶȻ������㣬Ŀ����ٶ�Ϊ�ȶ�ʱ�Ľ��ٶ� -18
    Roll_Speed_X.actual=actual_g*1.0;
    Roll_Speed_X.target=Roll_X.out + 2 ;

    // ���ٶȻ�������޷�
    Roll_Speed_X.omax=8000;
    Roll_Speed_X.omin=-8000;

    // ���ٶȻ��Ļ����ۻ��޷�

    Roll_Speed_X.imax=4000;

    *Motor_PWM1=-(int)PidLocCtrl(&Roll_Speed_X) -(int)Z_angle_out;//- Steering.out ; //1.25     //-
    *Motor_PWM2=-(int)PidLocCtrl(&Roll_Speed_X) +(int)Z_angle_out;//+ Steering.out ; //0.75     //+
    //tft180_show_float (0, 0,PidLocCtrl(&Roll_Speed_X)/*X_Distance*/, 4,4);
    if(*Motor_PWM1>=0) *Motor_PWM1=10000-*Motor_PWM1;         //100������С
    else              *Motor_PWM1=-10000-*Motor_PWM1;

    if(*Motor_PWM2>=0) *Motor_PWM2=10000-*Motor_PWM2;         //100������С
    else               *Motor_PWM2=-10000-*Motor_PWM2;
}
/*************************************************************************
 *  �������ƣ�void Roll_Aspect()
 *  ����˵�����ǶȻ�����
 *  ����˵����
 *  @param:
 *  �������أ����ؽǶȻ��� ���ٶ� ���
 *  �޸�ʱ�䣺2023��1��14�� ����
 *  ��    ע��ÿ10msִ��һ��ִ��һ�ε���
 *************************************************************************/
void Roll_X_Aspect(void)
{
    // �ǶȻ������㣬Ŀ��Ƕ�Ϊ�ٶȻ����
    Roll_X.actual=Roll;//����
    Roll_X.target= Mechanical_zero_x + X_Dynamic_Zero/*+ X_Dynamic_Zero*/+ Wheel_Speed.out;//��е���1//����

    // �ǶȻ�������޷�
    Roll_X.omax=8000;
    Roll_X.omin=-8000;

    // λ��ʽPIDӦ��
    PidLocCtrl(&Roll_X);
    tft180_show_float (0, 0,Roll_X.actual-Roll_X.target, 4,4);
    tft180_show_float (0, 20,Roll_X.integrator, 4,4);
    tft180_show_float (0, 40,Roll_X.last_error, 4,4);
    tft180_show_float (0, 60,Roll_X.out_p, 4,4);
    tft180_show_uint (0, 80, Roll_X.out,3);
}
/*************************************************************************
 *  �������ƣ�void Roll_X_Speed_Aspect()
 *  ����˵�����ٶȻ�����
 *  ����˵����
 *  �������أ������ٶȻ��� �Ƕ� ���
 *  �޸�ʱ�䣺2023��1��14�� ����
 *  ��    ע��ÿ100msִ��һ��ִ��һ�ε���
 *************************************************************************/

//�����˲�����CT
float Err_LowOut_last_X,Encoder_S_X;
float a_X=0.7;

void Roll_X_Speed_Aspect(void)
{

    /*�������ٶȻ�ȡ*/
    short int Front_Speed=0;
    short int Back_Speed=0;
    float Speed=0;
    // ˳ʱ��Ϊ-����ʱ��Ϊ+�����������ţ�
    Front_Speed = encoder_get_count(TIM5_ENCODER);    //����B
    Back_Speed = encoder_get_count(TIM2_ENCODER);     //����A
    //��������������
    encoder_clear_count(TIM5_ENCODER);
    encoder_clear_count(TIM2_ENCODER);

    B_Speed = Front_Speed;        // ȫ�ֱ�����ֵ
    A_Speed = -Back_Speed;

    Speed=(-Back_Speed+Front_Speed)*1.0/2;

    //CT
    //��ͨ�˲�
    float Err_X,Err_LowOut_X;
        //����ƫ��ֵ
        Err_X=Speed;
        //��ͨ�˲�
        Err_LowOut_X=(1-a_X)*Err_X+a_X*Err_LowOut_last_X;
        Err_LowOut_last_X=Err_LowOut_X;
        //����
        Encoder_S_X+=Err_LowOut_X;
        //�����޷�
        Encoder_S_X=Encoder_S_X>200?200:(Encoder_S_X<(-200)?(-200):Encoder_S_X);
    //CT

    // �ٶȻ������㣬Ŀ���ٶ�Ϊ0
    Wheel_Speed.actual = Err_LowOut_X;//Err_LowOut_X
    Wheel_Speed.target=0;

    // �ٶȻ�������޷�
    Wheel_Speed.omax=200;
    Wheel_Speed.omin=-200;

    // λ��ʽPIDӦ��
    PidLocCtrl(&Wheel_Speed);

}

void Steering_PID_Init(void)
{
    Steering.kp       = 0;
    Steering.ki       = 0;
    Steering.kd       = 0;    // ָ���Ƕ�ת��Ĳ����趨
    Steering_Speed.kp = 0;
    Steering_Speed.ki = 0;
    Steering_Speed.kd = 0;
}
/*************************************************************************
 *  �������ƣ�void Steering_Aspect()
 *  ����˵����ת�򻷵���
 *  ����˵����
 *  �������أ�
 *  �޸�ʱ�䣺2023��3��9�� ����
 *  ��    ע��ÿmsִ��һ��ִ��һ�ε���
 *************************************************************************/
void Steering_Aspect(float Turn, float target)
{
    //����ʵ��ֵ��Ŀ��ֵ�趨
    Steering.target = target + Steering_Speed.out;
    Steering.actual = Turn;

    // ���򻷵�����޷�
    Steering.omax = 5000;
    Steering.omin = -5000;
    // λ��ʽPIDӦ��
    PidLocCtrl(&Steering);
    Steering.out_d = -Steering.kd*(imu660ra_gyro_z - 1);
    //Steering.out += Steering.out_d;                     // �����ǻ�ȡ�Ľ��ٶ���Ϊ΢�������PID���� 1Ϊ������z����Ư
}
/*************************************************************************
 *  �������ƣ�void Steering_Speed_Aspect()
 *  ����˵����ת���ٶ��⻷����
 *  ����˵����Steering_Speed_Aspect(A_SPeed.B_Speed);
 *  �������أ�
 *  �޸�ʱ�䣺2023��5��9�� ����
 *  ��    ע��ÿ20msִ��һ��ִ��һ�ε���
 *************************************************************************/
void Steering_Speed_Aspect(float speed1,float speed2)
{

    float Difference_Speed = (speed1 - speed2);
    //����ʵ��ֵ��Ŀ��ֵ�趨
    Steering_Speed.target = 0;
    Steering_Speed.actual = Difference_Speed;

    // ���򻷵�����޷�
    Steering_Speed.omax = 100;
    Steering_Speed.omin = -100;
    // λ��ʽPIDӦ��
    PidLocCtrl(&Steering_Speed);
}

//CTת��
float Z_angle_Kp=30,Z_angle_Ki=0.150,Z_angle_Kd=0.3;//Z_angle_Kp=30,Z_angle_Ki=0.40,Z_angle_Kd=0.15;
float Z_angle_error,Z_angle_error_last,Z_angle_integration,Z_angle_deviation;
float Z_angle_out=0;
void Z_angle(float angle_Z_aim,float angle_Z)
{
    Z_angle_error=-angle_error;//angle_Z_aim-angle_Z;

    Z_angle_integration+=Z_angle_error;

    if (Z_angle_integration>6000)Z_angle_integration=7000;
    if (Z_angle_integration<-6000)Z_angle_integration=-7000;

    Z_angle_deviation=Z_angle_error-Z_angle_error_last;

    Z_angle_out=Z_angle_Kp*Z_angle_error + Z_angle_Ki*Z_angle_integration + Z_angle_Kd*Z_angle_deviation;

    if (Z_angle_out>0)Z_angle_out=Z_angle_out+(150+3*abs(Z_angle_error));//�н��������Ħ��������
    if (Z_angle_out<0)Z_angle_out=Z_angle_out-(150+3*abs(Z_angle_error));

    Z_angle_error_last=Z_angle_error;
}
//CTת���ٶȻ� //������
float Z_speed_Kp=0.00,Z_speed_Ki=0.0,Z_speed_Kd=0.0;
float Z_speed_error,Z_speed_error_last,Z_speed_integration,Z_speed_deviation;
float Z_speed_out=0;
void Z_speed(float speed1,float speed2)
{
    Z_speed_error=speed1 - speed2;
    Z_speed_integration+=Z_speed_error;

    if (Z_speed_integration>100)Z_speed_integration=100;
    if (Z_speed_integration<-100)Z_speed_integration=-100;

    Z_speed_deviation=Z_speed_error-Z_speed_error_last;

    Z_speed_out=Z_angle_Kp*Z_speed_error + Z_speed_Ki*Z_speed_integration + Z_speed_Kd*Z_speed_deviation;

    Z_speed_error_last=Z_speed_error;
}



// ��̬���
void Set_Dynamic_Zero(float line_speed, int MdValue, float* X_angle, float* Y_angle, float Alpha1, float Alpha2)
{
    *X_angle =  Alpha1 * line_speed  * line_speed * (MdValue - 49);
    *Y_angle =  Alpha2 * line_speed *  line_speed * (MdValue - 49);
    *X_angle = Constrain_Data(*X_angle, -8, 8);
    *Y_angle = Constrain_Data(*Y_angle, -8, 8);
}

//����ֵ����
void integrator_to_zero(void)
{
    integrator_Init(&Motor_Speed);
    integrator_Init(&Roll_Y);
    integrator_Init(&Wheel_Speed);
    integrator_Init(&Roll_X);
    integrator_Init(&Roll_Speed_X);
    integrator_Init(&Steering);
    integrator_Init(&Steering_Speed);
}
