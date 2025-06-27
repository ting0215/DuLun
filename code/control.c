/*
 * control.c
 *
 *  Created on: 2024年4月17日
 *      Author: 黄春华
 */


/*-----------------------------头文件声明区----------------------------------*/
#include <control.h>

/*------------------------------宏定义区------------------------------------*/
//电机频率
#define MOTOR_FREQUENCY    10000
//有刷电机PWM引脚
#define Brush_MotorN       ATOM0_CH1_P21_3                           // 有刷电机PWM正向输出口
#define Brush_MotorP       ATOM0_CH0_P21_2                           // 有刷电机PWM反向输出口

//无刷电机PWM引脚
#define Brushless_MotorA       ATOM0_CH5_P02_5        //PWM输出口  后轮
#define Brushless_MtorDirA     P02_4                  //方向引脚
#define Brushless_MtorStopA    P11_2                  //刹车引脚

#define Brushless_MotorB       ATOM0_CH7_P02_7        //PWM输出口  前轮
#define Brushless_MtorDirB     P02_6                  //方向引脚
#define Brushless_MtorStopB    P11_3                  //刹车引脚

//机械零点的角速度值
#define Mechanical_zero -17// -0.015

/*-----------------------------变量声明区-----------------------------------*/
float  Mechanical_zero_x = 0.15;              // x轴动态机械零点的角度值  直立（ROLL）-2.25 2.0     //-2.15
float  Mechanical_zero_x_basic = 0.15;         // x轴手动机械零点的角度值 -2.3                    //-2.15
float Mechanical_zero_y = -1.98;       // y轴动态机械零点的角度值-4.5    行进（PITCH）平衡时：-0.8    //-1.4 //-2.0 //-1.95
float Mechanical_zero_y_basic = -1.98; // y轴手动机械零点的角度值-4.5                             //-1.4 //-2.0 //-1.95

extern float Speed_Straght; // 直道速度

float angle_aim =0;

pid_param_t Motor_Speed; // 前后平衡y轴速度外环
pid_param_t Roll_Y;      // y轴角度内环
pid_param_t Roll_Y_Speed;
pid_param_t Wheel_Speed;                    // 动量轮速度外环
pid_param_t Roll_X;                         // 动量轮角度内环
pid_param_t Roll_Speed_X;                   // 动量轮角速度内环
pid_param_t Steering;                       // 方向环
pid_param_t Steering_Speed;                 // 方向环速度环

short int Y_Balance_Count=0;                // 定时标志位，1,5,25分别控制串级PID的调节周期2ms,10ms,50ms


/*------------------------------函数定义区-----------------------------*/

/*************************************************************************
 *  函数名称：void Brush_Motor_Init()
 *  功能说明：有刷直流电机初始化
 *  参数说明：
 *  函数返回：
 *  修改时间：2023年2月10日 安健
 *  修改作者：安健
 *  备    注：需要初始化PWM和方向控制线
 *************************************************************************/
void Brush_Motor_Init(void)
{
    pwm_init(Brush_MotorP, MOTOR_FREQUENCY, 0); // 有刷电机C正转的PWM输出口初始化
    pwm_init(Brush_MotorN, MOTOR_FREQUENCY, 0); // 有刷电机C反转的PWM输出口初始化
    Motor_Pid_Init();
}
/*************************************************************************
 *  函数名称：void Brush_Motor_Runnode-red
 *  )
 *  功能说明：有刷电机PWM赋值运行
 *  参数说明：@Puty:pwm值
 *  函数返回：
 *  修改时间：2023年2月10日
 *  修改作者：安健
 *  备    注：
 *************************************************************************/
void Brush_Motor_Run(int Puty, int stop)
{
    if (Puty > 3000)
        Puty = 3000;
    if (Puty < -3000)
        Puty = -3000;
    if (stop) // 停车保护标志
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
 *  函数名称：void Motor_Pid_Init()
 *  功能说明：前后平衡y轴的2个PID结构体初始化--角度环，速度环
 *  参数说明：
 *  函数返回：无
 *  修改时间：2023年2月10日 安健
 *  备    注：Motor_Pid_Init();
 *************************************************************************/
void Motor_Pid_Init(void)
{
    PidInit(&Roll_Y_Speed);
    PidInit(&Motor_Speed);
    PidInit(&Roll_Y);
    //角速度环PI设定
    Roll_Y_Speed.kp = -1.2;    //-1.5 -2.6平衡    //-1.0
    Roll_Y_Speed.ki = -2.5;     //-15 2.5        //-2.5
    Roll_Y_Speed.kd = 0;
    // 速度环的PD设定     //抖 还要调
    Motor_Speed.kp = 0.9;//1.5;      // -0.02
    Motor_Speed.ki = 0.15;//0.3;          // 0
    Motor_Speed.kd = 0.25;//0.5;
    // 角度环的PD设定     //抖 还要调
    Roll_Y.kp = 460;                       //420//440
    Roll_Y.ki = 0.450;      //              //0.3//0.45
    Roll_Y.kd = 2.00;                       //2.0//2.10
}
/*************************************************************************
 *  函数名称：void Motor_Speed_Get()
 *  功能说明：获取直流有刷c电机的速度
 *  参数说明：
 *  函数返回：经过平滑滤波后的速度值
 *  修改时间：2023年2月10日 安健
 *  备    注：Motor_Speed_Get();
 *************************************************************************/
//定义滤波参数
float Err_LowOut_last_Y,Encoder_S_Y;
float a_Y=0.7;
float Motor_Speed_Get(void)
{
    /*编码器实际速度换算*/
    float Speed = 0;
    // 顺时针为-，逆时针为+（代表正负号）
    Speed = encoder_get_count(TIM4_ENCODER) * 1.0;
    encoder_clear_count(TIM4_ENCODER);
    X_Distance = Distance_Get_x(Speed,Yaw_Tur);
    Y_Distance = Distance_Get_y(Speed,Yaw_Tur);
    //Distance_Get(Speed, Yaw);
    Speed = Speed * 50 * 2 * 3.14 / 9500;
    //Speed = Speed*2.514453125;//Speed/512*8.2*3.14/0.02;    实际速度值 cm/s
    // 速度的平滑滤波
    //    Speed=Data_Mean_Filter(Speed,speed_Record,SPEED_RECORD_NUM);

    //CT
    float Err_Y,Err_LowOut_Y;
        //计算偏差值
        Err_Y=Speed;
        //低通滤波
        Err_LowOut_Y=(1-a_Y)*Err_Y+a_Y*Err_LowOut_last_Y;
        Err_LowOut_last_Y=Err_LowOut_Y;
        //积分
        Encoder_S_Y+=Err_LowOut_Y;
        //积分限幅
        Encoder_S_Y=Encoder_S_Y>200?200:(Encoder_S_Y<(-200)?(-200):Encoder_S_Y);
    //CT

    return Err_LowOut_Y;
}

/*************************************************************************
 *  函数名称：void Roll_Y_Speed()
 *  功能说明: 前后平衡y轴的角速度环控制
 *  参数说明：@angular_velocity 实际角速度
 *  函数返回：无
 *  修改时间：2024年5月10日 黄春华
 *  备    注：Roll_Y_Aspect();
 *************************************************************************/
////定义滤波参数CT
//float Err_LowOut_last_y,Encoder_S_y;
//float a_y=0.7;
void Roll_Y_speed(float angle_velocity, int *Out)
{
//    //CT//待确认效果
//    float Err_y,Err_LowOut_y;
//        //计算偏差值
//        Err_y=angle_velocity;
//        //低通滤波
//        Err_LowOut_y=(1-a_y)*Err_y+a_y*Err_LowOut_last_y;
//        Err_LowOut_last_y=Err_LowOut_y;
//        //积分
//        Encoder_S_y+=Err_LowOut_y;
//        //积分限幅
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
 *  函数名称：void Roll_Y_Aspect()
 *  功能说明: 前后平衡y轴的角度环控制
 *  参数说明：@angle 实际角度
 *         @angular_velocity 实际角速度
 *  函数返回：无
 *  修改时间：2023年2月10日 安健
 *  备    注：Roll_Y_Aspect();
 *************************************************************************/
void Roll_Y_Aspect(float angle)
{
    Roll_Y.actual = angle;
    Roll_Y.target = Mechanical_zero_y + Motor_Speed.out;

        // 速度环的输出限幅
    Roll_Y.omax =3000;
    Roll_Y.omin = -3000;

        // 位置式PID应用
    PidLocCtrl(&Roll_Y);
    //*Out = (int)AngleControlOut;
}
//void Roll_Y_Aspect(float angle, float angular_velocity, int *Out)
//{
//    float Bias_Angle;
//    float AngleControlOut;
//
//    Bias_Angle = (Mechanical_zero_y + Motor_Speed.out) - angle; // Mechanical_zero_y机械零点的角度值，Motor_Speed.out参与构成串级PID
//
//    // PD控制器得到输出
//    AngleControlOut = Bias_Angle * Roll_Y.kp + (angular_velocity - 6) * Roll_Y.kd; // 14零漂
//
//    //    if(AngleControlOut>=0) AngleControlOut=AngleControlOut+150;                               // 150死区大小
//    //    else                   AngleControlOut=AngleControlOut-150;
//    // 角度环的输出限幅
//    AngleControlOut = Constrain_Data(AngleControlOut, -3000.0, 3000.0);
//
//    Roll_Y.out = AngleControlOut;
//    *Out = (int)AngleControlOut;
//}
/*************************************************************************
 *  函数名称：void Motor_Speed_Aspect(float Speed)
 *  功能说明：速度环调节
 *  参数说明：@Speed :速度实际值
 *  函数返回：返回速度环的 角度 输出
 *  修改时间：2023年2月10日 安健
 *  备    注：每20ms执行一次执行一次调节
 *************************************************************************/
void Motor_Speed_Aspect(float Speed)
{
    // 速度环误差计算，目标速度为0
    Motor_Speed.actual = Speed;
    Motor_Speed.target = Speed_Straght;//Speed_Straght;

    // 速度环的输出限幅
    Motor_Speed.omax = 10;
    Motor_Speed.omin = -10;

    // 位置式PID应用
    PidLocCtrl(&Motor_Speed);
}
/*************************************************************************
*  函数名称：void Brushless_Motor_Init()
*  功能说明：无刷电机初始化
*  参数说明：
*  函数返回：
*  修改时间：2023年1月14日
*  修改作者：安健
*  备    注：需要初始化PWM和编码器口，刹车线，方向控制线
*************************************************************************/
void Brushless_Motor_Init(void)
{
    pwm_init(Brushless_MotorB, 10000, MOTOR_FREQUENCY);                 // 无刷电机 B PWM输出引脚初始化,满占空比停转
    gpio_init(Brushless_MtorDirB, GPO, GPIO_HIGH, GPO_PUSH_PULL);       // B方向线初始化，1为顺时针
    gpio_init(Brushless_MtorStopB, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // 刹车线初始化。输出0刹车

    pwm_init(Brushless_MotorA, 10000, MOTOR_FREQUENCY);                 // 无刷电机 A PWM输出引脚初始化,满占空比停转
    gpio_init(Brushless_MtorDirA, GPO, GPIO_HIGH, GPO_PUSH_PULL);       // A方向线初始化，1为顺时针
    gpio_init(Brushless_MtorStopA, GPO, GPIO_HIGH, GPO_PUSH_PULL);      // 刹车线初始化。输出0刹车

    Momentum_Pid_Init();
}
/*************************************************************************
*  函数名称：void Brushless_Motor_Run()
*  功能说明：无刷电机PWM赋值运行
*  参数说明：
*  函数返回：
*  修改时间：2023年1月15日
*  修改作者：安健
*  备    注：需要初始化PWM的正负需要改变方向线的高低电平
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
        gpio_set_level(Brushless_MtorDirA,0);// A方向逆时针
        A_Puty=0-A_Puty;
    }
    else
    {
        gpio_set_level(Brushless_MtorDirA,1);// A方向顺时针
    }
    if(B_Puty<0)
    {
        gpio_set_level(Brushless_MtorDirB,1);// B方向逆时针
        B_Puty=0-B_Puty;
    }
    else
    {
        gpio_set_level(Brushless_MtorDirB,0);// B方向顺时针
    }
    if(Stop_Car)
    {
        pwm_set_duty(Brushless_MotorB, 0);    // 无刷电机 B PWM输出0
        pwm_set_duty(Brushless_MotorA, 0);    // 无刷电机 A PWM输出0
        gpio_set_level(Brushless_MtorStopA,0);// A刹车
        gpio_set_level(Brushless_MtorStopB,0);// B刹车
    }
    else
    {
        pwm_set_duty(Brushless_MotorB, B_Puty);// 无刷电机 B PWM输出
        pwm_set_duty(Brushless_MotorA, A_Puty);// 无刷电机 A PWM输出
        gpio_set_level(Brushless_MtorStopA,1); // 松开刹车
        gpio_set_level(Brushless_MtorStopB,1); // 松开刹车
    }
}
/*************************************************************************
 *  函数名称：void Momentum_Pid_Init()
 *  功能说明：动量轮的四个PID结构体初始化
 *  参数说明：
 *  函数返回：无
 *  修改时间：2023年1月9日 安健
 *  备    注：All_Pid_Init();
 *************************************************************************/
void Momentum_Pid_Init(void)
{
    PidInit(&Roll_Speed_X);
    PidInit(&Roll_X);
    PidInit(&Wheel_Speed);
    PidInit(&Steering);
    PidInit(&Steering_Speed);

    //直立角速度环(PI)
    Roll_Speed_X.kp = -2.80;//-3.5  -6  -5    //-2.8
    Roll_Speed_X.ki = -1.3;// -1 -1.2 -0.5   //-1.3
    Roll_Speed_X.kd = -0.0;//0

    //直立角度环(PD)
    Roll_X.kp = 150;//139 250    160      //150
    Roll_X.ki = 5.0;//4  2      2.5       //5
    Roll_X.kd = 20;//      0              //20

    //直立速度环
    Wheel_Speed.kp= -0.018;  //0.0005     //0.018
    Wheel_Speed.ki= 0;   //0.0005      0
    Wheel_Speed.kd= 0;

    //转向角度环
    Steering.kp       = 30;     //30
    Steering.ki       = 0;
    Steering.kd       = 0;    //定角度转弯的参数设定

    //转向速度环
    Steering_Speed.kp = 0;
    Steering_Speed.ki = 0;
    Steering_Speed.kd = 0;
}
/*************************************************************************
 *  函数名称：void Wheel_Speed_Get()
 *  功能说明：获取动量轮的速度
 *  参数说明：
 *  函数返回：无
 *  修改时间：2023年1月9日 安健
 *  备    注：All_Pid_Init();
 *************************************************************************/
void Wheel_Speed_Get(void)
{
    /*动量轮实际速度换算*/
    short int Front_Speed=0;
    short int Back_Speed=0;
    // 顺时针为-，逆时针为+（代表正负号）
    Front_Speed=encoder_get_count(TIM5_ENCODER);    //飞轮B
    Back_Speed=encoder_get_count(TIM2_ENCODER);     //飞轮A
}
/*************************************************************************
 *  函数名称：int Roll_Speed_X_Aspect()
 *  功能说明：角速度环调节
 *  参数说明：
 *  @param:actual_g 实际角速度值
 *  函数返回：返回角速度环的PWM输出
 *  修改时间：2023年1月14日 安健
 *  备    注：每2ms执行一次执行一次调节
 *************************************************************************/
void Roll_Speed_X_Aspect(int actual_g,int *Motor_PWM1,int *Motor_PWM2)
{
    // 角速度环误差计算，目标角速度为稳定时的角速度 -18
    Roll_Speed_X.actual=actual_g*1.0;
    Roll_Speed_X.target=Roll_X.out + 2 ;

    // 角速度环的输出限幅
    Roll_Speed_X.omax=8000;
    Roll_Speed_X.omin=-8000;

    // 角速度环的积分累积限幅

    Roll_Speed_X.imax=4000;

    *Motor_PWM1=-(int)PidLocCtrl(&Roll_Speed_X) -(int)Z_angle_out;//- Steering.out ; //1.25     //-
    *Motor_PWM2=-(int)PidLocCtrl(&Roll_Speed_X) +(int)Z_angle_out;//+ Steering.out ; //0.75     //+
    //tft180_show_float (0, 0,PidLocCtrl(&Roll_Speed_X)/*X_Distance*/, 4,4);
    if(*Motor_PWM1>=0) *Motor_PWM1=10000-*Motor_PWM1;         //100死区大小
    else              *Motor_PWM1=-10000-*Motor_PWM1;

    if(*Motor_PWM2>=0) *Motor_PWM2=10000-*Motor_PWM2;         //100死区大小
    else               *Motor_PWM2=-10000-*Motor_PWM2;
}
/*************************************************************************
 *  函数名称：void Roll_Aspect()
 *  功能说明：角度环调节
 *  参数说明：
 *  @param:
 *  函数返回：返回角度环的 角速度 输出
 *  修改时间：2023年1月14日 安健
 *  备    注：每10ms执行一次执行一次调节
 *************************************************************************/
void Roll_X_Aspect(void)
{
    // 角度环误差计算，目标角度为速度环输出
    Roll_X.actual=Roll;//正常
    Roll_X.target= Mechanical_zero_x + X_Dynamic_Zero/*+ X_Dynamic_Zero*/+ Wheel_Speed.out;//机械零点1//正常

    // 角度环的输出限幅
    Roll_X.omax=8000;
    Roll_X.omin=-8000;

    // 位置式PID应用
    PidLocCtrl(&Roll_X);
    tft180_show_float (0, 0,Roll_X.actual-Roll_X.target, 4,4);
    tft180_show_float (0, 20,Roll_X.integrator, 4,4);
    tft180_show_float (0, 40,Roll_X.last_error, 4,4);
    tft180_show_float (0, 60,Roll_X.out_p, 4,4);
    tft180_show_uint (0, 80, Roll_X.out,3);
}
/*************************************************************************
 *  函数名称：void Roll_X_Speed_Aspect()
 *  功能说明：速度环调节
 *  参数说明：
 *  函数返回：返回速度环的 角度 输出
 *  修改时间：2023年1月14日 安健
 *  备    注：每100ms执行一次执行一次调节
 *************************************************************************/

//定义滤波参数CT
float Err_LowOut_last_X,Encoder_S_X;
float a_X=0.7;

void Roll_X_Speed_Aspect(void)
{

    /*动量轮速度获取*/
    short int Front_Speed=0;
    short int Back_Speed=0;
    float Speed=0;
    // 顺时针为-，逆时针为+（代表正负号）
    Front_Speed = encoder_get_count(TIM5_ENCODER);    //飞轮B
    Back_Speed = encoder_get_count(TIM2_ENCODER);     //飞轮A
    //编码器计数清零
    encoder_clear_count(TIM5_ENCODER);
    encoder_clear_count(TIM2_ENCODER);

    B_Speed = Front_Speed;        // 全局变量赋值
    A_Speed = -Back_Speed;

    Speed=(-Back_Speed+Front_Speed)*1.0/2;

    //CT
    //低通滤波
    float Err_X,Err_LowOut_X;
        //计算偏差值
        Err_X=Speed;
        //低通滤波
        Err_LowOut_X=(1-a_X)*Err_X+a_X*Err_LowOut_last_X;
        Err_LowOut_last_X=Err_LowOut_X;
        //积分
        Encoder_S_X+=Err_LowOut_X;
        //积分限幅
        Encoder_S_X=Encoder_S_X>200?200:(Encoder_S_X<(-200)?(-200):Encoder_S_X);
    //CT

    // 速度环误差计算，目标速度为0
    Wheel_Speed.actual = Err_LowOut_X;//Err_LowOut_X
    Wheel_Speed.target=0;

    // 速度环的输出限幅
    Wheel_Speed.omax=200;
    Wheel_Speed.omin=-200;

    // 位置式PID应用
    PidLocCtrl(&Wheel_Speed);

}

void Steering_PID_Init(void)
{
    Steering.kp       = 0;
    Steering.ki       = 0;
    Steering.kd       = 0;    // 指定角度转弯的参数设定
    Steering_Speed.kp = 0;
    Steering_Speed.ki = 0;
    Steering_Speed.kd = 0;
}
/*************************************************************************
 *  函数名称：void Steering_Aspect()
 *  功能说明：转向环调节
 *  参数说明：
 *  函数返回：
 *  修改时间：2023年3月9日 安健
 *  备    注：每ms执行一次执行一次调节
 *************************************************************************/
void Steering_Aspect(float Turn, float target)
{
    //方向环实际值和目标值设定
    Steering.target = target + Steering_Speed.out;
    Steering.actual = Turn;

    // 方向环的输出限幅
    Steering.omax = 5000;
    Steering.omin = -5000;
    // 位置式PID应用
    PidLocCtrl(&Steering);
    Steering.out_d = -Steering.kd*(imu660ra_gyro_z - 1);
    //Steering.out += Steering.out_d;                     // 陀螺仪获取的角速度做为微分项进行PID控制 1为陀螺仪z轴零漂
}
/*************************************************************************
 *  函数名称：void Steering_Speed_Aspect()
 *  功能说明：转向速度外环调节
 *  参数说明：Steering_Speed_Aspect(A_SPeed.B_Speed);
 *  函数返回：
 *  修改时间：2023年5月9日 安健
 *  备    注：每20ms执行一次执行一次调节
 *************************************************************************/
void Steering_Speed_Aspect(float speed1,float speed2)
{

    float Difference_Speed = (speed1 - speed2);
    //方向环实际值和目标值设定
    Steering_Speed.target = 0;
    Steering_Speed.actual = Difference_Speed;

    // 方向环的输出限幅
    Steering_Speed.omax = 100;
    Steering_Speed.omin = -100;
    // 位置式PID应用
    PidLocCtrl(&Steering_Speed);
}

//CT转向环
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

    if (Z_angle_out>0)Z_angle_out=Z_angle_out+(150+3*abs(Z_angle_error));//行进轮与地面摩擦力死区
    if (Z_angle_out<0)Z_angle_out=Z_angle_out-(150+3*abs(Z_angle_error));

    Z_angle_error_last=Z_angle_error;
}
//CT转向速度环 //不启用
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



// 动态零点
void Set_Dynamic_Zero(float line_speed, int MdValue, float* X_angle, float* Y_angle, float Alpha1, float Alpha2)
{
    *X_angle =  Alpha1 * line_speed  * line_speed * (MdValue - 49);
    *Y_angle =  Alpha2 * line_speed *  line_speed * (MdValue - 49);
    *X_angle = Constrain_Data(*X_angle, -8, 8);
    *Y_angle = Constrain_Data(*Y_angle, -8, 8);
}

//积分值清零
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
