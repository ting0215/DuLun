/*
 * Init_ALL.c
 *
 *  Created on: 2024年4月17日
 *      Author: 黄春华
 */

#include "Init_All.h"

struct External_equipment Equipments;       //外设结构体Equipments定义
extern char UDPMsg[4096];
extern Once_Dimensional_Kalman_Parameter imu660ra_KalmanFliter_gyro_Z;
//extern float Bias1, Bias2, Bias, Yaw_Bias;
extern int Bias_count;
extern int short_t;
int mode =0, in_mode, out_mode1, out_mode2;

External_equipment Equipments=
{
    1,          //Key按键
    0,          //Led
    0,          //Beep蜂鸣器
    1,          //Uart
    1,          //imu660ra
    1,          //TFT/OLED
    0,          //Wifi
    0,          //ADC
    1,          //Brushless_Motor   无刷电机
    1,          //Brushless_ENC     无刷电机编码器初始化
    1,          //Brush_Motor_Init  有刷电机
    1,          //Brush_ENC         有刷电机编码器初始化
    1,          //Cameral           摄像头
    0,          //TOF               TOF
    0,          //光流模块
    1,          //CCU60_CH0         计时用
    1,          //CCU60_CH1
    1,          //CCU61_CH0         底层控制用
    0,          //CCU61_CH1
};

/*************************************************************************
*  函数名称：void Peripheral_Init()
*  功能说明：外设初始化
*  参数说明：
*  函数返回：
*  修改时间：2023年1月13日
*  修改作者：安健
*  备    注：使用不同外设时注意修改结构体的值
*************************************************************************/
void Peripheral_Init(void)
{
    // 按键初始化
    if(Equipments.Key==1)               key_init(10);
    // LED灯所用P10.6和P10.5初始化
    if(Equipments.Led==1)               GPIO_LED_Init();
    // Beep蜂鸣器的P33-10初始化
//    if(Equipments.Led==1)               Beep_Init();
    // 串口P14.0管脚输出,P14.1输入，波特率115200
    if(Equipments.Uart==1){
        // 初始化串口0 波特率115200 发送引脚使用P14_0 接收引脚使用P14_1
        uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);
    }
    if(Equipments.TFO){
        if(Equipments.TFO == 1){
            tft180_set_dir(TFT180_CROSSWISE_180);       //横屏模式
            tft180_init();                          // TFT180初始化
            tft180_clear();                         // 清屏
            tft180_show_string(20, 20, "chouxiang");    //列  行
        }
        else{
            oled_init();
            //oled_show_string(25, 7, "chouxiang");
        }
    }
    if(mode == 1){
        while(1){
            key_scanner();
            system_delay_ms(50);
            tft180_clear();
            tft180_show_string(0, 20, "1:Out1");
            tft180_show_string(0, 40, "2:Out2");
            tft180_show_string(0, 60, "3:in");
            if(KEY_detect(KEY_1)){
                out_mode1 = 1;
                tft180_clear();
                tft180_show_string(0, 20, "Out1 OK");
                break;
            }
            else if(KEY_detect(KEY_2)){
                out_mode2 = 1;
                tft180_clear();
                tft180_show_string(0, 20, "Out2 OK");
                break;
            }
            else if(KEY_detect(KEY_3)){
                in_mode = 1;
                tft180_clear();
                tft180_show_string(0, 20, "in OK");
                break;
            }
        }
    }
    //Icm20602初始化
    if(Equipments.imu660ra==1){
        int Bias_flag1 = 0, flash_in = 0, flash_out = 0;
        imu660ra_init();
        once_dimensional_kalman_init(&imu660ra_KalmanFliter_gyro_Z,0, 0.1, 1, 0.01, 1, 0.1);
//这一段是选择功能的人机交互部分
//        while(1){
//            key_scanner();
//            system_delay_ms(50);
//            if(KEY_detect(KEY_1))
//            {
//                system_delay_ms(100);
//                Bias_flag1 = 1;
//                break;
//            }
//            if(KEY_detect(KEY_2))
//            {
//                system_delay_ms(100);
//                flash_out = 1;
//                break;
//            }
//        }
//这一段是选择功能的人机交互部分
//这一段是选择功能的实际生效部分
//        if(Bias_flag1 == 1)
//        {
//            tft180_clear();
//            system_delay_ms(2000);
//            pit_init(CCU61_CH1, 2*1000);
//            while(Bias_count < 1000);
//
//            Bias1 = Yaw_Bias1;
//            system_delay_ms(2000);
//
//            while(Bias_count < 10000);
//
//            Bias2 = Yaw_Bias1;
//            Bias = (Bias2 - Bias1) / 2000;
//             char display_str2[] = "";
//             sprintf(display_str2,"Bias: %.5f",Bias);
//             tft180_show_string(20,20,display_str2);
//             pit_close(CCU61_CH1);
//             Bias_flag1 = 0;
//             flash_in = 1;
//         }
//        if(flash_in == 1)
//        {
//            tft180_show_string(20,60,"write flash");
//            flash_buffer_clear();                                                       // 清空缓冲区
//            flash_erase_page(FLASH_SECTION_INDEX, 10);                    // 擦除这一页
//            flash_union_buffer[0].float_type  = Bias;
//            flash_write_page_from_buffer(FLASH_SECTION_INDEX, 10);
//            flash_in = 0;
//        }
        if(/*flash_out == */1)
        {
            tft180_clear();
            tft180_show_string(20,60,"read flash");
            flash_buffer_clear();
            flash_read_page_to_buffer(FLASH_SECTION_INDEX, 10);           // 将数据从 flash 读取到缓冲区
            Bias = flash_union_buffer[0].float_type;
            flash_out = 0;
        }
    }
//这一段是选择功能的实际生效部分
    //TFT初始化
    if(Equipments.Wifi)
    {
        system_delay_ms(50);
        tft180_clear();
        tft180_show_string(0, 0, "WIFI INT");
        Equipments.Wifi = 1;
        Wifi_UDP_Connect(2);
        UDP_IP(UDPMsg);
        tft180_clear();
//        while(0)
//        {
//            system_delay_ms(50);
//            tft180_clear();
//            tft180_show_string(20, 20, "Wating Choose");
//            if(KEY_detect(KEY_1))       //vofa+
//            {
//                Equipments.Wifi = 2;
//                tft180_clear();
//                Wifi_UDP_Connect(1);
//                //Param_Send_Set(2);
//                break;
//            }
//            else if(KEY_detect(KEY_2))       //node-red
//            {
//                Equipments.Wifi = 1;
//                tft180_clear();
//                Wifi_UDP_Connect(2);
////                Momentum_Pid_Init();
////                Motor_Pid_Init();
//                //UDP_Parameter_Send_Control2(UDPMsg);    //直立node--red调参
//                //Read_Flash();
//                UDP_Parameter_Send_Control(UDPMsg);
//                //Param_Send_Set(1);
//                break;
//            }
//            else if(KEY_detect(KEY_3))       //no_wifi
//            {
//                Equipments.Wifi = 0;
//                tft180_clear();
//                Wifi_UDP_Connect(3);
//                //Param_Send_Set(3);
//                break;
//            }
//        }

    }

    //无刷电机初始化(用户自定义)
    if(Equipments.Brushless_Motor==1)   Brushless_Motor_Init();
    //无刷电机编码器初始化(用户自定义)
    if(Equipments.Brushless_ENC==1)
    {
        encoder_dir_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);// 无刷电机A编码器初始化
        encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);// 无刷电机B编码器初始化
    }
    //有刷直流电机初始化(用户自定义)
    if(Equipments.Brush_Motor_init==1)  Brush_Motor_Init();
    //有刷直流电机编码器初始化(用户自定义)
    if(Equipments.Brush_ENC==1)         encoder_dir_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9); //有刷电机编码器初始化
    //摄像头初始化,帧率100帧
    if(Equipments.Cameral==1){
        mt9v03x_init();
        Distance_Int();
    }
    //TOF测距模块初始化
   if(Equipments.TOF==1)
    {
        while(1)
        {
            if(dl1a_init())
                gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
            else
                break;
            system_delay_ms(1000);                                                  // 闪灯表示异常
        }
    }
   //中断CCU0通道0初始化
       //if(Equipments.CCU60_CH0==1)         pit_init(CCU60_CH0, 2*1000);   //10ms进入一次中断 pit_init(CCU60_CH0, 50*1000);  // 50ms进入一次中断

       if(!gpio_get_level(P33_11) && !gpio_get_level(P33_10))  // 拨码开关全在为1时进入人机交互
       {
            short_t = 1;
            save_path();//processMenuLevel1();            // 人机交互
            short_t = 0;
       }
       putin();
       //tft180_clear();
       tft180_show_string(0,80,"finished");
       char display_str4[] = "";
       sprintf(display_str4,"num: %d",numpath);
       tft180_show_string(20,40,display_str4);
       if(Equipments.CCU60_CH0==1)
       {
           //pit_close(CCU60_CH0);
           //pit_init(CCU60_CH0, 50*1000);
           //tft180_show_float (60, 40,Z_angle_out/*Y_Distance*/, 4,4);
       }
   /*
    if(Equipments.GuangLiu==1)
    {
        Mini_pixel_flow_init();
        uart_init(UART_3,19200,UART3_TX_P15_6,UART3_RX_P15_7);
    }
    */
    //中断CCU0通道0初始化
    //if(Equipments.CCU60_CH0==1)         pit_init(CCU60_CH0, 50*1000);  // 50ms进入一次中断
    //中断CCU0通道1初始化
    if(Equipments.CCU60_CH1==1)         pit_init(CCU60_CH1, 50*1000);   // 50ms进入一次中断
    //中断CCU1通道0初始化
    if(Equipments.CCU61_CH0==1)         pit_init(CCU61_CH0, 2*1000);    // 2ms进入一次中断
    //中断CCU1通道1初始化
    if(Equipments.CCU61_CH1==1)         pit_init(CCU61_CH1, 10*1000);       // 2ms进入一次中断
}


