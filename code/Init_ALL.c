/*
 * Init_ALL.c
 *
 *  Created on: 2024��4��17��
 *      Author: �ƴ���
 */

#include "Init_All.h"

struct External_equipment Equipments;       //����ṹ��Equipments����
extern char UDPMsg[4096];
extern Once_Dimensional_Kalman_Parameter imu660ra_KalmanFliter_gyro_Z;
//extern float Bias1, Bias2, Bias, Yaw_Bias;
extern int Bias_count;
extern int short_t;
int mode =0, in_mode, out_mode1, out_mode2;

External_equipment Equipments=
{
    1,          //Key����
    0,          //Led
    0,          //Beep������
    1,          //Uart
    1,          //imu660ra
    1,          //TFT/OLED
    0,          //Wifi
    0,          //ADC
    1,          //Brushless_Motor   ��ˢ���
    1,          //Brushless_ENC     ��ˢ�����������ʼ��
    1,          //Brush_Motor_Init  ��ˢ���
    1,          //Brush_ENC         ��ˢ�����������ʼ��
    1,          //Cameral           ����ͷ
    0,          //TOF               TOF
    0,          //����ģ��
    1,          //CCU60_CH0         ��ʱ��
    1,          //CCU60_CH1
    1,          //CCU61_CH0         �ײ������
    0,          //CCU61_CH1
};

/*************************************************************************
*  �������ƣ�void Peripheral_Init()
*  ����˵���������ʼ��
*  ����˵����
*  �������أ�
*  �޸�ʱ�䣺2023��1��13��
*  �޸����ߣ�����
*  ��    ע��ʹ�ò�ͬ����ʱע���޸Ľṹ���ֵ
*************************************************************************/
void Peripheral_Init(void)
{
    // ������ʼ��
    if(Equipments.Key==1)               key_init(10);
    // LED������P10.6��P10.5��ʼ��
    if(Equipments.Led==1)               GPIO_LED_Init();
    // Beep��������P33-10��ʼ��
//    if(Equipments.Led==1)               Beep_Init();
    // ����P14.0�ܽ����,P14.1���룬������115200
    if(Equipments.Uart==1){
        // ��ʼ������0 ������115200 ��������ʹ��P14_0 ��������ʹ��P14_1
        uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);
    }
    if(Equipments.TFO){
        if(Equipments.TFO == 1){
            tft180_set_dir(TFT180_CROSSWISE_180);       //����ģʽ
            tft180_init();                          // TFT180��ʼ��
            tft180_clear();                         // ����
            tft180_show_string(20, 20, "chouxiang");    //��  ��
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
    //Icm20602��ʼ��
    if(Equipments.imu660ra==1){
        int Bias_flag1 = 0, flash_in = 0, flash_out = 0;
        imu660ra_init();
        once_dimensional_kalman_init(&imu660ra_KalmanFliter_gyro_Z,0, 0.1, 1, 0.01, 1, 0.1);
//��һ����ѡ���ܵ��˻���������
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
//��һ����ѡ���ܵ��˻���������
//��һ����ѡ���ܵ�ʵ����Ч����
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
//            flash_buffer_clear();                                                       // ��ջ�����
//            flash_erase_page(FLASH_SECTION_INDEX, 10);                    // ������һҳ
//            flash_union_buffer[0].float_type  = Bias;
//            flash_write_page_from_buffer(FLASH_SECTION_INDEX, 10);
//            flash_in = 0;
//        }
        if(/*flash_out == */1)
        {
            tft180_clear();
            tft180_show_string(20,60,"read flash");
            flash_buffer_clear();
            flash_read_page_to_buffer(FLASH_SECTION_INDEX, 10);           // �����ݴ� flash ��ȡ��������
            Bias = flash_union_buffer[0].float_type;
            flash_out = 0;
        }
    }
//��һ����ѡ���ܵ�ʵ����Ч����
    //TFT��ʼ��
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
//                //UDP_Parameter_Send_Control2(UDPMsg);    //ֱ��node--red����
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

    //��ˢ�����ʼ��(�û��Զ���)
    if(Equipments.Brushless_Motor==1)   Brushless_Motor_Init();
    //��ˢ�����������ʼ��(�û��Զ���)
    if(Equipments.Brushless_ENC==1)
    {
        encoder_dir_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);// ��ˢ���A��������ʼ��
        encoder_dir_init(TIM5_ENCODER, TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);// ��ˢ���B��������ʼ��
    }
    //��ˢֱ�������ʼ��(�û��Զ���)
    if(Equipments.Brush_Motor_init==1)  Brush_Motor_Init();
    //��ˢֱ�������������ʼ��(�û��Զ���)
    if(Equipments.Brush_ENC==1)         encoder_dir_init(TIM4_ENCODER,TIM4_ENCODER_CH1_P02_8, TIM4_ENCODER_CH2_P00_9); //��ˢ�����������ʼ��
    //����ͷ��ʼ��,֡��100֡
    if(Equipments.Cameral==1){
        mt9v03x_init();
        Distance_Int();
    }
    //TOF���ģ���ʼ��
   if(Equipments.TOF==1)
    {
        while(1)
        {
            if(dl1a_init())
                gpio_toggle_level(LED1);                                            // ��ת LED ���������ƽ ���� LED ���� ��ʼ����������ƻ����ĺ���
            else
                break;
            system_delay_ms(1000);                                                  // ���Ʊ�ʾ�쳣
        }
    }
   //�ж�CCU0ͨ��0��ʼ��
       //if(Equipments.CCU60_CH0==1)         pit_init(CCU60_CH0, 2*1000);   //10ms����һ���ж� pit_init(CCU60_CH0, 50*1000);  // 50ms����һ���ж�

       if(!gpio_get_level(P33_11) && !gpio_get_level(P33_10))  // ���뿪��ȫ��Ϊ1ʱ�����˻�����
       {
            short_t = 1;
            save_path();//processMenuLevel1();            // �˻�����
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
    //�ж�CCU0ͨ��0��ʼ��
    //if(Equipments.CCU60_CH0==1)         pit_init(CCU60_CH0, 50*1000);  // 50ms����һ���ж�
    //�ж�CCU0ͨ��1��ʼ��
    if(Equipments.CCU60_CH1==1)         pit_init(CCU60_CH1, 50*1000);   // 50ms����һ���ж�
    //�ж�CCU1ͨ��0��ʼ��
    if(Equipments.CCU61_CH0==1)         pit_init(CCU61_CH0, 2*1000);    // 2ms����һ���ж�
    //�ж�CCU1ͨ��1��ʼ��
    if(Equipments.CCU61_CH1==1)         pit_init(CCU61_CH1, 10*1000);       // 2ms����һ���ж�
}


