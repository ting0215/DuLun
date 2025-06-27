/*
 * Wifi.c
 *
 *  Created on: 2024��4��28��
 *      Author: �ƴ���
 */


#include "Wifi.h"

int Start_Car;   // ������־λ

uint16 data_length;

extern char UDPMsg[4096];//char UDPMsg_C[4096];

extern pid_param_t Motor_Speed; // ǰ��ƽ��y���ٶ��⻷
extern pid_param_t Roll_Y;      // y��Ƕ��ڻ�
extern pid_param_t Wheel_Speed;                    // �������ٶ��⻷
extern pid_param_t Roll_X;                         // �����ֽǶ��ڻ�
extern pid_param_t Roll_Speed_X;                   // �����ֽ��ٶ��ڻ�
extern pid_param_t Steering;                       // ����
extern pid_param_t Steering_Speed;                 // �����ٶȻ�
extern float Mechanical_zero_x;               // x�ᶯ̬��е���ĽǶ�ֵ  ֱ����ROLL��
extern float Mechanical_zero_y;       // y�ᶯ̬��е���ĽǶ�ֵ-4.5    �н���PITCH��

/*
    ��������UDP_Msg_Add
    ���������ʽ��ΪJSON��UDP��Ϣ�ַ��������һ����ֵ��
    ������
        - Msg��ָ��Ҫ��Ӽ�ֵ�Ե���Ϣ�ַ�����ָ��
        - name��ָ������Ƶ�ָ��
        - value����ֵ
        - Is_end����־��ָʾ�˼�ֵ���Ƿ�Ϊ��Ϣ�е����һ����ֵ��
        - data_type����־��ָʾֵ���������ͣ�0��ʾ���ͣ�1��ʾ�����ͣ�
    ����ֵ����
*/
void UDP_Msg_Add(char* Msg, char* name, double value, int Is_end, char data_type)
{
    // �����Ϣ�ַ���Ϊ�գ�����������ź͵�һ����ֵ��
    if(Msg[0] == '')
    {
        sprintf(Msg,"%s{",Msg);
        if(data_type)   sprintf(Msg,"%s\"%s\":%.2f",Msg,name,value); // ��Ӹ�����ֵ
        else            sprintf(Msg,"%s\"%s\":%d",Msg,name,(int)value); // �������ֵ
        if(Is_end) sprintf(Msg,"%s}" ,Msg); // ���������Ϣ�е����һ����ֵ�ԣ�������һ�����
        return;
    }
    // ���������Ϣ�еĵ�һ����ֵ��
    if(!Is_end)
    {
        if(data_type)   sprintf(Msg,"%s,\"%s\":%.2f",Msg,name,value); // ��Ӹ�����ֵ
        else            sprintf(Msg,"%s,\"%s\":%d",Msg,name,(int)value); // �������ֵ
    }
    // �������Ϣ�е����һ����ֵ��
    else
    {
        if(data_type)   sprintf(Msg,"%s,\"%s\":%.2f}",Msg,name,value); // ��Ӹ�����ֵ���һ�����
        else            sprintf(Msg,"%s,\"%s\":%d}",Msg,name,(int)value); // �������ֵ���һ�����
    }
}


/*****************************UDP�����ݷ���**************************/
/*
                                    *
    ���ܣ�UDP��ͼ����Ϣ���� *
    ������Msg Ԥ������Ϣ��         *
    ������Icode ͼ�����
                                    *
*/
void WiFi_UDP_ImgSend(uint8* Icode,uint8* Mid)
{
    int i = 0;
    for(i = 0;i <= 749; i++)
    {
        NO0_SendChar(*(Icode+i));
    }
    for(i = 0;i <= 59; i++)
    {
        NO0_SendChar(*(Mid+i));
    }
//    wifi_uart_send_buffer(Icode, 750);
//    wifi_uart_send_buffer(Mid, 60);
}
/************************************
                                    *
    ���ܣ�UDP��Json��Ϣ���� *
    ������Msg Ԥ������Ϣ��         *
                                    *
*************************************/
void WiFi_UDP_JsonSend(char* Msg)
{
    int i = 0;
    while(*(Msg+i) != '}')
    {
        NO0_SendChar(*(Msg+i++));
    }
    NO0_SendChar('}');
//    uint32 len = strlen(Msg); // ���� Msg ����ĳ���
//    wifi_uart_send_buffer((uint8*)Msg, len);
}
/************************************
                                    *
    ���ܣ�UDP��ͼ���Json��Ϣ�ܷ��� *
    ������Msg Ԥ������Ϣ��         *
    ������Icode ͼ�����
    ������Mid ����
                                    *
*************************************/
void WiFi_UDPSend(char* Msg)
{
    WiFi_UDP_JsonSend(Msg);
}

/*****************************UDP���ݽ��ܲ�����**************************/
/************************************
                                    *
    ���ܣ�����UDP��Json������ֵ����*
    ������Msg �ǽ��յ���Json��Ϣ
    ������name ��Ҫ���ҵ����ͼ���

    ���أ� -1ʧ��  ����Ϊ��õļ�ֵ
                                    *
*************************************/
int UDP_Msg_KeyGet_Int(char* Msg,char* name)
{
    int value = -1;
    char temp_value[128];
    char temp_value_Point;
    char* Pointer;
    memset(temp_value,0,128);
    temp_value_Point = 0;
    Pointer = strstr(Msg,name);
    if(Pointer)
    {
        while(*Pointer != ',' && *Pointer != '}')
        {
            if((*Pointer >= '0' && *Pointer <= '9') || *Pointer == '-')
                temp_value[temp_value_Point++] = *Pointer;
            Pointer++;
        }
        value = atoi(temp_value);
    }
    return value;
}
/************************************
                                    *
    ���ܣ�����UDP��Json��������ֵ����*
    ������Msg �ǽ��յ���Json��Ϣ
    ������name ��Ҫ���ҵĸ����ͼ���
    ���أ� -1ʧ��  ����Ϊ��õļ�ֵ
                                    *
*************************************/
double UDP_Msg_KeyGet_Float(char* Msg,char* name)
{
    double value = -1;
    char temp_value[128];
    char temp_value_Point;
    char* Pointer;
    memset(temp_value,0,128);//��ʼ�� temp_value ȫ����0
    temp_value_Point = 0;
    Pointer = strstr(Msg,name);//+strlen(name)+2;
    if(Pointer)
    {
        while(*Pointer != ',' && *Pointer != '}')
        {
            //printf("%s",Pointer);
            if((*Pointer >= '0' && *Pointer <= '9') || *Pointer == '.' || *Pointer == '-')
                temp_value[temp_value_Point++] = *Pointer;
            Pointer++;
        }
        value = atof(temp_value);
    }
    return value;
}


void UDP_Parameter_Send_Control2(char* Msg)
{
    memset(Msg,0,4096);//��ʼ������ Msg
    UDP_Msg_Add(Msg,"Steering_kp",Steering.kp,0,1);                  //  ת�򻷲���p1
    UDP_Msg_Add(Msg,"Steering_kI",Steering.ki,0,1);                  //  ת�򻷲���p2
    UDP_Msg_Add(Msg,"Steering_kd",Steering.kd,0,1);                  //  ת�򻷲���p3
    UDP_Msg_Add(Msg,"Steering_Speed_kp",Steering_Speed.kp,0,1);                        //  ת��i
    UDP_Msg_Add(Msg,"Steering_Speed_ki",Steering_Speed.ki,0,1);                        //  ת��d
    UDP_Msg_Add(Msg,"Steering_Speed_kd",Steering_Speed.kd,0,1);            //  ֱ�������ٶ�
    UDP_Msg_Add(Msg,"SS",Speed_Straght,1,0);              //  ��������ٶ�
    WiFi_UDP_JsonSend(Msg);
}

/*****************************����Ӧ��ʵ��**************************/
/*
                                    *
    ���ܣ�����UDP�ĳ��ڲ�������ʾ��(���������и���)*
    ������Msg Ԥ���͵�Json��Ϣ��
    ������Icode ͼ���������
    ������Mid ��������
                                    *
*/
void UDP_Parameter_Send_Control(char* Msg)
{
    memset(Msg,0,4096);
    UDP_Msg_Add(Msg,"Roll_Y_kp",Roll_Y.kp,0,1);
    UDP_Msg_Add(Msg,"Roll_Y_kd",Roll_Y.kd,0,1);
    UDP_Msg_Add(Msg,"Motor_Speed_kp",Motor_Speed.kp,0,1);
    UDP_Msg_Add(Msg,"Motor_Speed_ki",Motor_Speed.ki,0,1);
    UDP_Msg_Add(Msg,"Roll_Speed_X_kp",Roll_Speed_X.kp,0,1);
    UDP_Msg_Add(Msg,"Roll_Speed_X_ki",Roll_Speed_X.ki,0,1);
    UDP_Msg_Add(Msg,"Roll_X_kp",Roll_X.kp,0,1);
    UDP_Msg_Add(Msg,"Roll_X_kd",Roll_X.kd,0,1);
    UDP_Msg_Add(Msg,"Wheel_Speed_kp",Wheel_Speed.kp,0,1);
    UDP_Msg_Add(Msg,"Wheel_Speed_ki",Wheel_Speed.ki,0,1);
    UDP_Msg_Add(Msg,"Mechanical_zero_y",Mechanical_zero_y,0,1);
    UDP_Msg_Add(Msg,"Mechanical_zero_x",Mechanical_zero_x,1,1);
    WiFi_UDP_JsonSend(Msg);
}

void UDPMsg_Get2(char* topic)
{
    char Order[256];
    //int temp;
    double ftemp;
    memset(Order,0,256);
    system_delay_ms(50);
    data_length = (uint16)wifi_uart_read_buffer((uint8*)Order,sizeof(Order));
    if(data_length)                                                         // ������յ����� ��������������ж�
    {
        if(strstr(Order,topic))
        {
                ftemp = UDP_Msg_KeyGet_Float(Order,"Steering_kp");
                if(ftemp != -1)
                    Steering.kp = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Steering_ki");
                if(ftemp != -1)
                    Steering.ki = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Steering_kd");
                if(ftemp != -1)
                    Steering.kd = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Steering_Speed_kp");
                if(ftemp != -1)
                    Steering_Speed.kp = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Steering_Speed_ki");
                if(ftemp != -1)
                    Steering_Speed.ki = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Steering_Speed_kd");
                if(ftemp != -1)
                    Steering_Speed.kd = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"SS");
                if(ftemp != -1)
                    Speed_Straght =  ftemp;
                // �Ƿ�Ҫ������д��flash
//                temp = UDP_Msg_KeyGet_Int(Order,"WF");
//                if(temp != -1)
//                {
//                    Write_Flash();
//                }
            }
        }
}

/*
                                    *
    ���ܣ�����UDP�ĳ��ڲ�������ʾ��(���������и���)*
    ������Msg Ԥ���͵�Json��Ϣ��
    ������Icode ͼ���������
    ������Mid ��������
                                    *
*/
void UDP_Parameter_Send_Carrun(char* Msg)
{
    memset(Msg,0,4096);
    UDP_Msg_Add(Msg,"ROLL",Roll,0,1);
    UDP_Msg_Add(Msg,"PITCH",Pitch,0,1);
    UDP_Msg_Add(Msg,"YAW",Yaw_Tur,0,1);
    UDP_Msg_Add(Msg,"PA",0,0,0);
    UDP_Msg_Add(Msg,"SA",A_Speed,0,0);
    UDP_Msg_Add(Msg,"PB",0,0,0);
    UDP_Msg_Add(Msg,"SB",B_Speed,0,0);
    UDP_Msg_Add(Msg,"PC",0,0,0);
    UDP_Msg_Add(Msg,"SC",C_Speed,1,1);
    WiFi_UDP_JsonSend(Msg);
}

// /*
//  * @brief ��ֵ��ͼ���������ESP8266���Ͷ�ֵ��ͼ��
//  * @parma Image_Cod ����������
//  * @param Bin_Image ��ֵ������
//  * @return ��
//  * @see Image_Coding();
//  * @author ������
//  * @date 2021/5/14
//  * @�޸�����:2022/6/28   2023/3/3
//  * @�޸���:������
//  *
//  * */
// void Image_Coding(uint8 binImage[ImgH][ImgW], uint8 Image_Cod[ImgH * ImgW / 8])
// {
//     int k = 0;               // ѹ�����ѭ������
//     unsigned char i, j;      // ��ֵ��ѭ������
//     unsigned char index = 0; // ��λ����
//     for (i = 0; i < ImgH; i++)
//     {
//         for (j = 0; j < ImgW; j++)
//         {
//             if (index < 8)
//             {
//                 Image_Cod[k] = (Image_Cod[k] << 1) + binImage[i][j]; // ����һλ
//                 index++;
//             }
//             else
//             {
//                 index = 0;
//                 k++;
//                 Image_Cod[k] = (Image_Cod[k] << 1) + binImage[i][j]; // ����һλ
//                 index++;
//             }


/*
                                    *
    ���ܣ�����UDP��VOFA+�������*
    ������data1-4 Ҫ���͵��ĸ�����
                                    *
*/
void UDP_Send_double(float data1, float data2, float data3, float data4)
{
    char send_str[128];
    sprintf(send_str, "%.2f,%.2f,%.2f,%.2f\r\n",data1,data2,data3,data4);
    NO0_SendString( send_str);
}

/*
                                    *
    ���ܣ�����UDP����λ�����ݽ���ʾ��(���������и���)*
    ������topic ���յ����ݵ�֡ͷ
                                    *
*/
void UDPMsg_Get(char* topic)
{
    char Order[256];
    //int temp;
    double ftemp;
    memset(Order,0,256);//�� Order ָ����ڴ������ǰ 256 ���ֽ�ȫ������Ϊ 0 ��ֵ  ��ʼ��Order����
    system_delay_ms(50);
    data_length = (uint16)wifi_uart_read_buffer((uint8*)Order,sizeof(Order));
    if(data_length)                                                         // ������յ����� ��������������ж�
    {
        if(strstr(Order,topic))
        {
                ftemp = UDP_Msg_KeyGet_Float(Order,"Roll_Y_kp")*1.0;
                if(ftemp != -1)
                    Roll_Y.kp = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Roll_Y_kd")*1.0;
                if(ftemp != -1)
                    Roll_Y.kd = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Motor_Speed_kp")*1.0;
                if(ftemp != -1)
                    Motor_Speed.kp = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Motor_Speed_ki")*1.0;
                if(ftemp != -1)
                    Motor_Speed.ki = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Roll_Speed_X_kp")*1.0;
                if(ftemp != -1)
                    Roll_Speed_X.kp =  ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Roll_Speed_X_ki")*1.0;
                if(ftemp != -1)
                    Roll_Speed_X.ki = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Roll_X_kp")*1.0;
                if(ftemp != -1)
                    Roll_X.kp = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Roll_X_kd")*1.0;
                if(ftemp != -1)
                    Roll_X.kd = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Wheel_Speed_kp")*1.0;
                if(ftemp != -1)
                    Wheel_Speed.kp = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Wheel_Speed_ki")*1.0;
                if(ftemp != -1)
                    Wheel_Speed.ki = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Mechanical_zero_x")*1.0;
                if(ftemp != -1)
                    Mechanical_zero_x = ftemp;
                ftemp = UDP_Msg_KeyGet_Float(Order,"Mechanical_zero_y")*1.0;
                if(ftemp != -1)
                    Mechanical_zero_y = ftemp;
               // temp = UDP_Msg_KeyGet_Int(Order,"WF");
                //if(temp != -1)
                //{
                 //  Write_Flash();
               // }
            }
        }
}
/**
 * @brief ����UDP���Ӻ���
 * @para: mode=1 ������vofa+��λ����ʾ����udp����
 *        mode=2 ������node-red��λ����UDP����
 */
void Wifi_UDP_Connect(int mode)
{
    int Protect_time=10;    //����ı���ʱ��
    tft180_show_float (0, 40,0.81, 4,4);
    while(wifi_uart_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST, WIFI_UART_STATION) )
    {
        system_delay_ms(500);                                                   // ��ʼ��ʧ�� �ȴ� 500ms
        Protect_time--;
        if(Protect_time<=0)     break;
    }
    tft180_show_float (0, 20,0.82, 4,4);
    // zf_device_wifi_uart.h �ļ��ڵĺ궨����Ը���ģ������(����) WIFI ֮���Ƿ��Զ����� TCP ������������ UDP ���ӡ����� TCP �������Ȳ���
    if(2 != WIFI_UART_AUTO_CONNECT)                                             // ���û�п����Զ����� ����Ҫ�ֶ�����Ŀ�� IP
    {
        if(mode == 2)
        {
            if(wifi_uart_connect_udp_client(                                        // ��ָ��Ŀ�� IP �Ķ˿ڽ��� UDP ����
                WIFI_UART_TARGET_IP_NODE_RED_VOFA,                                                // ����ʹ�����Զ�����ʱһ����Ŀ�� IP ʵ��ʹ��ʱҲ����ֱ����дĿ�� IP �ַ���
                WIFI_UART_TARGET_PORT_NODE_RED,                                              // ����ʹ�����Զ�����ʱһ����Ŀ��˿� ʵ��ʹ��ʱҲ����ֱ����дĿ��˿��ַ���
                WIFI_UART_LOCAL_PORT,                                               // ����ʹ�����Զ�����ʱһ���ı��ض˿� Ҳ�����Լ�ͨ��ʹ�õĶ˿ں�  ʵ��ʹ��ʱҲ����ֱ����д�����ı��ض˿��ַ���
                WIFI_UART_SERIANET))                                                // ���������ģʽ ��Ȼ����Ըĳ�͸��ģʽ ʵ���ϲ�𲢲��Ǻܴ�
            {
                // ���һֱ����ʧ�� ����һ���ǲ���û�н�Ӳ����λ
                tft180_show_string(0, 0, "UDP Con error");
//                oled_show_string(0, 0, "UDP Con error");
                system_delay_ms(500);                                               // ��������ʧ�� �ȴ� 500ms
            }
        }
        else
        {
            if(wifi_uart_connect_udp_client(                                        // ��ָ��Ŀ�� IP �Ķ˿ڽ��� UDP ����
                WIFI_UART_TARGET_IP_NODE_RED_VOFA,                                                // ����ʹ�����Զ�����ʱһ����Ŀ�� IP ʵ��ʹ��ʱҲ����ֱ����дĿ�� IP �ַ���
                WIFI_UART_TARGET_PORT_VOFA,                                              // ����ʹ�����Զ�����ʱһ����Ŀ��˿� ʵ��ʹ��ʱҲ����ֱ����дĿ��˿��ַ���
                WIFI_UART_LOCAL_PORT,                                               // ����ʹ�����Զ�����ʱһ���ı��ض˿� Ҳ�����Լ�ͨ��ʹ�õĶ˿ں�  ʵ��ʹ��ʱҲ����ֱ����д�����ı��ض˿��ַ���
                WIFI_UART_SERIANET))                                                // ���������ģʽ ��Ȼ����Ըĳ�͸��ģʽ ʵ���ϲ�𲢲��Ǻܴ�
            {
                // ���һֱ����ʧ�� ����һ���ǲ���û�н�Ӳ����λ
                tft180_show_string(0, 0, "UDP Con error");
//                oled_show_string(0, 0, "UDP Con error");
                system_delay_ms(500);                                               // ��������ʧ�� �ȴ� 500ms
            }
        }
    }
//    oled_show_string(0, 0, (char*)wifi_uart_information.wifi_uart_local_ip);
//    oled_show_string(0,2,(char*)wifi_uart_information.wifi_uart_local_port);
    tft180_show_string(0, 15, (char*)wifi_uart_information.wifi_uart_local_ip);
    tft180_show_string(0,30,(char*)wifi_uart_information.wifi_uart_local_port);

}

/*
 * ��������Param_Send_Set
 * ���ܣ������ڵĲ��ֿ��Ʋ�����������λ������ͨ��UDPԶ���޸Ĳ���ģʽ���޸Ĳ�����
 * ������
 *     ConFlag��UDPԶ���޸Ĳ���ģʽ��־��1��ʾ������0��ʾ�رա�
 * ����ֵ���ޡ�
 */
void Param_Send_Set(int ConFlag)
{
    uint8 i;
    uint8 protect_times=3;
    //�����ڵĲ��ֿ��Ʋ�����������λ��
    for(i=0;i<protect_times;i++)
    {
        system_delay_ms(25);
        UDP_Parameter_Send_Control(UDPMsg);
        system_delay_ms(25);
        tft180_show_string(0,50,"Send OK");
    }
    //udpԶ���޸Ĳ���ģʽ
    if(ConFlag==1)
    {
        //��Start_CarΪ0ʱ��ѭ��ִ�����²���
        while(!Start_Car)
        {
            UDPMsg_Get("DULUN");                // ͨ��UDP����ָ����ʽ������

            key_scanner();                      // ɨ�谴��״̬
            if(key_get_state(KEY_1))            // �жϰ����Ƿ񱻰���
            {
                Start_Car=1;                    // �޸�Start_Car������ֵΪ1���˳�ѭ��
            }
        }
    }
    tft180_clear();                             // TFT����
}

void Write_Flash(void)
{
    flash_buffer_clear();                                                       // ��ջ�����
    flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                    // ������һҳ
    flash_union_buffer[0].float_type  = Mechanical_zero_x;
    flash_union_buffer[1].float_type  = Mechanical_zero_y;
    flash_union_buffer[2].float_type  = Roll_Y.kp;
    flash_union_buffer[3].float_type  = Roll_Y.kd;
    flash_union_buffer[4].float_type  = Motor_Speed.kp;
    flash_union_buffer[5].float_type  = Motor_Speed.ki;
    flash_union_buffer[6].float_type  = Roll_Speed_X.kp;
    flash_union_buffer[7].float_type  = Roll_Speed_X.ki;
    flash_union_buffer[8].float_type  = Roll_X.kp;
    flash_union_buffer[9].float_type  = Roll_X.kd;
    flash_union_buffer[10].float_type  = Wheel_Speed.kp;
    flash_union_buffer[11].float_type  = Wheel_Speed.ki;
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // ��ָ�� Flash ������ҳ��д�뻺��������
}


void Read_Flash(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // �����ݴ� flash ��ȡ��������
    Mechanical_zero_x = flash_union_buffer[0].float_type;
    Mechanical_zero_y = flash_union_buffer[1].float_type;
    Roll_Y.kp = flash_union_buffer[2].float_type;
    Roll_Y.kd = flash_union_buffer[3].float_type;
    Motor_Speed.kp = flash_union_buffer[4].float_type;
    Motor_Speed.ki = flash_union_buffer[5].float_type;
    Roll_Speed_X.kp = flash_union_buffer[6].float_type;
    Roll_Speed_X.ki = flash_union_buffer[7].float_type;
    Roll_X.kp = flash_union_buffer[8].float_type;
    Roll_X.kd = flash_union_buffer[9].float_type;
    Wheel_Speed.kp = flash_union_buffer[10].float_type;
    Wheel_Speed.ki = flash_union_buffer[11].float_type;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//��������ֱ�ӷ��� ʹ��ʾ��
////UDP_OUT_Int4(UDPMsg, &A, NULL, NULL, NULL);
//����ٴ���дNULLռλ,NULL�Ӻ���ǰ��
void UDP_OUT_Int4(char* Msg, int *Test1, int *Test2, int *Test3, int *Test4){
    system_delay_ms(60);//���η������ݼ������Ҫ�����
    memset(Msg,0,4096);//��ʼ��
    int i = (Test1 != NULL) + (Test2 != NULL) + (Test3 != NULL) + (Test4 != NULL);
    if(i){
        if(i)UDP_Msg_Add(Msg,"ITO",(float)*Test1,!(i - 1),0), i--;              //Test
        if(i)UDP_Msg_Add(Msg,"ITT",(float)*Test2,!(i - 1),0), i--;              //Test
        if(i)UDP_Msg_Add(Msg,"ITS",(float)*Test3,!(i - 1),0), i--;              //Test
        if(i)UDP_Msg_Add(Msg,"ITF",(float)*Test4,!(i - 1),0);                   //Test
        WiFi_UDP_JsonSend(Msg);
    }
}

//UDP_In_Int4("TTT",&A, NULL, NULL, NULL);
//����ٴ���дNULLռλ,NULL�Ӻ���ǰ��
void UDP_In_Int4(char* topic, int *Test1, int *Test2, int *Test3, int *Test4){
    char Order[256];
    double ftemp;
    memset(Order,0,256);
    system_delay_ms(50);
    data_length = (uint16)wifi_uart_read_buffer((uint8*)Order,sizeof(Order));
    int i = (Test1 != NULL) + (Test2 != NULL) + (Test3 != NULL) + (Test4 != NULL);
    if(i){
        if(data_length){
            if(strstr(Order,topic)){
                ftemp = UDP_Msg_KeyGet_Float(Order,"ITO")*1.0;
                tft180_show_float (20, 100,ftemp, 4,4);
                if(i&&(ftemp != -1))*Test1 = ftemp, i--;

                ftemp = UDP_Msg_KeyGet_Float(Order,"ITT")*1.0;
                if(i&&(ftemp != -1))*Test2 = ftemp, i--;

                ftemp = UDP_Msg_KeyGet_Float(Order,"ITS")*1.0;
                if(i&&(ftemp != -1))*Test3 = ftemp, i--;

                ftemp = UDP_Msg_KeyGet_Float(Order,"ITF")*1.0;
                if(i&&(ftemp != -1))*Test4 = ftemp, i--;
            }
        }
    }
}

//��������ֱ�ӷ��� ʹ��ʾ��
//UDP_OUT_Float4(UDPMsg, &A, NULL, NULL, NULL);
//����ٴ���дNULLռλ,NULL�Ӻ���ǰ��
void UDP_OUT_Float4(char* Msg, float *Test1, float *Test2, float *Test3, float *Test4){
    system_delay_ms(60);//���η������ݼ������Ҫ�����
    memset(Msg,0,4096);//��ʼ��
    int i = (Test1 != NULL) + (Test2 != NULL) + (Test3 != NULL) + (Test4 != NULL);
    if(i){
        if(i) UDP_Msg_Add(Msg,"FTO",*Test1,!(i - 1),1), i--;                          //Test
        if(i) UDP_Msg_Add(Msg,"FTT",*Test2,!(i - 1),1), i--;                          //Test
        if(i) UDP_Msg_Add(Msg,"FTS",*Test3,!(i - 1),1), i--;                          //Test
        if(i) UDP_Msg_Add(Msg,"FTF",*Test4,!(i - 1),1);                       //Test
        WiFi_UDP_JsonSend(Msg);
    }
}

//��������ֱ�ӽ��� ʹ��ʾ��
//UDP_In_Float4("TTT", &A, NULL, NULL, NULL);
//����ٴ���дNULLռλ,NULL�Ӻ���ǰ��
void UDP_In_Float4(char* topic, float *Test1, float *Test2, float *Test3, float *Test4){
    char Order[256];
    double ftemp;
    memset(Order,0,256);
    system_delay_ms(50);
    data_length = (uint16)wifi_uart_read_buffer((uint8*)Order,sizeof(Order));
    int i = (Test1 != NULL) + (Test2 != NULL) + (Test3 != NULL) + (Test4 != NULL);
    if(i){
        if(data_length){
            if(strstr(Order,topic)){
                ftemp = UDP_Msg_KeyGet_Float(Order,"FTO")*1.0;
                tft180_show_float (20, 100,ftemp, 4,4);
                if(i&&(ftemp != -1))*Test1 = ftemp, i--;

                ftemp = UDP_Msg_KeyGet_Float(Order,"FTT")*1.0;
                if(i&&(ftemp != -1))*Test2 = ftemp, i--;

                ftemp = UDP_Msg_KeyGet_Float(Order,"FTS")*1.0;
                if(i&&(ftemp != -1))*Test3 = ftemp, i--;

                ftemp = UDP_Msg_KeyGet_Float(Order,"FTF")*1.0;
                if(i&&(ftemp != -1))*Test4 = ftemp, i--;
            }
        }
    }
}

//Test ʹ�ô���ֱ���޸�
//Test���������޸�Ϊ�������� ���������޸�Test����
//UDP_FTest(UDPMsg,Test)
void UDP_FTest(char* Msg, float Test){
    system_delay_ms(60);//���η������ݼ������Ҫ�����
    memset(Msg,0,4096);//��ʼ��
    UDP_Msg_Add(Msg,"Test",Test,1, 1);                   //Test
    WiFi_UDP_JsonSend(Msg);
}

//Test ʹ�ô���ֱ���޸�
//Test���������޸�Ϊ�������� ���������޸�Test����
//UDP_ITest(UDPMsg,Test)
void UDP_ITest(char* Msg, int Test){
    system_delay_ms(60);//���η������ݼ������Ҫ�����
    memset(Msg,0,4096);//��ʼ��
    UDP_Msg_Add(Msg,"Test", (float)Test, 1, 0);                  //Test
    WiFi_UDP_JsonSend(Msg);
}

//Car
void UDP_Car(char* Msg){
    system_delay_ms(60);//���η������ݼ������Ҫ�����
    memset(Msg,0,4096);
    snprintf(Msg, sizeof(Msg), "{\"Stop_Car\": \"%c\", \"Start_Car_Para\": \"%c\"}", Stop_Car, Start_Car_Para); //
    WiFi_UDP_JsonSend(Msg);
}

//IP
void UDP_IP(char* Msg){
    system_delay_ms(60);//���η������ݼ������Ҫ�����
    memset(Msg,0,4096);
    //printf("IP:\n");//(QwQ)
    char ip_str[17];
    for (int i = 0; i < 16; i++){
        ip_str[i] = (char)wifi_uart_information.wifi_uart_local_ip[i];
        //printf("%c", ip_str[i]);//(QwQ)
        if (ip_str[i] == '\0'){
            //printf("\nBreak\n");//(QwQ)
            break;
        }  // ������ֹ����ǰ����
    }
    ip_str[16] = '\0';  // ȷ���ַ�����ֹ
    snprintf(Msg,4096,"{\"wifi_uart_local_ip\": \"%s\"}",ip_str);//����תJSON
    WiFi_UDP_JsonSend(Msg);
    //printf("WIFIEND\n");//(QwQ)
    UDP_ITest(Msg,1);
    UDP_Data(Msg);
}

//�������ݶ�ȡ���
//�����������޸ı���ֵ
//��֡ͷ ��������������
void UDPMsg_ReadTest(){
    int Test = 0;
    char Order[256];
    //int temp;
    double ftemp;
    memset(Order,0,256);//��ʼ������
    system_delay_ms(50);
    data_length = (uint16)wifi_uart_read_buffer((uint8*)Order,sizeof(Order));//��ȡ���鳤��
    if(data_length){
        ftemp = UDP_Msg_KeyGet_Float(Order,"SG");
        if(ftemp != -1){
            Test = ftemp;
            UDP_ITest(UDPMsg,Test);
            UDP_Data(UDPMsg);
        }
    }
}

uint8_t Data = 0;
//�������ڿ��Ƴ�ģʽ 0 �ر� 1-ֱ�� 2-�ж�
//����δʵ�� ��Ԥ������
//UDP_Data(UDPMsg,1)
void UDP_Data(char* Msg){
    system_delay_ms(60);//���η������ݼ������Ҫ�����
    memset(Msg,0,4096);//��ʼ��
    UDP_Msg_Add(Msg,"Data", (float)Data, 1, 0);                  //Test
    WiFi_UDP_JsonSend(Msg);
}

void UDP_Control(){
    if(Data == 0){
        Stop_Car = 1;//��ȫֹͣ
    }
    if(Data == 1){
        Stop_Car = 2;//ֱ��
    }
    if(Data == 2){
        Stop_Car = 2;//ǰ��
    }
}

//UDPMsg_A("AAA")
//Data
void UDPMsg_A(char* topic){
    char Order[256];
    double ftemp;
    memset(Order,0,256);
    system_delay_ms(50);
    data_length = (uint16)wifi_uart_read_buffer((uint8*)Order,sizeof(Order));
    if(data_length){
        if(strstr(Order,topic)){
            ftemp = UDP_Msg_KeyGet_Float(Order,"Data")*1.0;
            if(ftemp != -1)Data = ftemp;
            UDP_Control();
        }
    }
    UDP_Control();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

