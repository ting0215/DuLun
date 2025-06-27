/*
 * Wifi.c
 *
 *  Created on: 2024年4月28日
 *      Author: 黄春华
 */


#include "Wifi.h"

int Start_Car;   // 发车标志位

uint16 data_length;

extern char UDPMsg[4096];//char UDPMsg_C[4096];

extern pid_param_t Motor_Speed; // 前后平衡y轴速度外环
extern pid_param_t Roll_Y;      // y轴角度内环
extern pid_param_t Wheel_Speed;                    // 动量轮速度外环
extern pid_param_t Roll_X;                         // 动量轮角度内环
extern pid_param_t Roll_Speed_X;                   // 动量轮角速度内环
extern pid_param_t Steering;                       // 方向环
extern pid_param_t Steering_Speed;                 // 方向环速度环
extern float Mechanical_zero_x;               // x轴动态机械零点的角度值  直立（ROLL）
extern float Mechanical_zero_y;       // y轴动态机械零点的角度值-4.5    行进（PITCH）

/*
    函数名：UDP_Msg_Add
    描述：向格式化为JSON的UDP消息字符串中添加一个键值对
    参数：
        - Msg：指向要添加键值对的消息字符串的指针
        - name：指向键名称的指针
        - value：键值
        - Is_end：标志，指示此键值对是否为消息中的最后一个键值对
        - data_type：标志，指示值的数据类型（0表示整型，1表示浮点型）
    返回值：无
*/
void UDP_Msg_Add(char* Msg, char* name, double value, int Is_end, char data_type)
{
    // 如果消息字符串为空，则添加左花括号和第一个键值对
    if(Msg[0] == '')
    {
        sprintf(Msg,"%s{",Msg);
        if(data_type)   sprintf(Msg,"%s\"%s\":%.2f",Msg,name,value); // 添加浮点型值
        else            sprintf(Msg,"%s\"%s\":%d",Msg,name,(int)value); // 添加整型值
        if(Is_end) sprintf(Msg,"%s}" ,Msg); // 如果这是消息中的最后一个键值对，则添加右花括号
        return;
    }
    // 如果不是消息中的第一个键值对
    if(!Is_end)
    {
        if(data_type)   sprintf(Msg,"%s,\"%s\":%.2f",Msg,name,value); // 添加浮点型值
        else            sprintf(Msg,"%s,\"%s\":%d",Msg,name,(int)value); // 添加整型值
    }
    // 如果是消息中的最后一个键值对
    else
    {
        if(data_type)   sprintf(Msg,"%s,\"%s\":%.2f}",Msg,name,value); // 添加浮点型值和右花括号
        else            sprintf(Msg,"%s,\"%s\":%d}",Msg,name,(int)value); // 添加整型值和右花括号
    }
}


/*****************************UDP的数据发送**************************/
/*
                                    *
    功能：UDP的图像信息发送 *
    参数：Msg 预发送信息流         *
    参数：Icode 图像编码
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
    功能：UDP的Json信息发送 *
    参数：Msg 预发送信息流         *
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
//    uint32 len = strlen(Msg); // 计算 Msg 数组的长度
//    wifi_uart_send_buffer((uint8*)Msg, len);
}
/************************************
                                    *
    功能：UDP的图像和Json信息总发送 *
    参数：Msg 预发送信息流         *
    参数：Icode 图像编码
    参数：Mid 中线
                                    *
*************************************/
void WiFi_UDPSend(char* Msg)
{
    WiFi_UDP_JsonSend(Msg);
}

/*****************************UDP数据接受并查找**************************/
/************************************
                                    *
    功能：基于UDP的Json键整型值查找*
    参数：Msg 是接收到的Json信息
    参数：name 需要查找的整型键名

    返回： -1失败  其余为查得的键值
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
    功能：基于UDP的Json键浮点型值查找*
    参数：Msg 是接收到的Json信息
    参数：name 需要查找的浮点型键名
    返回： -1失败  其余为查得的键值
                                    *
*************************************/
double UDP_Msg_KeyGet_Float(char* Msg,char* name)
{
    double value = -1;
    char temp_value[128];
    char temp_value_Point;
    char* Pointer;
    memset(temp_value,0,128);//初始化 temp_value 全部置0
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
    memset(Msg,0,4096);//初始化数组 Msg
    UDP_Msg_Add(Msg,"Steering_kp",Steering.kp,0,1);                  //  转向环参数p1
    UDP_Msg_Add(Msg,"Steering_kI",Steering.ki,0,1);                  //  转向环参数p2
    UDP_Msg_Add(Msg,"Steering_kd",Steering.kd,0,1);                  //  转向环参数p3
    UDP_Msg_Add(Msg,"Steering_Speed_kp",Steering_Speed.kp,0,1);                        //  转向环i
    UDP_Msg_Add(Msg,"Steering_Speed_ki",Steering_Speed.ki,0,1);                        //  转向环d
    UDP_Msg_Add(Msg,"Steering_Speed_kd",Steering_Speed.kd,0,1);            //  直道期望速度
    UDP_Msg_Add(Msg,"SS",Speed_Straght,1,0);              //  弯道期望速度
    WiFi_UDP_JsonSend(Msg);
}

/*****************************函数应用实例**************************/
/*
                                    *
    功能：基于UDP的车内参数发送示例(按需求自行更改)*
    参数：Msg 预发送的Json信息流
    参数：Icode 图像编码数据
    参数：Mid 中线数据
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
    if(data_length)                                                         // 如果接收到数据 则进行数据类型判断
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
                // 是否要将参数写入flash
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
    功能：基于UDP的车内参数发送示例(按需求自行更改)*
    参数：Msg 预发送的Json信息流
    参数：Icode 图像编码数据
    参数：Mid 中线数据
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
//  * @brief 二值化图像编码用于ESP8266发送二值化图像
//  * @parma Image_Cod 编码后的数组
//  * @param Bin_Image 二值化数组
//  * @return 无
//  * @see Image_Coding();
//  * @author 何永旗
//  * @date 2021/5/14
//  * @修改日期:2022/6/28   2023/3/3
//  * @修改人:陈悦鑫
//  *
//  * */
// void Image_Coding(uint8 binImage[ImgH][ImgW], uint8 Image_Cod[ImgH * ImgW / 8])
// {
//     int k = 0;               // 压缩后的循环变量
//     unsigned char i, j;      // 二值化循环变量
//     unsigned char index = 0; // 移位变量
//     for (i = 0; i < ImgH; i++)
//     {
//         for (j = 0; j < ImgW; j++)
//         {
//             if (index < 8)
//             {
//                 Image_Cod[k] = (Image_Cod[k] << 1) + binImage[i][j]; // 左移一位
//                 index++;
//             }
//             else
//             {
//                 index = 0;
//                 k++;
//                 Image_Cod[k] = (Image_Cod[k] << 1) + binImage[i][j]; // 左移一位
//                 index++;
//             }


/*
                                    *
    功能：基于UDP的VOFA+串口输出*
    参数：data1-4 要发送的四个数据
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
    功能：基于UDP的上位机数据接收示例(按需求自行更改)*
    参数：topic 接收到数据的帧头
                                    *
*/
void UDPMsg_Get(char* topic)
{
    char Order[256];
    //int temp;
    double ftemp;
    memset(Order,0,256);//将 Order 指向的内存区域的前 256 个字节全部设置为 0 的值  初始化Order数组
    system_delay_ms(50);
    data_length = (uint16)wifi_uart_read_buffer((uint8*)Order,sizeof(Order));
    if(data_length)                                                         // 如果接收到数据 则进行数据类型判断
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
 * @brief 建立UDP连接函数
 * @para: mode=1 适用于vofa+上位机显示无线udp串口
 *        mode=2 适用于node-red上位机的UDP传输
 */
void Wifi_UDP_Connect(int mode)
{
    int Protect_time=10;    //五秒的保护时间
    tft180_show_float (0, 40,0.81, 4,4);
    while(wifi_uart_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST, WIFI_UART_STATION) )
    {
        system_delay_ms(500);                                                   // 初始化失败 等待 500ms
        Protect_time--;
        if(Protect_time<=0)     break;
    }
    tft180_show_float (0, 20,0.82, 4,4);
    // zf_device_wifi_uart.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接、创建 TCP 服务器等操作
    if(2 != WIFI_UART_AUTO_CONNECT)                                             // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        if(mode == 2)
        {
            if(wifi_uart_connect_udp_client(                                        // 向指定目标 IP 的端口建立 UDP 连接
                WIFI_UART_TARGET_IP_NODE_RED_VOFA,                                                // 这里使用与自动连接时一样的目标 IP 实际使用时也可以直接填写目标 IP 字符串
                WIFI_UART_TARGET_PORT_NODE_RED,                                              // 这里使用与自动连接时一样的目标端口 实际使用时也可以直接填写目标端口字符串
                WIFI_UART_LOCAL_PORT,                                               // 这里使用与自动连接时一样的本地端口 也就是自己通信使用的端口号  实际使用时也可以直接填写其他的本地端口字符串
                WIFI_UART_SERIANET))                                                // 采用命令传输模式 当然你可以改成透传模式 实际上差别并不是很大
            {
                // 如果一直建立失败 考虑一下是不是没有接硬件复位
                tft180_show_string(0, 0, "UDP Con error");
//                oled_show_string(0, 0, "UDP Con error");
                system_delay_ms(500);                                               // 建立连接失败 等待 500ms
            }
        }
        else
        {
            if(wifi_uart_connect_udp_client(                                        // 向指定目标 IP 的端口建立 UDP 连接
                WIFI_UART_TARGET_IP_NODE_RED_VOFA,                                                // 这里使用与自动连接时一样的目标 IP 实际使用时也可以直接填写目标 IP 字符串
                WIFI_UART_TARGET_PORT_VOFA,                                              // 这里使用与自动连接时一样的目标端口 实际使用时也可以直接填写目标端口字符串
                WIFI_UART_LOCAL_PORT,                                               // 这里使用与自动连接时一样的本地端口 也就是自己通信使用的端口号  实际使用时也可以直接填写其他的本地端口字符串
                WIFI_UART_SERIANET))                                                // 采用命令传输模式 当然你可以改成透传模式 实际上差别并不是很大
            {
                // 如果一直建立失败 考虑一下是不是没有接硬件复位
                tft180_show_string(0, 0, "UDP Con error");
//                oled_show_string(0, 0, "UDP Con error");
                system_delay_ms(500);                                               // 建立连接失败 等待 500ms
            }
        }
    }
//    oled_show_string(0, 0, (char*)wifi_uart_information.wifi_uart_local_ip);
//    oled_show_string(0,2,(char*)wifi_uart_information.wifi_uart_local_port);
    tft180_show_string(0, 15, (char*)wifi_uart_information.wifi_uart_local_ip);
    tft180_show_string(0,30,(char*)wifi_uart_information.wifi_uart_local_port);

}

/*
 * 函数名：Param_Send_Set
 * 功能：将车内的部分控制参数发送至上位机，并通过UDP远程修改参数模式来修改参数。
 * 参数：
 *     ConFlag：UDP远程修改参数模式标志，1表示开启，0表示关闭。
 * 返回值：无。
 */
void Param_Send_Set(int ConFlag)
{
    uint8 i;
    uint8 protect_times=3;
    //将车内的部分控制参数发送至上位机
    for(i=0;i<protect_times;i++)
    {
        system_delay_ms(25);
        UDP_Parameter_Send_Control(UDPMsg);
        system_delay_ms(25);
        tft180_show_string(0,50,"Send OK");
    }
    //udp远程修改参数模式
    if(ConFlag==1)
    {
        //当Start_Car为0时，循环执行以下操作
        while(!Start_Car)
        {
            UDPMsg_Get("DULUN");                // 通过UDP接收指定格式的数据

            key_scanner();                      // 扫描按键状态
            if(key_get_state(KEY_1))            // 判断按键是否被按下
            {
                Start_Car=1;                    // 修改Start_Car变量的值为1，退出循环
            }
        }
    }
    tft180_clear();                             // TFT清屏
}

void Write_Flash(void)
{
    flash_buffer_clear();                                                       // 清空缓冲区
    flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);                    // 擦除这一页
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
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);        // 向指定 Flash 扇区的页码写入缓冲区数据
}


void Read_Flash(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);           // 将数据从 flash 读取到缓冲区
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
//批量数据直接发送 使用示例
////UDP_OUT_Int4(UDPMsg, &A, NULL, NULL, NULL);
//如果少传就写NULL占位,NULL从后往前填
void UDP_OUT_Int4(char* Msg, int *Test1, int *Test2, int *Test3, int *Test4){
    system_delay_ms(60);//两次发送数据间隔至少要这个数
    memset(Msg,0,4096);//初始化
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
//如果少传就写NULL占位,NULL从后往前填
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

//批量数据直接发送 使用示例
//UDP_OUT_Float4(UDPMsg, &A, NULL, NULL, NULL);
//如果少传就写NULL占位,NULL从后往前填
void UDP_OUT_Float4(char* Msg, float *Test1, float *Test2, float *Test3, float *Test4){
    system_delay_ms(60);//两次发送数据间隔至少要这个数
    memset(Msg,0,4096);//初始化
    int i = (Test1 != NULL) + (Test2 != NULL) + (Test3 != NULL) + (Test4 != NULL);
    if(i){
        if(i) UDP_Msg_Add(Msg,"FTO",*Test1,!(i - 1),1), i--;                          //Test
        if(i) UDP_Msg_Add(Msg,"FTT",*Test2,!(i - 1),1), i--;                          //Test
        if(i) UDP_Msg_Add(Msg,"FTS",*Test3,!(i - 1),1), i--;                          //Test
        if(i) UDP_Msg_Add(Msg,"FTF",*Test4,!(i - 1),1);                       //Test
        WiFi_UDP_JsonSend(Msg);
    }
}

//批量数据直接接受 使用示例
//UDP_In_Float4("TTT", &A, NULL, NULL, NULL);
//如果少传就写NULL占位,NULL从后往前填
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

//Test 使用代码直接修改
//Test可以随意修改为其他名称 可以随意修改Test类型
//UDP_FTest(UDPMsg,Test)
void UDP_FTest(char* Msg, float Test){
    system_delay_ms(60);//两次发送数据间隔至少要这个数
    memset(Msg,0,4096);//初始化
    UDP_Msg_Add(Msg,"Test",Test,1, 1);                   //Test
    WiFi_UDP_JsonSend(Msg);
}

//Test 使用代码直接修改
//Test可以随意修改为其他名称 可以随意修改Test类型
//UDP_ITest(UDPMsg,Test)
void UDP_ITest(char* Msg, int Test){
    system_delay_ms(60);//两次发送数据间隔至少要这个数
    memset(Msg,0,4096);//初始化
    UDP_Msg_Add(Msg,"Test", (float)Test, 1, 0);                  //Test
    WiFi_UDP_JsonSend(Msg);
}

//Car
void UDP_Car(char* Msg){
    system_delay_ms(60);//两次发送数据间隔至少要这个数
    memset(Msg,0,4096);
    snprintf(Msg, sizeof(Msg), "{\"Stop_Car\": \"%c\", \"Start_Car_Para\": \"%c\"}", Stop_Car, Start_Car_Para); //
    WiFi_UDP_JsonSend(Msg);
}

//IP
void UDP_IP(char* Msg){
    system_delay_ms(60);//两次发送数据间隔至少要这个数
    memset(Msg,0,4096);
    //printf("IP:\n");//(QwQ)
    char ip_str[17];
    for (int i = 0; i < 16; i++){
        ip_str[i] = (char)wifi_uart_information.wifi_uart_local_ip[i];
        //printf("%c", ip_str[i]);//(QwQ)
        if (ip_str[i] == '\0'){
            //printf("\nBreak\n");//(QwQ)
            break;
        }  // 遇到终止符提前结束
    }
    ip_str[16] = '\0';  // 确保字符串终止
    snprintf(Msg,4096,"{\"wifi_uart_local_ip\": \"%s\"}",ip_str);//数组转JSON
    WiFi_UDP_JsonSend(Msg);
    //printf("WIFIEND\n");//(QwQ)
    UDP_ITest(Msg,1);
    UDP_Data(Msg);
}

//测试数据读取情况
//本函数不会修改变量值
//无帧头 启用无条件进行
void UDPMsg_ReadTest(){
    int Test = 0;
    char Order[256];
    //int temp;
    double ftemp;
    memset(Order,0,256);//初始化数组
    system_delay_ms(50);
    data_length = (uint16)wifi_uart_read_buffer((uint8*)Order,sizeof(Order));//读取数组长度
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
//现在用于控制车模式 0 关闭 1-直立 2-行动
//功能未实现 先预备下来
//UDP_Data(UDPMsg,1)
void UDP_Data(char* Msg){
    system_delay_ms(60);//两次发送数据间隔至少要这个数
    memset(Msg,0,4096);//初始化
    UDP_Msg_Add(Msg,"Data", (float)Data, 1, 0);                  //Test
    WiFi_UDP_JsonSend(Msg);
}

void UDP_Control(){
    if(Data == 0){
        Stop_Car = 1;//完全停止
    }
    if(Data == 1){
        Stop_Car = 2;//直立
    }
    if(Data == 2){
        Stop_Car = 2;//前进
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

