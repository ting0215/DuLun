/*
 * Wifi.h
 *
 *  Created on: 2024��4��28��
 *      Author: �ƴ���
 */

#ifndef CODE_WIFI_H_
#define CODE_WIFI_H_

#include "zf_common_headfile.h"

#define NO0_SendChar(data)      (uart_write_byte((UART_2), (data)))
#define NO0_SendString(data)    uart_write_string(UART_2, data);

// ȡ����ɿ��еĺ궨��
#undef WIFI_UART_TARGET_IP
#undef WIFI_UART_TARGET_PORT
#undef WIFI_UART_LOCAL_PORT
#define FLASH_SECTION_INDEX       (0)                                 // �洢�����õ�����
#define FLASH_PAGE_INDEX          (11)                                // �洢�����õ�ҳ�� ������һ��ҳ��

//wifiģ������ӵ�WiFi�˺ź�����
#define WIFI_SSID_TEST      "4D421-422"//"iPhone20"//  //"MI 8"  "As"
#define WIFI_PASSWORD_TEST  "NO.0STUDIO"//"hch12345"//  //"fxq021006"
// �˿����¶���
#define WIFI_UART_TARGET_IP_NODE_RED      "192.168.1.171"         // nodered��λ����Ŀ��IP����̨��������
#define WIFI_UART_TARGET_IP_NODE_RED_VOFA "192.168.1.44"          // node-red��vofa+ģʽ��IP����������IP��
#define WIFI_UART_TARGET_PORT_NODE_RED    "1000"                  // NODE-red��UDP���ӵĶ˿�
#define WIFI_UART_TARGET_PORT_VOFA        "70"//"6661"                  // VOFA+��UDP���ӵĶ˿�

#define WIFI_UART_LOCAL_PORT    "2001"//"2333"      // �����˿�   ��Ƭ���˿�

void UDP_Msg_Add(char* Msg, char* name, double value, int Is_end, char data_type);
void WiFi_UDP_ImgSend(uint8* Icode,uint8* Mid);
void WiFi_UDP_JsonSend(char* Msg);
void WiFi_UDPSend(char* Msg);
int  UDP_Msg_KeyGet_Int(char* Msg,char* name);
double UDP_Msg_KeyGet_Float(char* Msg,char* name);
void UDP_Send_double(double data1, double data2, double data3, float data4);  //VOFA+���ͺ���

// ��Ҫ�û��޸���������ĺ���

void UDP_In_Float4(char* topic, float *Test1, float *Test2, float *Test3, float *Test4);
void UDP_In_Int4(char* topic, int *Test1, int *Test2, int *Test3, int *Test4);
void UDP_OUT_Float4(char* Msg, float *Test1, float *Test2, float *Test3, float *Test4);
void UDP_OUT_Int4(char* Msg, int *Test1, int *Test2, int *Test3, int *Test4);
void UDP_Parameter_Send_Control(char* Msg);
void UDP_Parameter_Send_Control2(char* Msg);
void UDP_Parameter_Send_Carrun(char* Msg);
void UDPMsg_Get(char* topic);
void UDPMsg_Get2(char* topic);

// ��ʼ������
void Wifi_UDP_Connect(int mode);

// �г����ݷ��ͺ���
void Param_Send_Set(int ConFlag);

//��дFlash
void Write_Flash(void);
void Read_Flash(void);

void UDP_FTest(char* Msg, float Test);
void UDP_ITest(char* Msg, int Test);
void UDP_Car(char* Msg);
void UDP_IP(char* Msg);
void UDPMsg_ReadTest();
void UDP_Data(char* Msg);
void UDPMsg_A(char* topic);
void UDP_Control();



#endif /* CODE_WIFI_H_ */
