/*********************************************************************************************************************
* TC377 Opensourec Library ����TC377 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC377 ��Դ���һ����
*
* TC377 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cpu1_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.20
* ����ƽ̨          TC377TP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-11-03       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#pragma section all "cpu1_disarm"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
float Actual_data[6];                           // ���ת���Ժ��������ʵ������(������)
float A_data[6];
float Yaw,Pitch,Roll,Yaw_B,Yaw_Tur, Pitch2, Yaw_B1, Yaw_B2, Yaw_Bias1;             // ����Ƕ�

char Stop_Car=1;                                // ͣ����־λ
char Start_Car_Para = 1;                        // ģʽ������־λ
int Start_Car_Time = 1;                         // ��ʼ��ʱ

float Speed_Straght = 0; //ֱ���ٶ�  120
float Speed_Turn = 0;   // ת���ٶ�0
float X_Distance = 0, Y_Distance = 0;

float X_Dynamic_Zero = 0;                       // X�ᶯ̬���ֵ
float Y_Dynamic_Zero = 0;                       // y�ᶯ̬���ֵ

float Yaw_Data_Mean_Filter[5];

float save_x[100]={0}, save_y[100]={0};
int numpath = 0;//·�������
int short_t = 0;

void core1_main(void)
{
    disable_Watchdog();                     // �رտ��Ź�
    interrupt_global_enable(0);             // ��ȫ���ж�
    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����
    // �˴���д�û����� ���������ʼ�������
    GPIO_LED_Init();
    //pit_init(CCU60_CH0, 2*1000);
    while(1)
    {

        //point_location_save();
    }

}
#pragma section all restore
