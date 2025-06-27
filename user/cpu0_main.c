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
* �ļ�����          cpu0_main
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
#pragma section all "cpu0_disarm"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
short Send_Flag = 0; // ��ʼ����UDP��Ϣ�ı�־λ
char UDPMsg[4096];
float Bias1, Bias2, Bias;
int Bias_count;
double Running_Time = 0; // ����ʼִ������ʱ�䣬�жϼ�ʱ��ʽ����
double Running_Time_Yaw = 0; // ����ʼִ������ʱ�䣬�жϼ�ʱ��ʽ����

short A_Speed = 0; // ������A���ٶ�
short B_Speed = 0; // ������B���ٶ�
short C_Speed = 0; // �н���C���ٶ�

//extern short C_Speed = 0;
//extern short B_Speed = 0;

short Speed_sum = 0;

extern int in_mode , out_mode1, out_mode2;
uint8_t Mmode = 1;//0��ʾ����ͷ�鿴 1��Ĭ��

int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    Peripheral_Init();              // ȫ�����ʼ��
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    //Speed_Straght = 0;//ǰ���ٶ�
    //Speed_Turn = 10;//����н����ٶ�

    //����δִ��
    while (1)
    {
        point_location_save();
//        if(Distance_aim<1)
//        {
//
//        }
//����ͷ���Դ���
//        while(!(Mmode)){
//            key_scanner();
//            system_delay_ms(20);
//            if(KEY_detect(KEY_1)){Mmode++;}
//            if(KEY_detect(KEY_2)){Mmode--;}
//            if(KEY_detect(KEY_3)){SGS++;}
//            if(KEY_detect(KEY_4)){SGS--;}
//            //tft180_show_float (0, 0, SG, 4,4);
//        }

//�������Դ����������ɾ�� �������Ŷ�Ӧû������
//��ISR.c��158������ϴ���ɾ���ǵ�һ��
//        while(1){
//            key_scanner();
//            system_delay_ms(20);
//            if(KEY_detect(KEY_1)){SG++,SG++;}//������������ 1 2 ������������ �� 20.6 20.7����
//            if(KEY_detect(KEY_2)){SG--,SG--;}//������33.12 33.13������������Ҳû������
//            if(KEY_detect(KEY_3)){SG++;}//3 4 �쳣 ��20.8 20.9�쳣
//            if(KEY_detect(KEY_4)){SG--;}
//        }

        key_scanner();
//        if (KEY_detect(KEY_2))                            // ȡ������������������ĸ�λ��
//        {
//            Stop_Car = 0;
//        }
        if (fabsf(Roll) > 15 || fabsf(Pitch) > 20)        // �����㵹����
            Stop_Car = 1;
    }
}

#pragma section all restore
// **************************** �������� ****************************

