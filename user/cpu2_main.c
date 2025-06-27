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
* �ļ�����          cpu2_main
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
#pragma section all "cpu2_disarm"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
void core2_main(void)
{
    disable_Watchdog();                     // �رտ��Ź�
    interrupt_global_enable(0);             // ��ȫ���ж�
    // �˴���д�û����� ���������ʼ�������

    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {
        //tft180_show_float (0, 0,Pitch/*X_Distance*/, 4,4);
        //tft180_show_float (60, 0, Yaw_Tur/*Y_Distance*/, 4,4);
        //tft180_show_float (0, 20, point_x[point_pointer], 4,4);
        //tft180_show_float (60, 20, point_y[point_pointer], 4,4);
        //tft180_show_float (0, 40, Yaw_Tur, 4,4);
        //tft180_show_float (60, 40, angle_error, 4,4);
        system_delay_ms(50);
        while(Mmode){
            if(mt9v03x_finish_flag){
                //Camera_Work(1);
                //Speed_Straght = 0;
                mt9v03x_finish_flag = 0;
            }
        }
        while(!Mmode){
            if(mt9v03x_finish_flag){

               //system_start();
               Camera_Work(1);
               //Camera_FGE(1);
               for(int i = 0;i < 188;i++){
                   mt9v03x_image[40][i] = 255;
               }//����
               tft180_displayimage03x((const uint8 *)mt9v03x_image, 160, 128);
               //tft180_show_gray_image(0,0,(const uint8 *)mt9v03x_out,MT9V03X_W+2, MT9V03X_H+2, 160, 128, 0);
               //printf("%d\n", (int)system_getval_us());
               mt9v03x_finish_flag = 0;
            }
        }
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore
