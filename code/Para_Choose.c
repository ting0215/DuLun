/*
 * Para_Choose.c
 *
 *  Created on: 2024��4��18��
 *      Author: �ƴ���
 */


#include <Para_Choose.h>

// ͼ��ȫ�ֲ����趨
// ��ʼ���ṹ��ȫ��
// extern External_equipment Equipments;

#define MENU_LEVEL1_ITEMS  2
int CurrentMode = 1;
extern int numpath;

void save_path(void)
{
    int i = 0; //i��ֵ���Ǵ洢��ĸ���
    while(1)
    {
        key_scanner();
        save_y[i] = Y_Distance;
        save_x[i] = X_Distance;
        char display_str[] = "";
        sprintf(display_str,"%d(%.1f,%.1f)",i,save_x[i],save_y[i]);
        tft180_show_string(0,22,display_str);//��ʾ�洢����š��洢��xy����
        if(KEY_detect(KEY_2)==1)//�̰�ֵΪ1������ֵΪ2������ֵΪ0
        {
            tft180_clear();
            i = i + 1;
        }
        if(KEY_detect(KEY_1)==1)
        {
            tft180_clear();
            break;
        }
    }


    flash_buffer_clear();
    flash_erase_page(FLASH_SECTION_INDEX, 0);                    // ������һҳ
    flash_union_buffer[0].int32_type  = i;                                      //��¼·��������
    for(int k=0; k < i;k++)
    {
        int pa = 2*k+1;
        flash_union_buffer[pa].float_type  = save_x[k];
        flash_union_buffer[pa+1].float_type  = save_y[k];
    }
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, 0);        // ��ָ�� Flash ������ҳ��д�뻺��������


}


void putin(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, 0);           // �����ݴ� flash ��ȡ��������
    numpath = flash_union_buffer[0].int32_type;                   // �洢·�������
    for(int i=0; i < numpath ;i++)                                // �洢����·��������
    {
        int sa_pa = 2*i+1;
        save_x[i] = flash_union_buffer[sa_pa].float_type;
        save_y[i] = flash_union_buffer[sa_pa+1].float_type;
    }
    char display_str6[] = "";
    sprintf(display_str6,"Bias: %.5f",Bias);
    tft180_show_string(20,20,display_str6);
}



void processMenuLevel1(void)
{
    Read_Flash();       // ��flash�ڵĲ�������
    oled_clear();
    while(1)
    {
        Stop_Car = 1;
        char display_str[] = "";
        sprintf(display_str,"CurrentMode: %d",CurrentMode);
        oled_show_string(0,1,display_str);
        if(KEY_detect(KEY_1))
        {
           CurrentMode +=1;
        }
        else if(KEY_detect(KEY_2))
        {
            CurrentMode -=1;
        }
        else if(KEY_detect(KEY_3))
        {
            oled_clear();
            //while (processMenuLevel2(CurrentMode)){}
        }
        else if(KEY_detect(KEY_4))
        {
            oled_clear();
            Stop_Car = 0;
            return;
        }
    }
}

