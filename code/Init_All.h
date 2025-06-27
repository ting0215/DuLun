/*
 * Init_All.h
 *
 *  Created on: 2024��4��17��
 *      Author: �ƴ���
 */

#ifndef CODE_INIT_ALL_H_
#define CODE_INIT_ALL_H_

#include "zf_common_headfile.h"

typedef struct External_equipment           //����ṹ�嶨��
{
    unsigned int Key;               //����
    unsigned int Led;               //���İ�ledָʾ��
    unsigned int Beep;              //Beep������
    unsigned int Uart;              //����
    unsigned int imu660ra;          //������
    unsigned int TFO;               //TFT/OLED
    unsigned int Wifi;              //wifiģ��
    unsigned int ADC;               //ADCģ��
    unsigned int Brushless_Motor;   //��ˢ���
    unsigned int Brushless_ENC;     //��ˢ���������
    unsigned int Brush_Motor_init;  //��ˢ���
    unsigned int Brush_ENC;         //��ˢ���������
    unsigned int Cameral;           //Cameral
    unsigned int TOF;               //TOFģ��
    unsigned int GuangLiu;          //����ģ��
    unsigned int CCU60_CH0;         //CCU60_CH0�ж�
    unsigned int CCU60_CH1;         //CCU60_CH1
    unsigned int CCU61_CH0;         //CCU61_CH0
    unsigned int CCU61_CH1;         //CCU61_CH1
}External_equipment;

extern External_equipment Equipments;
void Peripheral_Init(void);



#endif /* CODE_INIT_ALL_H_ */
