/*
 * Init_All.h
 *
 *  Created on: 2024年4月17日
 *      Author: 黄春华
 */

#ifndef CODE_INIT_ALL_H_
#define CODE_INIT_ALL_H_

#include "zf_common_headfile.h"

typedef struct External_equipment           //外设结构体定义
{
    unsigned int Key;               //按键
    unsigned int Led;               //核心板led指示灯
    unsigned int Beep;              //Beep蜂鸣器
    unsigned int Uart;              //串口
    unsigned int imu660ra;          //陀螺仪
    unsigned int TFO;               //TFT/OLED
    unsigned int Wifi;              //wifi模块
    unsigned int ADC;               //ADC模块
    unsigned int Brushless_Motor;   //无刷电机
    unsigned int Brushless_ENC;     //无刷电机编码器
    unsigned int Brush_Motor_init;  //有刷电机
    unsigned int Brush_ENC;         //有刷电机编码器
    unsigned int Cameral;           //Cameral
    unsigned int TOF;               //TOF模块
    unsigned int GuangLiu;          //光流模块
    unsigned int CCU60_CH0;         //CCU60_CH0中断
    unsigned int CCU60_CH1;         //CCU60_CH1
    unsigned int CCU61_CH0;         //CCU61_CH0
    unsigned int CCU61_CH1;         //CCU61_CH1
}External_equipment;

extern External_equipment Equipments;
void Peripheral_Init(void);



#endif /* CODE_INIT_ALL_H_ */
