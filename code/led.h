/*
 * led.h
 *
 *  Created on: 2024��4��18��
 *      Author: �ƴ���
 */

#ifndef CODE_LED_H_
#define CODE_LED_H_

#include "zf_common_headfile.h"

//����ģ���
typedef enum
{
    LED0=0,  //���İ���LED0
    LED1=1,  //���İ���LED1
    LED2=2,  //���İ���LED2
    LED3=3,   //���İ���LED3
    LEDALL=4
} LEDn_e;

typedef enum
{
    ON=0,  //��
    OFF=1, //��
    RVS=2, //��ת
}LEDs_e;

//����Ĺܽ�Ҫ��Ӧʵ�ʰ���
#define LED0p      P20_9   //���İ���LED0
#define LED1p      P20_8   //���İ���LED1
#define LED2p      P21_5   //������LED2
#define LED3p      P21_4   //������LED3

extern int Camera_t_t;
//
///*********************** UART���ܺ��� **************************/
//��ʼ��
void GPIO_LED_Init(void);


#endif /* CODE_LED_H_ */
