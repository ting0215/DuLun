/*
 * led.h
 *
 *  Created on: 2024年4月18日
 *      Author: 黄春华
 */

#ifndef CODE_LED_H_
#define CODE_LED_H_

#include "zf_common_headfile.h"

//定义模块号
typedef enum
{
    LED0=0,  //核心板上LED0
    LED1=1,  //核心板上LED1
    LED2=2,  //核心板上LED2
    LED3=3,   //核心板上LED3
    LEDALL=4
} LEDn_e;

typedef enum
{
    ON=0,  //亮
    OFF=1, //灭
    RVS=2, //反转
}LEDs_e;

//定义的管脚要对应实际按键
#define LED0p      P20_9   //核心板上LED0
#define LED1p      P20_8   //核心板上LED1
#define LED2p      P21_5   //核心上LED2
#define LED3p      P21_4   //核心上LED3

extern int Camera_t_t;
//
///*********************** UART功能函数 **************************/
//初始化
void GPIO_LED_Init(void);


#endif /* CODE_LED_H_ */
