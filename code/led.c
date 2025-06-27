/*
 * led.c
 *
 *  Created on: 2024年4月18日
 *      Author: 黄春华
 */

#include "Led.h"
#include "zf_device_tld7002.h"
#include "zf_device_dot_matrix_screen.h"

int Camera_t_t = 0;
//
///*************************************************************************
//*  函数名称：void LED_Init(void)
//*  功能说明：GPIO初始化函数 LED灯所用P10.6、P10.5、P15.4和P15.6初始化
//*  参数说明：无
//*  函数返回：无
//*  修改时间：2023年3月27日   安健
//*  备    注：
//*************************************************************************/
void GPIO_LED_Init(void)
{
//    gpio_init(LED0p, GPO, GPIO_LOW, GPO_PUSH_PULL);          // 初始化 LED1 输出 默认高电平 推挽输出模式
//    gpio_init(LED1p, GPO, GPIO_LOW, GPO_PUSH_PULL);         // 初始化 LED2 输出 默认高电平 推挽输出模式
//    gpio_init(LED2p, GPO, GPIO_LOW, GPO_PUSH_PULL);          // 初始化 LED3 输出 默认高电平 推挽输出模式
//    gpio_init(LED3p, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

