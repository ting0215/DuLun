/*
 * led.c
 *
 *  Created on: 2024��4��18��
 *      Author: �ƴ���
 */

#include "Led.h"
#include "zf_device_tld7002.h"
#include "zf_device_dot_matrix_screen.h"

int Camera_t_t = 0;
//
///*************************************************************************
//*  �������ƣ�void LED_Init(void)
//*  ����˵����GPIO��ʼ������ LED������P10.6��P10.5��P15.4��P15.6��ʼ��
//*  ����˵������
//*  �������أ���
//*  �޸�ʱ�䣺2023��3��27��   ����
//*  ��    ע��
//*************************************************************************/
void GPIO_LED_Init(void)
{
//    gpio_init(LED0p, GPO, GPIO_LOW, GPO_PUSH_PULL);          // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ
//    gpio_init(LED1p, GPO, GPIO_LOW, GPO_PUSH_PULL);         // ��ʼ�� LED2 ��� Ĭ�ϸߵ�ƽ �������ģʽ
//    gpio_init(LED2p, GPO, GPIO_LOW, GPO_PUSH_PULL);          // ��ʼ�� LED3 ��� Ĭ�ϸߵ�ƽ �������ģʽ
//    gpio_init(LED3p, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

