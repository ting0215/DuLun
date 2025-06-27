/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_disarm"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
short Send_Flag = 0; // 开始传输UDP信息的标志位
char UDPMsg[4096];
float Bias1, Bias2, Bias;
int Bias_count;
double Running_Time = 0; // 程序开始执行运行时间，中断计时方式测量
double Running_Time_Yaw = 0; // 程序开始执行运行时间，中断计时方式测量

short A_Speed = 0; // 动量轮A的速度
short B_Speed = 0; // 动量轮B的速度
short C_Speed = 0; // 行进轮C的速度

//extern short C_Speed = 0;
//extern short B_Speed = 0;

short Speed_sum = 0;

extern int in_mode , out_mode1, out_mode2;
uint8_t Mmode = 1;//0表示摄像头查看 1是默认

int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    Peripheral_Init();              // 全外设初始化
    debug_init();                   // 初始化默认调试串口
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    //Speed_Straght = 0;//前进速度
    //Speed_Turn = 10;//弯道行进轮速度

    //以下未执行
    while (1)
    {
        point_location_save();
//        if(Distance_aim<1)
//        {
//
//        }
//摄像头调试代码
//        while(!(Mmode)){
//            key_scanner();
//            system_delay_ms(20);
//            if(KEY_detect(KEY_1)){Mmode++;}
//            if(KEY_detect(KEY_2)){Mmode--;}
//            if(KEY_detect(KEY_3)){SGS++;}
//            if(KEY_detect(KEY_4)){SGS--;}
//            //tft180_show_float (0, 0, SG, 4,4);
//        }

//按键测试代码用完可以删除 按键引脚对应没有问题
//在ISR.c第158行有配合代码删除记得一起
//        while(1){
//            key_scanner();
//            system_delay_ms(20);
//            if(KEY_detect(KEY_1)){SG++,SG++;}//初步测试引脚 1 2 是正常工作的 即 20.6 20.7正常
//            if(KEY_detect(KEY_2)){SG--,SG--;}//测试了33.12 33.13两个开关引脚也没有问题
//            if(KEY_detect(KEY_3)){SG++;}//3 4 异常 即20.8 20.9异常
//            if(KEY_detect(KEY_4)){SG--;}
//        }

        key_scanner();
//        if (KEY_detect(KEY_2))                            // 取消保护，（车身保护后的复位）
//        {
//            Stop_Car = 0;
//        }
        if (fabsf(Roll) > 15 || fabsf(Pitch) > 20)        // 车身倾倒保护
            Stop_Car = 1;
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************

