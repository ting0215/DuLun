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
* 文件名称          cpu1_main
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
#pragma section all "cpu1_disarm"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************
float Actual_data[6];                           // 存放转换以后的陀螺仪实际数据(带符号)
float A_data[6];
float Yaw,Pitch,Roll,Yaw_B,Yaw_Tur, Pitch2, Yaw_B1, Yaw_B2, Yaw_Bias1;             // 三轴角度

char Stop_Car=1;                                // 停车标志位
char Start_Car_Para = 1;                        // 模式发车标志位
int Start_Car_Time = 1;                         // 开始计时

float Speed_Straght = 0; //直道速度  120
float Speed_Turn = 0;   // 转向速度0
float X_Distance = 0, Y_Distance = 0;

float X_Dynamic_Zero = 0;                       // X轴动态零点值
float Y_Dynamic_Zero = 0;                       // y轴动态零点值

float Yaw_Data_Mean_Filter[5];

float save_x[100]={0}, save_y[100]={0};
int numpath = 0;//路径点个数
int short_t = 0;

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    // 此处编写用户代码 例如外设初始化代码等
    GPIO_LED_Init();
    //pit_init(CCU60_CH0, 2*1000);
    while(1)
    {

        //point_location_save();
    }

}
#pragma section all restore
