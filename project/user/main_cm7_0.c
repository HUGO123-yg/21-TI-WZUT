/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0），或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至并未隐含任何适销特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时也收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源协议（附件协议） 以上为申明版本，以下为正文内容
* 正文本库的解释权为中文版本，任何冲突以中文版为准
* 正式协议文本参见 libraries 文件夹下的 GPL3_permission_statement.txt 文件
* 许可文本参见 libraries 文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序，但请勿删除或修改以下内容
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限责任公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "gnss_cache.h"

// **************************** 主函数 ****************************


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置，系统初始化【不可删除】
    debug_init();                       // 串口调试初始化
    // 此处用户可以编写 用户自己的初始化代码  例如初始化外设等

    BUZZER_init();
//    imu963ra_init();
//    imu660ra_init();
      imu660rb_init();


    flash_init();
    Init_Nag();

    yaw_fusion_init();
    gnss_init(TAU1201);
    gnss_auto_start();

    small_driver_uart_init();
    uart_receiver_init();
    remote_ctrl_init();
    balance_cascade_init();
    steer_control_init();
    ips_init(IPS200_TYPE_SPI);
    Key_init();
    system_delay_ms(1000);



    pit_ms_init(PIT_CH0,1);
    pit_ms_init(PIT_CH1,5);

    BUZZER_check(50);                       // 蜂鸣器自检



    // 此处用户可以编写 用户自己的初始化代码  例如初始化外设等
    while(true)
    {
        // GPS 数据解析与缓存保存
        if (gnss_flag)
        {
            gnss_flag = 0;
            if (gnss_data_parse() == 0 && gnss.state == 1)
            {
                gnss_cache_struct cache;
                cache.magic           = GNSS_CACHE_MAGIC;
                cache.latitude_e7     = (int32)(gnss.latitude * 1e7);
                cache.longitude_e7    = (int32)(gnss.longitude * 1e7);
                cache.altitude_cm     = (int32)(gnss.height * 100.0f);
                cache.ground_speed_cm = (int32)(gnss.speed * 100.0f);
                cache.heading_deg     = (int16)(gnss.direction * 100.0f);
                cache.year            = gnss.time.year;
                cache.month           = gnss.time.month;
                cache.day             = gnss.time.day;
                cache.hour            = gnss.time.hour;
                cache.minute          = gnss.time.minute;
                cache.second          = gnss.time.second;
                cache.satellite_used  = gnss.satellite_used;
                cache.save_count      = 1;
                memset(cache.reserved, 0, sizeof(cache.reserved));
                gnss_cache_save(&cache);
            }
        }

        Nag_Service();
        Menu();

        // 此处用户可以编写 需要循环执行的代码
    }
}

void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH0);

    pit_call_back();

}

void pit0_ch1_isr()                     // 定时器通道 1 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH1);

    key_scan();
}



// **************************** 主函数 ****************************