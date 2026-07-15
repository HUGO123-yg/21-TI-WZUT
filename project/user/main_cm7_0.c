/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
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
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
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
#include "config.h"
#include "Control_system.h"
#include "Menu.h"
#include "Pit_scheduler.h"
#include "Terrain_vision.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等

    // 先初始化逐飞 Work Flash 驱动，再读取项目层的双备份路径目录。
    // nav_flash_init() 首次上电只读取，不会无故擦写空白 Flash。
    flash_init();
    if (NAV_FLASH_STATUS_OK != nav_flash_init())
    {
        zf_log(0, "navigation flash init error.");
    }

    if (CONTROL_STATUS_OK != control_system_init())
    {
        zf_log(0, "control system init error.");
    }

    if (TERRAIN_VISION_STATUS_OK != terrain_vision_init())
    {
        zf_log(0, "terrain vision init error.");
    }

    menu_init();

    pit_scheduler_init();
    pit_ms_init(PIT_CH0,1);

    // 此处编写用户代码 例如外设初始化代码等
    while(true)
    {
        menu_task();
        terrain_vision_task();

        // 此处编写需要循环执行的代码
//          CYT2_D_motor_ctrl(1000,1000);
//      printf("%d,%d,%d\n",imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_gyro_z);
//        printf("%f,%f,%f\r\n",roll_balance_cascade.posture_value.pit, roll_balance_cascade.posture_value.rol, -roll_balance_cascade.posture_value.yaw);
//        printf("%d,%d\r\n",motor_value.receive_left_speed_data , motor_value.receive_right_speed_data );
//        printf("%d,%d\r\n",left_motor_duty ,right_motor_duty);
//        printf("%f\r\n",Car.mileage);

//      system_delay_ms(10);

        // Flash 页写入是阻塞操作，只允许在主循环执行；中断侧录制接口
        // 仅把采样写入 RAM 双缓冲，并通知这里处理待写页面。
        (void)nav_flash_service();

        // 此处编写需要循环执行的代码
    }
}

void pit0_ch0_isr()                     // 定时器通道 0 周期中断服务函数
{
    uint32 start_cycles;

    start_cycles = pit_scheduler_measure_begin();
    pit_isr_flag_clear(PIT_CH0);
    pit_scheduler_schedule_tick_from_isr();
    menu_tick_1ms();
    pit_scheduler_record_pit_isr(start_cycles);
}

void PendSV_Handler(void)
{
    pit_scheduler_run_deferred();
}

void pit0_ch1_isr()                     // 定时器通道 1 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH1);

}



// **************************** 代码区域 ****************************
