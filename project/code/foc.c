#include "foc.h"
#include "zf_common_headfile.h"

// 电机控制模块

small_device_value_struct motor_value = {0};

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       foc_set_duty
// 功能说明       设置左右电机占空比（通过小驱 UART 指令发送）
// 输入参数       left_duty    左电机占空比
//                right_duty   右电机占空比
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void foc_set_duty(int16 left_duty, int16 right_duty)
{
    small_driver_set_duty(&motor_value, left_duty, right_duty);
}
