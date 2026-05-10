//-------------------------------------------------------------------------------------------------------------------
// 文件名称       timer_control.c
// 功能说明       DWT 周期计数器 dt 测量模块实现
//               使用 ARM Cortex-M7 DWT->CYCCNT 寄存器（0xE0001004）
//               初始化时启用 TRCENA（CoreDebug->DEMCR bit 24）和 CYCCNTENA（DWT->CTRL bit 0）
// 修改记录
// 日期           作者           备注
// 2026-05-09    auto           first version — TCPWM-based
// 2026-05-10    auto           migrate to DWT cycle counter
//-------------------------------------------------------------------------------------------------------------------

#include "timer_control.h"

static uint8 dwt_initialized = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       dwt_init
// 功能说明       启用 DWT 周期计数器
//               设置 CoreDebug->DEMCR bit TRCENA = 1
//               清零并启动 DWT->CYCCNT
// 输入参数       void
// 返回参数       void
// 使用示例       dwt_init();     // 在 clock_init() 之后调用
// 备注信息       DWT 是 ARM 标准调试外设，所有 Cortex-M7 均支持
//               重复调用无害（已有初始化标志保护）
//-------------------------------------------------------------------------------------------------------------------
void dwt_init(void)
{
    if (!dwt_initialized)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     // 使能 DWT 访问
        DWT->CYCCNT = 0;                                    // 清零计数器
        DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;              // 使能周期计数
        dwt_initialized = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       get_timer
// 功能说明       读取 DWT->CYCCNT，计算与上次调用的时间差
//               CYCCNT 为 32-bit 增计数器，频率 = CPU 主频 (250MHz)
//               无符号算术自动处理回绕 (delta = now - last)
// 输入参数       timer_var   上次计数值（入/出参）
//               dt_out      输出时间差，单位：秒
// 返回参数       void
// 使用示例
//               static uint32 leg_time = 0;
//               float dt = 0.0f;
//               get_timer(&leg_time, &dt);
//-------------------------------------------------------------------------------------------------------------------
void get_timer(uint32 *timer_var, float *dt_out)
{
    uint32 now   = DWT->CYCCNT;
    uint32 delta = now - *timer_var;                         // 无符号减法自动处理 32-bit 回绕
    *timer_var   = now;
    *dt_out      = (float)delta / 250000000.0f;              // 250MHz → 秒
}
