#include "zf_common_headfile.h"

rotation_control_struct rotation = {
    .state         = ROT_IDLE,
    .dir           = ROT_CW,
    .turn_duty     = 0,
    .max_turn_duty = 1500,
    .duration_ms   = 2000,
    .elapsed       = 0,
};

// 函数功能：启动原地旋转
// 输入参数：dir — 旋转方向 (ROT_CW/ROT_CCW)；max_turn_duty — 最大差速值；duration_ms — 时长
// 返回值：  void
// 使用示例：rotation_start(ROT_CW, 1500, 2000);
// 注意事项：仅在 rotation.state == ROT_IDLE 时有效，duration_ms 建议 ≥500ms
void rotation_start(rotation_dir_enum dir, int16 max_turn_duty, uint32 duration_ms)
{
    if (rotation.state != ROT_IDLE) return;

    rotation.dir           = dir;
    rotation.turn_duty     = 0;
    rotation.max_turn_duty = func_limit_ab(max_turn_duty, 0, 3000);
    rotation.duration_ms   = (duration_ms > 0) ? duration_ms : 2000;
    rotation.elapsed       = 0;
    rotation.state         = ROT_RUNNING;
}

// 函数功能：旋转状态机 — 每 1ms 由 pit_call_back 调用
// 输入参数：void
// 返回值：  void
// 使用示例：rotation_run();
// 注意事项：在 ROT_RUNNING 阶段，前 30% 时间线性加速到 max_turn_duty，最后 20% 线性减速到 0
void rotation_run(void)
{
    if (rotation.state != ROT_RUNNING) return;

    rotation.elapsed++;

    float progress = (float)rotation.elapsed / (float)rotation.duration_ms;

    if (progress < 0.3f)
    {
        rotation.turn_duty = (int16)((float)rotation.max_turn_duty * (progress / 0.3f));
    }
    else if (progress > 0.8f)
    {
        rotation.turn_duty = (int16)((float)rotation.max_turn_duty * ((1.0f - progress) / 0.2f));
    }
    else
    {
        rotation.turn_duty = rotation.max_turn_duty;
    }

    if (rotation.dir == ROT_CCW)
    {
        rotation.turn_duty = -rotation.turn_duty;
    }

    if (rotation.elapsed >= rotation.duration_ms)
    {
        rotation.state     = ROT_DONE;
        rotation.turn_duty = 0;
    }
}

// 函数功能：强制停止旋转
// 输入参数：void
// 返回值：  void
// 使用示例：rotation_stop();
// 注意事项：将状态重置为 ROT_IDLE，差速清零
void rotation_stop(void)
{
    rotation.state     = ROT_IDLE;
    rotation.turn_duty = 0;
    rotation.elapsed   = 0;
}

// 函数功能：检查旋转是否完成
// 输入参数：void
// 返回值：  uint8 — 1=已完成，0=未完成
// 使用示例：if (rotation_is_done()) { func_index = MENU_L1_D; }
uint8 rotation_is_done(void)
{
    return (rotation.state == ROT_DONE);
}
