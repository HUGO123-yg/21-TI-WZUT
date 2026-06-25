#include "zf_common_headfile.h"

#define ROTATION_MAX_DUTY              (3000)
#define ROTATION_DEFAULT_DURATION_MS   (2000)
#define ROTATION_DEFAULT_TIMEOUT_MS    (12000)
#define ROTATION_BRAKE_MS              (180)
#define ROTATION_BRAKE_DUTY            (320)
#define ROTATION_RAMP_MS               (300)
#define ROTATION_SLOWDOWN_DEG          (270.0f)
#define ROTATION_MIN_RUN_DUTY          (240)
#define ROTATION_TARGET_TOL_DEG        (3.0f)

rotation_control_struct rotation = {
    .state         = ROT_IDLE,
    .mode          = ROT_MODE_TIME,
    .dir           = ROT_CW,
    .turn_duty     = 0,
    .max_turn_duty = 1500,
    .brake_duty    = ROTATION_BRAKE_DUTY,
    .duration_ms   = ROTATION_DEFAULT_DURATION_MS,
    .elapsed       = 0,
    .brake_elapsed = 0,
    .target_angle  = 0.0f,
    .accumulated_angle = 0.0f,
    .last_yaw      = 0.0f,
};

static float rotation_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float rotation_wrap_angle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle <= -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static int16 rotation_apply_dir(int16 duty)
{
    return (rotation.dir == ROT_CCW) ? (int16)-duty : duty;
}

static void rotation_reset_runtime(void)
{
    rotation.turn_duty         = 0;
    rotation.elapsed           = 0;
    rotation.brake_elapsed     = 0;
    rotation.accumulated_angle = 0.0f;
    rotation.last_yaw          = roll_balance_cascade.posture_value.yaw;
}

static void rotation_enter_braking(void)
{
    rotation.state         = ROT_BRAKING;
    rotation.brake_elapsed = 0;
    rotation.turn_duty     = (int16)-rotation_apply_dir(rotation.brake_duty);
}

static void rotation_update_yaw_accumulation(void)
{
    float yaw_now = roll_balance_cascade.posture_value.yaw;
    float yaw_delta = rotation_wrap_angle(yaw_now - rotation.last_yaw);

    rotation.accumulated_angle += rotation_absf(yaw_delta);
    rotation.last_yaw = yaw_now;
}

static int16 rotation_calc_time_duty(void)
{
    float progress = (float)rotation.elapsed / (float)rotation.duration_ms;
    float duty;

    if (progress < 0.3f)
    {
        duty = (float)rotation.max_turn_duty * (progress / 0.3f);
    }
    else if (progress > 0.8f)
    {
        duty = (float)rotation.max_turn_duty * ((1.0f - progress) / 0.2f);
    }
    else
    {
        duty = (float)rotation.max_turn_duty;
    }

    duty = func_limit_ab(duty, 0.0f, (float)rotation.max_turn_duty);
    return rotation_apply_dir((int16)duty);
}

static int16 rotation_calc_turns_duty(void)
{
    float remaining = rotation.target_angle - rotation.accumulated_angle;
    float duty = (float)rotation.max_turn_duty;

    if (remaining < ROTATION_SLOWDOWN_DEG)
    {
        duty = (float)ROTATION_MIN_RUN_DUTY
             + ((float)rotation.max_turn_duty - (float)ROTATION_MIN_RUN_DUTY)
             * (remaining / ROTATION_SLOWDOWN_DEG);
    }
    if (rotation.elapsed < ROTATION_RAMP_MS)
    {
        float ramp_duty = (float)rotation.max_turn_duty
                        * ((float)rotation.elapsed / (float)ROTATION_RAMP_MS);
        if (ramp_duty < duty)
        {
            duty = ramp_duty;
        }
    }

    duty = func_limit_ab(duty,
                         (rotation.elapsed < ROTATION_RAMP_MS) ? 0.0f : (float)ROTATION_MIN_RUN_DUTY,
                         (float)rotation.max_turn_duty);
    return rotation_apply_dir((int16)duty);
}

// 函数功能：启动原地旋转
// 输入参数：dir — 旋转方向 (ROT_CW/ROT_CCW)；max_turn_duty — 最大差速值；duration_ms — 时长
// 返回值：  void
// 使用示例：rotation_start(ROT_CW, 1500, 2000);
// 注意事项：仅在 rotation.state == ROT_IDLE 时有效，duration_ms 建议 ≥500ms
void rotation_start(rotation_dir_enum dir, int16 max_turn_duty, uint32 duration_ms)
{
    if (rotation_is_active()) return;

    rotation.mode          = ROT_MODE_TIME;
    rotation.dir           = dir;
    rotation.max_turn_duty = func_limit_ab(max_turn_duty, 0, ROTATION_MAX_DUTY);
    rotation.brake_duty    = func_limit_ab(ROTATION_BRAKE_DUTY, 0, ROTATION_MAX_DUTY);
    rotation.duration_ms   = (duration_ms > 0) ? duration_ms : ROTATION_DEFAULT_DURATION_MS;
    rotation.target_angle  = 0.0f;
    rotation_reset_runtime();
    rotation.state         = ROT_RUNNING;
}

// 函数功能：按圈数启动原地旋转
// 输入参数：dir — 旋转方向；max_turn_duty — 最大差速；turns — 目标圈数
// 返回值：  void
// 使用示例：rotation_start_turns(ROT_CW, 900, 3.0f);
// 注意事项：使用 IMU yaw 做跨 ±180° 的累计角度，末段降速并反向刹车
void rotation_start_turns(rotation_dir_enum dir, int16 max_turn_duty, float turns)
{
    if (rotation_is_active()) return;

    if (turns <= 0.0f)
    {
        turns = 1.0f;
    }

    rotation.mode          = ROT_MODE_TURNS;
    rotation.dir           = dir;
    rotation.max_turn_duty = func_limit_ab(max_turn_duty, ROTATION_MIN_RUN_DUTY, ROTATION_MAX_DUTY);
    rotation.brake_duty    = func_limit_ab(ROTATION_BRAKE_DUTY, 0, ROTATION_MAX_DUTY);
    rotation.duration_ms   = ROTATION_DEFAULT_TIMEOUT_MS;
    rotation.target_angle  = turns * 360.0f;
    rotation_reset_runtime();
    rotation.state         = ROT_RUNNING;
}

// 函数功能：旋转状态机 — 每 1ms 由 pit_call_back 调用
// 输入参数：void
// 返回值：  void
// 使用示例：rotation_run();
// 注意事项：定圈数模式末段降速，完成后短时反向差速刹车
void rotation_run(void)
{
    if (!rotation_is_active()) return;

    if (rotation.state == ROT_BRAKING)
    {
        int16 brake_duty;

        rotation.brake_elapsed++;
        brake_duty = rotation.brake_duty;
        if (rotation.brake_elapsed > (ROTATION_BRAKE_MS / 2))
        {
            uint32 remain = ROTATION_BRAKE_MS - rotation.brake_elapsed;
            brake_duty = (int16)((float)rotation.brake_duty
                       * ((float)remain / ((float)ROTATION_BRAKE_MS / 2.0f)));
        }
        brake_duty = func_limit_ab(brake_duty, 0, rotation.brake_duty);
        rotation.turn_duty = (int16)-rotation_apply_dir(brake_duty);

        if (rotation.brake_elapsed >= ROTATION_BRAKE_MS)
        {
            rotation.state     = ROT_DONE;
            rotation.turn_duty = 0;
        }
        return;
    }

    rotation.elapsed++;

    if (rotation.mode == ROT_MODE_TURNS)
    {
        rotation_update_yaw_accumulation();
        if ((rotation.accumulated_angle >= (rotation.target_angle - ROTATION_TARGET_TOL_DEG))
            || (rotation.elapsed >= rotation.duration_ms))
        {
            rotation_enter_braking();
            return;
        }
        rotation.turn_duty = rotation_calc_turns_duty();
    }
    else
    {
        if (rotation.elapsed >= rotation.duration_ms)
        {
            rotation_enter_braking();
            return;
        }
        rotation.turn_duty = rotation_calc_time_duty();
    }
}

// 函数功能：强制停止旋转
// 输入参数：void
// 返回值：  void
// 使用示例：rotation_stop();
// 注意事项：将状态重置为 ROT_IDLE，差速清零
void rotation_stop(void)
{
    rotation.state        = ROT_IDLE;
    rotation.mode         = ROT_MODE_TIME;
    rotation.target_angle = 0.0f;
    rotation_reset_runtime();
}

// 函数功能：检查旋转是否完成
// 输入参数：void
// 返回值：  uint8 — 1=已完成，0=未完成
// 使用示例：if (rotation_is_done()) { func_index = MENU_L1_D; }
uint8 rotation_is_done(void)
{
    return (rotation.state == ROT_DONE);
}

// 函数功能：检查旋转控制是否仍在占用电机差速
// 输入参数：void
// 返回值：  uint8 — 1=运行/刹车中，0=空闲或已完成
uint8 rotation_is_active(void)
{
    return ((rotation.state == ROT_RUNNING) || (rotation.state == ROT_BRAKING)) ? 1 : 0;
}

// 函数功能：旋转状态名
// 输入参数：state — 旋转状态
// 返回值：  const char*
const char *rotation_state_name(rotation_state_enum state)
{
    switch (state)
    {
    case ROT_IDLE:    return "Idle";
    case ROT_RUNNING: return "Running";
    case ROT_BRAKING: return "Braking";
    case ROT_DONE:    return "Done";
    default:          return "Unknown";
    }
}
