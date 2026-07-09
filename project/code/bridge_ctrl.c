#include "zf_common_headfile.h"
#include "bridge_ctrl.h"

//****************************************************************************
// 文件名称    bridge_ctrl.c
// 功能描述    单边桥控制模块实现
//             1. 5 状态状态机：IDLE -> ENTER -> CROSSING -> EXIT -> RECOVER -> IDLE
//             2. 腿高补偿：锁存桥上最大几何前馈 h_cg，并叠加单边桥专用 roll PD 闭环
//             3. 速度补偿：h_track = 2 * h_cg，Δv = v * (sqrt(h_track^2 + l^2) / l - 1)
// 依赖模块    zf_common_function (func_abs), arm_math (PI, sinf, sqrtf)
//****************************************************************************

static bridge_state_e s_state = BRIDGE_IDLE;
static float s_last_roll = 0.0f;
static float s_last_speed = 0.0f;
static float s_enter_mileage = 0.0f;
static float s_locked_cg_height_offset = 0.0f;
static uint32 s_state_timer = 0;
static uint32 s_detect_counter = 0;
static uint32 s_recover_counter = 0;
static int8 s_roll_sign = 0;
static uint8 s_forced_enter = 0;

static int8 bridge_get_side_sign(void)
{
    if (s_roll_sign != 0)
    {
        return s_roll_sign;
    }
    return (s_last_roll >= 0.0f) ? 1 : -1;
}

static float bridge_calc_cg_height_offset(float roll_abs)
{
    float half_width = BRIDGE_CAR_WIDTH / 2.0f;
    return half_width * sinf(roll_abs * DEG_TO_RAD);
}

static float bridge_get_cg_height_offset(void)
{
    float current_height_offset = bridge_calc_cg_height_offset(func_abs(s_last_roll));
    if (s_state != BRIDGE_IDLE && s_state != BRIDGE_RECOVER && s_locked_cg_height_offset > current_height_offset)
    {
        return s_locked_cg_height_offset;
    }
    return current_height_offset;
}

static void bridge_lock_cg_height_offset(float roll_abs)
{
    float current_height_offset = bridge_calc_cg_height_offset(roll_abs);
    if (current_height_offset > s_locked_cg_height_offset)
    {
        s_locked_cg_height_offset = current_height_offset;
    }
}

static float bridge_get_signed_cg_height_offset(void)
{
    float cg_height_offset = bridge_get_cg_height_offset();
    return (bridge_get_side_sign() > 0) ? cg_height_offset : -cg_height_offset;
}

static float bridge_get_track_height_delta(void)
{
    return 2.0f * bridge_get_cg_height_offset();
}

static float bridge_get_roll_pd_height_offset(void)
{
    float roll_rate_dps = BRIDGE_ROLL_PD_RATE_SIGN * (float)BRIDGE_ROLL_RATE_GYRO_DATA / GYRO_TRANSITION_FACTOR;
    float pd_height_offset = BRIDGE_ROLL_PD_KP * s_last_roll + BRIDGE_ROLL_PD_KD * roll_rate_dps;
    return func_limit_ab(pd_height_offset, -BRIDGE_ROLL_PD_MAX_CM, BRIDGE_ROLL_PD_MAX_CM);
}

//--------------------------------------------------------------------------------
// 函数介绍    初始化单边桥状态机与内部变量
// 返回参数    void
// 使用示例    bridge_init();
//--------------------------------------------------------------------------------
void bridge_init(void)
{
    s_state = BRIDGE_IDLE;
    s_last_roll = 0.0f;
    s_last_speed = 0.0f;
    s_enter_mileage = 0.0f;
    s_locked_cg_height_offset = 0.0f;
    s_state_timer = 0;
    s_detect_counter = 0;
    s_recover_counter = 0;
    s_roll_sign = 0;
    s_forced_enter = 0;
}

void bridge_force_enter(float roll_angle, float mileage)
{
    float roll_abs = func_abs(roll_angle);

    s_last_roll = roll_angle;
    s_enter_mileage = mileage;
    s_state = BRIDGE_ENTER;
    s_state_timer = 0;
    s_detect_counter = BRIDGE_DETECT_CYCLES;
    s_recover_counter = 0;
    s_forced_enter = 1;
    if(roll_angle > BRIDGE_ENTER_CANCEL_ROLL_THRESH)
    {
        s_roll_sign = 1;
    }
    else if(roll_angle < -BRIDGE_ENTER_CANCEL_ROLL_THRESH)
    {
        s_roll_sign = -1;
    }
    else if(s_roll_sign == 0)
    {
        s_roll_sign = 1;
    }
    if(roll_abs < BRIDGE_FORCE_MIN_ROLL_DEG)
    {
        roll_abs = BRIDGE_FORCE_MIN_ROLL_DEG;
    }
    bridge_lock_cg_height_offset(roll_abs);
}

//--------------------------------------------------------------------------------
// 函数介绍    获取当前单边桥状态
// 返回参数    bridge_state_e — 当前状态枚举值
//--------------------------------------------------------------------------------
bridge_state_e bridge_get_state(void)
{
    return s_state;
}

//--------------------------------------------------------------------------------
// 函数介绍    单边桥主状态机运行函数
// 参数说明    roll_angle — 当前 IMU roll 角（°）
//             mileage    — 当前车体累计里程（cm）
//             speed      — 当前车体速度（RPM）
// 返回参数    void
// 备注信息    状态转换：
//             IDLE -> ENTER   : |roll| > BRIDGE_ROLL_THRESHOLD 持续 BRIDGE_DETECT_CYCLES 周期
//             ENTER -> CROSSING: 固定延时 BRIDGE_ENTER_DELAY_CYCLES 到期
//             CROSSING -> EXIT : 里程增加 >= BRIDGE_LENGTH 或超过最大保持周期
//             EXIT -> RECOVER  : 固定延时 BRIDGE_EXIT_DELAY_CYCLES 到期
//             RECOVER -> IDLE  : |roll| < BRIDGE_RECOVER_ROLL_THRESH 持续 BRIDGE_RECOVER_CYCLES 周期
//--------------------------------------------------------------------------------
void bridge_run(float roll_angle, float mileage, float speed)
{
    float roll_abs = func_abs(roll_angle);

    s_last_roll = roll_angle;
    s_last_speed = speed;

    switch (s_state)
    {
        case BRIDGE_IDLE:
        {
            if (roll_abs > BRIDGE_ROLL_THRESHOLD)
            {
                int8 current_roll_sign = (roll_angle > 0.0f) ? 1 : -1;
                if (s_roll_sign != 0 && s_roll_sign != current_roll_sign)
                {
                    s_detect_counter = 0;
                    s_locked_cg_height_offset = 0.0f;
                }
                s_roll_sign = current_roll_sign;
                bridge_lock_cg_height_offset(roll_abs);
                s_detect_counter++;
                if (s_detect_counter >= BRIDGE_DETECT_CYCLES)
                {
                    s_state = BRIDGE_ENTER;
                    s_state_timer = 0;
                    s_enter_mileage = mileage;
                }
            }
            else
            {
                s_detect_counter = 0;
                s_locked_cg_height_offset = 0.0f;
                s_roll_sign = 0;
            }
            break;
        }

        case BRIDGE_ENTER:
        {
            int8 current_roll_sign = (roll_angle > 0.0f) ? 1 : -1;
            if (!s_forced_enter &&
                (roll_abs < BRIDGE_ENTER_CANCEL_ROLL_THRESH || current_roll_sign != s_roll_sign))
            {
                bridge_init();
                break;
            }
            bridge_lock_cg_height_offset(roll_abs);
            s_state_timer++;
            if (s_state_timer >= BRIDGE_ENTER_DELAY_CYCLES)
            {
                s_state = BRIDGE_CROSSING;
                s_state_timer = 0;
                s_forced_enter = 0;
            }
            break;
        }

        case BRIDGE_CROSSING:
        {
            float mileage_delta = func_abs(mileage - s_enter_mileage);
            bridge_lock_cg_height_offset(roll_abs);
            s_state_timer++;
            if (mileage_delta >= BRIDGE_LENGTH || s_state_timer >= BRIDGE_CROSSING_TIMEOUT_CYCLES)
            {
                s_state = BRIDGE_EXIT;
                s_state_timer = 0;
            }
            break;
        }

        case BRIDGE_EXIT:
        {
            s_state_timer++;
            if (s_state_timer >= BRIDGE_EXIT_DELAY_CYCLES)
            {
                s_state = BRIDGE_RECOVER;
                s_state_timer = 0;
                s_recover_counter = 0;
            }
            break;
        }

        case BRIDGE_RECOVER:
        {
            if (roll_abs < BRIDGE_RECOVER_ROLL_THRESH)
            {
                s_recover_counter++;
                if (s_recover_counter >= BRIDGE_RECOVER_CYCLES)
                {
                    s_state = BRIDGE_IDLE;
                    s_recover_counter = 0;
                    s_detect_counter = 0;
                    s_locked_cg_height_offset = 0.0f;
                    s_roll_sign = 0;
                }
            }
            else
            {
                s_recover_counter = 0;
            }
            break;
        }

        default:
        {
            bridge_init();
            break;
        }
    }
}

//--------------------------------------------------------------------------------
// 函数介绍    获取腿高补偿量（cm）
// 参数说明    left_delta  — 输出左腿高度补偿量（cm）
//             right_delta — 输出右腿高度补偿量（cm）
// 返回参数    void
// 备注信息    公式：L = BRIDGE_CAR_WIDTH / 2
//                  h_cg = L * sin(|roll| * DEG_TO_RAD)，进入桥后锁存最大值，避免回正后撤补偿
//                  h_signed = sign(roll) * h_cg + Kp * roll + Kd * roll_rate
//                  left = -h_signed , right = +h_signed
//--------------------------------------------------------------------------------
void bridge_get_leg_delta(float *left_delta, float *right_delta)
{
    if (s_state == BRIDGE_IDLE || s_state == BRIDGE_RECOVER)
    {
        *left_delta = 0.0f;
        *right_delta = 0.0f;
        return;
    }

    float signed_height_offset = bridge_get_signed_cg_height_offset() + bridge_get_roll_pd_height_offset();

    *left_delta = -signed_height_offset;
    *right_delta = +signed_height_offset;
}

//--------------------------------------------------------------------------------
// 函数介绍    获取速度补偿量（RPM）
// 参数说明    left_extra  — 输出左轮额外速度补偿（RPM）
//             right_extra — 输出右轮额外速度补偿（RPM）
// 返回参数    void
// 备注信息    公式：h_track = |left_delta - right_delta|，表示左右轮轨迹高度差
//                  l = BRIDGE_LENGTH
//                  v_extra = speed * (sqrt(h_track^2 + l^2) / l - 1)
//                  roll > 0 : left = v_extra , right = 0
//                  roll < 0 : left = 0 , right = v_extra
//--------------------------------------------------------------------------------
void bridge_get_speed_extra(float *left_extra, float *right_extra)
{
    if (s_state == BRIDGE_IDLE || s_state == BRIDGE_RECOVER)
    {
        *left_extra = 0.0f;
        *right_extra = 0.0f;
        return;
    }

    float h = bridge_get_track_height_delta();
    float l = BRIDGE_LENGTH;
    float v_extra = 0.0f;

    if (l > 0.0f)
    {
        v_extra = s_last_speed * (sqrtf(h * h + l * l) / l - 1.0f);
    }

    if (bridge_get_side_sign() > 0)
    {
        *left_extra = v_extra;
        *right_extra = 0.0f;
    }
    else
    {
        *left_extra = 0.0f;
        *right_extra = v_extra;
    }
}
