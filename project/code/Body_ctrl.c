#include "zf_common_headfile.h"

#define WHEEL_CIRCUMFERENCE  (6.4f)
#define SPEED_TO_ANGLE_GAIN  (0.003f)    // 速度环输出 → 角度目标 (度/out单位)
#define STRAIGHT_TEST_SPEED  (200.0f)
#define STRAIGHT_SLOWDOWN_CM (9900.0f)
#define STRAIGHT_TARGET_CM   (10000.0f)
#define JUMP_PREPARE_RATIO   (0.35f)
#define JUMP_MOTOR_BOOST_MIN (0)
#define JUMP_MOTOR_BOOST_MAX (1200)
#define STARTUP_SENSOR_SETTLE_MS       (650)
#define STARTUP_ZERO_SAMPLE_START_MS   (120)
#define STARTUP_ZERO_SAMPLE_END_MS     (620)
#define STARTUP_ZERO_SAMPLE_MIN        (80)
#define STARTUP_ZERO_GYRO_MAX          (80)
#define STARTUP_MECH_ZERO_MIN          (-12.0f)
#define STARTUP_MECH_ZERO_MAX          (2.0f)
#define STARTUP_CONTROL_RAMP_MS        (900)
#define STARTUP_CONTROL_RAMP_MIN       (0.15f)

//================================================================================
// 全局变量
//================================================================================
int16   balance_duty_max  = 3000;
int16   turn_duty_max     = 3000;
float   target_speed      = 0;
int     run_state         = 1;
volatile uint32  sys_times         = 0;
int     STOP_FLAG         = 1;
int32   car_distance      = 0;
int16   left_motor_duty   = 0;
int16   right_motor_duty  = 0;
int     jump_time         = 0;          // 保留兼容，由 jump_cfg.elapsed 替代
static int16 jump_motor_boost_duty = 0;
static pid_cycle_struct jump_saved_angular_speed_cycle;
static pid_cycle_struct jump_saved_angle_cycle;
static pid_cycle_struct jump_saved_speed_cycle;

//================================================================================
// 跳跃配置 — 默认值
//================================================================================
jump_config_struct jump_cfg = {
    // 阶段时长（PIT ticks）
    .prepare_ticks          = 70,
    .charge_ticks           = 90,
    .launch_ticks           = 45,
    .airborne_timeout       = 260,
    .landing_ticks          = 90,
    .recover_ticks          = 200,

    // 舵机占空比偏移
    .charge_duty            = 1200,
    .launch_duty            = 1700,
    .preland_duty           = 800,
    .land_damping_duty      = 28,

    // 前进动量
    .forward_tilt_target    = 5.0f,
    .forward_motor_boost    = 300.0f,
    .speed_recovery_rate    = 0.3f,

    // PID 抑制
    .airborne_pid_scale     = 0.12f,
    .landing_pid_scale      = 0.3f,
    .recover_pid_ramp_rate  = 0.01f,

    // IMU 检测阈值
    .airborne_acc_threshold = 0.3f,
    .landing_acc_threshold  = 1.8f,
    .max_tilt_abort         = 55.0f,

    // 视觉接口
    .vision_jump_trigger    = NULL,
    .vision_jump_enable     = 0,
    .vision_obstacle_dist   = 0.0f,
    .vision_min_dist        = 100.0f,
    .vision_max_dist        = 800.0f,

    // 运行时
    .state                  = JUMP_IDLE,
    .elapsed                = 0,
    .jump_count             = 0,
    .peak_acc_magnitude     = 0.0f,
    .stored_p_angle         = 0.0f,
    .stored_p_speed         = 0.0f,
    .stored_speed_target    = 0.0f,
    .last_trigger_result    = JUMP_TRIGGER_OK,
};

//================================================================================
// 计算当前总倾斜角（roll + pitch 合成）
//================================================================================
static float compute_tilt_angle(void)
{
    float rol = func_abs(roll_balance_cascade.posture_value.rol);
    float pit = func_abs(roll_balance_cascade.posture_value.pit);
    return sqrt(rol * rol + pit * pit);
}

static void jump_reset_pid_memory(void)
{
    roll_balance_cascade.angle_cycle.i_value = 0;
    roll_balance_cascade.angle_cycle.out = 0;
    roll_balance_cascade.angle_cycle.p_value_last = 0;
    roll_balance_cascade.angular_speed_cycle.i_value = 0;
    roll_balance_cascade.angular_speed_cycle.out = 0;
    roll_balance_cascade.angular_speed_cycle.p_value_last = 0;
    roll_balance_cascade.speed_cycle.i_value = 0;
    roll_balance_cascade.speed_cycle.out = 0;
    roll_balance_cascade.speed_cycle.p_value_last = 0;
    pitch_balance_cascade.angle_cycle.i_value = 0;
    pitch_balance_cascade.angle_cycle.out = 0;
    pitch_balance_cascade.angle_cycle.p_value_last = 0;
}

static uint8 jump_speed_loop_should_update(void)
{
    switch (jump_cfg.state)
    {
    case JUMP_PREPARE:
    case JUMP_CHARGE:
    case JUMP_LANDING:
    case JUMP_RECOVER:
        return 1;
    default:
        return 0;
    }
}

static uint8 balance_speed_loop_enabled(void)
{
    if (stair_px_angle_control_active())
        return 0;
    if (jump_cfg.state == JUMP_IDLE)
        return 1;
    return jump_speed_loop_should_update();
}

static void balance_speed_loop_clear(void)
{
    roll_balance_cascade.speed_cycle.i_value = 0;
    roll_balance_cascade.speed_cycle.out = 0;
    roll_balance_cascade.speed_cycle.p_value_last = 0;
}

static void jump_apply_fixed_pid(void)
{
    roll_balance_cascade.angle_cycle = jump_saved_angle_cycle;
    roll_balance_cascade.angular_speed_cycle = jump_saved_angular_speed_cycle;
    roll_balance_cascade.speed_cycle = jump_saved_speed_cycle;

    roll_balance_cascade.angle_cycle.i = 0.0f;
    roll_balance_cascade.angle_cycle.d = 0.0f;
    roll_balance_cascade.angular_speed_cycle.i = 0.0f;
    roll_balance_cascade.angular_speed_cycle.d = 0.0f;
    roll_balance_cascade.speed_cycle.p = jump_saved_speed_cycle.p;
    roll_balance_cascade.speed_cycle.i = 0.0f;
    roll_balance_cascade.speed_cycle.d = 0.0f;

    jump_reset_pid_memory();
}

static uint8 startup_zero_ready = 0;
static float startup_pitch_sum = 0.0f;
static uint16 startup_pitch_count = 0;

static void startup_clear_pid_state(void)
{
    roll_balance_cascade.angle_cycle.i_value = 0;
    roll_balance_cascade.angle_cycle.out = 0;
    roll_balance_cascade.speed_cycle.i_value = 0;
    roll_balance_cascade.speed_cycle.out = 0;
    roll_balance_cascade.angular_speed_cycle.i_value = 0;
    roll_balance_cascade.angular_speed_cycle.out = 0;
    pitch_balance_cascade.angle_cycle.i_value = 0;
    target_speed = 0.0f;
}

static void startup_mechanical_match_update(void)
{
    float matched_zero;

    if (startup_zero_ready)
    {
        return;
    }

    if (sys_times >= STARTUP_ZERO_SAMPLE_START_MS
        && sys_times <= STARTUP_ZERO_SAMPLE_END_MS
        && func_abs(imu660rb_gyro_y) <= STARTUP_ZERO_GYRO_MAX
        && compute_tilt_angle() <= jump_cfg.max_tilt_abort)
    {
        startup_pitch_sum += roll_balance_cascade.posture_value.pit;
        startup_pitch_count++;
    }

    if (sys_times < STARTUP_SENSOR_SETTLE_MS)
    {
        return;
    }

    if (startup_pitch_count >= STARTUP_ZERO_SAMPLE_MIN)
    {
        matched_zero = startup_pitch_sum / (float)startup_pitch_count;
        matched_zero = func_limit_ab(matched_zero,
                                     STARTUP_MECH_ZERO_MIN,
                                     STARTUP_MECH_ZERO_MAX);
        roll_balance_cascade.posture_value.mechanical_zero = matched_zero;
        roll_balance_cascade_resave.posture_value.mechanical_zero = matched_zero;
    }

    startup_zero_ready = 1;
    startup_clear_pid_state();
}

static float startup_control_ramp(void)
{
    uint32 ramp_time;
    float ramp;

    if (sys_times <= STARTUP_SENSOR_SETTLE_MS)
    {
        return 0.0f;
    }

    ramp_time = sys_times - STARTUP_SENSOR_SETTLE_MS;
    if (ramp_time >= STARTUP_CONTROL_RAMP_MS)
    {
        return 1.0f;
    }

    ramp = STARTUP_CONTROL_RAMP_MIN
         + (1.0f - STARTUP_CONTROL_RAMP_MIN)
         * ((float)ramp_time / (float)STARTUP_CONTROL_RAMP_MS);
    return func_limit_ab(ramp, STARTUP_CONTROL_RAMP_MIN, 1.0f);
}

static float jump_phase_progress(uint16 elapsed, uint16 total)
{
    float progress;

    if (total == 0)
        return 1.0f;

    progress = (float)elapsed / (float)total;
    return func_limit_ab(progress, 0.0f, 1.0f);
}

static void jump_restore_saved_control(uint8 stop_speed)
{
    roll_balance_cascade.angle_cycle = jump_saved_angle_cycle;
    roll_balance_cascade.angular_speed_cycle = jump_saved_angular_speed_cycle;
    roll_balance_cascade.speed_cycle = jump_saved_speed_cycle;
    pitch_balance_cascade.angle_cycle.i_value = 0;
    pitch_balance_cascade.angle_cycle.out = 0;
    pitch_balance_cascade.angle_cycle.p_value_last = 0;

    target_speed = stop_speed ? 0.0f : jump_cfg.stored_speed_target;
}

static void jump_enter_state(uint8 state)
{
    jump_cfg.state   = state;
    jump_cfg.elapsed = 0;
}

//================================================================================
// 跳跃舵机物理偏移换算
//================================================================================
static int16 jump_steer_duty_from_offset(const steer_control_struct *control_data, int16 physical_offset)
{
    int16 duty = control_data->center_num + physical_offset * control_data->steer_dir;
    return func_limit_ab(duty, 0, 10000);
}

static void steer_set_default_pose(void)
{
    steer_duty_set(&steer_1, jump_steer_duty_from_offset(&steer_1, STEER_1_DEFAULT_OFFSET));
    steer_duty_set(&steer_2, jump_steer_duty_from_offset(&steer_2, STEER_2_DEFAULT_OFFSET));
    steer_duty_set(&steer_3, jump_steer_duty_from_offset(&steer_3, STEER_3_DEFAULT_OFFSET));
    steer_duty_set(&steer_4, jump_steer_duty_from_offset(&steer_4, STEER_4_DEFAULT_OFFSET));
}

static void jump_set_all_leg_offset(int16 offset)
{
    steer_duty_set(&steer_1, jump_steer_duty_from_offset(&steer_1, STEER_1_DEFAULT_OFFSET + offset));
    steer_duty_set(&steer_2, jump_steer_duty_from_offset(&steer_2, STEER_2_DEFAULT_OFFSET + offset));
    steer_duty_set(&steer_3, jump_steer_duty_from_offset(&steer_3, STEER_3_DEFAULT_OFFSET + offset));
    steer_duty_set(&steer_4, jump_steer_duty_from_offset(&steer_4, STEER_4_DEFAULT_OFFSET + offset));
}

static void jump_set_neutral_leg_offset(void)
{
    steer_set_default_pose();
}

//================================================================================
// 跳跃触发
//================================================================================
uint8 jump_trigger(void)
{
    if (jump_cfg.state != JUMP_IDLE)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_BUSY;
        return JUMP_TRIGGER_BUSY;
    }
    if (!run_state)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_NOT_RUNNING;
        return JUMP_TRIGGER_NOT_RUNNING;
    }
    if (compute_tilt_angle() > jump_cfg.max_tilt_abort)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_TILT;
        return JUMP_TRIGGER_TILT;
    }

    jump_cfg.state               = JUMP_PREPARE;
    jump_cfg.elapsed             = 0;
    jump_cfg.peak_acc_magnitude  = 0.0f;
    jump_cfg.last_trigger_result = JUMP_TRIGGER_OK;

    // 保存进入跳跃前的 PID 参数，用于恢复
    jump_saved_angle_cycle         = roll_balance_cascade.angle_cycle;
    jump_saved_angular_speed_cycle = roll_balance_cascade.angular_speed_cycle;
    jump_saved_speed_cycle         = roll_balance_cascade.speed_cycle;
    jump_cfg.stored_p_angle            = roll_balance_cascade.angle_cycle.p;
    jump_cfg.stored_p_speed            = roll_balance_cascade.speed_cycle.p;
    jump_cfg.stored_speed_target = target_speed;

    jump_apply_fixed_pid();
    jump_set_neutral_leg_offset();
    target_speed = jump_cfg.stored_speed_target;

    return JUMP_TRIGGER_OK;
}

//================================================================================
// 紧急终止跳跃
//================================================================================
void jump_abort(void)
{
    uint8 was_active = (jump_cfg.state != JUMP_IDLE) ? 1 : 0;

    jump_enter_state(JUMP_IDLE);
    jump_motor_boost_duty = 0;

    if (was_active)
    {
        jump_restore_saved_control(1);
    }
    jump_set_neutral_leg_offset();
}

//================================================================================
// 视觉模块更新障碍物距离
//================================================================================
void jump_vision_update(float distance_mm)
{
    jump_cfg.vision_obstacle_dist = distance_mm;

    if (!jump_cfg.vision_jump_enable)                          return;
    if (jump_cfg.state != JUMP_IDLE)                           return;
    if (distance_mm > jump_cfg.vision_max_dist)                return;
    if (distance_mm < jump_cfg.vision_min_dist)                return;
    if (jump_cfg.vision_jump_trigger != NULL)
    {
        jump_cfg.vision_jump_trigger(distance_mm);
    }
    else
    {
        (void)jump_trigger();
    }
}

//================================================================================
// 重置跳跃参数为默认值
//================================================================================
void jump_config_default(void)
{
    if (jump_cfg.state != JUMP_IDLE)
    {
        jump_abort();
    }

    jump_cfg.prepare_ticks         = 70;
    jump_cfg.charge_ticks          = 90;
    jump_cfg.launch_ticks          = 45;
    jump_cfg.airborne_timeout      = 260;
    jump_cfg.landing_ticks         = 90;
    jump_cfg.recover_ticks         = 200;
    jump_cfg.charge_duty           = 1200;
    jump_cfg.launch_duty           = 1700;
    jump_cfg.preland_duty          = 800;
    jump_cfg.land_damping_duty     = 28;
    jump_cfg.forward_tilt_target   = 5.0f;
    jump_cfg.forward_motor_boost   = 300.0f;
    jump_cfg.speed_recovery_rate   = 0.3f;
    jump_cfg.airborne_pid_scale    = 0.12f;
    jump_cfg.landing_pid_scale     = 0.3f;
    jump_cfg.recover_pid_ramp_rate = 0.01f;
    jump_cfg.airborne_acc_threshold = 0.3f;
    jump_cfg.landing_acc_threshold  = 1.8f;
    jump_cfg.max_tilt_abort        = 55.0f;
    jump_cfg.vision_jump_enable    = 0;
    jump_cfg.vision_min_dist       = 100.0f;
    jump_cfg.vision_max_dist       = 800.0f;
    jump_cfg.state                 = JUMP_IDLE;
    jump_cfg.elapsed               = 0;
    jump_cfg.peak_acc_magnitude    = 0.0f;
    jump_cfg.last_trigger_result   = JUMP_TRIGGER_OK;
    jump_motor_boost_duty          = 0;
    jump_set_neutral_leg_offset();
}

//================================================================================
// 检查是否可以触发跳跃
//================================================================================
uint8 jump_can_trigger(void)
{
    if (jump_cfg.state != JUMP_IDLE)                           return 0;
    if (!run_state)                                            return 0;
    if (compute_tilt_angle() > jump_cfg.max_tilt_abort)        return 0;
    return 1;
}

const char *jump_state_name(uint8 state)
{
    switch (state)
    {
    case JUMP_IDLE:     return "IDLE";
    case JUMP_PREPARE:  return "PREP";
    case JUMP_CHARGE:   return "CHARGE";
    case JUMP_LAUNCH:   return "LAUNCH";
    case JUMP_AIRBORNE: return "AIR";
    case JUMP_LANDING:  return "LAND";
    case JUMP_RECOVER:  return "RECOVER";
    default:            return "UNKNOWN";
    }
}

const char *jump_trigger_result_name(uint8 result)
{
    switch (result)
    {
    case JUMP_TRIGGER_OK:          return "OK";
    case JUMP_TRIGGER_BUSY:        return "BUSY";
    case JUMP_TRIGGER_NOT_RUNNING: return "STOP";
    case JUMP_TRIGGER_TILT:        return "TILT";
    default:                       return "UNKNOWN";
    }
}

uint8 jump_is_active(void)
{
    return (jump_cfg.state != JUMP_IDLE) ? 1 : 0;
}

//================================================================================
// 车辆状态计算 — 含跳跃 PID 管理
//================================================================================
void car_state_calculate(void)
{
    //---------- 倾斜保护 ----------
    if (compute_tilt_angle() > jump_cfg.max_tilt_abort)
    {
        if (jump_cfg.state != JUMP_IDLE)
        {
            jump_abort();
        }
        run_state = 0;
        roll_balance_cascade.angular_speed_cycle.i_value  = 0;
        pitch_balance_cascade.angle_cycle.i_value          = 0;
    }
    else
    {
        if (run_state == 0)
        {
            sys_times = 0;
        }
        run_state = 1;
    }

    //---------- PID 软启动（非跳跃时）----------
    if (jump_cfg.state == JUMP_IDLE)
    {
        float ramp = startup_control_ramp();
        if (ramp < 1.0f)
        {
            roll_balance_cascade.angle_cycle.p  = roll_balance_cascade_resave.angle_cycle.p  * ramp;
            roll_balance_cascade.speed_cycle.p  = roll_balance_cascade_resave.speed_cycle.p  * ramp;
            roll_balance_cascade.angle_cycle.i_value = 0;
            roll_balance_cascade.speed_cycle.i_value = 0;
            target_speed = 0.0f;
        }
        else
        {
            roll_balance_cascade.angle_cycle.p  = roll_balance_cascade_resave.angle_cycle.p;
            roll_balance_cascade.speed_cycle.p  = roll_balance_cascade_resave.speed_cycle.p;
        }
        return;
    }

    //---------- 跳跃中的 PID 管理 ----------
    // 姿态环始终保持原始 Kp；速度环只在接地相关阶段用纯 P 更新。
    roll_balance_cascade.angle_cycle.p = jump_saved_angle_cycle.p;
    roll_balance_cascade.angle_cycle.i = 0.0f;
    roll_balance_cascade.angle_cycle.d = 0.0f;
    roll_balance_cascade.angular_speed_cycle.p = jump_saved_angular_speed_cycle.p;
    roll_balance_cascade.angular_speed_cycle.i = 0.0f;
    roll_balance_cascade.angular_speed_cycle.d = 0.0f;
    roll_balance_cascade.speed_cycle.p = jump_speed_loop_should_update()
                                       ? jump_saved_speed_cycle.p
                                       : 0.0f;
    roll_balance_cascade.speed_cycle.i = 0.0f;
    roll_balance_cascade.speed_cycle.d = 0.0f;
    if (!jump_speed_loop_should_update())
    {
        balance_speed_loop_clear();
    }
    pitch_balance_cascade.angle_cycle.i_value = 0;
}

//================================================================================
// 舵机控制 — 含 7 阶段跳跃状态机
//================================================================================
void car_steer_control(void)
{
    int16 steer_location_offset[4] = {0};
    int16 steer_target_offset[4]   = {0};
    int16 bridge_steer_offset[4]   = {0};
    float steer_balance_angle      = 0;
    float steer_balance_angle_count_local = 0;
    float steer_output_duty_filter = 0;
    static float s_filter          = 0;
    static float s_angle_count     = 0;
    int16 speed_steer;
    int16 normal_steer_rate = (int16)(2.0f + 8.0f * startup_control_ramp());
    if (normal_steer_rate < 1)
        normal_steer_rate = 1;

    //---------- 俯仰对转向的影响 ----------
    float pitch_offset = (30.0f - func_limit_ab(func_abs(
        roll_balance_cascade.posture_value.rol + roll_balance_cascade.posture_value.mechanical_zero
    ), 0.0f, 30.0f)) / 30.0f;

    if (stair_px_angle_control_active())
    {
        speed_steer = 0;
    }
    else
    {
        speed_steer = func_limit_ab((int16)(roll_balance_cascade.speed_cycle.out / 7.0f), -250, 250) * 6;
        speed_steer = (int16)((float)speed_steer * pitch_offset);
    }

    s_filter = (s_filter * 8 + (float)speed_steer) / 10.0f;
    steer_output_duty_filter = s_filter;

    //---------- 转向平衡角（跳跃时冻结）----------
    if (jump_cfg.state == JUMP_IDLE)
    {
        if (sys_times < 2000)
        {
            steer_balance_angle = 0;
            pitch_balance_cascade.angle_cycle.i_value = 0;
        }
        else
        {
            steer_balance_angle = func_limit_ab(pitch_balance_cascade.angle_cycle.out, -300, 300) * 6;
        }
        s_angle_count = steer_balance_angle;
    }
    steer_balance_angle_count_local = s_angle_count;

    //---------- 计算舵机位置偏差 ----------
    steer_location_offset[0] = (steer_1.now_location - steer_1.center_num) * steer_1.steer_dir;
    steer_location_offset[1] = (steer_2.now_location - steer_2.center_num) * steer_2.steer_dir;
    steer_location_offset[2] = (steer_3.now_location - steer_3.center_num) * steer_3.steer_dir;
    steer_location_offset[3] = (steer_4.now_location - steer_4.center_num) * steer_4.steer_dir;

    //---------- 正常模式目标偏移 ----------
    if (rotation_is_active())
    {
        // 旋转模式：双腿对称差速，左右两侧获得相反修正
        steer_target_offset[0] = (int16)(STEER_1_DEFAULT_OFFSET + steer_output_duty_filter + steer_balance_angle_count_local);
        steer_target_offset[1] = (int16)(STEER_2_DEFAULT_OFFSET + steer_output_duty_filter - steer_balance_angle_count_local);
        steer_target_offset[2] = (int16)(STEER_3_DEFAULT_OFFSET - steer_output_duty_filter + steer_balance_angle_count_local);
        steer_target_offset[3] = (int16)(STEER_4_DEFAULT_OFFSET - steer_output_duty_filter - steer_balance_angle_count_local);
    }
    else
    {
        steer_target_offset[0] = (int16)(STEER_1_DEFAULT_OFFSET + steer_output_duty_filter - (steer_balance_angle_count_local > 0 ? 0 : steer_balance_angle_count_local));
        steer_target_offset[1] = (int16)(STEER_2_DEFAULT_OFFSET + steer_output_duty_filter + (steer_balance_angle_count_local < 0 ? 0 : steer_balance_angle_count_local));
        steer_target_offset[2] = (int16)(STEER_3_DEFAULT_OFFSET - steer_output_duty_filter - (steer_balance_angle_count_local > 0 ? 0 : steer_balance_angle_count_local));
        steer_target_offset[3] = (int16)(STEER_4_DEFAULT_OFFSET - steer_output_duty_filter + (steer_balance_angle_count_local < 0 ? 0 : steer_balance_angle_count_local));
    }

    one_bridge_get_steer_offsets(bridge_steer_offset);
    steer_target_offset[0] += bridge_steer_offset[0];
    steer_target_offset[1] += bridge_steer_offset[1];
    steer_target_offset[2] += bridge_steer_offset[2];
    steer_target_offset[3] += bridge_steer_offset[3];

    if (run_state == 0)
    {
        // 停机 — 缓慢回默认站姿
        steer_control(&steer_1, func_limit_ab(STEER_1_DEFAULT_OFFSET - steer_location_offset[0], -1, 1));
        steer_control(&steer_2, func_limit_ab(STEER_2_DEFAULT_OFFSET - steer_location_offset[1], -1, 1));
        steer_control(&steer_3, func_limit_ab(STEER_3_DEFAULT_OFFSET - steer_location_offset[2], -1, 1));
        steer_control(&steer_4, func_limit_ab(STEER_4_DEFAULT_OFFSET - steer_location_offset[3], -1, 1));
        return;
    }

    //================================================================
    // 跳跃状态机
    //================================================================
    if (jump_cfg.state == JUMP_IDLE)
    {
        // 正常平衡转向
        steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -normal_steer_rate, normal_steer_rate));
        steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -normal_steer_rate, normal_steer_rate));
        steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -normal_steer_rate, normal_steer_rate));
        steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -normal_steer_rate, normal_steer_rate));
        return;
    }

    jump_cfg.elapsed++;

    jump_motor_boost_duty = 0;

    switch (jump_cfg.state)
    {
        //----------------------------------------------------------------
        // PREPARE — 降低重心，前倾蓄势
        //----------------------------------------------------------------
        case JUMP_PREPARE:
        {
            float progress = jump_phase_progress(jump_cfg.elapsed, jump_cfg.prepare_ticks);
            int16 crouch = -(int16)((float)jump_cfg.charge_duty * JUMP_PREPARE_RATIO * progress);
            jump_set_all_leg_offset(crouch);
            target_speed = jump_cfg.stored_speed_target;

            if (jump_cfg.elapsed >= jump_cfg.prepare_ticks)
            {
                jump_enter_state(JUMP_CHARGE);
            }
        }
            break;

        //----------------------------------------------------------------
        // CHARGE — 四腿同步压缩储能 + 保持车身稳定
        //----------------------------------------------------------------
        case JUMP_CHARGE:
        {
            float progress = jump_phase_progress(jump_cfg.elapsed, jump_cfg.charge_ticks);
            int16 start_crouch = -(int16)((float)jump_cfg.charge_duty * JUMP_PREPARE_RATIO);
            int16 end_crouch   = -jump_cfg.charge_duty;
            int16 crouch = start_crouch + (int16)((float)(end_crouch - start_crouch) * progress);
            jump_set_all_leg_offset(crouch);

            // 蓄力阶段保持原速度目标，避免起跳前额外加速把车身带倒。
            target_speed = jump_cfg.stored_speed_target;

            if (jump_cfg.elapsed >= jump_cfg.charge_ticks)
            {
                jump_enter_state(JUMP_LAUNCH);
            }
        }
            break;

        //----------------------------------------------------------------
        // LAUNCH — 释放能量 + 电机助推前冲
        //----------------------------------------------------------------
        case JUMP_LAUNCH:
            jump_motor_boost_duty = func_limit_ab((int16)jump_cfg.forward_motor_boost,
                                                  JUMP_MOTOR_BOOST_MIN,
                                                  JUMP_MOTOR_BOOST_MAX);

            // 舵机：从下蹲位快速伸腿，形成向上的冲量。
            // steer: fast extension from crouch to produce upward impulse.
            jump_set_all_leg_offset(jump_cfg.launch_duty);

            // 前进动量：额外电机推力
            // forward momentum: extra motor boost
            target_speed = jump_cfg.stored_speed_target + jump_cfg.forward_motor_boost * 0.006f;

            if (jump_cfg.elapsed >= jump_cfg.launch_ticks)
            {
                jump_enter_state(JUMP_AIRBORNE);
            }
            break;

        //----------------------------------------------------------------
        // AIRBORNE — 固定腾空时间，等待落地缓冲
        //----------------------------------------------------------------
        case JUMP_AIRBORNE:
            // 腿部微伸 — 预着陆位，增大落地缓冲行程
            jump_set_all_leg_offset(jump_cfg.preland_duty);

            // 超时保护
            // timeout protection
            if (jump_cfg.elapsed >= jump_cfg.airborne_timeout)
            {
                jump_enter_state(JUMP_LANDING);
            }
            break;

        //----------------------------------------------------------------
        // LANDING — 主动缓冲吸收冲击
        //----------------------------------------------------------------
        case JUMP_LANDING:
        {
            // 四轮同时内收，吸收冲击
            // all 4 wheels retract inward for impact absorption
            int16 damp = jump_cfg.land_damping_duty;
            steer_control(&steer_1, -damp);
            steer_control(&steer_2, -damp);
            steer_control(&steer_3, -damp);
            steer_control(&steer_4, -damp);

            // 前进动量恢复
            target_speed = jump_cfg.stored_speed_target * jump_cfg.speed_recovery_rate;

            if (jump_cfg.elapsed >= jump_cfg.landing_ticks)
            {
                jump_enter_state(JUMP_RECOVER);
            }
        }
            break;

        //----------------------------------------------------------------
        // RECOVER — 逐步恢复 PID 和正常平衡
        //----------------------------------------------------------------
        case JUMP_RECOVER:
        {
            jump_motor_boost_duty = 0;

            // 恢复转向控制
            int16 rate = jump_cfg.land_damping_duty / 2;
            if (rate < 1)
                rate = 1;
            steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -rate, rate));
            steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -rate, rate));
            steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -rate, rate));
            steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -rate, rate));

            // PID 由 car_state_calculate() 管理爬升
            // PID ramp managed by car_state_calculate()
            target_speed = jump_cfg.stored_speed_target
                         * (jump_cfg.speed_recovery_rate
                            + (1.0f - jump_cfg.speed_recovery_rate)
                            * jump_phase_progress(jump_cfg.elapsed, jump_cfg.recover_ticks));

            if (jump_cfg.elapsed >= jump_cfg.recover_ticks)
            {
                jump_enter_state(JUMP_IDLE);
                jump_cfg.jump_count++;

                // 完全恢复 PID
                jump_restore_saved_control(0);
            }
        }
            break;

        default:
            jump_motor_boost_duty = 0;
            break;
    }
}

//================================================================================
// 电机控制 — 含跳跃助推
//================================================================================
void car_motor_control(void)
{
    car_distance += ((float)car_speed / 60.0f * WHEEL_CIRCUMFERENCE * PI * 0.001f);

    if (run_state)
    {
        left_motor_duty  = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);
        right_motor_duty = func_limit_ab((int16)roll_balance_cascade.angular_speed_cycle.out, -balance_duty_max, balance_duty_max);

        // 跳跃助推：起跳阶段施加额外前冲力
        // jump boost: extra forward thrust during launch
        if (jump_cfg.state == JUMP_LAUNCH)
        {
            int16 boost = (int16)jump_cfg.forward_motor_boost;
            left_motor_duty  = func_limit_ab(left_motor_duty  + boost, -balance_duty_max, balance_duty_max);
            right_motor_duty = func_limit_ab(right_motor_duty + boost, -balance_duty_max, balance_duty_max);
        }

        // Z 轴陀螺仪辅助转向
        left_motor_duty   = func_limit_ab(left_motor_duty  + imu660rb_gyro_z / 3, -turn_duty_max, turn_duty_max);
        right_motor_duty  = func_limit_ab(right_motor_duty - imu660rb_gyro_z / 3, -turn_duty_max, turn_duty_max);
    }
    else
    {
        left_motor_duty  = 0;
        right_motor_duty = 0;
    }

    small_driver_set_duty(left_motor_duty, -right_motor_duty);
}

//================================================================================
// 直行100m综合测试模块
//================================================================================
straight_test_struct straight_test = { STRAIGHT_IDLE, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, false };

void straight_test_start(void)
{
    memset(&straight_test, 0, sizeof(straight_test));
    straight_test.state = STRAIGHT_LOCKING;
}

void straight_test_stop(void)
{
    straight_test.state = STRAIGHT_IDLE;
    target_speed = 0;
    straight_test.completed = false;
}

void straight_test_run(void)
{
    if (straight_test.state == STRAIGHT_IDLE)
        return;

    switch (straight_test.state)
    {
    case STRAIGHT_IDLE:
        return;

    case STRAIGHT_LOCKING:
        // 尝试获取GPS航向
        if (gnss.antenna_direction_state == 1)
        {
            straight_test.target_heading = gnss.antenna_direction;
            straight_test.gps_available  = true;
        }
        else if (gnss.state == 1 && gnss.speed > 1.0f)
        {
            straight_test.target_heading = gnss.direction;
            straight_test.gps_available  = true;
        }
        else if (gnss.state == 1 && gnss.satellite_used >= 6)
        {
            // GPS已定位但无法获取航向（静止时无运动航向，无双天线）
            // 以当前IMU yaw作为临时参考
            straight_test.target_heading = roll_balance_cascade.posture_value.yaw;
            straight_test.gps_available  = false;
        }
        else if (yaw_fusion_is_gyro_bias_ready())
        {
            straight_test.target_heading = roll_balance_cascade.posture_value.yaw;
            straight_test.gps_available  = false;
        }
        else
        {
            return;  // 等待GPS航向或静止IMU零偏标定完成
        }

        // 航向获取成功 → 记录起点，进入运行
        straight_test.start_lat     = (float)gnss.latitude;
        straight_test.start_lon     = (float)gnss.longitude;
        straight_test.start_mileage = Car.mileage;
        straight_test.state         = STRAIGHT_RUNNING;
        target_speed = STRAIGHT_TEST_SPEED;
        STOP_FLAG    = 1;
        break;

    case STRAIGHT_RUNNING:
    {
        // 距离计数
        float cur_mileage = Car.mileage;
        straight_test.distance = cur_mileage - straight_test.start_mileage;

        // 航向偏差PID → steer_correction
        float yaw_error = angle_plan((float)(roll_balance_cascade.posture_value.yaw - straight_test.target_heading));
        pid_control(&track_cascade.track_cycle, 0.0f, yaw_error);
        straight_test.steer_correction = track_cascade.track_cycle.out;

        // 100m到达 → 停止
        if (straight_test.distance >= STRAIGHT_SLOWDOWN_CM)  // 99m提前减速，编码器误差补偿
        {
            target_speed = 0;
            if (straight_test.distance >= STRAIGHT_TARGET_CM)
            {
                straight_test.state = STRAIGHT_DONE;
                // 航向漂移
                straight_test.yaw_drift = (float)angle_plan((double)(
                    roll_balance_cascade.posture_value.yaw - straight_test.target_heading));

                if (straight_test.gps_available && gnss.state == 1)
                {
                    // 计算侧偏
                    float end_lat  = (float)gnss.latitude;
                    float end_lon  = (float)gnss.longitude;
                    float d_total  = (float)get_two_points_distance(
                        straight_test.start_lat, straight_test.start_lon,
                        end_lat, end_lon);
                    float azimuth  = (float)get_two_points_azimuth(
                        straight_test.start_lat, straight_test.start_lon,
                        end_lat, end_lon);
                    float heading_ref = (straight_test.target_heading > 180.0f)
                        ? straight_test.target_heading - 360.0f : straight_test.target_heading;
                    float angle_diff = (float)angle_plan((double)(azimuth - heading_ref));
                    straight_test.lateral_deviation = d_total * sin(angle_diff * 0.01745329f);

                    // 评分
                    float abs_dev = (straight_test.lateral_deviation > 0)
                        ? straight_test.lateral_deviation : -straight_test.lateral_deviation;
                    if      (abs_dev < 0.3f) straight_test.rating = 5;
                    else if (abs_dev < 0.6f) straight_test.rating = 4;
                    else if (abs_dev < 1.0f) straight_test.rating = 3;
                    else if (abs_dev < 2.0f) straight_test.rating = 2;
                    else                     straight_test.rating = 1;
                }
                else
                {
                    straight_test.lateral_deviation = 0.0f;
                    straight_test.rating = 0;
                }

                straight_test.completed = true;
                STOP_FLAG = 0;
            }
        }
        break;
    }

    case STRAIGHT_DONE:
        break;  // 保持停止状态，等待Menu查询结果后reset
    }
}

static void balance_angle_loop_step(void)
{
    float speed_angle_offset = balance_speed_loop_enabled()
                             ? (roll_balance_cascade.speed_cycle.out * SPEED_TO_ANGLE_GAIN)
                             : 0.0f;
    float angle_target = (0.0f - roll_balance_cascade.posture_value.mechanical_zero)
                       - speed_angle_offset
                       - stair_get_px_angle_offset();

    pid_control(&roll_balance_cascade.angle_cycle,
                angle_target,
                -roll_balance_cascade.posture_value.pit);
}

static void balance_angular_speed_loop_step(void)
{
    pid_control(&roll_balance_cascade.angular_speed_cycle,
                roll_balance_cascade.angle_cycle.out,
                imu660rb_gyro_y);
}

static void balance_speed_loop_step(void)
{
    car_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) / 2;

    if (balance_speed_loop_enabled())
    {
        pid_control(&roll_balance_cascade.speed_cycle, target_speed, (float)car_speed);
    }
    else
    {
        balance_speed_loop_clear();
    }

    if (fuxian == 1)
    {
        pid_control(&track_cascade.track_cycle, N.Final_Out, 0);
    }
}

static int16 body_motor_steer_adj(void)
{
    int16 steer_adj = ((jump_cfg.state == JUMP_IDLE)
                    && !one_bridge_is_active()
                    && (fuxian == 1)) ? (int16)(N.Final_Out * 10) : 0;

    if (straight_test.state == STRAIGHT_RUNNING)
    {
        steer_adj += (int16)straight_test.steer_correction;
    }
    steer_adj += stair_get_heading_motor_adj();
    steer_adj += one_bridge_get_motor_adj();
    steer_adj += remote_ctrl_get_steer_adj();

    if (rotation_is_active())
    {
        target_speed = 0;
        rotation_run();
        steer_adj += rotation.turn_duty;
    }

    return steer_adj;
}

static void body_motor_output_step(void)
{
    float output_ramp = startup_control_ramp();
    int16 steer_adj = body_motor_steer_adj();
    int16 motor_base = func_limit_ab(-(int16)roll_balance_cascade.angular_speed_cycle.out
                                     + jump_motor_boost_duty,
                                     M_MIN,
                                     M_MAX);

    if (output_ramp < 1.0f)
    {
        motor_base = (int16)((float)motor_base * output_ramp);
        steer_adj  = (int16)((float)steer_adj  * output_ramp);
    }
    CYT2_D_motor_ctrl(
        motor_base + steer_adj,
        motor_base - steer_adj);
}

//================================================================================
// PIT 回调 — 1ms 控制级联
//================================================================================
void pit_call_back(void)
{
    sys_times++;

    imu660rb_get_gyro();
    imu660rb_get_acc();
    yaw_fusion_calibrate_gyro_bias(imu660rb_gyro_z);
    imu660rb_gyro_z -= (int16)yaw_fusion_gyro_bias;
    quaternion_module_calculate(&roll_balance_cascade);
    yaw_fusion_update();

    startup_mechanical_match_update();
    if (sys_times <= STARTUP_SENSOR_SETTLE_MS)
    {
        CYT2_D_motor_ctrl(0, 0);
        return;
    }

    car_state_calculate();
    if (sys_times <= STARTUP_SENSOR_SETTLE_MS)
    {
        CYT2_D_motor_ctrl(0, 0);
        return;
    }

    if (sys_times > STARTUP_SENSOR_SETTLE_MS)
    {
        if (sys_times % 5 == 0)
        {
            CYT2_get_distance();
            Nag_System();
        }

        stair_service_1ms();
        balance_angle_loop_step();
        balance_angular_speed_loop_step();

        one_bridge_service_1ms();
        car_steer_control();
        straight_test_run();
        remote_ctrl_update_1ms();

        if (startup_control_ramp() < 1.0f)
        {
            target_speed = 0.0f;
        }

        if (sys_times % 20 == 0)
        {
            balance_speed_loop_step();
        }

        if (STOP_FLAG == 1)
        {
            body_motor_output_step();
        }
        else
        {
            CYT2_D_motor_ctrl(0, 0);
        }
    }
}
