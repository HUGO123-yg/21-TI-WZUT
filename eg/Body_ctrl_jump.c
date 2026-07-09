//================================================================================
// 文件名：Body_ctrl_jump.c
// 功能描述：跳跃参数管理、PID 保存/恢复、跳跃触发与状态转换 API
// 依赖关系：被 pit_call_back / Menu / Remote_ctrl 等调用
// 注意事项：所有函数可能在 1ms ISR 或主循环中调用，需保证临界区安全；
//           jump_cfg 为全局可变状态，修改前需确认当前状态为 JUMP_IDLE。
//================================================================================
#include "Body_ctrl_internal.h"

// 起跳前保存的 PID 参数副本，用于跳跃结束后恢复
static pid_cycle_struct jump_saved_angular_speed_cycle;  // 角速度环备份
static pid_cycle_struct jump_saved_angle_cycle;          // 角度环备份
static pid_cycle_struct jump_saved_speed_cycle;          // 速度环备份

//================================================================================
// 跳跃配置 — 默认值
//================================================================================
// 以下结构体实例为全局跳跃配置，包含各阶段时间、舵机占空比、PID 参数及视觉触发相关字段
jump_config_struct jump_cfg = {
    .prepare_ticks          = JUMP_PREPARE_TICKS,        // 预备阶段持续 tick 数
    .charge_ticks           = JUMP_CHARGE_TICKS,         // 蓄力阶段持续 tick 数
    .launch_ticks           = JUMP_LAUNCH_TICKS,         // 起跳阶段持续 tick 数
    .airborne_timeout       = JUMP_AIRBORNE_TIMEOUT,     // 腾空阶段超时 tick 数
    .landing_ticks          = JUMP_LANDING_TICKS,        // 着陆阶段持续 tick 数
    .recover_ticks          = JUMP_RECOVER_TICKS,        // 恢复阶段持续 tick 数

    .charge_duty            = JUMP_CHARGE_DUTY,          // 蓄力深度占空比（负向弯曲量）
    .launch_duty            = JUMP_LAUNCH_DUTY,          // 起跳伸展占空比
    .preland_duty           = JUMP_PRELAND_DUTY,         // 预着陆姿态占空比
    .land_damping_duty      = JUMP_LAND_DAMPING_DUTY,    // 着陆阻尼占空比

    .forward_tilt_target    = JUMP_FORWARD_TILT_TARGET,  // 起跳前目标俯仰角
    .forward_motor_boost    = JUMP_FORWARD_MOTOR_BOOST,  // 起跳电机助推占空比
    .speed_recovery_rate    = JUMP_SPEED_RECOVERY_RATE,  // 速度恢复比率（0~1）

    .airborne_pid_scale     = JUMP_AIRBORNE_PID_SCALE,   // 腾空阶段 PID 缩放系数
    .landing_pid_scale      = JUMP_LANDING_PID_SCALE,    // 着陆阶段 PID 缩放系数
    .recover_pid_ramp_rate  = JUMP_RECOVER_PID_RAMP_RATE,// 恢复阶段 PID 斜坡速率

    .airborne_acc_threshold = JUMP_AIRBORNE_ACC_THRESHOLD,// 腾空触发加速度阈值
    .landing_acc_threshold  = JUMP_LANDING_ACC_THRESHOLD, // 着陆检测加速度阈值
    .max_tilt_abort         = JUMP_MAX_TILT_ABORT,       // 允许起跳的最大倾斜角

    .vision_jump_trigger    = NULL,                      // 视觉触发回调函数指针
    .vision_jump_enable     = 0,                         // 视觉自动跳跃使能标志
    .vision_obstacle_dist   = 0.0f,                      // 视觉测得的障碍物距离（mm）
    .vision_min_dist        = JUMP_VISION_MIN_DIST,      // 视觉触发最小距离
    .vision_max_dist        = JUMP_VISION_MAX_DIST,      // 视觉触发最大距离

    .state                  = JUMP_IDLE,                 // 当前跳跃状态机状态
    .elapsed                = 0,                         // 当前阶段已运行 tick 数
    .jump_count             = 0,                         // 累计跳跃成功次数
    .peak_acc_magnitude     = 0.0f,                      // 本次跳跃峰值加速度模长
    .stored_p_angle         = 0.0f,                      // 起跳前保存的角度环 P 参数
    .stored_p_speed         = 0.0f,                      // 起跳前保存的速度环 P 参数
    .stored_speed_target    = 0.0f,                      // 起跳前保存的目标速度
    .last_trigger_result    = JUMP_TRIGGER_OK,           // 上次触发结果码
};

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    清零 roll/pitch 三级 PID 的积分项、输出值及上次比例项
// 参数说明：    无
// 返回值：      无
// 调用方：      jump_apply_fixed_pid、body_jump_restore_saved_control
// 注意事项：    仅清零记忆型变量（i_value / out / p_value_last），不修改 PID 系数（p/i/d）；
//              用于跳跃阶段切换时防止积分累积突变
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    应用固定 PID 参数（仅保留 P 项，清零 I/D 项与记忆）
// 参数说明：    无
// 返回值：      无
// 调用方：      jump_trigger
// 注意事项：    起跳瞬间调用，将 roll 三阶 PID 的 I、D 系数置零，速度环仅保留 P 系数；
//              避免跳跃过程中积分饱和与微分冲击导致姿态失控
//-------------------------------------------------------------------------------------------------------------------
static void jump_apply_fixed_pid(void)
{
    roll_balance_cascade.angle_cycle = jump_saved_angle_cycle;
    roll_balance_cascade.angular_speed_cycle = jump_saved_angular_speed_cycle;
    roll_balance_cascade.speed_cycle = jump_saved_speed_cycle;

    roll_balance_cascade.angle_cycle.i = 0.0f;          // 角度环关闭积分
    roll_balance_cascade.angle_cycle.d = 0.0f;          // 角度环关闭微分
    roll_balance_cascade.angular_speed_cycle.i = 0.0f;  // 角速度环关闭积分
    roll_balance_cascade.angular_speed_cycle.d = 0.0f;  // 角速度环关闭微分
    roll_balance_cascade.speed_cycle.p = jump_saved_speed_cycle.p;  // 速度环保留原 P 系数
    roll_balance_cascade.speed_cycle.i = 0.0f;          // 速度环关闭积分
    roll_balance_cascade.speed_cycle.d = 0.0f;          // 速度环关闭微分

    jump_reset_pid_memory();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    判断当前跳跃状态下速度环是否应继续更新
// 参数说明：    无
// 返回值：      uint8，1 表示速度环需要更新，0 表示冻结
// 调用方：      body_jump_manage_pid、跳跃状态机相关逻辑
// 注意事项：    仅在 PREPARE / CHARGE / LANDING / RECOVER 四个阶段允许速度环更新；
//              LAUNCH 与 AIRBORNE 阶段冻结速度环，防止空中速度积分漂移
//-------------------------------------------------------------------------------------------------------------------
uint8 body_jump_speed_loop_should_update(void)
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

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    计算当前阶段进度比例（0.0 ~ 1.0）
// 参数说明：    elapsed — uint16，已运行 tick 数
//              total   — uint16，阶段总 tick 数
// 返回值：      float，限幅后的进度比例
// 调用方：      car_steer_control 跳跃状态机各阶段
// 注意事项：    当 total 为 0 时直接返回 1.0，避免除零；
//              结果经 func_limit_ab 限幅到 [0,1]，可用于线性插值
//-------------------------------------------------------------------------------------------------------------------
float body_jump_phase_progress(uint16 elapsed, uint16 total)
{
    float progress;

    if (total == 0)
        return 1.0f;

    progress = (float)elapsed / (float)total;
    return func_limit_ab(progress, 0.0f, 1.0f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    切换跳跃状态机状态并清零阶段计时
// 参数说明：    state — uint8，目标状态（JUMP_xxx 枚举值）
// 返回值：      无
// 调用方：      car_steer_control 状态机、jump_trigger、jump_abort 等
// 注意事项：    直接修改全局 jump_cfg.state，调用方需确保状态转换的合法性；
//              切换后 elapsed 归零，用于新阶段的进度计算
//-------------------------------------------------------------------------------------------------------------------
void body_jump_enter_state(uint8 state)
{
    jump_cfg.state = state;
    jump_cfg.elapsed = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    恢复起跳前保存的 PID 参数与速度目标
// 参数说明：    stop_speed — uint8，1 表示将目标速度置 0，0 表示恢复保存值
// 返回值：      无
// 调用方：      jump_abort、跳跃恢复阶段（RECOVER 结束）
// 注意事项：    恢复 roll 三级 PID 完整参数，pitch 角度环仅清零记忆；
//              若 stop_speed 为 1，通常用于异常终止后的安全停车
//-------------------------------------------------------------------------------------------------------------------
void body_jump_restore_saved_control(uint8 stop_speed)
{
    roll_balance_cascade.angle_cycle = jump_saved_angle_cycle;
    roll_balance_cascade.angular_speed_cycle = jump_saved_angular_speed_cycle;
    roll_balance_cascade.speed_cycle = jump_saved_speed_cycle;
    pitch_balance_cascade.angle_cycle.i_value = 0;
    pitch_balance_cascade.angle_cycle.out = 0;
    pitch_balance_cascade.angle_cycle.p_value_last = 0;

    target_speed = stop_speed ? 0.0f : jump_cfg.stored_speed_target;  // 根据参数选择停车或恢复速度
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    根据物理偏移量计算单腿舵机占空比
// 参数说明：    control_data    — const steer_control_struct *，舵机控制参数（中位值、方向）
//              physical_offset — int16，物理偏移量（相对于默认中位）
// 返回值：      int16，限幅后的舵机占空比（0~10000）
// 调用方：      steer_set_default_pose、body_jump_set_all_leg_offset
// 注意事项：    占空比 = center_num + physical_offset * steer_dir；
//              结果经 func_limit_ab 限幅到 [0, 10000]，防止舵机过驱
//-------------------------------------------------------------------------------------------------------------------
static int16 jump_steer_duty_from_offset(const steer_control_struct *control_data, int16 physical_offset)
{
    int16 duty = control_data->center_num + physical_offset * control_data->steer_dir;
    return func_limit_ab(duty, 0, 10000);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    设置四腿舵机为默认中位姿态
// 参数说明：    无
// 返回值：      无
// 调用方：      body_jump_set_neutral_leg_offset、jump_trigger、jump_config_default
// 注意事项：    直接调用 steer_duty_set 写入占空比，无平滑过渡；
//              通常用于跳跃前初始化或异常恢复
//-------------------------------------------------------------------------------------------------------------------
static void steer_set_default_pose(void)
{
    steer_duty_set(&steer_1, jump_steer_duty_from_offset(&steer_1, STEER_1_DEFAULT_OFFSET));
    steer_duty_set(&steer_2, jump_steer_duty_from_offset(&steer_2, STEER_2_DEFAULT_OFFSET));
    steer_duty_set(&steer_3, jump_steer_duty_from_offset(&steer_3, STEER_3_DEFAULT_OFFSET));
    steer_duty_set(&steer_4, jump_steer_duty_from_offset(&steer_4, STEER_4_DEFAULT_OFFSET));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    同步设置四腿舵机的统一偏移量
// 参数说明：    offset — int16，统一的物理偏移量（相对于默认中位）
// 返回值：      无
// 调用方：      car_steer_control 跳跃状态机各阶段
// 注意事项：    四腿使用相同 offset，实现同步下蹲/伸展；
//              offset 为正表示向 steer_dir 方向偏转，负表示反向
//-------------------------------------------------------------------------------------------------------------------
void body_jump_set_all_leg_offset(int16 offset)
{
    steer_duty_set(&steer_1, jump_steer_duty_from_offset(&steer_1, STEER_1_DEFAULT_OFFSET + offset));
    steer_duty_set(&steer_2, jump_steer_duty_from_offset(&steer_2, STEER_2_DEFAULT_OFFSET + offset));
    steer_duty_set(&steer_3, jump_steer_duty_from_offset(&steer_3, STEER_3_DEFAULT_OFFSET + offset));
    steer_duty_set(&steer_4, jump_steer_duty_from_offset(&steer_4, STEER_4_DEFAULT_OFFSET + offset));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    设置四腿为中性姿态（默认中位）
// 参数说明：    无
// 返回值：      无
// 调用方：      jump_abort、jump_config_default、jump_trigger
// 注意事项：    封装 steer_set_default_pose，供外部模块调用
//-------------------------------------------------------------------------------------------------------------------
void body_jump_set_neutral_leg_offset(void)
{
    steer_set_default_pose();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    管理跳跃各阶段的 PID 参数动态调整
// 参数说明：    无
// 返回值：      无
// 调用方：      pit_call_back 或跳跃状态机相关逻辑
// 注意事项：    roll 角度环与角速度环仅保留 P 项，速度环仅在特定阶段保留 P 项；
//              当速度环不应更新时，调用 body_balance_speed_loop_clear 清零速度环状态，
//              防止空中/起跳阶段速度积分漂移
//-------------------------------------------------------------------------------------------------------------------
void body_jump_manage_pid(void)
{
    roll_balance_cascade.angle_cycle.p = jump_saved_angle_cycle.p;
    roll_balance_cascade.angle_cycle.i = 0.0f;
    roll_balance_cascade.angle_cycle.d = 0.0f;
    roll_balance_cascade.angular_speed_cycle.p = jump_saved_angular_speed_cycle.p;
    roll_balance_cascade.angular_speed_cycle.i = 0.0f;
    roll_balance_cascade.angular_speed_cycle.d = 0.0f;
    roll_balance_cascade.speed_cycle.p = body_jump_speed_loop_should_update()
                                       ? jump_saved_speed_cycle.p
                                       : 0.0f;  // 非更新阶段将速度环 P 项置零，冻结速度控制
    roll_balance_cascade.speed_cycle.i = 0.0f;
    roll_balance_cascade.speed_cycle.d = 0.0f;
    if (!body_jump_speed_loop_should_update())
    {
        body_balance_speed_loop_clear();
    }
    pitch_balance_cascade.angle_cycle.i_value = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    触发一次跳跃流程
// 参数说明：    无
// 返回值：      uint8，JUMP_TRIGGER_OK 表示成功触发，其他值表示失败原因
// 调用方：      Menu 菜单回调、Remote_ctrl 遥控指令、jump_vision_update
// 注意事项：    触发前检查：状态必须为 IDLE、车辆必须运行、倾斜角不超过阈值；
//              触发后保存当前 PID 参数与目标速度，并应用固定 PID；
//              若触发失败，last_trigger_result 记录失败原因
//-------------------------------------------------------------------------------------------------------------------
uint8 jump_trigger(void)
{
    if (jump_cfg.state != JUMP_IDLE)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_BUSY;  // 状态非空闲，记录忙状态
        return JUMP_TRIGGER_BUSY;
    }
    if (!run_state)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_NOT_RUNNING;  // 车辆未运行
        return JUMP_TRIGGER_NOT_RUNNING;
    }
    if (body_compute_tilt_angle() > jump_cfg.max_tilt_abort)
    {
        jump_cfg.last_trigger_result = JUMP_TRIGGER_TILT;  // 倾斜角过大，禁止起跳
        return JUMP_TRIGGER_TILT;
    }

    jump_cfg.state = JUMP_PREPARE;  // 进入预备阶段
    jump_cfg.elapsed = 0;
    jump_cfg.peak_acc_magnitude = 0.0f;
    jump_cfg.last_trigger_result = JUMP_TRIGGER_OK;

    jump_saved_angle_cycle = roll_balance_cascade.angle_cycle;          // 备份角度环参数
    jump_saved_angular_speed_cycle = roll_balance_cascade.angular_speed_cycle;  // 备份角速度环参数
    jump_saved_speed_cycle = roll_balance_cascade.speed_cycle;          // 备份速度环参数
    jump_cfg.stored_p_angle = roll_balance_cascade.angle_cycle.p;       // 记录角度环 P 值
    jump_cfg.stored_p_speed = roll_balance_cascade.speed_cycle.p;       // 记录速度环 P 值
    jump_cfg.stored_speed_target = target_speed;                        // 保存当前目标速度

    jump_apply_fixed_pid();        // 应用固定 PID（关闭 I/D 项）
    body_jump_set_neutral_leg_offset();  // 四腿回归中位，准备起跳
    target_speed = jump_cfg.stored_speed_target;  // 保持目标速度不变

    return JUMP_TRIGGER_OK;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    强制中止当前跳跃流程并恢复安全状态
// 参数说明：    无
// 返回值：      无
// 调用方：      Menu 菜单紧急停止、异常保护逻辑
// 注意事项：    无论当前处于何状态，立即回到 IDLE；
//              若跳跃曾处于活跃状态，恢复保存的 PID 并将目标速度置 0；
//              最后将四腿回归中性姿态
//-------------------------------------------------------------------------------------------------------------------
void jump_abort(void)
{
    uint8 was_active = (jump_cfg.state != JUMP_IDLE) ? 1 : 0;

    body_jump_enter_state(JUMP_IDLE);  // 强制回到空闲状态
    body_jump_motor_boost_duty = 0;    // 关闭电机助推

    if (was_active)
    {
        body_jump_restore_saved_control(1);  // 恢复 PID 并将目标速度置 0
    }
    body_jump_set_neutral_leg_offset();      // 四腿回归中位
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    视觉模块距离更新回调，支持视觉自动触发跳跃
// 参数说明：    distance_mm — float，视觉测得的障碍物距离（毫米）
// 返回值：      无
// 调用方：      视觉模块中断或轮询任务
// 注意事项：    仅当 vision_jump_enable 使能且状态为 IDLE 时才会触发；
//              距离需在 [vision_min_dist, vision_max_dist] 范围内；
//              若注册了 vision_jump_trigger 回调，则调用回调；否则直接触发跳跃
//-------------------------------------------------------------------------------------------------------------------
void jump_vision_update(float distance_mm)
{
    jump_cfg.vision_obstacle_dist = distance_mm;

    if (!jump_cfg.vision_jump_enable)                          return;  // 视觉自动跳跃未使能
    if (jump_cfg.state != JUMP_IDLE)                           return;  // 跳跃状态机非空闲
    if (distance_mm > jump_cfg.vision_max_dist)                return;  // 距离超过最大触发阈值
    if (distance_mm < jump_cfg.vision_min_dist)                return;  // 距离小于最小触发阈值
    if (jump_cfg.vision_jump_trigger != NULL)
    {
        jump_cfg.vision_jump_trigger(distance_mm);  // 调用用户注册的视觉触发回调
    }
    else
    {
        (void)jump_trigger();  // 无回调时直接触发跳跃
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    将跳跃配置恢复为出厂默认值
// 参数说明：    无
// 返回值：      无
// 调用方：      Menu 菜单恢复默认配置
// 注意事项：    若当前处于跳跃状态，先调用 jump_abort 安全终止；
//              恢复后会保留 vision_jump_trigger 回调指针与 vision_jump_enable 标志，
//              避免视觉配置被意外清除
//-------------------------------------------------------------------------------------------------------------------
void jump_config_default(void)
{
    if (jump_cfg.state != JUMP_IDLE)
    {
        jump_abort();  // 非空闲时先强制中止，防止配置变更导致状态混乱
    }

    static const jump_config_struct default_template = {
        .prepare_ticks          = JUMP_PREPARE_TICKS,        // 预备阶段持续 tick 数
        .charge_ticks           = JUMP_CHARGE_TICKS,         // 蓄力阶段持续 tick 数
        .launch_ticks           = JUMP_LAUNCH_TICKS,         // 起跳阶段持续 tick 数
        .airborne_timeout       = JUMP_AIRBORNE_TIMEOUT,     // 腾空阶段超时 tick 数
        .landing_ticks          = JUMP_LANDING_TICKS,        // 着陆阶段持续 tick 数
        .recover_ticks          = JUMP_RECOVER_TICKS,        // 恢复阶段持续 tick 数

        .charge_duty            = JUMP_CHARGE_DUTY,          // 蓄力深度占空比
        .launch_duty            = JUMP_LAUNCH_DUTY,          // 起跳伸展占空比
        .preland_duty           = JUMP_PRELAND_DUTY,         // 预着陆姿态占空比
        .land_damping_duty      = JUMP_LAND_DAMPING_DUTY,    // 着陆阻尼占空比

        .forward_tilt_target    = JUMP_FORWARD_TILT_TARGET,  // 起跳前目标俯仰角
        .forward_motor_boost    = JUMP_FORWARD_MOTOR_BOOST,  // 起跳电机助推占空比
        .speed_recovery_rate    = JUMP_SPEED_RECOVERY_RATE,  // 速度恢复比率

        .airborne_pid_scale     = JUMP_AIRBORNE_PID_SCALE,   // 腾空阶段 PID 缩放系数
        .landing_pid_scale      = JUMP_LANDING_PID_SCALE,    // 着陆阶段 PID 缩放系数
        .recover_pid_ramp_rate  = JUMP_RECOVER_PID_RAMP_RATE,// 恢复阶段 PID 斜坡速率

        .airborne_acc_threshold = JUMP_AIRBORNE_ACC_THRESHOLD,// 腾空触发加速度阈值
        .landing_acc_threshold  = JUMP_LANDING_ACC_THRESHOLD, // 着陆检测加速度阈值
        .max_tilt_abort         = JUMP_MAX_TILT_ABORT,       // 允许起跳的最大倾斜角

        .vision_jump_trigger    = NULL,                      // 视觉触发回调函数指针
        .vision_jump_enable     = 0,                         // 视觉自动跳跃使能标志
        .vision_obstacle_dist   = 0.0f,                      // 视觉测得的障碍物距离
        .vision_min_dist        = JUMP_VISION_MIN_DIST,      // 视觉触发最小距离
        .vision_max_dist        = JUMP_VISION_MAX_DIST,      // 视觉触发最大距离

        .state                  = JUMP_IDLE,                 // 当前跳跃状态机状态
        .elapsed                = 0,                         // 当前阶段已运行 tick 数
        .jump_count             = 0,                         // 累计跳跃成功次数
        .peak_acc_magnitude     = 0.0f,                      // 本次跳跃峰值加速度模长
        .stored_p_angle         = 0.0f,                      // 起跳前保存的角度环 P 参数
        .stored_p_speed         = 0.0f,                      // 起跳前保存的速度环 P 参数
        .stored_speed_target    = 0.0f,                      // 起跳前保存的目标速度
        .last_trigger_result    = JUMP_TRIGGER_OK,           // 上次触发结果码
    };

    void (*saved_vision_cb)(float) = jump_cfg.vision_jump_trigger;  // 暂存视觉回调指针
    uint8 saved_vision_enable = jump_cfg.vision_jump_enable;        // 暂存视觉使能标志

    jump_cfg = default_template;  // 整体拷贝默认模板

    jump_cfg.vision_jump_trigger = saved_vision_cb;    // 恢复视觉回调指针
    jump_cfg.vision_jump_enable = saved_vision_enable; // 恢复视觉使能标志

    body_jump_motor_boost_duty = 0;  // 关闭电机助推
    body_jump_set_neutral_leg_offset();  // 四腿回归中位
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    判断当前是否允许触发跳跃
// 参数说明：    无
// 返回值：      uint8，1 表示允许触发，0 表示不允许
// 调用方：      Menu 菜单状态显示、Remote_ctrl 遥控使能判断
// 注意事项：    检查条件与 jump_trigger 前置条件一致：状态 IDLE、车辆运行、倾斜角合格
//-------------------------------------------------------------------------------------------------------------------
uint8 jump_can_trigger(void)
{
    if (jump_cfg.state != JUMP_IDLE)                           return 0;  // 跳跃状态机非空闲
    if (!run_state)                                            return 0;  // 车辆未运行
    if (body_compute_tilt_angle() > jump_cfg.max_tilt_abort)   return 0;  // 倾斜角过大
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    将跳跃状态枚举值转换为可读的字符串名称
// 参数说明：    state — uint8，跳跃状态枚举值
// 返回值：      const char *，状态名称字符串
// 调用方：      调试打印、Menu 菜单显示、日志记录
// 注意事项：    返回静态字符串常量，无需释放；
//              未知状态返回 "UNKNOWN"
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    将跳跃触发结果枚举值转换为可读的字符串名称
// 参数说明：    result — uint8，触发结果枚举值
// 返回值：      const char *，结果名称字符串
// 调用方：      调试打印、Menu 菜单显示
// 注意事项：    返回静态字符串常量，无需释放
//-------------------------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------------------------
// 函数功能：    判断跳跃状态机是否处于活跃状态（非 IDLE）
// 参数说明：    无
// 返回值：      uint8，1 表示正在跳跃，0 表示空闲
// 调用方：      pit_call_back、Menu 菜单状态显示
// 注意事项：    无
//-------------------------------------------------------------------------------------------------------------------
uint8 jump_is_active(void)
{
    return (jump_cfg.state != JUMP_IDLE) ? 1 : 0;
}
