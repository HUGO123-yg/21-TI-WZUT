/*
 * Stair_test.c — 上台阶 / 跳跃测试模块
 *
 *  Created on: 2026-6-19
 *      Author: HUGO
 *
 * 三种测试模式，共享里程/速度检测逻辑。
 * 每个模式由 start() 初始化，run() 每帧驱动状态机并刷新 LCD，
 * is_done() 供 Menu 判断是否退回一级菜单。
 */

#include "zf_common_headfile.h"

// ============================================================
// 内部状态枚举
// ============================================================
typedef enum {
    STAIR_PHASE_RUNUP    = 0,
    STAIR_PHASE_JUMPING  = 1,
    STAIR_PHASE_STOPPING = 2,
    STAIR_PHASE_DONE     = 3,
} stair_phase_enum;

// ============================================================
// 每个模式的独立上下文
// ============================================================
typedef struct {
    uint8            active;
    stair_phase_enum phase;
    uint8            jump_index;
    uint8            jump_ok;          // 本次跳跃是否触发成功
    uint8            fail_reason;      // 0=无故障，1=触发失败，2=超距未停
    uint8            last_trigger_result;
    float            segment_start_x;
    float            stop_x_cm;
    float            x_cm;
    float            y_cm;
    float            yaw_ref_deg;
    float            last_mileage;
    float            x_target_cm;
    uint16           phase_timer;
    uint16           trigger_wait_timer;
} stair_ctx;

static stair_ctx ctx_jump;
static stair_ctx ctx_single;
static stair_ctx ctx_seq;
static uint8 stop_stable_count = 0;
static pid_cycle_struct stair_px_pid;
static pid_cycle_struct stair_yaw_pid;
static float stair_px_angle_offset = 0.0f;
static uint8 stair_px_enable = 0;
static int16 stair_heading_motor_adj = 0;

// ============================================================
// 辅助函数
// ============================================================

// 当前平均里程 (cm)
static float current_mileage(void)
{
    return Car.mileage;
}

static float stair_wrap_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static void stair_px_pid_init(void)
{
    stair_px_pid.p = STAIR_PX_KP;
    stair_px_pid.i = STAIR_PX_KI;
    stair_px_pid.d = STAIR_PX_KD;
    stair_px_pid.i_value = 0.0f;
    stair_px_pid.p_value_last = 0.0f;
    stair_px_pid.out = 0.0f;
    stair_px_pid.i_value_max = STAIR_PX_INTEGRAL_MAX;
    stair_px_pid.i_value_pro = STAIR_PX_INTEGRAL_PRO;
    stair_px_pid.out_max = STAIR_PX_OUT_MAX;
}

static void stair_yaw_pid_init(void)
{
    stair_yaw_pid.p = STAIR_YAW_KP;
    stair_yaw_pid.i = STAIR_YAW_KI;
    stair_yaw_pid.d = STAIR_YAW_KD;
    stair_yaw_pid.i_value = 0.0f;
    stair_yaw_pid.p_value_last = 0.0f;
    stair_yaw_pid.out = 0.0f;
    stair_yaw_pid.i_value_max = STAIR_YAW_INTEGRAL_MAX;
    stair_yaw_pid.i_value_pro = STAIR_YAW_INTEGRAL_PRO;
    stair_yaw_pid.out_max = STAIR_YAW_OUT_MAX;
}

static void stair_ctx_begin(stair_ctx *c)
{
    c->active             = 1;
    c->phase              = STAIR_PHASE_RUNUP;
    c->jump_index         = 0;
    c->jump_ok            = 0;
    c->fail_reason        = 0;
    c->last_trigger_result = JUMP_TRIGGER_OK;
    c->segment_start_x    = 0.0f;
    c->stop_x_cm          = 0.0f;
    c->x_cm               = 0.0f;
    c->y_cm               = 0.0f;
    c->yaw_ref_deg        = roll_balance_cascade.posture_value.yaw;
    c->last_mileage       = current_mileage();
    c->x_target_cm        = 0.0f;
    c->phase_timer        = 0;
    c->trigger_wait_timer = 0;
    stop_stable_count = 0;
    stair_px_pid_init();
    stair_yaw_pid_init();
    stair_px_angle_offset = 0.0f;
    stair_px_enable = 0;
    stair_heading_motor_adj = 0;
}

static float stair_coord_update(stair_ctx *c)
{
    float mileage_now;
    float ds;
    float yaw_error;
    float yaw_rad;

    if (!c->active)
        return 0.0f;

    mileage_now = current_mileage();
    ds = mileage_now - c->last_mileage;
    c->last_mileage = mileage_now;

    yaw_error = stair_wrap_angle(roll_balance_cascade.posture_value.yaw - c->yaw_ref_deg);
    yaw_rad = yaw_error * 0.01745329f;
    c->x_cm += ds * cosf(yaw_rad);
    c->y_cm += ds * sinf(yaw_rad);

    return yaw_error;
}

static void stair_heading_update(stair_ctx *c, float yaw_error)
{
    if (!c->active)
    {
        stair_heading_motor_adj = 0;
        return;
    }

    pid_control(&stair_yaw_pid, 0.0f, yaw_error);
    stair_heading_motor_adj = (int16)(STAIR_YAW_MOTOR_DIR * stair_yaw_pid.out);
}

static void stair_pose_update(stair_ctx *c)
{
    float yaw_error = stair_coord_update(c);
    stair_heading_update(c, yaw_error);
}

// 当前段沿车体 x 轴的行进距离 (cm)
static float stair_x_delta(const stair_ctx *c)
{
    return c->x_cm - c->segment_start_x;
}

static void stair_px_set_target(stair_ctx *c, float x_target_cm)
{
    c->x_target_cm = x_target_cm;
    stair_px_enable = 1;
}

static void stair_px_disable(void)
{
    stair_px_enable = 0;
    stair_px_angle_offset = 0.0f;
    stair_px_pid.i_value = 0.0f;
    stair_px_pid.p_value_last = 0.0f;
    stair_px_pid.out = 0.0f;
}

static void stair_heading_disable(void)
{
    stair_heading_motor_adj = 0;
    stair_yaw_pid.i_value = 0.0f;
    stair_yaw_pid.p_value_last = 0.0f;
    stair_yaw_pid.out = 0.0f;
}

static void stair_px_update(stair_ctx *c)
{
    if (!stair_px_enable || !c->active || jump_is_active())
    {
        stair_px_angle_offset = 0.0f;
        return;
    }

    pid_control(&stair_px_pid, c->x_target_cm, c->x_cm);
    stair_px_angle_offset = STAIR_PX_ANGLE_DIR * stair_px_pid.out;
    target_speed = STAIR_PX_HOLD_SPEED;
}

// 车速是否已降到阈值以下 — 连续稳定检测，避免编码器噪声
static uint8 speed_stopped(void)
{
    if (func_abs(car_speed) <= (int16)STAIR_STOP_SPEED)
        stop_stable_count++;
    else
        stop_stable_count = 0;
    return (stop_stable_count >= STAIR_STOP_STABLE_MS) ? 1 : 0;
}

// 将 jump_cfg 设为上台阶参数
static void apply_stair_params(void)
{
    jump_cfg.prepare_ticks        = STAIR_PREPARE_TICKS;
    jump_cfg.charge_duty          = STAIR_CHARGE_DUTY;
    jump_cfg.launch_duty          = STAIR_LAUNCH_DUTY;
    jump_cfg.charge_ticks         = STAIR_CHARGE_TICKS;
    jump_cfg.launch_ticks         = STAIR_LAUNCH_TICKS;
    jump_cfg.preland_duty         = STAIR_PRELAND_DUTY;
    jump_cfg.forward_motor_boost  = STAIR_MOTOR_BOOST;
    jump_cfg.land_damping_duty    = STAIR_LAND_DAMPING;
    jump_cfg.airborne_timeout     = STAIR_AIRBORNE_TIMEOUT;
    jump_cfg.landing_ticks        = STAIR_LANDING_TICKS;
    jump_cfg.recover_ticks        = STAIR_RECOVER_TICKS;
    jump_cfg.landing_acc_threshold = STAIR_LANDING_ACC_THRESHOLD;
}

// 恢复 jump_cfg 为默认值
static void restore_jump_defaults(void)
{
    jump_config_default();
}

static void stair_finish(stair_ctx *c, uint8 ok, uint8 fail_reason)
{
    target_speed = 0.0f;
    stair_px_disable();
    stair_heading_disable();
    c->jump_ok = ok;
    c->fail_reason = fail_reason;
    c->active = 0;
    c->phase = STAIR_PHASE_DONE;
}

static void stair_finish_with_defaults(stair_ctx *c, uint8 ok, uint8 fail_reason)
{
    restore_jump_defaults();
    stair_finish(c, ok, fail_reason);
}

static uint8 stair_trigger_jump(stair_ctx *c)
{
    uint8 result = jump_trigger();

    c->last_trigger_result = result;
    c->jump_ok = (result == JUMP_TRIGGER_OK) ? 1 : 0;
    c->fail_reason = c->jump_ok ? STAIR_FAIL_NONE : STAIR_FAIL_TRIGGER;
    c->phase = STAIR_PHASE_JUMPING;
    c->phase_timer = 0;
    c->trigger_wait_timer = 0;
    if (!c->jump_ok)
    {
        target_speed = 0.0f;
    }

    return c->jump_ok;
}

static void stair_seq_apply_final_brake(void)
{
    if ((ctx_seq.phase_timer <= STAIR_FINAL_BRAKE_MS)
        && (car_speed > (int16)STAIR_STOP_SPEED))
    {
        target_speed = STAIR_FINAL_BRAKE_SPEED;
    }
    else
    {
        target_speed = 0.0f;
    }
}

static uint8 stair_jump_timeout(stair_ctx *c, uint8 restore_defaults)
{
    if (c->phase_timer <= STAIR_JUMP_TIMEOUT_MS)
        return 0;

    jump_abort();
    if (restore_defaults)
        restore_jump_defaults();

    stair_finish(c, 0, STAIR_FAIL_JUMP_TO);
    return 1;
}

static uint8 stair_stop_timeout(stair_ctx *c, uint8 restore_defaults)
{
    if (c->phase_timer <= STAIR_STOP_TIMEOUT_MS)
        return 0;

    if (restore_defaults)
        restore_jump_defaults();

    stair_finish(c, 0, STAIR_FAIL_STOP_TO);
    return 1;
}

// ============================================================
// 公用 LCD 显示
// ============================================================
static void stair_display(const char *title, stair_phase_enum phase,
                          uint8 jump_ok, uint8 fail_reason, uint8 last_trigger_result,
                          float x_cm, float y_cm, uint8 jump_idx, uint8 jump_total)
{
    static uint32 last_display_ms = 0;
    int32 x_i;
    int32 y_i;

    if (last_display_ms != 0 && (uint32)(sys_times - last_display_ms) < STAIR_LCD_PERIOD_MS)
        return;
    last_display_ms = sys_times;

    x_i = func_limit_ab((int32)x_cm, -9999, 9999);
    y_i = func_limit_ab((int32)y_cm, -9999, 9999);

    ips_show_string(8 * 0, 16 * 0, title);
    ips_show_string(8 * 0, 16 * 2, "P:");
    switch (phase)
    {
    case STAIR_PHASE_RUNUP:
        ips_show_string(8 * 3, 16 * 2, "WAIT ");
        break;
    case STAIR_PHASE_JUMPING:
        ips_show_string(8 * 3, 16 * 2, "JUMP ");
        break;
    case STAIR_PHASE_STOPPING:
        ips_show_string(8 * 3, 16 * 2, "STOP ");
        break;
    case STAIR_PHASE_DONE:
        ips_show_string(8 * 3, 16 * 2, jump_ok ? "DONE " : "FAIL ");
        break;
    }
    ips_show_string(8 * 0, 16 * 3, "S:");
    ips_show_string(8 * 3, 16 * 3, jump_state_name(jump_cfg.state));
    ips_show_string(8 * 0, 16 * 4, "V:");
    ips_show_int(8 * 3, 16 * 4, car_speed, 5);
    ips_show_string(8 * 0, 16 * 5, "X:");
    ips_show_int(8 * 3, 16 * 5, x_i, 5);
    ips_show_string(8 * 0, 16 * 6, "F:");
    ips_show_int(8 * 3, 16 * 6, fail_reason, 1);
    ips_show_string(8 * 5, 16 * 6, jump_ok ? "OK " : "-- ");
    ips_show_string(8 * 0, 16 * 7, "Y:");
    ips_show_int(8 * 3, 16 * 7, y_i, 5);
    ips_show_string(8 * 0, 16 * 8, "T:");
    ips_show_string(8 * 3, 16 * 8, jump_trigger_result_name(last_trigger_result));
    if (jump_total > 1)
    {
        ips_show_string(8 * 0, 16 * 9, "J:");
        ips_show_int(8 * 3, 16 * 9, jump_idx, 1);
        ips_show_string(8 * 4, 16 * 9, "/");
        ips_show_int(8 * 5, 16 * 9, jump_total, 1);
    }
}

// ============================================================
// 模式 0: 平地跳跃测试 (stair_jump)
//   — 原 Menu.c fun_d34_jump_test 移入
// ============================================================
void stair_jump_start(void)
{
    stair_abort();
    restore_jump_defaults();               // 清理可能残留的上台阶参数
    stair_ctx_begin(&ctx_jump);
    target_speed          = JUMP_RUNUP_SPEED;
}

static void stair_jump_step(void)
{
    if (!ctx_jump.active) return;
    ctx_jump.phase_timer++;
    stair_pose_update(&ctx_jump);

    switch (ctx_jump.phase)
    {
    case STAIR_PHASE_RUNUP:
        if (ctx_jump.phase_timer > JUMP_RUNUP_TICKS)
        {
            (void)stair_trigger_jump(&ctx_jump);
        }
        break;

    case STAIR_PHASE_JUMPING:
        if (stair_jump_timeout(&ctx_jump, 0))
        {
            break;
        }
        if (!ctx_jump.jump_ok && ctx_jump.phase_timer > 10)
        {
            stair_finish(&ctx_jump, 0, ctx_jump.fail_reason);
        }
        else if (jump_cfg.state == JUMP_IDLE && ctx_jump.phase_timer > 10)
        {
            target_speed         = 0;
            ctx_jump.phase       = STAIR_PHASE_STOPPING;
            ctx_jump.phase_timer = 0;
            ctx_jump.stop_x_cm = ctx_jump.x_cm;
            stop_stable_count = 0;
        }
        break;

    case STAIR_PHASE_STOPPING:
        if (speed_stopped())
        {
            stair_finish(&ctx_jump, 1, STAIR_FAIL_NONE);
        }
        else
        {
            (void)stair_stop_timeout(&ctx_jump, 0);
        }
        break;

    case STAIR_PHASE_DONE:
        break;
    }
}

void stair_jump_run(void)
{
    stair_display("Jump Test", ctx_jump.phase, ctx_jump.jump_ok, ctx_jump.fail_reason,
                  ctx_jump.last_trigger_result, ctx_jump.x_cm, ctx_jump.y_cm, 0, 0);
}

uint8 stair_jump_is_done(void)
{
    return (ctx_jump.phase == STAIR_PHASE_DONE) ? 1 : 0;
}

// ============================================================
// 模式 1: 单次上台阶 (stair_single)
// ============================================================
void stair_single_start(void)
{
    stair_abort();
    stair_ctx_begin(&ctx_single);
    target_speed            = STAIR_PX_HOLD_SPEED;
    stair_px_set_target(&ctx_single, STAIR_RUNUP_DIST1);
}

static void stair_single_step(void)
{
    if (!ctx_single.active) return;
    ctx_single.phase_timer++;
    stair_pose_update(&ctx_single);

    switch (ctx_single.phase)
    {
    case STAIR_PHASE_RUNUP:
        stair_px_update(&ctx_single);
        if (stair_x_delta(&ctx_single) >= STAIR_RUNUP_DIST1
            && ctx_single.phase_timer > 20)
        {
            apply_stair_params();
            stair_px_disable();
            (void)stair_trigger_jump(&ctx_single);
        }
        break;

    case STAIR_PHASE_JUMPING:
        if (stair_jump_timeout(&ctx_single, 1))
        {
            break;
        }
        if (!ctx_single.jump_ok && ctx_single.phase_timer > 10)
        {
            stair_finish_with_defaults(&ctx_single, 0, ctx_single.fail_reason);
        }
        else if (jump_cfg.state == JUMP_IDLE && ctx_single.phase_timer > 10)
        {
            target_speed           = 0;
            ctx_single.phase       = STAIR_PHASE_STOPPING;
            ctx_single.phase_timer = 0;
            ctx_single.stop_x_cm = ctx_single.x_cm;
            stop_stable_count = 0;
        }
        break;

    case STAIR_PHASE_STOPPING:
        if (speed_stopped())
        {
            stair_finish_with_defaults(&ctx_single, 1, STAIR_FAIL_NONE);
        }
        else
        {
            (void)stair_stop_timeout(&ctx_single, 1);
        }
        break;

    case STAIR_PHASE_DONE:
        break;
    }
}

void stair_single_run(void)
{
    stair_display("Stair Test", ctx_single.phase, ctx_single.jump_ok, ctx_single.fail_reason,
                  ctx_single.last_trigger_result, ctx_single.x_cm, ctx_single.y_cm, 0, 0);
}

uint8 stair_single_is_done(void)
{
    return (ctx_single.phase == STAIR_PHASE_DONE) ? 1 : 0;
}

// ============================================================
// 模式 2: 连续上台阶 3 跳 (stair_seq)
//   识别到台阶后立即触发第一跳，后续两跳按固定时间间隔触发。
//   第三跳后强制刹停（平台仅 250mm，前方 22° 下坡）。
// ============================================================
void stair_seq_start(void)
{
    stair_abort();
    stair_ctx_begin(&ctx_seq);
    apply_stair_params();                 // 全程使用上台阶参数
    target_speed          = STAIR_RUNUP_SPEED;
    (void)stair_trigger_jump(&ctx_seq);   // 识别到台阶后立即进入第一跳
}

static void stair_seq_step(void)
{
    if (!ctx_seq.active) return;
    ctx_seq.phase_timer++;
    stair_pose_update(&ctx_seq);

    switch (ctx_seq.phase)
    {
    case STAIR_PHASE_RUNUP:
        stair_px_update(&ctx_seq);
        if (ctx_seq.phase_timer >= STAIR_SEQ_INTERVAL_MS)
        {
            stair_px_disable();
            (void)stair_trigger_jump(&ctx_seq);
        }
        break;

    case STAIR_PHASE_JUMPING:
        if (stair_jump_timeout(&ctx_seq, 1))
        {
            break;
        }
        if (!ctx_seq.jump_ok && ctx_seq.phase_timer > 10)
        {
            stair_finish_with_defaults(&ctx_seq, 0, ctx_seq.fail_reason);
        }
        else if (jump_cfg.state == JUMP_IDLE && ctx_seq.phase_timer > 10)
        {
            ctx_seq.jump_index++;

            if (ctx_seq.jump_index < STAIR_SEQ_TOTAL_JUMPS)
            {
                ctx_seq.phase         = STAIR_PHASE_RUNUP;
                ctx_seq.phase_timer   = 0;
                ctx_seq.trigger_wait_timer = 0;
                ctx_seq.segment_start_x = ctx_seq.x_cm;
                stair_px_set_target(&ctx_seq, ctx_seq.x_cm + STAIR_RUNUP_DISTN);
                target_speed          = STAIR_RUNUP_SPEED;
            }
            else
            {
                target_speed     = STAIR_FINAL_BRAKE_SPEED;
                ctx_seq.phase    = STAIR_PHASE_STOPPING;
                ctx_seq.phase_timer = 0;
                ctx_seq.stop_x_cm = ctx_seq.x_cm;           // 记录刹停起点
                stop_stable_count = 0;
            }
        }
        break;

    case STAIR_PHASE_STOPPING:
        stair_seq_apply_final_brake();

        if (speed_stopped())
        {
            stair_finish_with_defaults(&ctx_seq, 1, STAIR_FAIL_NONE);
        }
        else if ((ctx_seq.x_cm - ctx_seq.stop_x_cm) > STAIR_FINAL_STOP_DIST)
        {
            // 超距未停：强制终止（Step 3 仅 250mm，前方下坡）
            stair_finish_with_defaults(&ctx_seq, 0, STAIR_FAIL_OVER_DIST);
        }
        else
        {
            (void)stair_stop_timeout(&ctx_seq, 1);
        }
        break;

    case STAIR_PHASE_DONE:
        break;
    }
}

void stair_seq_run(void)
{
    uint8 display_index = (ctx_seq.jump_index < STAIR_SEQ_TOTAL_JUMPS)
                         ? (uint8)(ctx_seq.jump_index + 1)
                         : ctx_seq.jump_index;

    stair_display("Stair Seq", ctx_seq.phase, ctx_seq.jump_ok, ctx_seq.fail_reason,
                  ctx_seq.last_trigger_result, ctx_seq.x_cm, ctx_seq.y_cm, display_index, STAIR_SEQ_TOTAL_JUMPS);
}

uint8 stair_seq_is_done(void)
{
    return (ctx_seq.phase == STAIR_PHASE_DONE) ? 1 : 0;
}

// ============================================================
// 紧急终止
// ============================================================
void stair_abort(void)
{
    jump_abort();
    target_speed = 0;
    stair_px_disable();
    stair_heading_disable();
    restore_jump_defaults();

    ctx_jump.active = 0;
    ctx_jump.phase  = STAIR_PHASE_DONE;
    ctx_single.active = 0;
    ctx_single.phase  = STAIR_PHASE_DONE;
    ctx_seq.active = 0;
    ctx_seq.phase  = STAIR_PHASE_DONE;
}

void stair_service_1ms(void)
{
    stair_jump_step();
    stair_single_step();
    stair_seq_step();
}

uint8 stair_is_active(void)
{
    return (ctx_jump.active || ctx_single.active || ctx_seq.active) ? 1 : 0;
}

float stair_get_px_angle_offset(void)
{
    return stair_px_angle_offset;
}

uint8 stair_px_angle_control_active(void)
{
    return (stair_px_enable && !jump_is_active()) ? 1 : 0;
}

int16 stair_get_heading_motor_adj(void)
{
    return stair_heading_motor_adj;
}
