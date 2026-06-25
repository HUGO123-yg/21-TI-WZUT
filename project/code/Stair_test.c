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
    float            start_mileage;
    float            stop_mileage;
    uint16           phase_timer;
    uint16           trigger_wait_timer;
} stair_ctx;

static stair_ctx ctx_jump;
static stair_ctx ctx_single;
static stair_ctx ctx_seq;
static uint8 stop_stable_count = 0;

// ============================================================
// 辅助函数
// ============================================================

// 当前平均里程 (cm)
static float current_mileage(void)
{
    return Car.mileage;
}

static void stair_ctx_begin(stair_ctx *c)
{
    c->active             = 1;
    c->phase              = STAIR_PHASE_RUNUP;
    c->jump_index         = 0;
    c->jump_ok            = 0;
    c->fail_reason        = 0;
    c->last_trigger_result = JUMP_TRIGGER_OK;
    c->start_mileage      = current_mileage();
    c->stop_mileage       = 0.0f;
    c->phase_timer        = 0;
    c->trigger_wait_timer = 0;
    stop_stable_count = 0;
}

// 自起跑以来的行进距离 (cm)
static float mileage_delta(const stair_ctx *c)
{
    return current_mileage() - c->start_mileage;
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
                          float distance_cm, uint8 jump_idx, uint8 jump_total)
{
    static uint32 last_display_ms = 0;
    int32 dist_i;

    if (last_display_ms != 0 && (uint32)(sys_times - last_display_ms) < STAIR_LCD_PERIOD_MS)
        return;
    last_display_ms = sys_times;

    dist_i = (int32)distance_cm;
    dist_i = func_limit_ab(dist_i, -9999, 9999);

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
    ips_show_string(8 * 0, 16 * 5, "D:");
    ips_show_int(8 * 3, 16 * 5, dist_i, 5);
    ips_show_string(8 * 0, 16 * 6, "F:");
    ips_show_int(8 * 3, 16 * 6, fail_reason, 1);
    ips_show_string(8 * 5, 16 * 6, jump_ok ? "OK " : "-- ");
    ips_show_string(8 * 0, 16 * 8, "T:");
    ips_show_string(8 * 3, 16 * 8, jump_trigger_result_name(last_trigger_result));
    if (jump_total > 1)
    {
        ips_show_string(8 * 0, 16 * 7, "J:");
        ips_show_int(8 * 3, 16 * 7, jump_idx, 1);
        ips_show_string(8 * 4, 16 * 7, "/");
        ips_show_int(8 * 5, 16 * 7, jump_total, 1);
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
            ctx_jump.stop_mileage = current_mileage();
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
                  ctx_jump.last_trigger_result, mileage_delta(&ctx_jump), 0, 0);
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
    target_speed            = STAIR_RUNUP_SPEED;
}

static void stair_single_step(void)
{
    if (!ctx_single.active) return;
    ctx_single.phase_timer++;

    switch (ctx_single.phase)
    {
    case STAIR_PHASE_RUNUP:
        if (mileage_delta(&ctx_single) >= STAIR_RUNUP_DIST1
            && ctx_single.phase_timer > 20)
        {
            apply_stair_params();
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
            ctx_single.stop_mileage = current_mileage();
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
                  ctx_single.last_trigger_result, mileage_delta(&ctx_single), 0, 0);
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

    switch (ctx_seq.phase)
    {
    case STAIR_PHASE_RUNUP:
        target_speed = STAIR_RUNUP_SPEED;
        if (ctx_seq.phase_timer >= STAIR_SEQ_INTERVAL_MS)
        {
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
                ctx_seq.start_mileage = current_mileage();
                target_speed          = STAIR_RUNUP_SPEED;
            }
            else
            {
                target_speed     = STAIR_FINAL_BRAKE_SPEED;
                ctx_seq.phase    = STAIR_PHASE_STOPPING;
                ctx_seq.phase_timer = 0;
                ctx_seq.stop_mileage = current_mileage();   // 记录刹停起点
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
        else if ((current_mileage() - ctx_seq.stop_mileage) > STAIR_FINAL_STOP_DIST)
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
                  ctx_seq.last_trigger_result, mileage_delta(&ctx_seq), display_index, STAIR_SEQ_TOTAL_JUMPS);
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
