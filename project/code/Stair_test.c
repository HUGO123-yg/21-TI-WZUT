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
    float            start_mileage;
    uint16           phase_timer;
} stair_ctx;

static stair_ctx ctx_jump;
static stair_ctx ctx_single;
static stair_ctx ctx_seq;

// ============================================================
// 辅助函数
// ============================================================

// 当前平均里程 (cm)
static float current_mileage(void)
{
    return (Car.mileage_L + Car.mileage_R) * 0.5f;
}

// 自起跑以来的行进距离 (cm)
static float mileage_delta(const stair_ctx *c)
{
    return current_mileage() - c->start_mileage;
}

// 车速是否已降到阈值以下 — 连续稳定检测，避免编码器噪声
static uint8 stop_stable_count = 0;

static uint8 speed_stopped(void)
{
    if (func_abs(car_speed) <= (int16)STAIR_STOP_SPEED)
        stop_stable_count++;
    else
        stop_stable_count = 0;
    return (stop_stable_count > 10) ? 1 : 0;
}

// 将 jump_cfg 设为上台阶参数
static void apply_stair_params(void)
{
    jump_cfg.charge_duty          = STAIR_CHARGE_DUTY;
    jump_cfg.forward_motor_boost  = STAIR_MOTOR_BOOST;
    jump_cfg.land_damping_duty    = STAIR_LAND_DAMPING;
    jump_cfg.airborne_timeout     = 200;
    jump_cfg.landing_acc_threshold = 1.8f;
}

// 恢复 jump_cfg 为默认值
static void restore_jump_defaults(void)
{
    jump_config_default();
}

// ============================================================
// 公用 LCD 显示
// ============================================================
static void stair_display(const char *title, stair_phase_enum phase,
                          uint8 jump_ok, uint8 jump_idx, uint8 jump_total)
{
    ips_show_string(8 * 0, 16 * 0, title);
    ips_show_string(8 * 0, 16 * 2, "Phase:");
    switch (phase)
    {
    case STAIR_PHASE_RUNUP:
        ips_show_string(8 * 10, 16 * 2, "Runup...");
        break;
    case STAIR_PHASE_JUMPING:
        ips_show_string(8 * 10, 16 * 2, "Jumping...");
        break;
    case STAIR_PHASE_STOPPING:
        ips_show_string(8 * 10, 16 * 2, "Stopping...");
        break;
    case STAIR_PHASE_DONE:
        ips_show_string(8 * 10, 16 * 2, jump_ok ? "Done!" : "FAIL");
        break;
    }
    ips_show_string(8 * 0, 16 * 3, "State:"); ips_show_int(8 * 10, 16 * 3, jump_cfg.state, 3);
    ips_show_string(8 * 0, 16 * 4, "Speed:"); ips_show_float(8 * 10, 16 * 4, (float)car_speed, 5, 1);
    ips_show_string(8 * 0, 16 * 5, "Dist:");  ips_show_float(8 * 10, 16 * 5, current_mileage(), 6, 2);
    if (jump_total > 1)
    {
        ips_show_string(8 * 0, 16 * 6, "Jump:");  ips_show_int(8 * 10, 16 * 6, jump_idx, 1);
        ips_show_string(8 * 12, 16 * 6, "/");      ips_show_int(8 * 13, 16 * 6, jump_total, 1);
    }
}

// ============================================================
// 模式 0: 平地跳跃测试 (stair_jump)
//   — 原 Menu.c fun_d34_jump_test 移入
// ============================================================
void stair_jump_start(void)
{
    restore_jump_defaults();               // 清理可能残留的上台阶参数
    ctx_jump.active       = 1;
    ctx_jump.phase        = STAIR_PHASE_RUNUP;
    ctx_jump.phase_timer  = 0;
    target_speed          = JUMP_RUNUP_SPEED;
}

void stair_jump_run(void)
{
    if (!ctx_jump.active) return;
    ctx_jump.phase_timer++;

    switch (ctx_jump.phase)
    {
    case STAIR_PHASE_RUNUP:
        if (ctx_jump.phase_timer > JUMP_RUNUP_TICKS)
        {
            jump_trigger();
            ctx_jump.jump_ok    = (jump_cfg.state != JUMP_IDLE);
            ctx_jump.phase      = STAIR_PHASE_JUMPING;
            ctx_jump.phase_timer = 0;
        }
        break;

    case STAIR_PHASE_JUMPING:
        if (!ctx_jump.jump_ok && ctx_jump.phase_timer > 10)
        {
            ctx_jump.active = 0;
            ctx_jump.phase  = STAIR_PHASE_DONE;
        }
        else if (jump_cfg.state == JUMP_IDLE && ctx_jump.phase_timer > 10)
        {
            target_speed         = 0;
            ctx_jump.phase       = STAIR_PHASE_STOPPING;
            ctx_jump.phase_timer = 0;
        }
        break;

    case STAIR_PHASE_STOPPING:
        if (speed_stopped())
        {
            ctx_jump.phase = STAIR_PHASE_DONE;
        }
        break;

    case STAIR_PHASE_DONE:
        break;
    }

    stair_display("Jump Test", ctx_jump.phase, ctx_jump.jump_ok, 0, 0);
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
    ctx_single.active       = 1;
    ctx_single.phase        = STAIR_PHASE_RUNUP;
    ctx_single.phase_timer  = 0;
    ctx_single.start_mileage = current_mileage();
    target_speed            = STAIR_RUNUP_SPEED;
}

void stair_single_run(void)
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
            jump_trigger();
            ctx_single.jump_ok    = (jump_cfg.state != JUMP_IDLE);
            ctx_single.phase      = STAIR_PHASE_JUMPING;
            ctx_single.phase_timer = 0;
        }
        break;

    case STAIR_PHASE_JUMPING:
        if (!ctx_single.jump_ok && ctx_single.phase_timer > 10)
        {
            restore_jump_defaults();
            ctx_single.active = 0;
            ctx_single.phase  = STAIR_PHASE_DONE;
        }
        else if (jump_cfg.state == JUMP_IDLE && ctx_single.phase_timer > 10)
        {
            target_speed           = 0;
            ctx_single.phase       = STAIR_PHASE_STOPPING;
            ctx_single.phase_timer = 0;
        }
        break;

    case STAIR_PHASE_STOPPING:
        if (speed_stopped())
        {
            restore_jump_defaults();
            ctx_single.phase = STAIR_PHASE_DONE;
        }
        break;

    case STAIR_PHASE_DONE:
        break;
    }

    stair_display("Stair Test", ctx_single.phase, ctx_single.jump_ok, 0, 0);
}

uint8 stair_single_is_done(void)
{
    return (ctx_single.phase == STAIR_PHASE_DONE) ? 1 : 0;
}

// ============================================================
// 模式 2: 连续上台阶 3 跳 (stair_seq)
//   地面→Step1→Step2→Step3，每跳后短助跑再跳。
//   第三跳后强制刹停（平台仅 250mm，前方 22° 下坡）。
// ============================================================
void stair_seq_start(void)
{
    ctx_seq.active        = 1;
    ctx_seq.phase         = STAIR_PHASE_RUNUP;
    ctx_seq.jump_index    = 0;
    ctx_seq.phase_timer   = 0;
    ctx_seq.start_mileage = current_mileage();
    target_speed          = STAIR_RUNUP_SPEED;
    apply_stair_params();                 // 全程使用上台阶参数
}

void stair_seq_run(void)
{
    if (!ctx_seq.active) return;
    ctx_seq.phase_timer++;

    switch (ctx_seq.phase)
    {
    case STAIR_PHASE_RUNUP:
    {
        float need_dist = (ctx_seq.jump_index == 0)
                          ? STAIR_RUNUP_DIST1
                          : STAIR_RUNUP_DISTN;
        if (mileage_delta(&ctx_seq) >= need_dist
            && ctx_seq.phase_timer > 20)
        {
            jump_trigger();
            ctx_seq.jump_ok     = (jump_cfg.state != JUMP_IDLE);
            ctx_seq.phase       = STAIR_PHASE_JUMPING;
            ctx_seq.phase_timer = 0;
        }
    }
        break;

    case STAIR_PHASE_JUMPING:
        if (!ctx_seq.jump_ok && ctx_seq.phase_timer > 10)
        {
            restore_jump_defaults();
            ctx_seq.active = 0;
            ctx_seq.phase  = STAIR_PHASE_DONE;
        }
        else if (jump_cfg.state == JUMP_IDLE && ctx_seq.phase_timer > 10)
        {
            ctx_seq.jump_index++;

            if (ctx_seq.jump_index < 3)
            {
                ctx_seq.phase         = STAIR_PHASE_RUNUP;
                ctx_seq.phase_timer   = 0;
                ctx_seq.start_mileage = current_mileage();
                target_speed          = STAIR_RUNUP_SPEED;
            }
            else
            {
                target_speed     = 0;
                ctx_seq.phase    = STAIR_PHASE_STOPPING;
                ctx_seq.phase_timer = 0;
                ctx_seq.start_mileage = current_mileage();   // 记录刹停起点
            }
        }
        break;

    case STAIR_PHASE_STOPPING:
        if (speed_stopped())
        {
            restore_jump_defaults();
            ctx_seq.phase = STAIR_PHASE_DONE;
        }
        else if (mileage_delta(&ctx_seq) > STAIR_FINAL_STOP_DIST)
        {
            // 超距未停：强制终止（Step 3 仅 250mm，前方下坡）
            restore_jump_defaults();
            ctx_seq.jump_ok = 0;
            ctx_seq.active  = 0;
            ctx_seq.phase   = STAIR_PHASE_DONE;
        }
        break;

    case STAIR_PHASE_DONE:
        break;
    }

    stair_display("Stair Seq", ctx_seq.phase, ctx_seq.jump_ok, ctx_seq.jump_index, 3);
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

    ctx_jump.active   = 0;
    ctx_single.active = 0;
    ctx_seq.active    = 0;
}
