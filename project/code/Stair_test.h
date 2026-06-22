/*
 * Stair_test.h — 上台阶 / 跳跃测试模块
 *
 *  Created on: 2026-6-19
 *      Author: HUGO
 *
 * 三个测试模式：
 *   stair_jump   — 平地跳跃测试（加速→跳→减速→停）
 *   stair_single — 单次上台阶（可调参数，距离触发）
 *   stair_seq   — 连续上台阶（3 跳序列：地面→S1→S2→S3）
 */

#ifndef CODE_STAIR_TEST_H_
#define CODE_STAIR_TEST_H_

#include "zf_common_headfile.h"

// ============================================================
// 可调参数 — 集中一处，方便现场调参
// ============================================================

// ---- 平地跳跃测试 ----
#define JUMP_RUNUP_SPEED      200.0f
#define JUMP_RUNUP_TICKS      150       // super-loop 循环数

// ---- 单次 / 连续上台阶 ----
#define STAIR_RUNUP_SPEED     280.0f    // 助跑速度
#define STAIR_RUNUP_DIST1     30.0f     // 第一跳助跑距离 (cm)
#define STAIR_RUNUP_DISTN     20.0f     // 后续跳助跑距离 (cm)
#define STAIR_FINAL_STOP_DIST 15.0f     // 最后一跳刹停距离上限 (cm)
#define STAIR_CHARGE_TICKS    140       // 上台阶蓄力时间 (ms)
#define STAIR_LAUNCH_TICKS    35        // 起跳释放时间 (ms)
#define STAIR_CHARGE_DUTY     3800      // X 型压缩量 (2000~5000)
#define STAIR_PRELAND_DUTY    1500      // 腾空后预着陆伸腿量
#define STAIR_MOTOR_BOOST     800.0f    // 起跳电机助推
#define STAIR_LAND_DAMPING    35        // 落地缓冲步进
#define STAIR_AIRBORNE_TIMEOUT 220      // 腾空超时保护 (ms)
#define STAIR_LANDING_ACC_THRESHOLD 1.8f // 落地冲击阈值 (g)
#define STAIR_STOP_SPEED      5.0f      // 判定停止的速度阈值

// ============================================================
// API — 三个测试模式 + 终止
// ============================================================

// 平地跳跃测试
void stair_jump_start(void);
void stair_jump_run(void);
uint8 stair_jump_is_done(void);

// 单次上台阶
void stair_single_start(void);
void stair_single_run(void);
uint8 stair_single_is_done(void);

// 连续上台阶 (3 跳)
void stair_seq_start(void);
void stair_seq_run(void);
uint8 stair_seq_is_done(void);

// 紧急终止（所有模式）
void stair_abort(void);

#endif /* CODE_STAIR_TEST_H_ */
