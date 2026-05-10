#ifndef _state_estimator_h_
#define _state_estimator_h_

// 状态估计器头文件
// 互补滤波器：融合 IMU 俯仰角和电机加速度，估计速度和位置

#include "zf_common_headfile.h"

//===== 互补滤波器增益 (可调) =====
// [可调] 观测修正增益 — pitch_angle → v_correction 的比例系数
//   增大 → 加速度计权重↑，速度估计更依赖姿态；减小 → 更依赖模型预测
#define ESTIMATOR_OBS_GAIN       (0.05f)
// [可调] 互补融合权重 — 模型预测的比例 (1-alpha 为观测比例)
//   范围: (0.5, 0.99)，越大越信任模型预测(电机加速度)
#define ESTIMATOR_FUSION_ALPHA   (0.95f)

//===== 估计值合理性检查 =====
#define ESTIMATOR_V_MAX          (10.0f)     // 速度上限 (m/s)

//===== 函数原型 =====

void state_estimator_init(void);
void state_estimator_update(float pitch_angle, float motor_accel, float dt);

#endif
