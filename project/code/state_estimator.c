#include "state_estimator.h"
#include "control.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数名称       state_estimator_init
// 功能说明       状态估计器初始化，清零速度和位置估计值
// 输入参数       void
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
void state_estimator_init(void)
{
    v_hat = 0.0f;
    x_hat = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 调参指南:
//   1. 机器人静止放置，观察 v_hat 是否收敛到 0
//   2. 如果 v_hat 漂移 → 增大 ESTIMATOR_OBS_GAIN (增加姿态修正)
//   3. 如果 v_hat 抖动 → 减小 ESTIMATOR_OBS_GAIN (减小观测噪声)
//   4. 如果速度估计滞后 → 减小 ESTIMATOR_FUSION_ALPHA (增加观测权重)
//   5. 如果速度估计噪声大 → 增大 ESTIMATOR_FUSION_ALPHA (增加模型权重)
//-------------------------------------------------------------------------------------------------------------------
// 函数名称       state_estimator_update
// 功能说明       互补滤波器：融合 IMU 俯仰角和电机加速度，估计速度和位置
//                 步骤1: v_pred = v_hat + motor_accel * dt                    (模型预测)
//                 步骤2: v_correction = pitch_angle * ESTIMATOR_OBS_GAIN      (观测修正)
//                 步骤3: v_hat = ESTIMATOR_FUSION_ALPHA*v_pred + (1-ALPHA)*v_correction  (互补融合)
//                 步骤4: x_hat += v_hat * dt                                   (位置积分)
// 输入参数       pitch_angle     IMU 俯仰角 (rad)
//                motor_accel     电机加速度 (m/s²)
//                dt              控制周期 (s)
// 返回参数       void
// 使用注意       增益和权重定义在 state_estimator.h 中，需在硬件平台上实测标定
//-------------------------------------------------------------------------------------------------------------------
void state_estimator_update(float pitch_angle, float motor_accel, float dt)
{
    float v_pred, v_correction;

    //===== 步骤1：模型预测速度 =====
    v_pred = v_hat + motor_accel * dt;

    //===== 步骤2：观测修正 =====
    v_correction = pitch_angle * ESTIMATOR_OBS_GAIN;

    //===== 步骤3：互补融合 =====
    // 高通滤波（模型预测）+ 低通滤波（观测修正）
    v_hat = ESTIMATOR_FUSION_ALPHA * v_pred + (1.0f - ESTIMATOR_FUSION_ALPHA) * v_correction;

    //===== 步骤4：位置积分 =====
    x_hat += v_hat * dt;

    // 估计值合理性检查
    if (v_hat > ESTIMATOR_V_MAX)  v_hat =  ESTIMATOR_V_MAX;
    if (v_hat < -ESTIMATOR_V_MAX) v_hat = -ESTIMATOR_V_MAX;
}
