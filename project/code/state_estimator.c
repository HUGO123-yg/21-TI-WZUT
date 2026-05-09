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
// 函数名称       state_estimator_update
// 功能说明       互补滤波器：融合 IMU 俯仰角和电机加速度，估计速度和位置
//                 步骤1: v_pred = v_hat + motor_accel * dt         (模型预测)
//                 步骤2: v_correction = pitch_angle * 0.05f         (观测修正)
//                 步骤3: v_hat = 0.95f * v_pred + 0.05f * v_correction   (互补融合)
//                 步骤4: x_hat += v_hat * dt                        (位置积分)
// 输入参数       pitch_angle     IMU 俯仰角 (rad)
//                motor_accel     电机加速度 (m/s²)
//                dt              控制周期 (s)
// 返回参数       void
// 使用注意       滤波器增益和融合权重为初始猜测值，需在硬件平台上实测标定
//-------------------------------------------------------------------------------------------------------------------
void state_estimator_update(float pitch_angle, float motor_accel, float dt)
{
    float v_pred, v_correction;

    //===== 步骤1：模型预测速度 =====
    v_pred = v_hat + motor_accel * dt;

    //===== 步骤2：观测修正 =====
    // [TODO: 标定滤波器增益]
    v_correction = pitch_angle * 0.05f;

    //===== 步骤3：互补融合 =====
    // 高通滤波（模型预测，0.95）+ 低通滤波（观测修正，0.05）
    // [TODO: 标定融合权重]
    v_hat = 0.95f * v_pred + 0.05f * v_correction;

    //===== 步骤4：位置积分 =====
    x_hat += v_hat * dt;
}
