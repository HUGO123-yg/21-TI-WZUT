#ifndef _state_estimator_h_
#define _state_estimator_h_

// 状态估计器头文件
// 互补滤波器：融合 IMU 俯仰角和电机加速度，估计速度和位置

#include "zf_common_headfile.h"

//===== 函数原型 =====

void state_estimator_init(void);
void state_estimator_update(float pitch_angle, float motor_accel, float dt);

#endif
