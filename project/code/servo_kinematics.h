#ifndef _servo_kinematics_h_
#define _servo_kinematics_h_

// 腿舵机逆运动学模块
// 提供 SERVO 引脚别名、死区标定常量、舵机控制查表函数

#include "zf_common_headfile.h"
#include "servo.h"

//===== SERVO 引脚别名 → 来自 servo.h 的 STEER 宏 =====

#define SERVO_1    (STEER_1_PWM)     // 左腿上舵机 → 同 STEER_1
#define SERVO_2    (STEER_2_PWM)     // 左腿下舵机 → 同 STEER_2
#define SERVO_3    (STEER_3_PWM)     // 右腿上舵机 → 同 STEER_3
#define SERVO_4    (STEER_4_PWM)     // 右腿下舵机 → 同 STEER_4

#define SERVO1_MID (STEER_1_CENTER)  // 左腿上舵机中位值 = 1000
#define SERVO2_MID (STEER_2_CENTER)  // 左腿下舵机中位值 = 1000
#define SERVO3_MID (STEER_3_CENTER)  // 右腿上舵机中位值 = 1000
#define SERVO4_MID (STEER_4_CENTER)  // 右腿下舵机中位值 = 1000

//===== 死区标定常量 (可调) =====
// [可调] 实测标定 — 根据电机最小启动占空比填入
//   标定方法: 从小到大增大占空比，记录轮子开始转动时的值
#define L_dead_zone_correct    (200)   // 左轮正转死区补偿
#define L_dead_zone_negative   (200)   // 左轮反转死区补偿
#define R_dead_zone_correct    (200)   // 右轮正转死区补偿
#define R_dead_zone_negative   (200)   // 右轮反转死区补偿

//===== 四杆机构尺寸参数 (单位: mm) =====
// [可调] 上连杆长度 — 待 MATLAB 仿真或实机测量标定
#define L1_MM              (42.0f)
// [可调] 下连杆长度
#define L2_MM              (38.0f)
// [可调] 上下舵机轴间距
#define D_MM               (24.0f)

//===== 舵机转换参数 =====
// [可调] 舵机最大机械角度 (deg) — 单向行程
#define SERVO_ANGLE_MAX    (90.0f)
// [可调] 每度对应的 PWM 偏移量 (PWM/deg)
//   计算参考: 舵机 1000us~2000us 对应 ±90°, PWM 范围 ±10000
//   PWM_PER_DEG = 10000 / 90 ≈ 111.11
#define PWM_PER_DEG        (111.0f)

// 内部辅助函数
void servo_angle_to_pwm(float angle_deg, int16 *out_pwm);

//===== 外部变量声明 =====

extern float mid_point;              // 机械中位偏移量 (视觉模块输入)

//===== 函数声明 =====

void servo_control_table(float p, float angle, int16 *out_ph1, int16 *out_ph4);

#endif
