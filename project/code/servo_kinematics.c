/*-------------------------------------------------------------------------------------------------------------------*/
/* 文件名称       servo_kinematics.c                                                                                  */
/* 功能说明       腿舵机逆运动学查表 — 根据腿高 p 和角度 angle 解算两舵机 PWM 偏移                                      */
/*-------------------------------------------------------------------------------------------------------------------*/

#include "servo_kinematics.h"
#include "control.h"    // RAD_TO_DEG 宏
#include <stdio.h>

//----* 机械中位偏移量 *-----
// 由视觉模块运行时写入，初值置零
//
float mid_point = 0;

/*-------------------------------------------------------------------------------------------------------------------
// 函数名称       servo_control_table
// 功能说明       腿舵机逆运动学查表 — 根据当前姿态解算相位1/4舵机的 PWM 偏移
// 输入参数       p          腿高 (目标伸缩量)
//               angle      腿角度 (目标偏转角, 度)
// 输出参数       out_ph1    相位 1 舵机 PWM 偏移
//               out_ph4    相位 4 舵机 PWM 偏移
// 返回参数       void
// 使用示例       servo_control_table(5.5, 0.0, &pwm_ph1, &pwm_ph4);
// 备注信息       [TODO: 四杆机构逆运动学解算 — 当前使用简化线性映射]
//               调用方 (left_leg_control / right_leg_control) 将 out_ph1/out_ph4 叠加到
//               SERVO1_MID~SERVO4_MID 后通过 pwm_set_duty 输出
-------------------------------------------------------------------------------------------------------------------*/

void servo_angle_to_pwm(float angle_deg, int16 *out_pwm)
{
    float pwm_f = angle_deg * PWM_PER_DEG;
    pwm_f = func_limit(pwm_f, 10000.0f);
    *out_pwm = (int16)pwm_f;
}

static uint8 param_warned = 0;

void servo_control_table(float p, float angle, int16 *out_ph1, int16 *out_ph4)
{
    // 参数合理性检查（首次调用时输出一次警告）
    if ((L1_MM < 10.0f || L2_MM < 10.0f || D_MM < 5.0f) && !param_warned)
    {
        param_warned = 1;
#ifdef KINEMATICS_DEBUG
        printf("[servo_kinematics] WARNING: L1/L2/D may be unreasonable (L1=%.1f, L2=%.1f, D=%.1f). Check kinematics params.\n", L1_MM, L2_MM, D_MM);
#endif
    }

    // [TODO: 四杆机构逆运动学解算]
    // 输入: p = 腿端伸缩量(mm), angle = 腿端偏转角(deg)
    // 输出: out_ph1 = 下舵机 PWM 偏移, out_ph4 = 上舵机 PWM 偏移
    //
    // 机构参数（待 MATLAB 仿真后填入实测值）:
    //   L1 = L1_MM (上连杆长度), L2 = L2_MM (下连杆长度), d = D_MM (舵机轴间距)
    //
    // 解算步骤（余弦定理）:
    //   1. 腿端点在世界坐标系的坐标: x = p*cos(angle), y = p*sin(angle)
    //   2. 上舵机轴→端点距离: R1 = sqrt(x² + (y - d/2)²)
    //   3. 下舵机轴→端点距离: R2 = sqrt(x² + (y + d/2)²)
    //   4. 余弦定理求舵机转角:
    //      cos(theta1) = (L1² + R1² - L2²) / (2*L1*R1)
    //      cos(theta2) = (L2² + R2² - L1²) / (2*L2*R2)
    //   5. theta1 = acos(cos_theta1), theta2 = acos(cos_theta2) → 弧度转度
    //   6. 角度限幅 → PWM 转换

    float x, y, R1, R2;
    float cos_theta1, cos_theta2;
    float theta1_deg, theta2_deg;

    // 步骤1: 腿端点坐标 (角度→弧度)
    float angle_rad = angle / RAD_TO_DEG;
    x = p * cosf(angle_rad);
    y = p * sinf(angle_rad);

    // 步骤2: 各舵机轴到端点的距离
    R1 = sqrtf(x*x + (y - D_MM/2.0f)*(y - D_MM/2.0f));
    R2 = sqrtf(x*x + (y + D_MM/2.0f)*(y + D_MM/2.0f));

    // 步骤3: 余弦定理求角
    cos_theta1 = (L1_MM*L1_MM + R1*R1 - L2_MM*L2_MM) / (2.0f * L1_MM * R1);
    cos_theta2 = (L2_MM*L2_MM + R2*R2 - L1_MM*L1_MM) / (2.0f * L2_MM * R2);

    // 步骤4: acos 定义域安全限幅到 [-1, 1]
    if (cos_theta1 > 1.0f) cos_theta1 = 1.0f;
    if (cos_theta1 < -1.0f) cos_theta1 = -1.0f;
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

    theta1_deg = acosf(cos_theta1) * 57.29577951308232f;
    theta2_deg = acosf(cos_theta2) * 57.29577951308232f;

    // 步骤5: 角度限幅到机械范围
    theta1_deg = func_limit(theta1_deg, SERVO_ANGLE_MAX);
    theta2_deg = func_limit(theta2_deg, SERVO_ANGLE_MAX);

    // 步骤6: 角度 → PWM 偏移
    servo_angle_to_pwm(theta2_deg, out_ph1);  // 下舵机角度→PWM1
    servo_angle_to_pwm(theta1_deg, out_ph4);  // 上舵机角度→PWM4
}
