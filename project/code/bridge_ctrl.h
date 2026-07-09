#ifndef __BRIDGE_CTRL_H__
#define __BRIDGE_CTRL_H__

//****************************************************************************
// 文件名称    bridge_ctrl.h
// 功能描述    单边桥控制模块头文件
//             定义单边桥检测状态机、腿高补偿与速度补偿参数及对外接口。
// 使用方式    1. 调用 bridge_init() 初始化状态机；
//             2. 在主控制循环中周期性调用 bridge_run(roll, mileage, speed) 更新状态；
//             3. 通过 bridge_get_leg_delta() 与 bridge_get_speed_extra() 获取补偿量，
//                由调用方转换为舵机 duty 或叠加到电机目标速度。
// 注意事项    1. bridge_run() 建议以 PIT 中断频率（约 1kHz）调用；
//             2. 所有 *_CYCLES 宏基于调用周期计数，若调用频率改变需同步调整。
//****************************************************************************

// 单边桥可调参数集中在 config.h，头文件只保留状态枚举与对外接口。

//============================ 辅助宏 ============================
#define DEG_TO_RAD                  (PI / 180.0f)

//============================ 状态枚举 ============================
typedef enum
{
    BRIDGE_IDLE = 0,        // 空闲：正常行驶，等待 roll 超阈值
    BRIDGE_ENTER,           // 进入桥：已确认倾斜，进行腿高调整/固定延时
    BRIDGE_CROSSING,        // 过桥中：持续输出腿高补偿与速度补偿，跟踪里程
    BRIDGE_EXIT,            // 退出桥：过渡阶段，等待舵机回中前的固定延时
    BRIDGE_RECOVER          // 恢复稳定：补偿量归零，等待姿态收敛后回到 IDLE
} bridge_state_e;

//============================ 对外接口 ============================

// 函数介绍    初始化单边桥状态机与内部变量
// 返回参数    void
// 使用示例    bridge_init();
void bridge_init(void);
void bridge_force_enter(float roll_angle, float mileage);

// 函数介绍    单边桥主状态机运行函数
// 参数说明    roll_angle — 当前 IMU roll 角（°），建议传入 roll_balance_cascade.posture_value.rol
//             mileage    — 当前车体累计里程（cm），建议传入 Car.mileage
//             speed      — 当前车体速度（RPM），建议传入 car_speed 或 target_speed
// 返回参数    void
// 使用示例    bridge_run(roll_balance_cascade.posture_value.rol, Car.mileage, (float)car_speed);
// 备注信息    需在主控制循环中周期性调用；内部基于周期计数实现延时与防抖
void bridge_run(float roll_angle, float mileage, float speed);

// 函数介绍    获取当前单边桥状态
// 返回参数    bridge_state_e — 当前状态枚举值
// 使用示例    if (bridge_get_state() == BRIDGE_CROSSING) { ... }
bridge_state_e bridge_get_state(void);

// 函数介绍    获取腿高补偿量（cm）
// 参数说明    left_delta  — 输出左腿高度补偿量（cm），桥左侧高时为负
//             right_delta — 输出右腿高度补偿量（cm），桥右侧高时为正
// 返回参数    void
// 使用示例    bridge_get_leg_delta(&left_h, &right_h);
// 备注信息    腿高量以重心侧向高度偏差 h_cg 为几何前馈，并叠加单边桥专用 roll PD；
//             仅在 BRIDGE_ENTER / BRIDGE_CROSSING / BRIDGE_EXIT 状态下非零；
//             调用方负责将 cm 差值映射为舵机 duty（简化映射，不做五连杆逆解）
void bridge_get_leg_delta(float *left_delta, float *right_delta);

// 函数介绍    获取速度补偿量（RPM）
// 参数说明    left_extra  — 输出左轮额外速度补偿（RPM）
//             right_extra — 输出右轮额外速度补偿（RPM）
// 返回参数    void
// 使用示例    bridge_get_speed_extra(&left_v, &right_v);
// 备注信息    基于左右轮轨迹高度差 h_track 计算 Δv = v * (sqrt(h_track^2 + l^2) / l - 1)；
//             仅在较高一侧轮子输出额外速度，另一侧为 0；
//             输出单位仍为速度量，调用方需按实车整定系数换算为电机 duty；
//             IDLE / RECOVER 状态下输出 0
void bridge_get_speed_extra(float *left_extra, float *right_extra);

#endif // __BRIDGE_CTRL_H__
