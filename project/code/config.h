#ifndef BODY_CTRL_CONFIG_H_
#define BODY_CTRL_CONFIG_H_

/*
 * 车体控制可调参数集中配置。
 * 状态编号、数组下标和单位换算常量仍保留在控制代码中。
 */

/* 上电默认状态 */
#define BODY_TARGET_SPEED_DEFAULT              (0.0f)    // 默认目标速度
#define BODY_RUN_STATE_DEFAULT                 (1)       // 默认允许车体控制
#define BODY_STOP_FLAG_DEFAULT                 (1)       // 默认允许电机输出

/* 车体和电机输出 */
#define BODY_WHEEL_DIAMETER_CM                 (6.4f)    // 车轮直径，影响里程换算
#define BODY_BALANCE_DUTY_MAX                  (3000)    // 平衡输出限幅
#define BODY_TURN_DUTY_MAX                     (3000)    // 叠加转向后的总输出限幅
#define BODY_YAW_GYRO_DIVISOR                  (3)       // Z 轴角速度转为差速补偿的衰减系数

/* 安全保护和 PID 启动 */
#define BODY_TILT_LIMIT_DEG                    (40.0f)   // 横滚或俯仰超过该角度时停机
#define BODY_TILT_RECOVER_DEG                  (30.0f)   // 倾倒后回到该角度内才重新允许启动
#define BODY_PID_RAMP_CYCLES                   (500U)    // PID 从弱到强的渐变周期
#define BODY_PID_RAMP_INITIAL_SCALE            (0.2f)    // 渐变开始时的 P 参数比例
#define BODY_PID_RAMP_SCALE_RANGE              (0.8f)    // 渐变过程中增加的 P 参数比例
#define BODY_JUMP_PID_SCALE                    (0.5f)    // 跳跃时平衡 P 参数比例

/* 前后姿态闭环：保持现有 IMU 安装方向，可在实车符号检查时只修改 SIGN */
#define BODY_PITCH_TARGET_DEG                  (0.0f)    // 前后姿态目标
#define BODY_PITCH_MECHANICAL_ZERO_DEG         (-6.0f)   // 车体能够直立时的机械零点
#define BODY_PITCH_ANGLE_FEEDBACK_SIGN         (-1.0f)   // pit 反馈方向
#define BODY_PITCH_RATE_FEEDBACK_SIGN          (1.0f)    // gyro_y 反馈方向
#define BODY_PITCH_ANGLE_KP                    (700.0f)
#define BODY_PITCH_ANGLE_KI                    (1.0f)
#define BODY_PITCH_ANGLE_KD                    (50.0f)
#define BODY_PITCH_ANGLE_I_VALUE_MAX           (1000.0f)
#define BODY_PITCH_ANGLE_I_VALUE_PRO           (2.0f)
#define BODY_PITCH_RATE_TARGET_MAX             (10000.0f)
#define BODY_PITCH_RATE_KP                     (1.1f)
#define BODY_PITCH_RATE_KI                     (0.0f)
#define BODY_PITCH_RATE_KD                     (0.0f)
#define BODY_PITCH_RATE_I_VALUE_MAX            (1000.0f)
#define BODY_PITCH_RATE_I_VALUE_PRO            (0.1f)
#define BODY_PITCH_MOTOR_OUTPUT_MAX            (10000.0f)

/* 横滚姿态串级闭环：角度环输出目标角速度，角速度环输出左右腿差分 PWM */
#define BODY_ROLL_CONTROL_ENABLE               (1)
#define BODY_ROLL_TARGET_DEG                   (0.0f)
#define BODY_ROLL_MECHANICAL_ZERO_DEG          (0.0f)
#define BODY_ROLL_ANGLE_FEEDBACK_SIGN          (1.0f)    // rol 反馈方向，实车若正反馈则改为 -1
#define BODY_ROLL_RATE_FEEDBACK_SIGN           (1.0f)    // gyro_x 反馈方向，实车若正反馈则改为 -1
#define BODY_ROLL_ANGLE_KP                     (18.0f)
#define BODY_ROLL_ANGLE_KI                     (0.0f)
#define BODY_ROLL_ANGLE_KD                     (0.0f)
#define BODY_ROLL_ANGLE_I_VALUE_MAX            (100.0f)
#define BODY_ROLL_ANGLE_I_VALUE_PRO            (0.02f)
#define BODY_ROLL_RATE_TARGET_MAX              (300.0f)
#define BODY_ROLL_RATE_KP                      (0.8f)
#define BODY_ROLL_RATE_KI                      (0.0f)
#define BODY_ROLL_RATE_KD                      (0.0f)
#define BODY_ROLL_RATE_I_VALUE_MAX             (100.0f)
#define BODY_ROLL_RATE_I_VALUE_PRO             (0.02f)
#define BODY_ROLL_STEER_OUTPUT_MAX             (300.0f)

/* 舵机姿态和速度辅助 */
#define STEER_PITCH_ATTENUATION_LIMIT_DEG      (30.0f)   // 倾角达到该值时速度辅助衰减到零
#define STEER_SPEED_OUTPUT_DIVISOR             (7.0f)    // 速度环输出到舵机辅助量的缩小系数
#define STEER_SPEED_OUTPUT_LIMIT               (250)     // 缩小后的速度辅助限幅
#define STEER_SPEED_OUTPUT_GAIN                (6)       // 速度辅助输出增益
#define STEER_FILTER_HISTORY_WEIGHT            (8.0f)    // 滤波器历史值权重
#define STEER_FILTER_INPUT_WEIGHT              (1.0f)    // 滤波器新输入权重
#define STEER_FILTER_DIVISOR                   (10.0f)   // 滤波器总除数
#define STEER_ROLL_ENABLE_DELAY_CYCLES         (2000U)   // 上电后延迟启用横滚姿态闭环
#define STEER_ROLL_OUTPUT_LIMIT                (300)     // 横滚闭环到左右腿差分的限幅
#define STEER_ROLL_OUTPUT_GAIN                 (1.0f)    // 横滚闭环到舵机 PWM 的增益
#define STEER_ROLL_OUTPUT_SIGN                 (1.0f)    // 左右腿补偿方向，实车若正反馈则改为 -1
#define STEER_NORMAL_STEP_LIMIT                (10)      // 正常控制时单周期最大舵机步进
#define STEER_STOP_RETURN_STEP_LIMIT           (1)       // 停机回中时单周期最大舵机步进

/* 跳跃四阶段时序和舵机动作 */
#define JUMP_EXTEND_CYCLES                     (110)     // 起跳阶段周期
#define JUMP_HOLD_CYCLES                       (110)     // 收腿/保持阶段周期
#define JUMP_PRELOAD_CYCLES                    (30)      // 预备缓冲阶段周期
#define JUMP_EXECUTE_CYCLES                    (100)     // 执行缓冲阶段周期
#define JUMP_EXTEND_DUTY_OFFSET                (2500)    // 起跳阶段相对舵机中值偏移
#define JUMP_PRELOAD_DUTY_OFFSET               (1400)    // 预备缓冲阶段相对舵机中值偏移
#define JUMP_EXECUTE_STEER_STEP                (-14)     // 执行缓冲阶段舵机步进

/* 主控制循环调度 */
#define BODY_CONTROL_STARTUP_DELAY_CYCLES      (500U)    // 上电后开始闭环控制的等待周期
#define BODY_ANGLE_LOOP_DIVIDER                (5U)      // 角度环执行分频
#define BODY_SPEED_LOOP_DIVIDER                (20U)     // 速度环执行分频
#define BODY_TRACK_OUTPUT_GAIN                 (10.0f)   // 导航输出叠加到左右电机的增益

#if BODY_YAW_GYRO_DIVISOR == 0
#error "BODY_YAW_GYRO_DIVISOR must not be zero"
#endif

#if BODY_PID_RAMP_CYCLES == 0 || BODY_ANGLE_LOOP_DIVIDER == 0 || BODY_SPEED_LOOP_DIVIDER == 0
#error "Body control cycle count and dividers must not be zero"
#endif

#endif
