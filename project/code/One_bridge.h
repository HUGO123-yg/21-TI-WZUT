/*
 * One_bridge.h — 单边桥自动通过控制模块
 *
 * 通过 IMU roll 自动识别单边桥，输出左右腿差动偏移和高侧轮差速补偿。
 */

#ifndef CODE_ONE_BRIDGE_H_
#define CODE_ONE_BRIDGE_H_

#include "zf_common_headfile.h"

// ============================================================
// 可调参数 — 现场调参优先改这里
// ============================================================

#define ONE_BRIDGE_AUTO_ENABLE_DEFAULT      (1)
#define ONE_BRIDGE_ENTRY_ROLL_DEG           (4.0f)      // roll 持续超过该阈值进入桥模式
#define ONE_BRIDGE_EXIT_ROLL_DEG            (1.2f)      // roll 回落阈值
#define ONE_BRIDGE_ENTRY_STABLE_MS          (40)        // 进入判定连续时间
#define ONE_BRIDGE_EXIT_STABLE_MS           (100)       // 退出判定连续时间
#define ONE_BRIDGE_MIN_SPEED                (40.0f)     // 低于该速度不自动触发
#define ONE_BRIDGE_MAX_SPEED                (160.0f)    // 桥上目标速度上限
#define ONE_BRIDGE_RUN_DISTANCE_CM          (36.0f)     // 通过距离保护
#define ONE_BRIDGE_MIN_EXIT_DISTANCE_CM     (22.0f)     // 至少行驶该距离后才允许 roll 退出
#define ONE_BRIDGE_MAX_ACTIVE_MS            (1600)      // 超时保护
#define ONE_BRIDGE_COOLDOWN_MS              (350)       // 退出后冷却，避免重复触发

#define ONE_BRIDGE_ROLL_ZERO_DEG            (0.0f)
#define ONE_BRIDGE_ROLL_FILTER_ALPHA        (0.85f)
#define ONE_BRIDGE_LEG_P_DUTY_PER_DEG       (35.0f)
#define ONE_BRIDGE_LEG_I_DUTY_PER_DEG_MS    (0.08f)
#define ONE_BRIDGE_LEG_MAX_DUTY             (650.0f)
#define ONE_BRIDGE_LEG_DIR                  (1.0f)      // 若腿差动方向相反，改为 -1.0f

// 单边桥高度 5cm、长度 20cm 时，斜坡轮路径比水平投影约多 11.8%。
#define ONE_BRIDGE_PATH_RATIO               (0.118f)
#define ONE_BRIDGE_SPEED_TO_DUTY_GAIN       (5.0f)
#define ONE_BRIDGE_MOTOR_COMP_MAX           (260)
#define ONE_BRIDGE_MOTOR_DIR                (1)         // 若差速方向相反，改为 -1

// ============================================================
// 状态与上下文
// ============================================================

typedef enum
{
    ONE_BRIDGE_IDLE    = 0,
    ONE_BRIDGE_RUNNING = 1,
    ONE_BRIDGE_EXITING = 2,
} one_bridge_state_enum;

typedef enum
{
    ONE_BRIDGE_SIDE_NONE     = 0,
    ONE_BRIDGE_SIDE_POS_ROLL = 1,
    ONE_BRIDGE_SIDE_NEG_ROLL = 2,
} one_bridge_side_enum;

typedef struct
{
    uint8   auto_enable;
    uint8   state;
    uint8   side;
    uint8   entry_stable_count;
    uint8   exit_stable_count;
    uint16  elapsed;
    uint16  cooldown;
    float   roll_error;
    float   roll_filtered;
    float   leg_integral;
    int16   leg_offset;
    int16   motor_adj;
    float   start_mileage;
    float   distance;
    float   saved_speed_target;
} one_bridge_control_struct;

extern one_bridge_control_struct one_bridge;

void one_bridge_service_1ms(void);
void one_bridge_abort(void);
void one_bridge_set_auto(uint8 enable);
uint8 one_bridge_is_active(void);
void one_bridge_get_steer_offsets(int16 offsets[4]);
int16 one_bridge_get_motor_adj(void);
const char *one_bridge_state_name(uint8 state);
const char *one_bridge_side_name(uint8 side);

#endif /* CODE_ONE_BRIDGE_H_ */
