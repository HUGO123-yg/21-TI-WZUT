//================================================================================
// 跳跃控制状态枚举
// jump control state enumeration
//================================================================================
typedef enum
{
    JUMP_IDLE       = 0,    // 空闲 — 正常平衡控制 / idle - normal balance control
    JUMP_PREPARE    = 1,    // 预备 — 平滑下蹲、前倾蓄势 / prepare - smooth crouch, forward lean
    JUMP_CHARGE     = 2,    // 蓄力 — 四腿同步压缩储能 / charge - all-leg compression
    JUMP_LAUNCH     = 3,    // 起跳 — 快速伸腿 + 电机助推 / launch - fast extension + motor boost
    JUMP_AIRBORNE   = 4,    // 腾空 — IMU 检测失重，维持姿态 / airborne - IMU detects freefall
    JUMP_LANDING    = 5,    // 着陆 — IMU 检测冲击，主动缓冲 / landing - IMU detects impact, active damping
    JUMP_RECOVER    = 6,    // 恢复 — 逐渐恢复 PID 增益和速度 / recover - gradual PID restoration
} jump_state_enum;

typedef enum
{
    JUMP_TRIGGER_OK          = 0,    // 触发成功
    JUMP_TRIGGER_BUSY        = 1,    // 跳跃状态机忙
    JUMP_TRIGGER_NOT_RUNNING = 2,    // 车体未运行 / STOP
    JUMP_TRIGGER_TILT        = 3,    // 倾角过大
} jump_trigger_result_enum;

//================================================================================
// 跳跃配置结构体 — 全部可自定义
// jump configuration struct — fully configurable
//================================================================================
typedef struct
{
    //---------- 阶段时长（PIT 节拍，1 tick ≈ 1ms）----------
    // phase timing (PIT ticks, 1 tick ≈ 1ms)
    uint16  prepare_ticks;          // 预备阶段时长
    uint16  charge_ticks;           // 蓄力阶段时长
    uint16  launch_ticks;           // 起跳阶段时长
    uint16  airborne_timeout;       // 腾空超时（防止无限悬空）
    uint16  landing_ticks;          // 着陆缓冲时长
    uint16  recover_ticks;          // 恢复阶段时长

    //---------- 舵机占空比偏移 ----------
    // steer duty offsets (PWM duty units)
    int16   charge_duty;            // 蓄力下蹲量（相对默认站姿，四腿同步）
    int16   launch_duty;            // 起跳伸腿量（相对默认站姿，四腿同步）
    int16   preland_duty;           // 预着陆偏移量
    int16   land_damping_duty;      // 着陆缓冲步进值

    //---------- 前进动量 ----------
    // forward momentum
    float   forward_tilt_target;    // 前倾目标角度（度），用于维持前进速度
    float   forward_motor_boost;    // 起跳时电机额外推力（占空比）
    float   speed_recovery_rate;    // 着陆后速度恢复速率（0~1，越大越快）

    //---------- PID 抑制 ----------
    // PID suppression
    float   airborne_pid_scale;     // 腾空阶段 PID 缩放系数（0~1）
    float   landing_pid_scale;      // 着陆阶段 PID 缩放系数（0~1）
    float   recover_pid_ramp_rate;  // 恢复阶段 PID 爬升速率

    //---------- IMU 检测阈值 ----------
    // IMU detection thresholds
    float   airborne_acc_threshold; // 加速度幅值低于此值判定为腾空（单位：g）
    float   landing_acc_threshold;  // 加速度幅值高于此值判定为着陆（单位：g）
    float   max_tilt_abort;         // 最大倾斜角度（度），超过则终止跳跃

    //---------- 视觉接口 ----------
    // vision interface — 后期与视觉模块对接
    void    (*vision_jump_trigger)(float obstacle_distance);
                                    // 视觉触发回调：传入障碍物距离(mm)，由视觉模块调用
    uint8   vision_jump_enable;     // 使能视觉触发跳跃
    float   vision_obstacle_dist;   // 当前障碍物距离（mm），由视觉模块更新
    float   vision_min_dist;        // 最小触发距离（mm），小于此距离才起跳
    float   vision_max_dist;        // 最大触发距离（mm），大于此距离不起跳

    //---------- 运行时状态（只读）----------
    // runtime state (read-only externally)
    uint8   state;                  // 当前状态（jump_state_enum）
    uint16  elapsed;                // 当前阶段已过节拍数
    uint8   jump_count;             // 累计跳跃次数
    float   peak_acc_magnitude;     // 本次跳跃峰值加速度（g）
    float   stored_p_angle;         // 进入跳跃前的角度 P（用于恢复）
    float   stored_p_speed;         // 进入跳跃前的速度 P（用于恢复）
    float   stored_speed_target;    // 进入跳跃前的目标速度
    uint8   last_trigger_result;    // 最近一次触发结果（jump_trigger_result_enum）
} jump_config_struct;

//================================================================================
// 全局跳跃配置实例
// global jump config instance
//================================================================================
extern jump_config_struct  jump_cfg;

//================================================================================
// 跳跃控制 API
// jump control API
//================================================================================

// 触发跳跃（由按键/菜单/视觉调用）
// trigger jump (called by key/menu/vision)
uint8 jump_trigger(void);

// 终止跳跃（紧急停止）
// abort jump (emergency stop)
void jump_abort(void);

// 视觉模块更新障碍物距离
// vision module updates obstacle distance
void jump_vision_update(float distance_mm);

// 将跳跃参数恢复到默认值
// reset jump parameters to defaults
void jump_config_default(void);

// 检查是否可以触发跳跃
// check if jump can be triggered
uint8 jump_can_trigger(void);

const char *jump_state_name(uint8 state);
const char *jump_trigger_result_name(uint8 result);
uint8 jump_is_active(void);

//---------- 原有 extern 保持不变 ----------
// existing externs preserved
extern float  target_speed;
extern volatile uint32 sys_times;
extern int16 left_motor_duty, right_motor_duty;
extern int    run_state;
extern int    STOP_FLAG;

//=================================================================
// 直行100m综合测试模块
//=================================================================
typedef enum
{
    STRAIGHT_IDLE    = 0,   // 空闲
    STRAIGHT_LOCKING = 1,   // 正在锁定GPS航向
    STRAIGHT_RUNNING = 2,   // 直行中
    STRAIGHT_DONE    = 3,   // 测试完成
} straight_test_state_enum;

typedef struct
{
    straight_test_state_enum state;     // 当前测试阶段
    float   target_heading;             // 锁定航向（度）
    float   start_lat;                  // 起点纬度
    float   start_lon;                  // 起点经度
    float   start_mileage;              // 起点编码器距离（cm）
    float   distance;                   // 已行驶距离（cm）
    float   lateral_deviation;          // 终点侧偏（m）
    float   yaw_drift;                  // 终点航向漂移（度）
    float   steer_correction;           // 当前舵机修正输出
    uint8   rating;                     // 评级 1-5
    bool    gps_available;              // 起点GPS航向是否有效
    bool    completed;                  // 是否完成（供Menu查询）
} straight_test_struct;

extern straight_test_struct straight_test;

void straight_test_start(void);         // 启动测试（Menu调用）
void straight_test_run(void);           // 测试FSM（pit_call_back每1ms调用）
void straight_test_stop(void);          // 紧急停止

void pit_call_back(void);
