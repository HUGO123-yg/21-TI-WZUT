#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// PIT_CH0 每 1 ms 运行一次，五个节拍产生 200 Hz 的 IMU 更新。滤波器
// 周期由这些值推导，因此调度器与估计器不会发生漂移。
#define IMU_SCHEDULER_TICK_PERIOD_S      (0.001f)
#define IMU_UPDATE_INTERVAL_TICKS        (5U)
#define IMU_UPDATE_PERIOD_S              \
    (IMU_SCHEDULER_TICK_PERIOD_S * (float)IMU_UPDATE_INTERVAL_TICKS)

// IMU660RB 量程由 zf_device_imu660rb.h 选择：+/-8 g 和 +/-2000 dps。
#define IMU_ACCEL_LSB_PER_G              (4098.0f)
#define IMU_GYRO_LSB_PER_DPS             (14.3f)

// 传感器轴到车体轴的映射。轴索引为 X=0, Y=1, Z=2。
// 这与旧平衡核心使用的 IMU 安装映射相同：
// X=原始 X，Y=-原始 Y，Z=-原始 Z。Imu.c 会拒绝 +/-1 以外的方向值。
#define IMU_BODY_X_SOURCE_AXIS           (0U)
#define IMU_BODY_Y_SOURCE_AXIS           (1U)
#define IMU_BODY_Z_SOURCE_AXIS           (2U)
#define IMU_BODY_X_DIRECTION             (1.0f)
#define IMU_BODY_Y_DIRECTION             (-1.0f)
#define IMU_BODY_Z_DIRECTION             (-1.0f)

// 以物理单位进行的额外软件校准。660RB 驱动已经
// 应用了其固定的原始计数值陀螺仪偏移（-7, +6, +2）。
#define IMU_ACCEL_BIAS_X_G               (0.0f)
#define IMU_ACCEL_BIAS_Y_G               (0.0f)
#define IMU_ACCEL_BIAS_Z_G               (0.0f)
#define IMU_GYRO_BIAS_X_DPS              (0.0f)
#define IMU_GYRO_BIAS_Y_DPS              (0.0f)
#define IMU_GYRO_BIAS_Z_DPS              (0.0f)

// 在车辆启动静止时估计剩余的陀螺仪偏移。
// 如果有效静止样本太少，初始化会明确失败；
// 控制栈绝不能从未校准的角速率信号开始平衡。
#define IMU_STARTUP_CALIBRATION_ENABLE          (1U)
#define IMU_STARTUP_CALIBRATION_SAMPLES         (200U)
#define IMU_STARTUP_CALIBRATION_MIN_VALID       (180U)
#define IMU_STARTUP_CALIBRATION_DELAY_MS        (5U)
#define IMU_STARTUP_CALIBRATION_MAX_GYRO_DPS    (5.0f)
#define IMU_STARTUP_CALIBRATION_MIN_G           (0.90f)
#define IMU_STARTUP_CALIBRATION_MAX_G           (1.10f)

// 两状态卡尔曼滤波参数：角度和陀螺仪偏置。
#define IMU_KALMAN_Q_ANGLE               (0.001f)
#define IMU_KALMAN_Q_BIAS                (0.003f)
#define IMU_KALMAN_R_MEASUREMENT         (0.030f)
#define IMU_KALMAN_INITIAL_VARIANCE      (1.0f)

// 在强动态加速度期间忽略加速度计角度修正。
#define IMU_ACCEL_CORRECTION_MIN_G       (0.80f)
#define IMU_ACCEL_CORRECTION_MAX_G       (1.20f)

// 控制调度器。姿态估计器和快速平衡控制器共享
// 一个 5 ms 采样周期，因此控制器不会混合不同周期的数据。
#define CONTROL_FAST_INTERVAL_TICKS       IMU_UPDATE_INTERVAL_TICKS
#define CONTROL_FAST_PERIOD_S             IMU_UPDATE_PERIOD_S
#define CONTROL_SPEED_INTERVAL_STEPS      (4U)    // 200 Hz 快速环路下为 50 Hz
#define CONTROL_LEG_INTERVAL_STEPS        (2U)    // 200 Hz 快速环路下为 100 Hz
#define CONTROL_WHEEL_REQUEST_INTERVAL_STEPS (4U) // 以 50 Hz 请求反馈

// PIT_CH0 只清中断标志、记录节拍并挂起最低优先级 PendSV。
// 完整控制计算仍按 1 ms 节拍执行，但不再拉长高优先级 PIT ISR。
// 将 DEFER 临时置 0 可在同一套 DWT 计数下测量改造前基线。
#define PIT_CONTROL_DEFER_ENABLE              (1U)
#define PIT_RUNTIME_PROFILING_ENABLE          (1U)
#define PIT_ISR_WCET_BUDGET_US                (10U)
#define PIT_CONTROL_TICK_WCET_BUDGET_US       (900U)
#define PIT_CONTROL_PENDING_TICK_LIMIT        (8U)
#define PIT_CONTROL_MAX_TICKS_PER_PENDSV      (2U)

// 平衡仅通过明确的菜单/控制请求来使能。启动时
// 初始化并监控传感器，两个轮子指令保持为零。
#define CONTROL_DEFAULT_STAND_ON_BOOT      (0U)
#define CONTROL_STAND_ARM_DELAY_MS         (300U)
#define CONTROL_STAND_ARM_MAX_PITCH_ERROR_RAD (0.17453293f) // 10 度
#define CONTROL_STAND_ARM_MAX_ROLL_RAD     (0.34906585f) // 20 度
#define CONTROL_FALL_PITCH_ERROR_RAD       (0.61086524f) // 35 度
#define CONTROL_FALL_ROLL_RAD              (0.78539816f) // 45 度
#define CONTROL_WHEEL_FEEDBACK_TIMEOUT_MS  (100U)

// 故障恢复始终是明确触发的。轮驱动器保持停止锁定
// 状态，直到 IMU 健康、底盘已被放正且腿回到
// 此安全姿态。软件紧急停止默认保持最后一个
// 有效的腿 PWM，以避免底盘不受控地坍塌；仅当
// 机构有独立支撑时才设置输出禁用开关。
#define CONTROL_FAULT_RECOVERY_MAX_PITCH_ERROR_RAD \
    CONTROL_STAND_ARM_MAX_PITCH_ERROR_RAD
#define CONTROL_FAULT_RECOVERY_MAX_ROLL_RAD  CONTROL_STAND_ARM_MAX_ROLL_RAD
#define CONTROL_ESTOP_DISABLE_LEG_OUTPUT     (0U)

// 两个顶层菜单将开发操作与竞赛
// 流程分开。按键从主循环中扫描；1 ms ISR 仅
// 置起服务标志。
#define MENU_ENABLE                        (1U)
#define MENU_DISPLAY_ENABLE                (1U)
#define MENU_KEY_SCAN_PERIOD_MS            (10U)
#define MENU_DISPLAY_REFRESH_MS            (100U)
#define CONTROL_COMPETITION_MODULES_ON_BOOT (0U)

// 轮驱动器协议及逻辑到硬件的符号。逻辑正方向表示
// 两个轮子均为车辆前进方向。这些值复现了经验证的旧
// 路径 CYT2_D_motor_ctrl(-left, +right) 和 speed=(raw_left-raw_right)/2。
// Wheel_driver.c 在任何方向不严格为 +1 或 -1 时拒绝输出。
#define WHEEL_DRIVER_UART                  (UART_2)
#define WHEEL_DRIVER_BAUDRATE              (460800U)
#define WHEEL_DRIVER_TX_PIN                (UART2_TX_P10_1)
#define WHEEL_DRIVER_RX_PIN                (UART2_RX_P10_0)
#define WHEEL_LEFT_COMMAND_DIRECTION       (-1)
#define WHEEL_RIGHT_COMMAND_DIRECTION      (1)
#define WHEEL_LEFT_SPEED_DIRECTION         (1)
#define WHEEL_RIGHT_SPEED_DIRECTION        (-1)
#define WHEEL_MAX_COMMAND                  (3000)
#define WHEEL_DIAMETER_M                    (0.062f)

// 平面导航持续运行，用于日志记录和路径记录。轮
// 差速航向保持禁用，直到在组装好的车辆上
// 测量出有效轮距。
#define NAVIGATION_ENABLE                       (1U)
#define NAVIGATION_USE_WHEEL_YAW_CORRECTION     (0U)
#define NAVIGATION_WHEEL_TRACK_WIDTH_M          (0.0f) // 待测量
#define NAVIGATION_WHEEL_YAW_RATE_WEIGHT        (0.05f)
#define NAVIGATION_STATIONARY_SPEED_M_S         (0.02f)
#define NAVIGATION_STATIONARY_GYRO_DPS          (1.0f)
#define NAVIGATION_GYRO_BIAS_LEARNING_RATE      (0.002f)

// 路径回放计算外环偏航角速率指令，但在
// 符号、轮距和增益验证通过之前不会将其
// 施加到轮控制器。
#define NAVIGATION_ROUTE_CONTROL_ENABLE         (0U)
#define NAVIGATION_HEADING_KP                    (2.0f)
#define NAVIGATION_MAX_YAW_RATE_RAD_S            (1.5f)

// 路线表只发布速度和动作意图，Control_system 仍是唯一执行仲裁者。
// route_distance_m 从每次路径回放起点清零，因此动作点不会受总里程漂移
// 影响。速度接管在航向回放完成实车标定前保持关闭；调度状态和动作点
// 仍会运行，便于先核对里程。
#define ROUTE_PLAN_ENABLE                        (1U)
#define ROUTE_PLAN_APPLY_SPEED_ENABLE            (0U)
#define ROUTE_PLAN_MAX_POINT_COUNT               (16U)
#define ROUTE_PLAN_MAX_SPEED_M_S                  (0.60f)
#define ROUTE_PLAN_SPEED_SLEW_M_S2                (0.40f)
#define ROUTE_PLAN_TRIGGER_EPSILON_M              (0.002f)
#define ROUTE_PLAN_DISTANCE_BACKTRACK_TOLERANCE_M (0.005f)
#define ROUTE_PLAN_ACTION_RETRY_LIMIT             (100U) // 200 Hz 下 500 ms

// 路线 1/2 先给出保守的速度分段。路线 3 展示完整的速度点和动作点
// 表结构；未拿到实测动作里程前，动作类型统一为 NONE。确认赛道里程后，
// 将对应 ROUTE_PLAN_ROUTE_3_ACTION_* 改成 ROUTE_ACTION_BRIDGE_LEFT/RIGHT、
// ROUTE_ACTION_BUMPY、ROUTE_ACTION_JUMP 或 ROUTE_ACTION_ROTATE_CW/CCW。
// 旋转动作的 parameter 表示圈数，其它动作填 0。
#define ROUTE_PLAN_ROUTE_1_POINTS \
    { 0.00f, 0.16f, ROUTE_ACTION_NONE, 0.0f }, \
    { 0.80f, 0.22f, ROUTE_ACTION_NONE, 0.0f }, \
    { 1.80f, 0.18f, ROUTE_ACTION_NONE, 0.0f }

#define ROUTE_PLAN_ROUTE_2_POINTS \
    { 0.00f, 0.16f, ROUTE_ACTION_NONE, 0.0f }, \
    { 1.20f, 0.24f, ROUTE_ACTION_NONE, 0.0f }, \
    { 2.80f, 0.18f, ROUTE_ACTION_NONE, 0.0f }

#define ROUTE_PLAN_ROUTE_3_ACTION_1              ROUTE_ACTION_NONE
#define ROUTE_PLAN_ROUTE_3_ACTION_1_PARAMETER    (0.0f)
#define ROUTE_PLAN_ROUTE_3_ACTION_2              ROUTE_ACTION_NONE
#define ROUTE_PLAN_ROUTE_3_ACTION_2_PARAMETER    (0.0f)
#define ROUTE_PLAN_ROUTE_3_ACTION_3              ROUTE_ACTION_NONE
#define ROUTE_PLAN_ROUTE_3_ACTION_3_PARAMETER    (0.0f)
#define ROUTE_PLAN_ROUTE_3_ACTION_4              ROUTE_ACTION_NONE
#define ROUTE_PLAN_ROUTE_3_ACTION_4_PARAMETER    (0.5f)

#define ROUTE_PLAN_ROUTE_3_POINTS \
    { 0.00f, 0.14f, ROUTE_ACTION_NONE, 0.0f }, \
    { 1.00f, 0.18f, ROUTE_PLAN_ROUTE_3_ACTION_1, \
      ROUTE_PLAN_ROUTE_3_ACTION_1_PARAMETER }, \
    { 1.50f, 0.22f, ROUTE_ACTION_NONE, 0.0f }, \
    { 2.40f, 0.18f, ROUTE_PLAN_ROUTE_3_ACTION_2, \
      ROUTE_PLAN_ROUTE_3_ACTION_2_PARAMETER }, \
    { 3.60f, 0.22f, ROUTE_ACTION_NONE, 0.0f }, \
    { 4.50f, 0.16f, ROUTE_PLAN_ROUTE_3_ACTION_3, \
      ROUTE_PLAN_ROUTE_3_ACTION_3_PARAMETER }, \
    { 5.20f, 0.14f, ROUTE_PLAN_ROUTE_3_ACTION_4, \
      ROUTE_PLAN_ROUTE_3_ACTION_4_PARAMETER }, \
    { 6.00f, 0.20f, ROUTE_ACTION_NONE, 0.0f }

// 无头 MT9V03X 地形识别。检测器内部保持一个道路
// 区域仅用于排除背景像素；它不发布转向线
// 也不绘制到 LCD。识别结果仅作观测用途，在
// 添加独立的仲裁器之前不会触发
// 桥、颠簸路面、台阶或跳跃控制。
#define TERRAIN_VISION_ENABLE                    (1U)
#define TERRAIN_VISION_DARK_SCENE_AVERAGE        (70U)
#define TERRAIN_VISION_THRESHOLD_FLOOR_DARK      (75U)
#define TERRAIN_VISION_THRESHOLD_FLOOR_NORMAL    (92U)
#define TERRAIN_VISION_BAND_DARK_PERCENT         (62U)
#define TERRAIN_VISION_BUMPY_MIN_WIDTH_PERCENT   (45U)
#define TERRAIN_VISION_BUMPY_MIN_STRIPS          (3U)
#define TERRAIN_VISION_SCORE_MAX                 (8U)
#define TERRAIN_VISION_BUMPY_CONFIRM_FRAMES      (2U)
#define TERRAIN_VISION_STEP_CONFIRM_FRAMES       (3U)
#define TERRAIN_VISION_BRIDGE_CONFIRM_FRAMES     (3U)
#define TERRAIN_VISION_OBSTACLE_CONFIRM_FRAMES   (3U)
#define TERRAIN_VISION_RELEASE_SCORE             (1U)
#define TERRAIN_VISION_EXPOSURE_UPDATE_FRAMES    (10U)
#define TERRAIN_VISION_EXPOSURE_MIN              (40U)
#define TERRAIN_VISION_EXPOSURE_MAX              (650U)
#define TERRAIN_VISION_EXPOSURE_STEP             (10U)

// 从经验证的旧 660RB 控制器转换而来的基础平衡级联。
// 旧的角度/角速率增益 700、50 和 1.1 作用于度和原始陀螺仪
// 计数（14.3 LSB/(度/秒)）；这些值在当前 200 Hz 速率下
// 在弧度、弧度/秒和驱动器指令单位中保持相同的小信号轮
// 响应。速度增益还包括旧的 0.003 度/输出耦合和轮
// RPM 到 m/s 的转换。积分增益在首次硬件调试前保持为零。
#define BALANCE_SPEED_KP                   (0.0806452f)
#define BALANCE_SPEED_KI                   (0.0f)
#define BALANCE_PITCH_KP                   (-48.9510f)
#define BALANCE_PITCH_KI                   (0.0f)
#define BALANCE_PITCH_KD                   (-0.0174825f)
#define BALANCE_RATE_KP                    (901.263f)
#define BALANCE_RATE_KI                    (0.0f)
#define BALANCE_YAW_RATE_KP                (0.0f)
#define BALANCE_YAW_RATE_KI                (0.0f)
#define BALANCE_MAX_PITCH_REFERENCE_RAD    (0.10471976f) // 6 度
#define BALANCE_MAX_PITCH_RATE_RAD_S       (3.0f)
#define BALANCE_MAX_RATE_INTEGRAL          (500.0f)
#define BALANCE_MAX_YAW_COMMAND            (800.0f)
#define BALANCE_PITCH_ZERO_RAD              (-0.10471976f) // 旧值 -6 度
#define BALANCE_WHEEL_OUTPUT_DIRECTION      (-1.0f)

// 零半径旋转是命令现有偏航角速率 PI 的外环角度
// 控制。正偏航预期为逆时针方向，因此顺时针
// 暂定为负。请在车辆抬起时验证此符号。
#define ROTATION_CONTROL_ENABLE             (1U)
#define ROTATION_CW_YAW_SIGN                (-1)
#define ROTATION_MAX_TURNS                  (5.0f)
#define ROTATION_MIN_YAW_RATE_RAD_S         (0.20f)
#define ROTATION_MAX_YAW_RATE_RAD_S         (1.20f)
#define ROTATION_ANGLE_KP_RAD_S_PER_RAD     (0.30f)
#define ROTATION_MAX_YAW_ACCEL_RAD_S2       (4.0f)
#define ROTATION_ANGLE_TOLERANCE_DEG        (3.0f)
#define ROTATION_SETTLE_YAW_RATE_RAD_S      (0.10f)
#define ROTATION_SETTLE_STEPS               (20U)   // 200 Hz 下 100 ms

// 超时随请求的圈数缩放。这些值允许大约两秒
// 的准备时间加上每整圈八秒，之后保持零速。
#define ROTATION_TIMEOUT_BASE_STEPS         (400U)
#define ROTATION_TIMEOUT_PER_TURN_STEPS     (1600U)

// 可选的四状态倒立摆反馈。在物理模型
// 辨识完成且 K 增益已计算之前保持禁用。增益
// 包含从模型力/力矩到轮驱动器指令的转换。
#define BALANCE_USE_STATE_FEEDBACK          (0U)
#define PENDULUM_K_POSITION                 (0.0f)
#define PENDULUM_K_SPEED                    (0.0f)
#define PENDULUM_K_PITCH                    (0.0f)
#define PENDULUM_K_PITCH_RATE               (0.0f)
#define PENDULUM_CART_EQUIVALENT_MASS_KG    (0.0f)   // 待测量
#define PENDULUM_BODY_MASS_KG               (0.0f)   // 待测量
#define PENDULUM_BODY_COM_HEIGHT_M          (0.0f)   // 尚未测量
#define PENDULUM_GRAVITY_M_S2               (9.80665f)

// 底盘在水平短连杆参考姿态下的离地间隙。这是
// 一个封装/地形值，不是倒立摆的质心高度。
#define BODY_REFERENCE_GROUND_CLEARANCE_M   (0.049f)
#define BODY_GROUND_CLEARANCE_TOLERANCE_M   (0.005f)

// 腿控制使用固定于每对电机枢轴的坐标系：
// +x 为车辆前进方向，+z 指向下方。四个相同的舵机
// 现已确认中位/行程；初始化保持水平姿态。
#define LEG_CONTROL_ENABLE                 (1U)
#define LEG_BASE_SPACING_M                 (0.038f)
#define LEG_LINK_A_PROXIMAL_M              (0.059f)
#define LEG_LINK_A_DISTAL_M                (0.090f)
#define LEG_LINK_B_PROXIMAL_M              (0.059f)
#define LEG_LINK_B_DISTAL_M                (0.090f)

// 在水平参考姿态下，关节 A 指向前方（0 弧度），关节 B
// 指向后方（pi 弧度）。当轮向下移动时这些分支
// 保持该装配模式。不要越过接近直链奇异点。
#define LEG_BRANCH_A                       (-1)
#define LEG_BRANCH_B                       (1)
#define LEG_REFERENCE_JOINT_A_RAD          (0.0f)
#define LEG_REFERENCE_JOINT_B_RAD          (3.14159265f)
#define LEG_JOINT_A_MIN_RAD                (-0.10471976f) // 允许旧的 -160 微调
#define LEG_JOINT_A_MAX_RAD                (1.57079633f)
#define LEG_JOINT_B_MIN_RAD                (1.57079633f)
#define LEG_JOINT_B_MAX_RAD                (3.24631241f)  // 允许旧的 -160 微调

// 公开的腿指令是相对于水平参考姿态的偏移量。正向
// 运动学计算其绝对 (x,z)，因此对调用者而言
// 法向姿态恰好为 (0,0)，即使逆运动学使用枢轴相对坐标。
#define LEG_DEFAULT_X_OFFSET_M             (0.0f)
#define LEG_DEFAULT_Z_OFFSET_M             (0.0f)
// 连杆机构在几何上可达约 145 mm，但已确认的舵机 1
// 工作极限仅为从水平位置 +2000 PWM（约 60 度）。将
// 正常目标保持在 140 mm 以下，使控制不依赖机械限位。
#define LEG_MECHANICAL_MAX_ABSOLUTE_Z_M    (0.145f)
#define LEG_SAFE_MAX_ABSOLUTE_Z_M          (0.140f)
#define LEG_MAX_ABSOLUTE_Z_M               LEG_SAFE_MAX_ABSOLUTE_Z_M
#define LEG_ROLL_KP_M_PER_RAD              (0.0f)
#define LEG_ROLL_KD_M_PER_RAD_S            (0.0f)
#define LEG_ROLL_DIRECTION                  (1.0f)
#define LEG_MAX_ROLL_OFFSET_M              (0.0f)
#define LEG_MAX_DIFFERENTIAL_Z_OFFSET_M    (0.030f)
#define LEG_MAX_TARGET_STEP_M              (0.001f)
#define LEG_FAULT_RECOVERY_X_OFFSET_M      LEG_DEFAULT_X_OFFSET_M
#define LEG_FAULT_RECOVERY_Z_OFFSET_M      LEG_DEFAULT_Z_OFFSET_M

// 第三届颠簸路面控制。官方凸起高度 20 mm、宽 25 mm，
// 间距约 100 mm。自动进入需要两次分离的加速度冲击，
// 这样单次落地或桥边缘不会抢占速度到腿的控制器。
// 路径/里程触发仍应优先使用。
#define BUMPY_CONTROL_ENABLE                (1U)
#define BUMPY_AUTO_DETECT_ENABLE            (1U)
#define BUMPY_IMPACT_DETECT_DELTA_G         (0.20f)
#define BUMPY_IMPACT_RELEASE_DELTA_G        (0.08f)
#define BUMPY_STABLE_DELTA_G                (0.05f)
#define BUMPY_IMPACT_REQUIRED_COUNT         (2U)
#define BUMPY_IMPACT_REFRACTORY_STEPS       (10U)  // 200 Hz 下 50 ms
#define BUMPY_DETECT_WINDOW_STEPS           (160U) // 800 ms

// 规则未定义颠簸段总长度。此距离为
// 临时路径参数，必须替换为实际赛道
// 测量值。超时保证漏读里程计更新时能释放控制。
#define BUMPY_CROSSING_DISTANCE_M           (1.00f)
#define BUMPY_CROSSING_TIMEOUT_STEPS        (1200U) // 6 秒
#define BUMPY_RECOVER_STABLE_STEPS          (20U)   // 100 ms
#define BUMPY_RECOVER_TIMEOUT_STEPS         (200U)  // 1 秒

// 通过期间，速度 PI 直接改变公共腿 x 偏移。
// Balance_ctrl 随后将俯仰参考固定在校准零点，仅使用
// 俯仰/俯仰角速率环路进行轮稳定。在腿 x 方向和
// 可用行程在支撑车上验证之前增益保持为零。
#define BUMPY_SPEED_KP_M_PER_M_S            (0.0f)
#define BUMPY_SPEED_KI_M_PER_M              (0.0f)
#define BUMPY_SPEED_TO_LEG_DIRECTION        (1.0f)
#define BUMPY_MAX_SPEED_LEG_X_OFFSET_M      (0.020f)
#define BUMPY_MAX_TOTAL_LEG_X_OFFSET_M      (0.025f)
#define BUMPY_MAX_LEG_X_STEP_M              (0.00025f)

// 两条腿各延伸 20 mm，为 20 mm
// 凸起创建间隙/柔顺性。速度和转向受限以
// 减少轮卸载和偏航冲击。
#define BUMPY_BODY_Z_OFFSET_M               (0.020f)
#define BUMPY_MAX_SPEED_M_S                 (0.30f)
#define BUMPY_YAW_RATE_SCALE                (0.50f)
#define BUMPY_MAX_YAW_RATE_RAD_S            (0.60f)

// 单侧桥控制。桥层让俯仰平衡环路
// 负责两个轮子，限制前进/偏航指令，并向
// Leg_ctrl 请求对称的左右 z 差值。当
// LEG_CONTROL_ENABLE 为零或 leg_ctrl_init() 未就绪时
// 自动保持禁用。
#define BRIDGE_CONTROL_ENABLE               (1U)
#define BRIDGE_AUTO_DETECT_ENABLE           (1U)

// 状态机阈值使用 200 Hz 快速控制采样。检测
// 时间特意短于基于距离的通过状态；不要
// 仅因为补偿本身已将横滚减小至零就退出通过状态。
#define BRIDGE_ROLL_DETECT_RAD              (0.08726646f) // 5 度
#define BRIDGE_ROLL_RECOVER_RAD             (0.01745329f) // 1 度
#define BRIDGE_DETECT_STEPS                 (8U)          // 40 ms
#define BRIDGE_ENTER_STEPS                  (20U)         // 100 ms
#define BRIDGE_EXIT_STEPS                   (20U)         // 100 ms
#define BRIDGE_RECOVER_STEPS                (20U)         // 100 ms 稳定
#define BRIDGE_RECOVER_TIMEOUT_STEPS        (400U)        // 2 秒保护
#define BRIDGE_CROSSING_TIMEOUT_STEPS       (800U)        // 4 秒保护
#define BRIDGE_CROSSING_DISTANCE_M          (0.20f)       // 需在赛道上验证

// 几何前馈将左右地面高度差的一半
// 估计为 0.5 * 支撑跨度 * tan(入口横滚)。0.10 m 跨度是
// 临时底盘值，必须替换为两条轮
// 接触线之间的实测距离。仅在
// 抬车横滚方向测试后更改方向符号。
#define BRIDGE_LATERAL_SUPPORT_SPAN_M       (0.10f)
#define BRIDGE_ROLL_TO_LEG_DIRECTION        (1.0f)
#define BRIDGE_FORCED_ENTRY_ROLL_RAD        BRIDGE_ROLL_DETECT_RAD

// 桥专用横滚 PD 叠加在锁存的几何前馈上。正
// 差分表示左腿 +z、右腿 -z。较大的 Kp 更强地
// 找平车身；较大的 Kd 增加阻尼。过大的值会导致
// 左右振荡，因此总差分及其变化率均受限。
#define BRIDGE_ROLL_KP_M_PER_RAD            (0.0172f)
#define BRIDGE_ROLL_KD_M_PER_RAD_S          (0.000012f)
#define BRIDGE_MAX_DIFFERENTIAL_OFFSET_M    (0.015f)
#define BRIDGE_MAX_DIFFERENTIAL_STEP_M      (0.00025f)

// 在差分行程激活期间使两腿远离水平参考，
// 限制桥速度，并减小可能导致轮子脱离
// 窄支撑面的转向。这些是临时的低速测试值。
#define BRIDGE_BODY_Z_OFFSET_M              (0.020f)
#define BRIDGE_MAX_SPEED_M_S                (0.26f)
#define BRIDGE_YAW_RATE_SCALE               (0.35f)
#define BRIDGE_MAX_YAW_RATE_RAD_S           (0.50f)

// 定时跳跃脚本。跳跃控制器直接命令每个目标，
// 而非等待接触或姿态事件。+z 延伸腿。
// 这些临时目标位于实测五连杆工作空间内，
// 但在地面跳跃前仍需在车辆支撑状态下检查。
#define JUMP_EXTEND_TIME_MS                 (100U)
#define JUMP_RETRACT_TIME_MS                (100U)
#define JUMP_BUFFER_TIME_MS                 (80U)
#define JUMP_LEG_X_OFFSET_M                 (0.0f)
#define JUMP_EXTEND_Z_OFFSET_M              (0.059f)
#define JUMP_RETRACT_Z_OFFSET_M             LEG_DEFAULT_Z_OFFSET_M
#define JUMP_BUFFER_Z_OFFSET_M              (0.020f)

// 四个舵机共用 steer_1 标定：水平 4500，每 90 度 3000 PWM
// 计数。正常延伸仅使用 2000 计数；从水平位置
// 实测的对侧行程为 1500 计数。镜像安装
// 以相反的 PWM 方向应用这些相对行程。
#define LEG_SERVO_COUNT                    (4U)
#define LEG_SERVO_FREQUENCY_HZ             (300U)
#define LEG_SERVO_CALIBRATION_COMPLETE     (1U)
#define LEG_SERVO_REFERENCE_PWM            (4500)
#define LEG_SERVO_90_DEG_TRAVEL_PWM        (3000)
#define LEG_SERVO_PWM_PER_RAD              (1909.85932f)
#define LEG_SERVO_1_MECHANICAL_MIN_PWM     (3000)
#define LEG_SERVO_1_MECHANICAL_MAX_PWM     (7500)
#define LEG_SERVO_SAFE_EXTEND_TRAVEL_PWM   (2000)
#define LEG_SERVO_SAFE_RETRACT_TRAVEL_PWM  (1500)
#define LEG_SERVO_1_PWM                    (TCPWM_CH10_P05_1)
#define LEG_SERVO_2_PWM                    (TCPWM_CH12_P05_3)
#define LEG_SERVO_3_PWM                    (TCPWM_CH09_P05_0)
#define LEG_SERVO_4_PWM                    (TCPWM_CH11_P05_2)
#define LEG_SERVO_1_CENTER                 LEG_SERVO_REFERENCE_PWM
#define LEG_SERVO_2_CENTER                 LEG_SERVO_REFERENCE_PWM
#define LEG_SERVO_3_CENTER                 LEG_SERVO_REFERENCE_PWM
#define LEG_SERVO_4_CENTER                 LEG_SERVO_REFERENCE_PWM
#define LEG_SERVO_1_DIRECTION              (1)
#define LEG_SERVO_2_DIRECTION              (-1)
#define LEG_SERVO_3_DIRECTION              (1)
#define LEG_SERVO_4_DIRECTION              (-1)
// 关节 B 在延伸过程中从 pi 向 pi/2 运动，因此上述
// 角度到 PWM 方向对 1..4 产生有效延伸符号 +、-、-、+。
#define LEG_SERVO_1_PWM_PER_RAD            LEG_SERVO_PWM_PER_RAD
#define LEG_SERVO_2_PWM_PER_RAD            LEG_SERVO_PWM_PER_RAD
#define LEG_SERVO_3_PWM_PER_RAD            LEG_SERVO_PWM_PER_RAD
#define LEG_SERVO_4_PWM_PER_RAD            LEG_SERVO_PWM_PER_RAD
#define LEG_SERVO_1_ZERO_RAD               (0.0f)
#define LEG_SERVO_2_ZERO_RAD               (0.0f)
#define LEG_SERVO_3_ZERO_RAD               (3.14159265f)
#define LEG_SERVO_4_ZERO_RAD               (3.14159265f)
#define LEG_SERVO_1_PWM_MIN                (LEG_SERVO_REFERENCE_PWM \
                                            - LEG_SERVO_SAFE_RETRACT_TRAVEL_PWM)
#define LEG_SERVO_1_PWM_MAX                (LEG_SERVO_REFERENCE_PWM \
                                            + LEG_SERVO_SAFE_EXTEND_TRAVEL_PWM)
#define LEG_SERVO_2_PWM_MIN                (LEG_SERVO_REFERENCE_PWM \
                                            - LEG_SERVO_SAFE_EXTEND_TRAVEL_PWM)
#define LEG_SERVO_2_PWM_MAX                (LEG_SERVO_REFERENCE_PWM \
                                            + LEG_SERVO_SAFE_RETRACT_TRAVEL_PWM)
#define LEG_SERVO_3_PWM_MIN                LEG_SERVO_2_PWM_MIN
#define LEG_SERVO_3_PWM_MAX                LEG_SERVO_2_PWM_MAX
#define LEG_SERVO_4_PWM_MIN                LEG_SERVO_1_PWM_MIN
#define LEG_SERVO_4_PWM_MAX                LEG_SERVO_1_PWM_MAX
#define LEG_LEFT_JOINT_A_SERVO             (0U) // 旧 steer_1，前/上
#define LEG_LEFT_JOINT_B_SERVO             (2U) // 旧 steer_3，后/下
#define LEG_RIGHT_JOINT_A_SERVO            (1U) // 旧 steer_2，前/上
#define LEG_RIGHT_JOINT_B_SERVO            (3U) // 旧 steer_4，后/下

// 导航路径记录器使用的 Work-Flash 布局。第 1 和第 2 页
// 包含交替的元数据副本。保留两份副本可防止
// 元数据更新期间断电导致唯一的路径目录损坏。
#define NAV_FLASH_ROUTE_COUNT             (3U)
#define NAV_FLASH_META_PAGE_A             (1U)
#define NAV_FLASH_META_PAGE_B             (2U)

// 每条路径拥有固定的递减页范围。固定分区
// 使满路径不可能覆盖另一条路径或元数据。
#define NAV_FLASH_ROUTE1_START_PAGE       (95U)
#define NAV_FLASH_ROUTE1_END_PAGE         (65U)
#define NAV_FLASH_ROUTE2_START_PAGE       (64U)
#define NAV_FLASH_ROUTE2_END_PAGE         (34U)
#define NAV_FLASH_ROUTE3_START_PAGE       (33U)
#define NAV_FLASH_ROUTE3_END_PAGE         (3U)

// 每个 512 字页中的前 500 字存储偏航样本。其余
// 字包含页标识、样本计数、生成号、CRC 和提交数据。
#define NAV_FLASH_SAMPLES_PER_PAGE        (500U)
#define NAV_FLASH_REPLAY_MAX_SAMPLES      (15500U)

// 每前进 5 cm 记录一个新的相对偏航样本。所有
// 导航和路径记录器的距离值均使用米。
#define NAV_FLASH_SAMPLE_DISTANCE_M       (0.05f)

// 偏航以有符号百分度存储而非浮点数。这避免了
// 在持久数据中存储编译器相关的浮点表示。
#define NAV_FLASH_YAW_SCALE               (100.0f)

// 持久格式标识符。每当 Flash 上的字布局
// 发生不兼容变化时更改 FORMAT_VERSION；旧数据
// 将被安全拒绝。
#define NAV_FLASH_METADATA_MAGIC          (0x4E41564DUL)
#define NAV_FLASH_DATA_MAGIC              (0x4E415644UL)
#define NAV_FLASH_FORMAT_VERSION          (1U)
#define NAV_FLASH_COMMIT_MARKER           (0x434F4D54UL)

// 将每个已写入页回读并比对后再发布元数据。
// 在车上保持启用；禁用仅减少开发延迟。
#define NAV_FLASH_VERIFY_AFTER_WRITE      (1U)
#define NAV_FLASH_WRITE_RETRY_COUNT       (2U)

#endif
