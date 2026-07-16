#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

// PIT_CH0 每 1 ms 运行一次，五个节拍产生 200 Hz 的 IMU 更新。滤波器
// 周期由这些值推导，因此调度器与估计器不会发生漂移。
#define IMU_SCHEDULER_TICK_PERIOD_S      (0.001f)
#define IMU_UPDATE_INTERVAL_TICKS        (5U)
#define IMU_UPDATE_PERIOD_S              \
    (IMU_SCHEDULER_TICK_PERIOD_S * (float)IMU_UPDATE_INTERVAL_TICKS)

// 临时 IMU 型号兼容开关。更换模块时只修改 IMU_SENSOR_TYPE：
// IMU_SENSOR_TYPE_660RA 使用 660RA，IMU_SENSOR_TYPE_660RB 使用 660RB。
#define IMU_SENSOR_TYPE_660RA             (0U)
#define IMU_SENSOR_TYPE_660RB             (1U)
#ifndef IMU_SENSOR_TYPE
#define IMU_SENSOR_TYPE                   IMU_SENSOR_TYPE_660RA
#endif

#if (IMU_SENSOR_TYPE == IMU_SENSOR_TYPE_660RA)
// 660RA 底层驱动完成 BMI270 配置文件加载；Imu.c 随后将加速度计
// 从驱动默认的 50 Hz 改为 200 Hz，并复核两路 ODR 和量程。
#define IMU660RA_ACC_CONF_200HZ            (0xA9U)
#define IMU660RA_GYR_CONF_200HZ            (0xA9U)
#define IMU660RA_ACC_RANGE_8G              (0x02U)
#define IMU660RA_GYR_RANGE_2000DPS         (0x00U)
#define IMU_ACCEL_LSB_PER_G                (4096.0f)
#define IMU_GYRO_LSB_PER_DPS               (16.384f)
#elif (IMU_SENSOR_TYPE == IMU_SENSOR_TYPE_660RB)
// 660RB 模块必须是规则允许的 ST LSM6DSR：项目层复核
// WHO_AM_I=0x6B。原驱动默认的加速度计仅为 52 Hz，Imu.c 会将
// CTRL1_XL 改为 208 Hz / +/-8 g，与 200 Hz 姿态周期匹配；
// 陀螺仪保持 208 Hz / +/-2000 dps。
#define IMU_LSM6DSR_EXPECTED_WHO_AM_I     (0x6BU)
#define IMU_LSM6DSR_CTRL1_XL_208HZ_8G     (0x5CU)
#define IMU_LSM6DSR_CTRL2_G_208HZ_2000DPS (0x5CU)
#define IMU_ACCEL_LSB_PER_G                (4098.0f)
#define IMU_GYRO_LSB_PER_DPS               (14.3f)
#else
#error "IMU_SENSOR_TYPE must select IMU_SENSOR_TYPE_660RA or IMU_SENSOR_TYPE_660RB"
#endif

// 传感器轴到车体轴的映射。轴索引为 X=0, Y=1, Z=2。
// 这与旧平衡核心使用的 IMU 安装映射相同：
// X=原始 X，Y=-原始 Y，Z=-原始 Z。Imu.c 会拒绝 +/-1 以外的方向值。
#define IMU_BODY_X_SOURCE_AXIS           (0U)
#define IMU_BODY_Y_SOURCE_AXIS           (1U)
#define IMU_BODY_Z_SOURCE_AXIS           (2U)
#define IMU_BODY_X_DIRECTION             (1.0f)
#define IMU_BODY_Y_DIRECTION             (-1.0f)
#define IMU_BODY_Z_DIRECTION             (-1.0f)

// 以物理单位进行的额外软件校准。底层 660RA/660RB 驱动均已
// 应用各自的固定原始计数值陀螺仪偏移。
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
// 限制姿态滤波器在线学习的 X/Y 残余陀螺零偏，避免动态工况下误修正持续累积。
#define IMU_KALMAN_MAX_BIAS_DPS           (5.0f)

// 在强动态加速度期间忽略加速度计角度修正。
#define IMU_ACCEL_CORRECTION_MIN_G       (0.80f)
#define IMU_ACCEL_CORRECTION_MAX_G       (1.20f)
// 加速度角与陀螺预测角差值超过此门限时，按线性加速度干扰处理。
// 减小可更强地抑制急加减速误倾角，过小会降低大扰动后的恢复能力。
#define IMU_ACCEL_MAX_INNOVATION_DEG     (12.0f)

// 在线偏置只在轮速、加速度模长和三轴角速度连续满足静止条件后学习。
// 增大确认样本数可降低短暂停顿或反馈滞后造成的误学习，但温漂跟踪更慢。
#define IMU_RUNTIME_BIAS_STATIONARY_SAMPLES     (20U) // 200 Hz 下为 100 ms
// 比较的是减去当前在线偏置后的三轴残余角速度。
#define IMU_RUNTIME_BIAS_MAX_GYRO_DPS           (1.0f)
#define IMU_RUNTIME_BIAS_MAX_WHEEL_SPEED_M_S    (0.02f)

// Z 轴缺少绝对航向观测，确认静止后以零角速度为参考缓慢学习温漂。
// 学习结果同时供姿态航向积分、旋转控制和导航使用。
#define IMU_RUNTIME_Z_BIAS_ENABLE              (1U)
// 增大可更快跟踪温漂，但也会放大误判静止时对真实慢转动的吸收。
#define IMU_RUNTIME_Z_BIAS_LEARNING_RATE       (0.002f)
// 增大允许补偿更大的温漂；过大会隐藏安装松动或传感器异常。
#define IMU_RUNTIME_Z_BIAS_MAX_DPS             (5.0f)

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
// ROUTE_ACTION_BUMPY 或 ROUTE_ACTION_STAIR_DESCENT_JUMP。单边桥区禁止配置
// 跳跃动作；台阶上行尚无经过实车验证的自动动作，不能用下台阶跳跃代替。
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
#define ROUTE_PLAN_ROUTE_3_ACTION_4_PARAMETER    (0.0f)

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

// 共享 MT9V03X 前端只负责采集、缩放、曝光和稳定帧发布。
// 地形、锥桶和雷区检测器消费同一帧，不能各自初始化或抢占相机。
#define VISION_PIPELINE_ENABLE                   (1U)
#define VISION_FRAME_ENABLE                      (1U)
#define VISION_FRAME_EXPOSURE_DARK_AVERAGE       (75U)
#define VISION_FRAME_EXPOSURE_BRIGHT_AVERAGE     (165U)
#define VISION_FRAME_EXPOSURE_UPDATE_FRAMES      (10U)
#define VISION_FRAME_EXPOSURE_MIN                (40U)
#define VISION_FRAME_EXPOSURE_MAX                (650U)
#define VISION_FRAME_EXPOSURE_STEP               (10U)

// 无头 MT9V03X 地形识别。检测器内部保持一个道路区域仅用于排除
// 背景像素；它不绘制到 LCD，也不调用桥、台阶、跳跃或电机控制。
#define TERRAIN_VISION_ENABLE                    (1U)
// 只有真实帧统计和实车符号测试通过后才改为 1；Mission_perception 强制门控。
#define TERRAIN_VISION_CALIBRATED                (0U)
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
// 图像空间道路几何只做归一化观测，不在相机高度、俯仰角和内参标定前
// 强行换算成米或弧度。近/远两个窗口同时有效后才向融合层发布。
#define TERRAIN_VISION_PATH_FAR_ROW_FIRST         (24U)
#define TERRAIN_VISION_PATH_FAR_ROW_LAST          (35U)
#define TERRAIN_VISION_PATH_NEAR_ROW_FIRST        (42U)
#define TERRAIN_VISION_PATH_NEAR_ROW_LAST         (55U)
#define TERRAIN_VISION_PATH_MIN_VALID_ROWS        (5U)

// 科目一锥桶观测只发布候选框和相邻锥桶间隙。灰度相机下的明暗极性
// 由现场背景决定，因此同时检测亮/暗目标；完成实拍标定前 calibrated=0。
#define CONE_VISION_ENABLE                        (1U)
#define CONE_VISION_CALIBRATED                    (0U)
#define CONE_VISION_ROI_TOP                       (8U)
#define CONE_VISION_CONTRAST_MIN                  (30U)
#define CONE_VISION_MIN_HEIGHT_PX                 (7U)
#define CONE_VISION_MIN_WIDTH_PX                  (3U)
#define CONE_VISION_MAX_WIDTH_PX                  (30U)
#define CONE_VISION_MIN_HEIGHT_WIDTH_PERCENT      (90U)
#define CONE_VISION_MIN_FILL_PERCENT              (20U)
#define CONE_VISION_GAP_MIN_WIDTH_PX              (6U)

// 科目二只识别白色边框的局部几何。现场其他白线会产生 boundary_visible，
// 因此任务层必须先用惯导位置进入雷区窗口，再消费此观测。实拍前不标定。
#define MINEFIELD_VISION_ENABLE                    (1U)
#define MINEFIELD_VISION_CALIBRATED                (0U)
#define MINEFIELD_VISION_ROI_TOP                   (6U)
#define MINEFIELD_VISION_WHITE_THRESHOLD_FLOOR    (170U)
#define MINEFIELD_VISION_WHITE_DELTA               (40U)
#define MINEFIELD_VISION_MIN_WIDTH_PX              (24U)
#define MINEFIELD_VISION_MIN_HEIGHT_PX             (12U)
#define MINEFIELD_VISION_SIDE_SPAN_PERCENT         (45U)
#define MINEFIELD_VISION_MIN_SIDE_ROWS_PERCENT     (50U)
#define MINEFIELD_VISION_HORIZONTAL_ROW_PERCENT    (45U)
#define MINEFIELD_VISION_MIN_HORIZONTAL_BANDS      (2U)
#define MINEFIELD_VISION_BOUNDARY_ROW_PERCENT      (25U)
#define MINEFIELD_VISION_BOUNDARY_WARNING_ROW      (44U)

// 视觉-惯导融合层当前只发布建议，不直接接入 Control_system。完成实车
// 相机标定、误差符号检查和失帧测试后，再由任务决策层选择是否消费。
#define PERCEPTION_FUSION_ENABLE                  (1U)
#define PERCEPTION_FUSION_VISION_TIMEOUT_MS       (250U)
#define PERCEPTION_FUSION_PATH_MIN_QUALITY        (45U)
// 新帧占比。增大可更快跟随弯折道路，但图像抖动会更明显。
#define PERCEPTION_FUSION_PATH_FILTER_ALPHA       (0.25f)
// 单位是 rad/s / 归一化图像误差；当前值是保守联调起点，不是实车定值。
#define PERCEPTION_FUSION_PATH_CENTER_YAW_KP      (0.60f)
#define PERCEPTION_FUSION_PATH_HEADING_YAW_KP     (0.80f)
#define PERCEPTION_FUSION_MAX_VISION_YAW_RATE_RAD_S (0.80f)
#define PERCEPTION_FUSION_MAX_SUGGESTED_YAW_RATE_RAD_S (1.50f)

// IMU 只给视觉地形标签增加“物理现象支持”标志和置信度，不反向修改
// 视觉分类，也不替代 Bridge_ctrl/Bumpy_ctrl 内部的安全检测。
#define PERCEPTION_FUSION_IMU_CONFIDENCE_BONUS    (15U)
#define PERCEPTION_FUSION_BUMPY_ACCEL_DELTA_G     (0.12f)
#define PERCEPTION_FUSION_BRIDGE_ROLL_DEG         (2.5f)
#define PERCEPTION_FUSION_STEP_PITCH_DEG          (3.0f)

// 科目上下文层只发布建议和任务进度。导航里程窗口外、视觉过期或未标定时，
// 锥桶/地形偏航建议均不可用；雷区旋转圈数仍由惯导连续累计。
#define MISSION_PERCEPTION_ENABLE                 (1U)
#define MISSION_PERCEPTION_VISUAL_TIMEOUT_MS      (250U)
#define MISSION_PERCEPTION_CONE_MIN_QUALITY       (45U)
#define MISSION_PERCEPTION_CONE_FILTER_ALPHA      (0.25f)
#define MISSION_PERCEPTION_CONE_YAW_KP            (0.80f)
#define MISSION_PERCEPTION_MAX_CONE_YAW_RATE_RAD_S (0.80f)
#define MISSION_PERCEPTION_MAX_SUGGESTED_YAW_RATE_RAD_S (1.50f)
// 官方要求雷区内至少旋转两周，任务上下文不接受更小目标。
#define MISSION_PERCEPTION_MINE_MIN_TURNS         (2.0f)
#define MISSION_PERCEPTION_MINE_MAX_TURNS         (10.0f)
// 200 Hz 下相邻航向跳变超过此值按姿态重置/数据不连续处理，不计入圈数。
#define MISSION_PERCEPTION_MAX_YAW_STEP_RAD       (0.35f)

// 路线 1/2/3 分别对应绕桩、雷区和地形科目。当前窗口覆盖整条已记录路线，
// 只用于采集观测；启用下方任何控制接管前，必须改成现场实测的局部窗口。
#define MISSION_PERCEPTION_ROUTE_1_WINDOW_START_M (0.0f)
#define MISSION_PERCEPTION_ROUTE_1_WINDOW_END_M   (1000.0f)
#define MISSION_PERCEPTION_ROUTE_2_WINDOW_START_M (0.0f)
#define MISSION_PERCEPTION_ROUTE_2_WINDOW_END_M   (1000.0f)
#define MISSION_PERCEPTION_ROUTE_3_WINDOW_START_M (0.0f)
#define MISSION_PERCEPTION_ROUTE_3_WINDOW_END_M   (1000.0f)

// 两个开关均须在真实图像标定、路线窗口、转向符号和失帧回退通过后开启。
// 第一个允许任务层建议覆盖导航偏航率；第二个允许路线 2 的雷区专用旋转。
#define MISSION_PERCEPTION_APPLY_GUIDANCE_ENABLE  (0U)
#define MISSION_PERCEPTION_MINE_ACTION_ENABLE     (0U)
// 雷区专用旋转只在视觉中心误差进入该范围后允许开始。
#define MISSION_PERCEPTION_MINE_CENTER_TOLERANCE_NORM (0.12f)
// 雷区动作运行时，边框观测丢失或接近边界会中止旋转，避免车轮越出白框。
#define MISSION_PERCEPTION_MINE_GUARD_REQUIRED    (1U)

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
// 航向增益先保持为零。抬轮确认正偏航和左右轮差分符号后先增加 KP，
// 再用很小的 KI 消除稳态误差；调试时观察 balance_state 中的
// yaw_command_limit、yaw_command_limited 和 yaw_integrator，避免航向环
// 在平衡输出已经占满轮端指令时积分堆积。两项同时为零时，Control_system
// 将 rotation_actuation_ready 置零并拒绝旋转/雷区动作，避免空转到超时。
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
