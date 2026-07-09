#ifndef BODY_CTRL_CONFIG_H_
#define BODY_CTRL_CONFIG_H_

//****************************************************************************
// 文件名称    config.h
// 功能描述    车体控制与跳跃动作模块可调参数集中配置文件
//             将物理/机械、安全保护、PID 启动、跳跃、转向、主循环时序等
//             硬编码常量抽离为宏，便于调试、整定和复用。
// 使用方式    在 Body_ctrl.c / Jump.c 中 #include "config.h"，所有参数以宏形式使用。
// 注意事项    1. 修改后需重新编译生效；
//             2. 单位与注释中保持一致；
//             3. 部分参数相互耦合（如 *_MULT 与 *_LIMIT 共同决定舵机运动量）。
//****************************************************************************

//============================ 物理/机械参数 ============================
#define USE_TEST3_BALANCE_CORE                 // 使用 TEST3 2 已验证的基础直立控制核心，先保证能稳定站住
// 下面扩展按当前调试目标打开/关闭；单边桥调试需要开启 USE_BRIDGE_CONTROL。
// #define USE_ROLL_BALANCE_CONTROL            // 横滚/腿高辅助
#define USE_BRIDGE_CONTROL                    // 单边桥状态机、腿高补偿与左右轮动力补偿
#define USE_TERRAIN_CONTROL                   // 路况仲裁层：自动触发单边桥/颠簸路段，草地按普通路处理
// #define USE_ROTATION_CONTROL                // 原地旋转差速叠加
#define WHEEL_CIRCUMFERENCE          (6.4f)   // 轮子直径（cm），用于里程计算
#define BALANCE_MECHANICAL_ZERO_DEG  (-6.0f)  // 前后平衡机械零点（°），对齐 TEST3 2 可直立参数

//============================ 电机输出限幅 ============================
#define BALANCE_DUTY_MAX             3000     // 前后平衡电机输出占空比上限
#define TURN_DUTY_MAX                3000     // 转向差速电机输出占空比上限
#define BALANCE_MOTOR_OUTPUT_SIGN    (-1)     // 非 TEST3 扩展模式使用；TEST3 核心固定按 -(angle speed out) 输出

//============================ 安全保护参数 ============================
#define BODY_TILT_LIMIT_DEG          60.0f    // 车体倾角保护阈值（°），横滚/俯仰绝对值超过则停机
#define STARTUP_RAMP_CYCLES          500      // 启动时 PID 参数渐变周期数（约 0.5s，假设 PIT 1ms）
#define CONTROL_STARTUP_CYCLES       500      // 开始闭环控制前等待周期数，等待姿态初步收敛

//============================ 单边桥控制参数 ============================
// 调参效果：
//   BRIDGE_ROLL_THRESHOLD：调大 -> 更不容易误触发；调小 -> 更早进入桥模式，但普通侧倾也可能触发。
//   BRIDGE_DETECT_CYCLES：调大 -> 检测更稳但进入更慢；调小 -> 响应更快但更怕瞬时抖动。
//   BRIDGE_ENTER_DELAY_CYCLES：调大 -> 进入桥后保持预补偿更久；调小 -> 更快进入 CROSSING。
//   BRIDGE_CROSSING_TIMEOUT_CYCLES：调大 -> 桥模式最长保持更久；调小 -> 里程异常时更快退出。
//   BRIDGE_EXIT_DELAY_CYCLES：调大 -> 下桥后补偿释放更慢；调小 -> 更快进入恢复阶段。
//   BRIDGE_RECOVER_CYCLES：调大 -> 回到 IDLE 更谨慎；调小 -> 更快允许下一次识别。
//   BRIDGE_LENGTH：调大 -> CROSSING 保持距离更长；调小 -> 更早退出桥模式。
//   BRIDGE_CAR_WIDTH：调大 -> 几何腿长和动力补偿都更强；调小 -> 补偿更弱。
//   BRIDGE_RECOVER_ROLL_THRESH：调大 -> 更容易判定恢复完成；调小 -> 回正要求更严格。
#define BRIDGE_ROLL_THRESHOLD            (5.0f)      // 进入单边桥 roll 角绝对值阈值（°）
#define BRIDGE_DETECT_CYCLES             (10)        // 连续超过阈值周期数，用于防抖确认
#define BRIDGE_ENTER_CANCEL_ROLL_THRESH  (2.0f)      // ENTER 阶段 roll 回落到该阈值内则取消本次触发
#define BRIDGE_ENTER_DELAY_CYCLES        (100)       // ENTER 阶段固定延时（周期），约 100ms @1kHz
#define BRIDGE_CROSSING_TIMEOUT_CYCLES   (3000)      // CROSSING 阶段最大保持周期，防止里程异常时桥模式无法退出
#define BRIDGE_EXIT_DELAY_CYCLES         (50)        // EXIT 阶段固定延时（周期），约 50ms @1kHz
#define BRIDGE_RECOVER_CYCLES            (20)        // RECOVER 阶段 roll 稳定持续周期数
#define BRIDGE_LENGTH                    (20.0f)     // 单边桥长度（cm，规则上限）
#define BRIDGE_CAR_WIDTH                 (10.0f)     // 车体宽度（cm），用于腿高补偿 L = CAR_WIDTH / 2
#define BRIDGE_RECOVER_ROLL_THRESH       (1.0f)      // 恢复阶段判定姿态稳定的 roll 角阈值（°）
#define BRIDGE_FORCE_MIN_ROLL_DEG        (BRIDGE_ROLL_THRESHOLD) // 里程强制进桥时用于预补偿的最小等效 roll
#define BRIDGE_FORCE_SIDE_SIGN           (1)         // 里程强制进桥且 roll 很小时的补偿方向：1 / -1，方向错就改符号

// 单边桥 roll PD：只修正桥上横滚，不替代前后倒立摆主平衡。
//   BRIDGE_ROLL_PD_KP：调大 -> 横滚角纠偏更强；过大容易左右摇摆。
//   BRIDGE_ROLL_PD_KD：调大 -> 阻尼更强、抑制快速侧倾；方向错会放大抖动。
//   BRIDGE_ROLL_PD_MAX_CM：调大 -> 允许更大 PD 修正；调小 -> 更温和但可能扶不住。
//   BRIDGE_ROLL_PD_RATE_SIGN：若桥上越补越抖或阻尼像正反馈，在 1.0f 与 -1.0f 之间切换。
//   BRIDGE_ROLL_RATE_GYRO_DATA：选择横滚角速度轴，默认 GYRO_DATA_X；轴选错会让 D 项无效或反向。
#define BRIDGE_ROLL_PD_KP                (0.03f)     // 单边桥横滚 P 补偿，单位约 cm/deg
#define BRIDGE_ROLL_PD_KD                (0.002f)    // 单边桥横滚 D 补偿，单位约 cm/(deg/s)
#define BRIDGE_ROLL_PD_MAX_CM            (1.5f)      // 横滚 PD 补偿限幅（cm）
#define BRIDGE_ROLL_PD_RATE_SIGN         (1.0f)      // roll 角速度方向修正
#define BRIDGE_ROLL_RATE_GYRO_DATA       (GYRO_DATA_X)

// 单边桥动力补偿：
//   BRIDGE_SPEED_EXTRA_DUTY_GAIN：调大 -> 高侧轮动力补偿更强；过大会抢主平衡输出。
//   BRIDGE_SPEED_EXTRA_DUTY_MAX：调大 -> 允许更大动力补偿；调小 -> 更安全但可能补偿不足。
//   BRIDGE_MIN_COMP_SPEED_RPM：调大 -> 低速/零速时仍有更强补偿；调小 -> 低速更柔和。
#define BRIDGE_SPEED_EXTRA_DUTY_GAIN     (30.0f)     // 几何速度补偿(RPM)到电机 duty 的换算系数
#define BRIDGE_SPEED_EXTRA_DUTY_MIN      80          // 桥模式下非零动力补偿的最小 duty，避免几何量过小被取整为 0
#define BRIDGE_SPEED_EXTRA_DUTY_MAX      500         // 单边桥动力补偿占空比限幅
#define BRIDGE_MIN_COMP_SPEED_RPM        80.0f       // 动力补偿最小参考速度，避免低速/零速时补偿失效
#define BRIDGE_TEST_SPEED_RPM            80.0f       // 菜单单边桥测试目标速度（RPM）

// 单边桥腿高到舵机 duty 的临时线性映射，后续可替换为标定表/五连杆逆解。
#define BRIDGE_LEG_TO_DUTY_RATIO         (100.0f)

//============================ 方格/颠簸路障控制参数 ============================
// 策略：进路障前/路障内清 PID 运行态，冻结速度环和循迹环积分，用小前馈推力低速通过；
//      如果编码器速度很低但平衡电机输出很大，判定为卡滞，短退一下再前冲。
#define USE_OBSTACLE_CONTROL                         // 方格/颠簸路障状态机与 PID 防积分饱和
#define OBSTACLE_AUTO_ARM_IN_SUBJECT3     (0)        // 进入菜单 fun_a33 科目三复现时自动武装；0=由路况仲裁层/KEY4 触发
#define OBSTACLE_START_DISTANCE_CM        (0.0f)     // 从 obstacle_arm() 里程起算，延迟多少 cm 进入路障模式
#define OBSTACLE_LENGTH_CM                (120.0f)   // 路障控制保持距离，规则颠簸路段约 >=100cm，默认留余量
#define OBSTACLE_ENTER_CYCLES             (80)       // 进入阶段清 PID 后预推时间，约 80ms @1kHz
#define OBSTACLE_CROSSING_TIMEOUT_CYCLES  (3500)     // CROSSING 最长保持时间，防止编码器/里程异常导致卡状态
#define OBSTACLE_RECOVER_CYCLES           (220)      // 离开路障后继续冻结积分的恢复时间，约 220ms
#define OBSTACLE_TARGET_SPEED_RPM         (70.0f)    // 路障区域速度上限；<=0 表示不限制 target_speed
#define OBSTACLE_FORWARD_TILT_DEG         (0.8f)     // 路障内叠加到前后平衡目标角的偏置；方向错就改符号
#define OBSTACLE_MOTOR_BOOST_DUTY         (120)      // 路障内双轮同向前馈推力；方向错就改符号
#define OBSTACLE_NAV_SCALE                (0.35f)    // 路障内导航差速缩放，避免方格冲击时循迹 PID 存量过大
#define OBSTACLE_STUCK_SPEED_RPM          (12)       // 卡滞判定：实际速度绝对值低于该值
#define OBSTACLE_STUCK_BALANCE_DUTY       (900)      // 卡滞判定：平衡电机输出绝对值高于该值
#define OBSTACLE_STUCK_DETECT_CYCLES      (180)      // 连续满足卡滞条件多久触发脱困，约 180ms
#define OBSTACLE_BACKOFF_CYCLES           (130)      // 脱困阶段：短暂反拖时间
#define OBSTACLE_BACKOFF_DUTY             (-450)     // 脱困阶段：双轮反拖 duty；方向错就改符号
#define OBSTACLE_BOOST_CYCLES             (210)      // 脱困阶段：前冲时间
#define OBSTACLE_BOOST_DUTY               (720)      // 脱困阶段：双轮前冲 duty；方向错就改符号
#define OBSTACLE_MAX_RETRY                (2)        // 单次路障内最多反拖/前冲次数
#define OBSTACLE_TILT_ABORT_DEG           (38.0f)    // 姿态绝对值超过该角度时退出路障模式

//============================ 路况仲裁参数 ============================
// 仲裁优先级：单边桥 > 颠簸路段 > 普通路；草地不单独识别，按普通路处理。
#define USE_TERRAIN_AUTO_BRIDGE                       // 自动识别单边桥并允许 bridge_ctrl 接管
#define USE_TERRAIN_AUTO_OBSTACLE                     // 自动识别颠簸路段并触发 obstacle_ctrl

// 科目三里程点触发：进入 fun_a33 时会记录当前 Car.mileage 为 0 点。
// 先跑一遍看 TERRAIN_DEBUG 的 course_mileage，再把桥/颠簸起止点填到下面。
#define USE_TERRAIN_MILEAGE_TRIGGER                   // 按科目三相对里程点触发/门控特殊路况
#define TERRAIN_MILEAGE_FORCE_TRIGGER      (1)        // 1=进入里程窗口立即触发；0=仅作为自动识别门控
#define TERRAIN_MILEAGE_GATE_AUTO          (1)        // 1=自动识别只在对应里程窗口内生效；0=保留全程自动识别
#define TERRAIN_MILEAGE_PRE_ARM_CM         (8.0f)     // 里程窗口前提前允许识别，补偿编码器/起点误差
#define TERRAIN_MILEAGE_POST_HOLD_CM       (12.0f)    // 里程窗口后继续保持一点距离，防止提前退出

#define TERRAIN_BRIDGE_MILEAGE_ENABLE      (0)        // 填好起止点后改 1
#define TERRAIN_BRIDGE_START_CM            (0.0f)     // 单边桥起点：科目三相对里程 cm
#define TERRAIN_BRIDGE_END_CM              (0.0f)     // 单边桥终点：科目三相对里程 cm
#define TERRAIN_OBSTACLE_MILEAGE_ENABLE    (0)        // 填好起止点后改 1
#define TERRAIN_OBSTACLE_START_CM          (0.0f)     // 颠簸路段起点：科目三相对里程 cm
#define TERRAIN_OBSTACLE_END_CM            (0.0f)     // 颠簸路段终点：科目三相对里程 cm

// 逐飞助手/屏幕调试采样：PIT 中只缓存数据，主循环限频发送，避免串口阻塞控制中断。
#define USE_TERRAIN_DEBUG
#define TERRAIN_DEBUG_SEND_ENABLE          (1)
#define TERRAIN_DEBUG_SEND_PERIOD_MS       (50)       // 20Hz，115200 串口较稳；想更细可调 20
#define TERRAIN_DEBUG_ASSISTANT_DEVICE     SEEKFREE_ASSISTANT_DEBUG_UART

#define TERRAIN_BRIDGE_ROLL_THRESHOLD      (BRIDGE_ROLL_THRESHOLD) // 自动桥触发 roll 阈值（°）
#define TERRAIN_BRIDGE_DETECT_CYCLES       (18)        // roll 连续超阈值周期，调大可减少颠簸误判为桥
#define TERRAIN_BRIDGE_MIN_HOLD_CM         (8.0f)      // 自动桥状态最短保持距离，避免刚触发就释放
#define TERRAIN_BRIDGE_IDLE_TIMEOUT_CYCLES (700)       // 桥模块未进入时的自动桥最大保持周期
#define TERRAIN_BRIDGE_AUTO_SPEED_RPM      (BRIDGE_MIN_COMP_SPEED_RPM) // 自动桥首次触发时给桥模块的速度参考

#define TERRAIN_STARTUP_IGNORE_CYCLES      (350)       // 运行刚使能后忽略自动路况，避开起步大输出误触发
#define TERRAIN_OBSTACLE_GYRO_THRESH_DPS   (260.0f)    // 颠簸识别：pitch/roll 角速度冲击阈值（deg/s）
#define TERRAIN_OBSTACLE_MOTOR_DUTY_THRESH (700)       // 颠簸识别：负载增大时平衡输出阈值
#define TERRAIN_OBSTACLE_SPEED_DROP_RPM    (45)        // 颠簸识别：负载增大时速度低于该值
#define TERRAIN_OBSTACLE_MAX_PITCH_DEG     (25.0f)     // 超过该俯仰角不触发颠簸，交给保护/其他动作
#define TERRAIN_OBSTACLE_MAX_ROLL_DEG      (8.0f)      // 超过该横滚角优先按单边桥/保护处理
#define TERRAIN_OBSTACLE_SCORE_ON          (10)        // 颠簸累计分达到该值触发
#define TERRAIN_OBSTACLE_SCORE_SHOCK_STEP  (3)         // 角速度冲击每次加分
#define TERRAIN_OBSTACLE_SCORE_LOAD_STEP   (1)         // 低速大输出每次加分
#define TERRAIN_OBSTACLE_SCORE_DECAY       (1)         // 未命中时每周期衰减
#define TERRAIN_OBSTACLE_COOLDOWN_CYCLES   (800)       // 颠簸结束后冷却时间，避免重复触发

//============================ PID 启动渐变参数 ============================
#define RAMP_START_RATIO             0.2f     // 启动时 P 参数初始比例
#define RAMP_END_RATIO               1.0f     // 渐变结束后 P 参数比例

//============================ 跳跃动作参数 ============================
// 六段式跳跃：加速/预动作 -> 起跳 -> 收腿 -> 放腿 -> 缓冲等待 -> 自平衡
// 说明：
//   1. JUMP_*_DUTY_* 不再写绝对 PWM，而是相对各舵机 center_num 的逻辑偏移；
//   2. 正值沿 STEER_x_DIR 方向动作，四条腿伸出量一致，避免后腿中心值不同导致一边伸不开；
//   3. 当前幅度对齐 TEST3 原跳跃动作：+2500 -> 回中 -> +1400 -> 回中。
#define JUMP_TIME_ACCEL              150      // 阶段 1：对齐 TEST3，保持大幅动作
#define JUMP_TIME_TAKEOFF            100      // 阶段 2：回到中心
#define JUMP_TIME_RETRACT            100       // 阶段 3：短时间中等幅度动作
#define JUMP_TIME_EXTEND             80      // 阶段 4：回到中心
#define JUMP_TIME_BUFFER             0        // 阶段 5：当前先关闭额外缓冲，保留直立环
#define JUMP_TIME_RECOVER            0        // 阶段 6：当前先关闭额外恢复，保留直立环

#define JUMP_ACCEL_DUTY_FRONT        3000     // 阶段 1：前腿逻辑偏移
#define JUMP_ACCEL_DUTY_REAR         3000     // 阶段 1：后腿逻辑偏移

#define JUMP_TAKEOFF_DUTY_FRONT      0        // 阶段 2：前腿回中
#define JUMP_TAKEOFF_DUTY_REAR       0        // 阶段 2：后腿回中

#define JUMP_RETRACT_DUTY_FRONT      1400     // 阶段 3：前腿逻辑偏移
#define JUMP_RETRACT_DUTY_REAR       1400     // 阶段 3：后腿逻辑偏移

#define JUMP_EXTEND_DUTY_FRONT       0        // 阶段 4：前腿回中
#define JUMP_EXTEND_DUTY_REAR        0        // 阶段 4：后腿回中

#define JUMP_RECOVER_DUTY_FRONT      0        // 恢复目标：前腿中心位
#define JUMP_RECOVER_DUTY_REAR       0        // 恢复目标：后腿中心位

#define JUMP_MOTOR_LOCK_DUTY         0        // 起跳后/落地缓冲期间电机输出占空比（0=自由滑行，可改为主动制动值）

// eg 跳跃状态机参数映射：默认沿用上面的 TEST3 动作序列。
#define JUMP_PREPARE_TICKS           0
#define JUMP_CHARGE_TICKS            JUMP_TIME_ACCEL
#define JUMP_LAUNCH_TICKS            JUMP_TIME_TAKEOFF
#define JUMP_AIRBORNE_TIMEOUT        JUMP_TIME_RETRACT
#define JUMP_LANDING_TICKS           JUMP_TIME_EXTEND
#define JUMP_RECOVER_TICKS           JUMP_TIME_RECOVER

#define JUMP_CHARGE_DUTY             JUMP_ACCEL_DUTY_FRONT
#define JUMP_LAUNCH_DUTY             JUMP_TAKEOFF_DUTY_FRONT
#define JUMP_PRELAND_DUTY            JUMP_RETRACT_DUTY_FRONT
#define JUMP_LAND_DAMPING_DUTY       0

#define JUMP_FORWARD_TILT_TARGET     0.0f
#define JUMP_FORWARD_MOTOR_BOOST     0.0f
#define JUMP_SPEED_RECOVERY_RATE     1.0f
#define JUMP_AIRBORNE_PID_SCALE      1.0f
#define JUMP_LANDING_PID_SCALE       1.0f
#define JUMP_RECOVER_PID_RAMP_RATE   1.0f
#define JUMP_AIRBORNE_ACC_THRESHOLD  0.35f
#define JUMP_LANDING_ACC_THRESHOLD   1.35f
#define JUMP_MAX_TILT_ABORT          35.0f
#define JUMP_VISION_MIN_DIST         50.0f
#define JUMP_VISION_MAX_DIST         500.0f

//============================ 转向控制参数 ============================
#define STEER_SPEED_SCALE_DIV        7.0f     // 速度环输出到基础转向 duty 的缩放除数
#define STEER_SPEED_LIMIT            250      // 速度环输出限幅（±）
#define STEER_SPEED_MULT             6        // 基础转向 duty 映射倍数（最终映射到 ±1500）
#define STEER_PITCH_MAX_DEG          30.0f    // 转向衰减计算中最大俯仰角（°）
#define STEER_BALANCE_LIMIT          300      // 左右平衡角度环输出限幅（±）
#define STEER_BALANCE_MULT           6        // 平衡角补偿映射倍数
#define STEER_NORMAL_RATE_LIMIT      10       // 正常行驶时每个周期舵机最大变化量（±）
#define STEER_EMERGENCY_RATE_LIMIT   1        // 异常停机时每个周期舵机回中最大变化量（±）
#define STEER_BALANCE_WAIT_CYCLES    500      // 启动后等待姿态收敛再引入左右平衡补偿的周期数（约 0.5s）

//============================ 转向输出低通滤波参数 ============================
// 滤波输出 = (历史值 * STEER_FILTER_OLD_WEIGHT + 新值 * (STEER_FILTER_NEW_WEIGHT - STEER_FILTER_OLD_WEIGHT)) / STEER_FILTER_NEW_WEIGHT
// 当前等效新值权重 = 1 - 0.8 = 0.2，即滤波系数 0.2
#define STEER_FILTER_OLD_WEIGHT      8        // 历史值权重
#define STEER_FILTER_NEW_WEIGHT      10       // 总权重（分母）

//============================ 主循环时序参数 ============================
#define LOOP_DIV_ANGLE_CYCLE         5        // 角度环 PID / 导航 / 里程计算周期分频（每 N 周期执行一次）
#define LOOP_DIV_SPEED_CYCLE         20       // 速度环 PID / 转向环周期分频（每 N 周期执行一次）

//============================ 转向差速陀螺仪比例参数 ============================
#define TURN_GYRO_SCALE_DIV          3        // IMU660RB Z 轴陀螺仪转向差速比例除数（越大转向响应越慢）

//============================ 原地旋转参数 ============================
#define ROTATION_MAX_DUTY            3000     // 最大差速占空比上限
#define ROTATION_DEFAULT_DURATION_MS 2000     // 默认旋转持续时间（ms）
#define ROTATION_DEFAULT_TIMEOUT_MS  12000    // 定圈数模式超时时间（ms）
#define ROTATION_BRAKE_MS            180      // 反向刹车持续时间（ms）
#define ROTATION_BRAKE_DUTY          320      // 反向刹车差速占空比
#define ROTATION_RAMP_MS             300      // 启动 ramp 时间（ms）
#define ROTATION_SLOWDOWN_DEG        270.0f   // 末段开始降速的剩余角度（°）
#define ROTATION_MIN_RUN_DUTY        240      // 最小运行占空比
#define ROTATION_TARGET_TOL_DEG      3.0f     // 目标角度容差（°）
#define ROTATION_STEER_SCALE         2.0f     // 舵机偏转缩放比例
#define ROTATION_STEER_MAX           600      // 舵机偏转最大偏移量
#define NAV_TURN_DIFF_MULT           10       // 对齐 TEST3 2：N.Final_Out * 10 叠加到左右轮差速

//============================ 上电自动直行参数 ============================
// 取消注释 AUTO_RUN_STRAIGHT 可进入上电自动直立/直行模式；默认使用菜单
// #define AUTO_RUN_STRAIGHT                   // 定义此宏：禁用菜单，上电后自动站立并直行
#define AUTO_RUN_SPEED               80       // 直行目标速度（RPM），0=先只平衡站立不前进
#define AUTO_RUN_ARM_DELAY_MS        500      // PIT 启动后等待姿态收敛再使能电机的时间（ms）

#ifndef USE_TEST3_BALANCE_CORE
#define USE_DIFFERENTIAL_STEERING

// 启用一阶互补滤波更新（在 pit_call_back() 四元数解算后调用）
#define USE_FIRST_ORDER_FILTER
#endif

#endif // BODY_CTRL_CONFIG_H_
