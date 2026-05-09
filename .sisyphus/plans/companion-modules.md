# 轮腿机器人控制模块 — 配套模块完整实现

## TL;DR

> **Quick Summary**: 为 control.c 创建 5 个配套模块（pid、euler、foc、timer_control、servo_kinematics）+ 更新 control.h，使轮腿机器人 LQR+PID 控制系统可以编译链接运行。
>
> **Deliverables**:
> - `pid.h` + `pid.c` — 位置式 PID 控制器（10参数初始化，5个API函数）
> - `euler.h` + `euler.c` — 姿态角结构体（留空壳，算法后续填入）
> - `foc.h` + `foc.c` — FOC 电机驱动（UART 包装 small_driver_set_duty）
> - `timer_control.h` + `timer_control.c` — get_timer() 时间测量（封装 zf_driver_timer）
> - `servo_kinematics.h` + `servo_kinematics.c` — 腿运动学解算 + 死区校准
> - `control.h` — 更新符号声明，整合所有模块
>
> **Estimated Effort**: Medium
> **Parallel Execution**: YES — 2 Waves
> **Critical Path**: Wave 1 (pid + timer) → Wave 2 (foc + servo + euler + control.h)

---

## Context

### Original Request
用户要求为 `control.c`（轮腿机器人 LQR+PID 级联控制系统）补全缺失的配套控制模块：pid_t, euler_angle, foc_set_duty, get_timer, servo_control_table 等，使系统可以实际编译运行。

### Interview Summary
**Key Discussions**:
- **补全深度**: 头文件 + 完整 .c 实现（5 模块 × 2 文件 = 10 文件）
- **舵机映射**: SERVO_1~4 = STEER_1~4（TCPWM_CH10_P05_0~3），已在 servo.h 定义
- **FOC 驱动方式**: 通过 UART 发给电机驱动器 → 包装 `small_driver_set_duty()`
- **姿态解算**: 只建结构体和 extern，融合算法后续填入（互补滤波/Mahony）
- **定时器通道**: TOM0_CH0~4 精确值待用户提供（先用 TODO 占位）

**Research Findings**:
- **pid_init 原型**: `void pid_init(pid_t*, float kp, float ki, float kd, float dt, float, float, bool/uint8, uint16 limit, pid_type_enum)` — 10 参数，Position_pid 枚举
- **pid_t 成员**: `.out` 为 `float`（唯一外部访问成员），内部含 target/observation/integral/kp/ki/kd/dt
- **get_timer 原型**: `void get_timer(timer_channel_enum ch, uint16 *counter, float *dt_out)`
- **servo_control_table 原型**: `void servo_control_table(float p, float angle, int16 *out1, int16 *out2)`
- **已有基础设施**: imu660ra_gyro_* (zf_device_imu660ra.h), pwm_set_duty (zf_driver_pwm.h), small_driver_set_duty (small_driver_uart_control.h), zf_driver_timer.{h,c}

### Metis Review
**Identified Gaps** (addressed in this plan):
- **pid_t 内部结构**: 需要至少 kp, ki, kd, dt, target, observation, integral, out, last_error, output_limit, integral_limit 字段
- **pid_init 参数含义**: 分析所有调用模式 → 含 integral_min, integral_max, use_d 参数
- **get_timer 实现**: 基于 zf_driver_timer 的 timer_get() 实现 delta-time 计算
- **servo_control_table 数学**: 四杆机构逆运动学，需腿长→转角→PWM 映射
- **TOM0_CH* 占位**: 用 0 占位 + TODO，后续替换为实际通道号
- **ASSERT 宏来源**: zf_common_debug.h 已提供 zf_assert()，可 `#define ASSERT zf_assert`

---

## Work Objectives

### Core Objective
创建 5 套配套控制模块（.h + .c），使 control.c 可以成功编译、链接、运行。

### Concrete Deliverables
| 模块 | .h 文件 | .c 文件 | 核心功能 |
|------|---------|---------|----------|
| PID | `pid.h` | `pid.c` | pid_t 类型 + 5 API 函数 |
| Euler | `euler.h` | `euler.c` | euler_angle 结构体 + extern 实例 |
| FOC | `foc.h` | `foc.c` | foc_set_duty() → UART 驱动 |
| Timer | `timer_control.h` | `timer_control.c` | get_timer() + TOM0_CH* 枚举 |
| Kinematics | `servo_kinematics.h` | `servo_kinematics.c` | servo_control_table() + dead_zone |
| Control | `control.h` | (更新) | 整合所有 extern + 函数原型 |

### Definition of Done
- [ ] 所有 5 个 .h 文件通过 IAR 语法检查（double-include 不报错）
- [ ] 所有 5 个 .c 文件在 IAR 中编译无错误
- [ ] control.c 在包含 control.h 后可以通过完整编译链接
- [ ] 符号覆盖率：control.c 使用的每个外部符号都可追溯到 include 链

### Must Have
- pid_t 完整结构体 + Position_pid 枚举 + 5 API 实现
- euler_angle_struct typedef + extern euler_angle
- foc_set_duty() 调用 small_driver_set_duty()
- get_timer() 基于 zf_driver_timer 的 delta-time 计算
- servo_control_table() 的逆运动学骨架（含杆长参数占位）
- clip2() 宏定义 + DEG_TO_RAD 宏 + 死区校准常量

### Must NOT Have (Guardrails)
- **不**实现姿态融合算法（euler.c 只定义变量，不写滤波器）
- **不**修改 control.c 已有逻辑
- **不**修改现有库文件（libraries/zf_*）
- **不**在 .h 中定义变量（只 extern）
- **不**使用大写 include guard（跟随 _module_h_ 风格）
- **不**将 TOM0_CH* 写成固定值（占位 TODO，等用户提供实际映射）

---

## Verification Strategy

> 嵌入式项目无自动化测试框架。验证方式：IAR 编译 + 符号审计。

### Test Decision
- **Infrastructure exists**: NO
- **Automated tests**: None — 编译验证
- **QA Policy**: Agent 通过 IAR 编译尝试 + grep 符号覆盖审计

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — 基础类型 + 独立模块):
├── Task 1: pid.h + pid.c        [quick] — 位置式 PID（无外部依赖）
├── Task 2: timer_control.h + .c  [quick] — get_timer()（仅依赖 zf_driver_timer）
└── Task 3: control.h 基础框架   [quick] — jump_control_struct + 通用宏

Wave 2 (After Wave 1 — 依赖 PID 类型和控制框架):
├── Task 4: euler.h + euler.c    [quick] — euler_angle 结构体（依赖 pid.h 的 float 约定）
├── Task 5: foc.h + foc.c        [quick] — foc_set_duty()（依赖 small_driver_uart_control.h）
├── Task 6: servo_kinematics.h + .c [unspecified-high] — 运动学解算（逆向运动学数学）
└── Task 7: control.h 整合       [quick] — 添加所有 extern + 函数原型

Wave FINAL (After ALL tasks):
├── Task F1: IAR 编译验证
├── Task F2: 符号覆盖审计
├── Task F3: 代码风格审查
└── Task F4: Scope 合规检查
```

### Dependency Matrix

- **1**: - - 4, 7
- **2**: - - 5, 6, 7
- **3**: - - 4, 5, 6, 7
- **4**: 1, 3 - 7
- **5**: 2, 3 - F1
- **6**: 2, 3 - F1
- **7**: 1, 2, 3, 4, 5, 6 - F1-F4

### Agent Dispatch Summary

- **Wave 1**: 3 tasks → `quick` × 3
- **Wave 2**: 4 tasks → `quick` × 3 + `unspecified-high` × 1
- **FINAL**: 4 tasks → `oracle` + `unspecified-high` × 3

---

- [x] 1. 创建 `pid.h` + `pid.c` — 位置式 PID 控制器

  **What to do**:

  ### pid.h
  - Include guard: `#ifndef _pid_h_` / `#define _pid_h_`
  - Include `"zf_common_headfile.h"` （类型 uint8/int16/float + FALSE/TRUE）
  - **pid_type_enum**: `typedef enum { Position_pid, Incremental_pid } pid_type_enum;`
  - **pid_t 结构体**:
    ```c
    typedef struct {
        float kp, ki, kd;           // PID 参数
        float dt;                   // 时间步长
        float target;               // 目标值
        float observation;          // 观测值
        float integral;             // 积分累加
        float integral_min;         // 积分下限
        float integral_max;         // 积分上限
        float output_limit;         // 输出限幅
        float out;                  // PID 输出
        float last_error;           // 上一次误差
        uint8 use_d;               // 是否启用微分项
        pid_type_enum type;         // 位置式/增量式
    } pid_t;
    ```
  - 函数原型:
    ```c
    void pid_init(pid_t *pid, float kp, float ki, float kd, float dt,
                  float integral_min, float integral_max, uint8 use_d,
                  float output_limit, pid_type_enum type);
    void pid_set_target(pid_t *pid, float target);
    void pid_get_observation(pid_t *pid, float observation);
    void pid_set_dt(pid_t *pid, float dt);
    void pid_run(pid_t *pid);
    ```

  ### pid.c
  - `pid_init()`: 赋值所有结构体成员，清零 integral 和 out
  - `pid_set_target()`: `pid->target = target;`
  - `pid_get_observation()`: `pid->observation = observation;`
  - `pid_set_dt()`: `pid->dt = dt;`
  - `pid_run()`: 标准位置式 PID 算法
    ```
    error = target - observation;
    integral += error * dt;
    // 积分限幅
    if (integral > integral_max) integral = integral_max;
    if (integral < integral_min) integral = integral_min;
    // 微分（如果有）
    derivative = use_d ? ((error - last_error) / dt) : 0;
    out = kp * error + ki * integral + kd * derivative;
    // 输出限幅
    out = clip(out, -output_limit, output_limit);
    last_error = error;
    ```

  **Must NOT do**:
  - 不实现增量式 PID（Position_pid 已覆盖当前所有调用）
  - 不添加防积分饱和之外的复杂算法

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: `[]` — 标准 C 数学，无特殊技能需求

  **Parallelization**:
  - **Can Run In Parallel**: YES（与 Task 2, 3 并行）
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 4, 7

  **References**:
  - `project/code/servo.h:1-54` — 头文件模板（include guard + typedef + extern + 函数原型）
  - `project/code/control.c:29,69-76,170-198` — pid_t 使用模式、pid_init 参数、pid_run 调用链
  - `libraries/zf_common/zf_common_typedef.h:1-50` — ZF_ENABLE/FALSE 等常量定义

  **Acceptance Criteria**:
  - [ ] pid.h double-include 不报错
  - [ ] pid_init() 接受 10 参数，匹配 control.c 中所有调用
  - [ ] pid_run() 更新 pid->out（位置式算法）
  - [ ] 位置式 PID 数学正确性（含积分限幅+输出限幅）

  **QA Scenarios**:
  ```
  Scenario: PID 基本功能测试 — 阶跃响应
    Tool: Bash (IAR 编译器 + 模拟验证)
    Steps:
      1. Create test harness: pid_init(&pid, 1.0, 0.1, 0.0, 0.01, 0, 0, FALSE, 100, Position_pid)
      2. pid_set_target(&pid, 100); pid_get_observation(&pid, 0)
      3. Call pid_run(&pid) 1000 times, observe convergence to target
    Expected Result: pid.out → 100，无振荡发散
    Evidence: .sisyphus/evidence/task-1-pid-step-response.txt

  Scenario: PID 积分限幅测试
    Steps:
      1. pid_init with integral_max = 50
      2. 持续给 large error，验证 integral 不超过 50
    Expected Result: integral clamped at ±50
    Evidence: .sisyphus/evidence/task-1-pid-clamp.txt
  ```

  **Commit**: YES
  - Message: `feat(pid): add position PID controller with 5 API functions`
  - Files: `project/code/pid.h`, `project/code/pid.c`

- [x] 2. 创建 `timer_control.h` + `timer_control.c` — 时间测量模块

  **What to do**:

  ### timer_control.h
  - Include guard: `#ifndef _timer_control_h_`
  - Include `"zf_common_headfile.h"` （获取 zf_driver_timer.h 的 TC_TIME2_CH* 枚举）
  - **timer_channel_enum**: `typedef enum { TOM0_CH0 = 0, TOM0_CH1, TOM0_CH2, TOM0_CH3, TOM0_CH4, ... } timer_channel_enum;`
    - 值用 0-4 占位，[TODO: 替换为实际 TOM 通道号]
  - 函数原型: `void get_timer(timer_channel_enum ch, uint16 *timer_var, float *dt_out);`

  ### timer_control.c
  - `get_timer()` 实现:
    ```c
    void get_timer(timer_channel_enum ch, uint16 *timer_var, float *dt_out) {
        uint16 now = timer_get((timer_index_enum)ch);  // 读取硬件定时器计数值
        uint16 delta = now - *timer_var;
        *timer_var = now;
        *dt_out = (float)delta / 1000000.0f;  // 假设定时器单位微秒
    }
    ```
  - 需要在调用前初始化定时器（可通过 init 函数或假设已由主程序初始化）

  **Must NOT do**:
  - 不擅自创建定时器初始化（假设主程序已配置 TOM0）

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES（与 Task 1, 3 并行）
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 5, 6, 7

  **References**:
  - `libraries/zf_driver/zf_driver_timer.h:1-50` — timer_get() 和 timer_index_enum 定义
  - `project/code/control.c:160-163,171,180,189,196,220` — get_timer 调用模式

  **QA Scenarios**:
  ```
  Scenario: get_timer delta 计算
    Tool: Bash (IAR 编译器)
    Steps:
      1. 编译 timer_control.c，验证可链接
      2. 手动注入 timer_get mock 返回值验证 delta 计算正确
    Expected Result: dt_out = (current - previous) / 1e6
    Evidence: .sisyphus/evidence/task-2-timer-delta.txt
  ```

  **Commit**: YES
  - Message: `feat(timer): add get_timer() with TOM0 channel enum`
  - Files: `project/code/timer_control.h`, `project/code/timer_control.c`

- [x] 3. 创建 control.h 基础框架 — 通用宏 + jump_control_struct

  **What to do**:

  ### control.h（第一阶段 — 基础定义）
  - Include guard: `#ifndef _control_h_`
  - Include `"zf_common_headfile.h"` + `"servo.h"`
  - **通用数学宏**:
    ```c
    #define DEG_TO_RAD   (57.29577951308232f)  // 180/PI，用作除数将角度转弧度
    #define ABS(x)       (((x) >= 0) ? (x) : -(x))
    #define clip2(x, max) (((x) > (max)) ? (max) : ((x) < -(max)) ? -(max) : (x))
    ```
  - **jump_control_struct**:
    ```c
    typedef struct {
        int min;               // 时间段下限（ms）
        int max;               // 时间段上限（ms）
        void (*handler)(int);  // 阶段回调
        const char *name;      // 阶段名称
    } jump_control_struct;
    ```
  - **[TODO] 注释标记**: 待 Task 4-6 完成后，Task 7 再补全 extern 和函数原型

  **Must NOT do**:
  - 不要在这一步添加 extern 变量声明（Task 7 统一处理）

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES（与 Task 1, 2 并行）
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 4, 5, 6

  **References**:
  - `project/code/servo.h:1-54` — 头文件模板
  - `project/code/control.c:5-21` — LQR_K 常量、jump_control_config 使用模式
  - `project/code/control.c:330-356` — clip2 使用模式（±10000 限幅）

  **QA Scenarios**:
  ```
  Scenario: clip2 宏边界测试
    Tool: Bash (grep 验证宏定义)
    Steps:
      1. 验证 #define clip2(x,max) 双括号保护
      2. 检查控制流：正值→限幅在max，负值→限幅在-max
    Expected Result: clip2(5000, 10000) = 5000, clip2(15000, 10000) = 10000, clip2(-15000, 10000) = -10000
    Evidence: .sisyphus/evidence/task-3-clip2-macro.txt
  ```

  **Commit**: YES
  - Message: `feat(control): add control.h base with macros and jump_control_struct`
  - Files: `project/code/control.h`

- [x] 4. 创建 `euler.h` + `euler.c` — 姿态角结构体

  **What to do**:

  ### euler.h
  - Include guard: `#ifndef _euler_h_`
  - Include `"zf_common_typedef.h"` (获取 float 类型)
  - **euler_angle_struct**:
    ```c
    typedef struct {
        float pitch;    // 俯仰角（deg）
        float roll;     // 横滚角（deg）
        float yaw;      // 偏航角（deg）
    } euler_angle_struct;
    extern euler_angle_struct euler_angle;
    ```

  ### euler.c
  - 定义全局实例: `euler_angle_struct euler_angle = {0};`
  - **不实现姿态融合**（算法后续填入，互补滤波或 Mahony 待定）
  - 添加 TODO 注释说明姿态更新函数签名待实现

  **Must NOT do**:
  - 不实现 IMU 融合算法（euler_angle 值需要外部持续更新）
  - 不依赖与姿态解算无关的库

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES（与 Task 5, 6 并行）
  - **Parallel Group**: Wave 2
  - **Blocked By**: Task 1, 3
  - **Blocks**: Task 7

  **References**:
  - `project/code/control.c:100,104-105,179,222,226` — euler_angle.pitch/roll 使用
  - `libraries/zf_device/zf_device_imu660ra.h:142-150` — 可用原始 IMU 数据（imu660ra_gyro_x/y/z）
  - `project/code/servo.h:47-51` — extern 声明模式

  **QA Scenarios**:
  ```
  Scenario: euler_angle 结构体编译
    Tool: Bash (IAR -fsyntax-only)
    Steps:
      1. 编译 euler.h + euler.c
      2. 验证 extern euler_angle 可被 control.c 引用
    Expected Result: 编译成功，无 undefined reference 错误
    Evidence: .sisyphus/evidence/task-4-euler-compile.txt
  ```

  **Commit**: YES
  - Message: `feat(euler): add euler angle struct and extern instance`
  - Files: `project/code/euler.h`, `project/code/euler.c`

- [x] 5. 创建 `foc.h` + `foc.c` — FOC 电机驱动 UART 接口

  **What to do**:

  ### foc.h
  - Include guard: `#ifndef _foc_h_`
  - Include `"zf_common_headfile.h"` + `"small_driver_uart_control.h"`
  - 函数原型: `void foc_set_duty(int16 left_duty, int16 right_duty);`
  - **motor_value 变量**: `extern small_device_value_struct motor_value;`
    - 用户确认 motor_value 是独立于 small_driver_value 的实例，在 foc.c 中定义

  ### foc.c
  - 定义 motor_value: `small_device_value_struct motor_value = {0};`
  - `foc_set_duty()`:
    ```c
    void foc_set_duty(int16 left_duty, int16 right_duty) {
        small_driver_set_duty(&motor_value, left_duty, right_duty);
    }
    ```
  - 如果 motor_value 和 small_driver_value 共享同一个 UART，需要适当修改（目前作为独立实例）

  **Must NOT do**:
  - 不直接操作 PWM 寄存器（已确认通过 UART 发指令给电机驱动器）
  - 不需要初始化 UART（由 small_driver_uart_control 模块处理）

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES（与 Task 4, 6 并行）
  - **Parallel Group**: Wave 2
  - **Blocked By**: Task 2, 3
  - **Blocks**: Task 7, F1

  **References**:
  - `project/code/control.c:129,200` — foc_set_duty(LO, -RO) 调用模式
  - `project/code/small_driver_uart_control.h:46` — small_driver_set_duty() 原型
  - `project/code/small_driver_uart_control.h:19-41` — small_device_value_struct 定义

  **QA Scenarios**:
  ```
  Scenario: foc_set_duty 编译验证
    Tool: Bash (IAR)
    Steps:
      1. 编译 foc.c，验证 small_driver_set_duty 链接成功
    Expected Result: 编译+链接通过
    Evidence: .sisyphus/evidence/task-5-foc-compile.txt
  ```

  **Commit**: YES
  - Message: `feat(foc): add foc_set_duty() UART motor driver wrapper`
  - Files: `project/code/foc.h`, `project/code/foc.c`

- [x] 6. 创建 `servo_kinematics.h` + `servo_kinematics.c` — 腿运动学

  **What to do**:

  ### servo_kinematics.h
  - Include guard: `#ifndef _servo_kinematics_h_`
  - Include `"zf_common_headfile.h"` + `"servo.h"`
  - **舵机引脚别名**（映射到 servo.h 的 STEER_1~4）:
    ```c
    #define SERVO_1    (STEER_1_PWM)     // 左腿上舵机
    #define SERVO_2    (STEER_2_PWM)     // 左腿下舵机
    #define SERVO_3    (STEER_3_PWM)     // 右腿上舵机
    #define SERVO_4    (STEER_4_PWM)     // 右腿下舵机
    #define SERVO1_MID (STEER_1_CENTER)  // 左腿上中位值 = 1000
    #define SERVO2_MID (STEER_2_CENTER)
    #define SERVO3_MID (STEER_3_CENTER)
    #define SERVO4_MID (STEER_4_CENTER)
    ```
  - **死区校准常量**（值待标定，用 0 占位 + TODO）:
    ```c
    #define L_DEAD_ZONE_CORRECT    (0)   // [TODO: 实测标定]
    #define L_DEAD_ZONE_NEGATIVE   (0)
    #define R_DEAD_ZONE_CORRECT    (0)
    #define R_DEAD_ZONE_NEGATIVE   (0)
    ```
  - **外部变量**: `extern float mid_point;`  // 视觉中点
  - 函数原型: `void servo_control_table(float p, float angle, int16 *out_ph1, int16 *out_ph4);`

  ### servo_kinematics.c
  - 定义 mid_point: `float mid_point = 0;`
  - `servo_control_table()`:
    ```c
    void servo_control_table(float p, float angle, int16 *out_ph1, int16 *out_ph4) {
        // [TODO: 四杆机构逆运动学解算]
        // 输入: p = 腿长(leg_length), angle = 摆角(deg)
        // 输出: out_ph1, out_ph4 = PWM 相位值（范围 ±10000）
        //
        // 杆长参数（待实测）:
        //   L1 = 上连杆长度 (mm)
        //   L2 = 下连杆长度 (mm)
        //   d  = 舵机轴间距 (mm)
        //
        // 简化实现（线性映射，待替换为真实运动学）:
        float scale = p / 5.5f;  // 以 5.5mm 为基准
        *out_ph1 = (int16)(scale * 500.0f);
        *out_ph4 = (int16)(scale * 500.0f);
    }
    ```
  - 函数体内需要实现舵机转角→绝对角度→PWM 占空比的完整映射链

  **Must NOT do**:
  - 不直接操作舵机硬件（调用 pwm_set_duty 在 control.c 的 left_leg_control 中完成）

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: 逆运动学涉及四杆机构数学建模，需要三角函数计算
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES（与 Task 4, 5 并行）
  - **Parallel Group**: Wave 2
  - **Blocked By**: Task 2, 3
  - **Blocks**: Task 7, F1

  **References**:
  - `project/code/control.c:368-408` — left_leg_control/right_leg_control 调用模式
  - `project/code/servo.h:11-36` — STEER_1~4 引脚定义
  - `libraries/zf_driver/zf_driver_pwm.h:106-111` — pwm_channel_enum + pwm_set_duty()

  **QA Scenarios**:
  ```
  Scenario: servo_control_table 输出范围验证
    Tool: Bash (IAR)
    Steps:
      1. 调用 servo_control_table(5.5, 0, &p1, &p4)
      2. 验证输出值在 int16 范围内且不为 0
    Expected Result: p1/p4 在合理 PWM 范围内（±10000）
    Evidence: .sisyphus/evidence/task-6-kinematics-output.txt
  ```

  **Commit**: YES
  - Message: `feat(kinematics): add leg servo kinematics with dead zone calibration`
  - Files: `project/code/servo_kinematics.h`, `project/code/servo_kinematics.c`

- [x] 7. 更新 control.h — 整合所有模块

  **What to do**:
  基于 Task 3 已创建的 control.h 骨架，追加：
  - **新增 include** (在现有 `"zf_common_headfile.h"` 和 `"servo.h"` 之后):
    ```c
    #include "pid.h"
    #include "euler.h"
    #include "foc.h"
    #include "timer_control.h"
    #include "servo_kinematics.h"
    ```
  - **extern 常量**（来自 control.c）:
    ```c
    extern const float LQR_K[8];
    extern const jump_control_struct jump_control_config[];
    extern const uint8 jump_step_num;
    extern const float Lmoto_K, Rmoto_K;
    ```
  - **extern PID 实例**（7个）:
    ```c
    extern pid_t leg_hight, turn_angle, turn_gyro, gyro, angle, speed, turn;
    ```
  - **extern 校准值**（6个 float）:
    ```c
    extern float angle_kd, pitch_mid, roll_mid, KP, KPP, KD, KDD;
    ```
  - **extern 时间步长**（7个 float）:
    ```c
    extern float dt_pid_gyro, dt_pid_angle, dt_pid_speed, dt_pid_turn, dt_leg, dt_pid_turn_angle, dt_pid_turn_gyro;
    ```
  - **extern 状态变量**:
    ```c
    extern float leg_long, set_speed, turn_out;
    extern uint8 jump_flag, speed_flag;
    ```
  - **函数原型**（control.c 中实现的 10 个函数）:
    ```c
    void pid_ctrl_Init(void);
    void LQR_control(float V_target, float th);
    float turn_control(float image_error);
    void pid_ctrl_Run(void);
    void leg_control(void);
    void jump_set_step(int step_num);
    void jump_control(void);
    void dead_compensate(int16 *input_L, int16 *input_R);
    void left_leg_control(float p, float angle);
    void right_leg_control(float p, float angle);
    ```
  - **外部模块变量**（来自未来模块 — TODO 标注）:
    ```c
    extern float image_error;   // [TODO: 视觉模块]
    extern float v_hat;         // [TODO: 状态估计器]
    extern float x_hat;         // [TODO: 状态估计器]
    ```

  **Must NOT do**:
  - 不覆盖 Task 3 已创建的 include guard、DEG_TO_RAD/ABS/clip2 宏、jump_control_struct
  - 不修改 control.c

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: NO（必须等所有 Wave 1+2 完成后执行）
  - **Parallel Group**: Wave 2（最后）
  - **Blocked By**: Task 1, 2, 3, 4, 5, 6
  - **Blocks**: F1, F2, F3, F4

  **References**:
  - `project/code/control.c:1-408` — 所有符号的唯一定义来源
  - 各子模块 .h 文件 — pid.h（pid_t）, euler.h（euler_angle_struct）, foc.h（small_device_value_struct）

  **QA Scenarios**:
  ```
  Scenario: 符号覆盖审计 — control.h 包含所有必需声明
    Tool: Bash (grep + diff)
    Steps:
      1. 提取 control.c 中所有文件作用域变量名和函数名
      2. 从 control.h + transitive includes 提取所有 extern/函数声明
      3. diff 验证全覆盖（排除 C 关键字、局部变量、库函数）
    Expected Result: 未覆盖符号仅限 imu660ra_*/pwm_set_duty（库已有）+ 配套模块符号（已通过 include 链解决）
    Evidence: .sisyphus/evidence/task-7-symbol-audit.txt
  ```

  **Commit**: YES
  - Message: `feat(control): complete control.h with all externs, prototypes, and module includes`
  - Files: `project/code/control.h`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE.

- [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. Verify:
  - Must Have: 5 个 .h + 5 个 .c 存在，pid_t 含 .out，servo_control_table 输出 int16*，foc_set_duty 调用 small_driver_set_duty
  - Must NOT Have: 无姿态算法实现，无库文件修改
  - Evidence 文件存在于 `.sisyphus/evidence/`
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [7/7] | VERDICT: APPROVE/REJECT`

- [x] F2. **Code Quality Review** — `deep` (combined with F3)
  Run IAR build 验证所有模块编译+链接。Review：
  - Include guard 正确性（每个 .h 独立检查 double-include）
  - 宏安全性（clip2/ABS/DEG_TO_RAD 参数括号保护）
  - 无 `#pragma` 关闭警告
  - 函数命名符合 snake_case 惯例
  Output: `Build [PASS/FAIL] | Lint [PASS/FAIL] | Style [N clean/N issues] | VERDICT`

- [x] F3. **Real Manual QA** — `deep` (combined with F2)
  执行所有 Task QA Scenarios：
  - Task 1: PID 阶跃响应 + 积分限幅
  - Task 2: get_timer delta 计算
  - Task 3: clip2 宏边界测试
  - Task 4: euler 编译
  - Task 5: foc 编译
  - Task 6: kinematics 输出范围
  - Task 7: 符号覆盖审计
  Save evidence to `.sisyphus/evidence/final-qa/`.
  Output: `Scenarios [7/7 pass] | VERDICT`

- [x] F4. **Scope Fidelity Check** — `deep`
  Verify:
  - 10 个新文件均在 `project/code/` 下，无文件泄露到 libraries/
  - control.c 零修改（git diff 限制在 project/code/）
  - 所有 TODO 占位变量已标注
  Output: `Tasks [7/7 compliant] | Files [CLEAN] | VERDICT`

---

## Commit Strategy

- **1**: `feat(pid): add position PID controller` — pid.h, pid.c
- **2**: `feat(timer): add get_timer() with TOM0 channel enum` — timer_control.h, timer_control.c
- **3**: `feat(control): add control.h base with macros and jump_control_struct` — control.h
- **4**: `feat(euler): add euler angle struct and extern instance` — euler.h, euler.c
- **5**: `feat(foc): add foc_set_duty() UART motor driver wrapper` — foc.h, foc.c
- **6**: `feat(kinematics): add leg servo kinematics with dead zone calibration` — servo_kinematics.h, servo_kinematics.c
- **7**: `feat(control): complete control.h with all externs, prototypes, and module includes` — control.h

---

## Success Criteria

### Verification Commands
```bash
# 文件存在性检查
ls project/code/pid.h project/code/pid.c
ls project/code/euler.h project/code/euler.c
ls project/code/foc.h project/code/foc.c
ls project/code/timer_control.h project/code/timer_control.c
ls project/code/servo_kinematics.h project/code/servo_kinematics.c
ls project/code/control.h

# IAR 编译
# 打开 project/iar/cyt4bb7.eww → F7 (Make)
# 预期：0 errors, 0 warnings for project/code/ files
```

### Final Checklist
- [ ] 5 个 .h 文件 double-include 无误
- [ ] 5 个 .c 文件编译通过
- [ ] control.h 整合所有依赖（include chain 完整）
- [ ] control.c 符号审计零遗漏
- [ ] 所有 TODO 占位变量已标注

---

## Known Limitations

| 项目 | 说明 | 后续工作 |
|------|------|----------|
| **TOM0_CH* 通道值** | 当前用 0-4 占位 | 用户提供实际 TOM 映射后替换 |
| **死区校准值** | 当前为 0 | 硬件标定后填入实测值 |
| **姿态解算** | 只建结构体，未实现算法 | 后续填入互补滤波/Mahony 代码 |
| **逆运动学** | 简化线性映射 | 实测杆长后替换为四杆机构解算 |
| **image_error/v_hat/x_hat** | extern 声明，值来源未知 | 视觉模块和状态估计器就位后定义 |


