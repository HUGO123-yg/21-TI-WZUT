# control.h — 轮腿机器人控制头文件

## TL;DR

> **Quick Summary**: 为 `project/code/control.c`（LQR+PID 级联轮腿控制系统）创建配套头文件，将舵机/定时器引脚定义置于文件顶部以便后期统一修改。
>
> **Deliverables**:
> - `project/code/control.h` — 含引脚宏、结构体 typedef、27 个 extern 声明、10 个函数原型
>
> **Estimated Effort**: Quick
> **Parallel Execution**: NO — 单任务
> **Critical Path**: Task 1 → F1-F4

---

## Context

### Original Request
用户提供 `control.c`（轮腿机器人 LQR+PID 控制程序），要求开发对应 `control.h`，GPIO/接口编号放在程序开头以保持后期更新的便利性。

### Interview Summary
**Key Discussions**:
- **舵机映射**: SERVO_1~4 复用 servo.h 的 STEER_1~4（TCPWM_CH10_P05_0~3），SERVO1_MID~4 = STEER_1_CENTER~4（1000）
- **电机变量**: `motor_value` 使用 `small_device_value_struct` 类型，是独立于 `small_driver_value` 的实例
- **外部传感器变量**: `image_error`, `mid_point`, `v_hat`, `x_hat` 全部在 control.h 中声明 extern
- **Scope**: 仅创建 control.h，配套模块（pid.h, euler.h, foc.h, timer_control.h, servo_kinematics.h）不在本次范围

**Research Findings**:
- `servo.h` 提供模板：`#ifndef _servo_h_` include guard，引脚 #define 在顶部（含中文标注），struct typedef + extern + 函数原型
- `small_driver_uart_control.h` 提供 `motor_value` 的类型定义（`small_device_value_struct`）
- `control.c` 中 27 个文件作用域变量需要 extern，10 个函数需要声明
- 以下符号仅靠 control.h 无法解决（来自未创建的配套模块）：`pid_t`, `euler_angle`, `foc_set_duty()`, `get_timer()`, `TOM0_CH*`, `servo_control_table()`, `ABS()`, `Position_pid` — 这属于已知限制

### Metis Review
**Identified Gaps** (addressed):
- **变量数量低估**: 实际 27 个需要 extern，非初估的 ~15 个 → 逐一核对并补全
- **`clip2()` 宏安全性**: 需要双参数括号保护 → 采用 `#define clip2(x, max) (((x)>(max))?(max):((x)<-(max))?-(max):(x))`
- **`DEG_TO_RAD` 命名歧义**: 实际值为 `180/π ≈ 57.29578`（使用时作除数），注释标注实际语义
- **`jump_control_struct` 需从使用模式反推定义**: 含 `int min; int max; void (*handler)(int); const char *name;` 四个字段
- **`ABS()` 未定义**: 需在 control.h 中添加通用取绝对值宏

---

## Work Objectives

### Core Objective
创建 `project/code/control.h`，使 control.c 的符号声明需求得到满足，引脚定义集中在文件顶部。

### Concrete Deliverables
- `project/code/control.h` — 完整的控制模块头文件

### Definition of Done
- [x] include guard 测试通过（double-include 不报错）
- [x] 所有 control.c 中定义的文件作用域变量在 .h 中有对应的 extern 声明
- [x] 所有 control.c 中定义的函数在 .h 中有对应的原型声明
- [x] 引脚/定时器/校准宏集中在文件顶部，含中文注释
- [x] 遵循 servo.h 的代码风格惯例

### Must Have
- `jump_control_struct` 完整 typedef
- 27 个 extern 声明（PID 实例、校准值、时间步长、状态变量）
- 10 个函数原型
- SERVO_1~4 / SERVO1_MID~4 引脚宏定义
- TOM0_CH0~4 定时器通道宏定义（值未知 → 用 `[TODO]` 占位）
- `clip2()`, `DEG_TO_RAD`, `ABS()` 通用宏
- `L_DEAD_ZONE_*`, `R_DEAD_ZONE_*` 死区校准宏

### Must NOT Have (Guardrails)
- **不**包含 `pid_t` 类型定义（属于 pid.h 配套模块，不在 scope）
- **不**声明 `euler_angle`（属于 euler.h 模块）
- **不**声明 `foc_set_duty()`（属于 foc.h 模块）
- **不**声明 `get_timer()`（属于 timer_control.h 模块）
- **不**声明 `servo_control_table()`（属于 servo_kinematics.h 模块）
- **不**修改 control.c（include guard 已由 `#include "zf_common_headfile.h"` 覆盖，无需改）
- **不**使用大写 include guard（用 `_control_h_`，跟随 servo.h 风格）
- **不**在 .h 中定义变量（仅 extern，定义保留在 .c）

---

## Verification Strategy

> 这是一个纯头文件任务，无运行时测试。验证方式为静态分析 + IAR 编译尝试。

### Test Decision
- **Infrastructure exists**: NO（无测试框架）
- **Automated tests**: None — 头文件验证
- **QA Policy**: Agent 通过编译尝试 + grep 符号覆盖审计验证

---

## Execution Strategy

### Parallel Execution Waves
单任务项目，无并行。

```
Wave 1 (唯一任务):
└── Task 1: 创建 project/code/control.h
```

### Dependency Matrix
- **1**: - - F1-F4（最终审查依赖 Task 1）

---

## TODOs

> **A task WITHOUT QA Scenarios is INCOMPLETE. No exceptions.**

- [ ] 1. 创建 `project/code/control.h` — 完整的控制模块头文件

  **What to do**:

  ### 1.1 文件骨架
  - 创建 `project/code/control.h`
  - Include guard: `#ifndef _control_h_` / `#define _control_h_`
  - Include: `#include "zf_common_headfile.h"` 和 `#include "servo.h"` (需要 SERVO 引脚和 `steer_control_struct`)
  - Copyright 块：`/* */` 格式，文件名为 `control`，备注 "轮腿机器人 LQR+PID 控制模块"

  ### 1.2 引脚 / 外设定义（文件顶部，中文分段注释）
  按以下结构组织，使用 `//=====...=====` 分隔：

  **舵机引脚定义**：
  ```
  // 舵机通道定义（与 servo.h 中的 STEER_1~4 对应）
  #define SERVO_1    (STEER_1_PWM)     // 左腿上舵机 → TCPWM_CH10_P05_0
  #define SERVO_2    (STEER_2_PWM)     // 左腿下舵机 → TCPWM_CH10_P05_1
  #define SERVO_3    (STEER_3_PWM)     // 右腿上舵机 → TCPWM_CH10_P05_2
  #define SERVO_4    (STEER_4_PWM)     // 右腿下舵机 → TCPWM_CH10_P05_3
  
  #define SERVO1_MID (STEER_1_CENTER)  // 左腿上舵机中位值 = 1000
  #define SERVO2_MID (STEER_2_CENTER)  // 左腿下舵机中位值 = 1000
  #define SERVO3_MID (STEER_3_CENTER)  // 右腿上舵机中位值 = 1000
  #define SERVO4_MID (STEER_4_CENTER)  // 右腿下舵机中位值 = 1000
  ```

  **定时器通道定义**（值待确认，用 0 占位 + TODO 注释）：
  ```
  // 定时器通道定义 [TODO: 确认 TOM0 实例对应的实际通道号]
  #define TOM0_CH0    (0)   // 腿高控制定时器
  #define TOM0_CH1    (0)   // 速度环定时器
  #define TOM0_CH2    (0)   // 角度环定时器
  #define TOM0_CH3    (0)   // 角速度环定时器
  #define TOM0_CH4    (0)   // 转向环定时器
  ```

  **死区校准参数**（值待标定，用 0 占位）：
  ```
  // 死区校准参数 [TODO: 实测标定]
  #define L_DEAD_ZONE_CORRECT    (0)     // 左轮正转死区补偿
  #define L_DEAD_ZONE_NEGATIVE   (0)     // 左轮反转死区补偿
  #define R_DEAD_ZONE_CORRECT    (0)     // 右轮正转死区补偿
  #define R_DEAD_ZONE_NEGATIVE   (0)     // 右轮反转死区补偿
  ```

  **通用数学宏**：
  ```
  // 通用数学宏
  #define DEG_TO_RAD   (57.29577951308232f)    // 180/π，作为除数使用将角度转为弧度
  #define ABS(x)       (((x) >= 0) ? (x) : -(x))
  #define clip2(x, max) (((x) > (max)) ? (max) : ((x) < -(max)) ? -(max) : (x))
  ```

  ### 1.3 结构体定义
  - `jump_control_struct` typedef（含 `_struct` 后缀）:
    ```c
    typedef struct {
        int min;               // 时间范围下限
        int max;               // 时间范围上限
        void (*handler)(int);  // 控制回调函数(参数为步骤编号)
        const char *name;      // 阶段名称
    } jump_control_struct;
    ```
  - 注释：`// 跳跃控制阶段配置结构体`

  ### 1.4 extern 声明（control.c 中定义的文件作用域变量）

  **PID 实例**（7个）：
  ```c
  extern pid_t leg_hight;      // 腿高 PID
  extern pid_t turn_angle;     // 转向角度 PID
  extern pid_t turn_gyro;      // 转向角速度 PID
  extern pid_t gyro;           // 角速度 PID
  extern pid_t angle;          // 角度 PID
  extern pid_t speed;          // 速度 PID
  extern pid_t turn;           // 转向 PID
  ```

  **校准/配置值**（6个 float）：
  ```c
  extern float angle_kd;       // 角度环 KD 值
  extern float pitch_mid;      // pitch 机械中值
  extern float roll_mid;       // roll 机械中值
  extern float KP;             // 转向 P 参数
  extern float KPP;            // 转向 PP 参数
  extern float KD;             // 转向 D 参数
  extern float KDD;            // 转向 DD 参数
  ```

  **PID 时间步长**（7个 float）：
  ```c
  extern float dt_pid_gyro;        // 角速度环时间步长
  extern float dt_pid_angle;       // 角度环时间步长
  extern float dt_pid_speed;       // 速度环时间步长
  extern float dt_pid_turn;        // 转向环时间步长
  extern float dt_leg;             // 腿高环时间步长
  extern float dt_pid_turn_angle;  // 转向角度环时间步长
  extern float dt_pid_turn_gyro;   // 转向角速度环时间步长
  ```

  **运行状态变量**（5个）：
  ```c
  extern float leg_long;       // 当前腿长
  extern float set_speed;      // 设定速度
  extern float turn_out;       // 转向输出值
  extern uint8 jump_flag;      // 跳跃标志位
  extern uint8 speed_flag;     // 速度标志位
  ```

  **常量**（4个）：
  ```c
  extern const float LQR_K[8];                         // LQR 增益矩阵
  extern const jump_control_struct jump_control_config[]; // 跳跃控制序列
  extern const uint8 jump_step_num;                     // 跳跃步数
  extern const float Lmoto_K;                           // 左电机力矩系数
  extern const float Rmoto_K;                           // 右电机力矩系数
  ```

  **外部模块变量**（来自未创建的配套模块 — 标注 TODO）：
  ```c
  extern small_device_value_struct motor_value;          // 电机驱动数据 [TODO: 需在 motor_control.c 中定义]
  extern float image_error;                              // 图像误差 [TODO: 需在 vision 模块中定义]
  extern float mid_point;                                // 视觉中点 [TODO: 需在 vision 模块中定义]
  extern float v_hat;                                    // 速度估计值 [TODO: 需在 estimator 模块中定义]
  extern float x_hat;                                    // 位置估计值 [TODO: 需在 estimator 模块中定义]
  ```

  ### 1.5 函数原型（10个）
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

  ### 1.6 结尾
  - `#endif` 闭合 include guard

  **Must NOT do**:
  - 不要定义 `pid_t`、`euler_angle`、`foc_set_duty()`、`get_timer()`、`servo_control_table()` — 这些属于配套模块
  - 不要在 .h 中定义变量（只 extern，定义保留在 .c）
  - 不要使用大写 include guard 风格
  - 不要修改 control.c

  **Recommended Agent Profile**:
  > 这是一个纯 C 头文件创建任务，需要精确的符号映射和代码惯例遵循。
  - **Category**: `quick` — 单文件创建，无复杂逻辑
    - Reason: 头文件骨架 + 已知符号表 + 既有模式参考
  - **Skills**: `[]`
    - 无需特殊技能 — 标准 C 头文件编写
  - **Skills Evaluated but Omitted**: N/A

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 1（唯一任务）
  - **Blocks**: F1, F2, F3, F4（最终审查依赖于本任务）
  - **Blocked By**: None（可立即开始）

  **References** (CRITICAL — Be Exhaustive):

  **Pattern References** (existing code to follow):
  - `project/code/servo.h:1-54` — 头文件模板：include guard 风格（`_servo_h_`），引脚 #define 顶部布局（中文分段注释），struct typedef 模式（`_struct` 后缀），extern 声明模式，函数原型 void 风格
  - `project/code/small_driver_uart_control.h:1-58` — `small_device_value_struct` 类型定义位置（motor_value 的类型来源），extern 声明模式

  **API/Type References** (contracts to implement against):
  - `project/code/control.c:1-408` — **所有声明的唯一真实来源**：逐行核对每个文件作用域变量名和类型、每个函数签名（参数类型和顺序）、jump_control_struct 的使用模式（.min, .max, .handler 字段访问）

  **External References**:
  - `project/code/servo.h:11-36` — STEER_1~4_PWM 和 STEER_1~4_CENTER 的精确值（SERVO_1~4 和 SERVO1_MID~4 映射来源）
  - `project/code/small_driver_uart_control.h:19-39` — `small_device_value_struct` 完整定义（验证 `.receive_left_speed_data`, `.receive_right_speed_data` 成员存在）

  **WHY Each Reference Matters**:
  - `servo.h` 是最直接的风格模板，同一目录下的现有头文件 —— 提供 include guard、中文注释、pin 定义布局的规范
  - `control.c` 是符号的唯一定义来源 —— 所有 extern 声明和函数原型必须与此文件精确匹配
  - `small_driver_uart_control.h` 是 motor_value 类型的唯一权威来源 —— 不查看此文件则无法确定 struct 成员

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY — task is INCOMPLETE without these)**:

  ```
  Scenario: control.h 语法自检 — double-include 不报错
    Tool: Bash (IAR 编译器)
    Preconditions: servo.h 和 zf_common_headfile.h 存在
    Steps:
      1. Create temp file: echo '#include "control.h"\n#include "control.h"\nint main(void){return 0;}' > /tmp/test_control.c
      2. Attempt compile with IAR: iarcc --cpu=Cortex-M7 /tmp/test_control.c (或使用 cc -fsyntax-only 快速检查)
      3. Check exit code = 0
    Expected Result: 编译成功，无 redefinition 错误
    Failure Indicators: "redefinition of ..." 错误；exit code ≠ 0
    Evidence: .sisyphus/evidence/task-1-double-include.txt (编译器输出)

  Scenario: 符号覆盖审计 — 所有 control.c 中的外部引用都能追溯到 include 链
    Tool: Bash (grep)
    Preconditions: control.h 已完成
    Steps:
      1. 提取 control.c 中所有非局部标识符: grep -oE '[a-zA-Z_][a-zA-Z0-9_]*' control.c | sort -u > /tmp/sym_used.txt
      2. 从 include 链中提取所有声明: grep -r 'extern\|#define\|typedef\|void \|float \|int16 \|uint8 ' control.h servo.h small_driver_uart_control.h zf_common_headfile.h zf_common_typedef.h | grep -oE '[a-zA-Z_][a-zA-Z0-9_]*' | sort -u > /tmp/sym_declared.txt
      3. diff /tmp/sym_used.txt /tmp/sym_declared.txt — 列出未覆盖的符号
    Expected Result: 未覆盖符号仅限于已知配套模块符号（pid_t, euler_angle, foc_set_duty, get_timer, servo_control_table, Position_pid, ABS, imu660ra_*, pwm_set_duty）
    Evidence: .sisyphus/evidence/task-1-symbol-audit.txt (diff 输出)

  Scenario: 负面测试 — 确认不在 scope 的符号未被错误声明
    Tool: Bash (grep)
    Steps:
      1. grep -n "pid_t\|euler_angle\|foc_set_duty\|get_timer\|servo_control_table" control.h
      2. 预期输出为空
    Expected Result: 零匹配 — 无 scope 越界声明
    Evidence: .sisyphus/evidence/task-1-scope-check.txt
  ```

  **Evidence to Capture**:
  - [ ] `.sisyphus/evidence/task-1-double-include.txt` — 编译器语法检查结果
  - [ ] `.sisyphus/evidence/task-1-symbol-audit.txt` — 符号覆盖 diff 输出
  - [ ] `.sisyphus/evidence/task-1-scope-check.txt` — scope 越界检查

  **Commit**: YES
  - Message: `feat(control): add control.h with servo/timer pin definitions and LQR+PID externs`
  - Files: `project/code/control.h`
  - Pre-commit: N/A (纯头文件，无运行时测试)

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE.

  - [x] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. Verify:
  - Must Have: 27 extern declarations present, 10 function prototypes present, jump_control_struct typedef present
  - Must NOT Have: No pid_t definition, no euler_angle extern, no foc_set_duty extern, no get_timer extern, no servo_control_table extern
  - Evidence files exist in `.sisyphus/evidence/`
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [1/1] | VERDICT: APPROVE/REJECT`

  - [x] F2. **Code Quality Review** — `unspecified-high`
  Run IAR build on CM7_0 project. Review control.h for:
  - Include guard correctness (`#ifndef _control_h_` ... `#endif`)
  - No `#pragma` warning suppression without documented reason
  - All macros properly parenthesized (clip2, ABS, DEG_TO_RAD)
  - Chinese-named sections consistent with servo.h style
  - Static `const float LQR_K[8]` properly declared as `extern const`
  Output: `Build [PASS/FAIL] | Lint [PASS/FAIL] | Style [N clean/N issues] | VERDICT`

  - [x] F3. **Real Manual QA** — `unspecified-high`
  Execute all QA scenarios from Task 1:
  - Scenario 1: double-include test
  - Scenario 2: symbol coverage audit
  - Scenario 3: scope boundary check
  Save evidence to `.sisyphus/evidence/final-qa/`.
  Output: `Scenarios [3/3 pass] | VERDICT`

  - [x] F4. **Scope Fidelity Check** — `deep`
  Verify:
  - control.h contains exactly the symbols declared in the plan (no missing, no extra)
  - No control.c modifications made (git diff shows only control.h added)
  - "Must NOT do" compliance confirmed
  Output: `Tasks [1/1 compliant] | Contamination [CLEAN] | Unaccounted [CLEAN] | VERDICT`

---

## Commit Strategy

- **1**: `feat(control): add control.h with servo/timer pin definitions and LQR+PID externs` — `project/code/control.h`

---

## Success Criteria

### Verification Commands
```bash
# 符号覆盖检查
cd project/code
grep -c "^extern" control.h  # Expected: >= 27 extern 声明
grep -c "^void\|^float\|^int16" control.h  # Expected: >= 10 函数原型

# 编译尝试
# IAR EWARM → 打开 project/iar/cyt4bb7.eww → F7
# 预期：control.h 无自身语法错误，但链接阶段会因配套模块缺失而失败（已知限制）
```

### Final Checklist
- [ ] Include guard: `#ifndef _control_h_` / `#define _control_h_`
- [ ] 27 extern 声明完备
- [ ] 10 函数原型完备
- [ ] SERVO_1~4, SERVO1_MID~4 宏定义正确映射到 servo.h
- [ ] TOM0_CH0~4 占位宏（含 TODO）
- [ ] clip2/DEG_TO_RAD/ABS 宏定义（参数已括号保护）
- [ ] 死区校准宏定义
- [ ] jump_control_struct typedef
- [ ] 未包含 pid_t/euler_angle/foc_set_duty/get_timer/servo_control_table

---

## Known Limitations（已知限制）

以下符号在 control.h 中**故意未声明**，因为它们属于未创建的配套模块。`control.c` 在以下模块就位之前**无法完整编译链接**：

| 所需符号 | 所需模块 | 当前状态 |
|----------|----------|----------|
| `pid_t`, `pid_init()`, `pid_run()`, `pid_set_target()`, `pid_get_observation()`, `pid_set_dt()`, `Position_pid` | `pid.h` | ❌ 不存在 |
| `euler_angle` (struct) | `euler.h` | ❌ 不存在 |
| `foc_set_duty()` | `foc.h` | ❌ 不存在 |
| `get_timer()`, `TOM0_CH*`（实际通道值） | `timer_control.h` | ❌ 不存在 |
| `servo_control_table()` | `servo_kinematics.h` | ❌ 不存在 |
| `imu660ra_get_gyro()`, `imu660ra_gyro_transition()`, `imu660ra_gyro_y`, `imu660ra_gyro_z` | IMU 库头文件 | 需确认路径 |

