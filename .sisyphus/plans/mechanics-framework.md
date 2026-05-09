# 力学参数框架 — 四杆机构逆运动学参数化

## TL;DR

> **Quick Summary**: 将 `servo_kinematics.c` 的简化线性映射替换为带力学参数的四杆机构逆运动学计算骨架，参数值留 TODO 占位（由 MATLAB 仿真和物理学解算后填入）。
>
> **Deliverables**:
> - `servo_kinematics.h` — 新增机械参数宏（L1, L2, d, 角度范围, PWM比例）
> - `servo_kinematics.c` — 替换线性映射为余弦定理解算骨架
>
> **Estimated Effort**: Quick
> **Parallel Execution**: NO — 单任务
> **Critical Path**: Task 1

---

## Context

### Original Request
用户要求适配力学模块，将 `servo_control_table()` 从简化线性映射升级为带参数的四杆机构逆运动学计算。力学参数（杆长、角度范围等）留 TODO 占位，等 MATLAB 仿真和物理学解算后再填入。

### Interview Summary
- **深度**: 仅建力学参数框架（推荐起步）
- **参数来源**: MATLAB 仿真 + 物理学解算（不在本次 scope）
- **已有基础**: servo_kinematics.h/c 存在，SERVO_1~4/MID 已定义
- **机械结构**: 四条腿，每条腿由上舵机+下舵机驱动四杆机构

### Mechanical Model (from control.c)
- **左腿**: `SERVO_1 = MID + pwm_ph4`(上), `SERVO_2 = MID - pwm_ph1`(下)
- **右腿**: `SERVO_3 = MID - pwm_ph4`(上), `SERVO_4 = MID + pwm_ph1`(下)
- **输入**: `p` = 腿端伸缩量(mm), `angle` = 腿端偏转角(deg)
- **输出**: `pwm_ph1`, `pwm_ph4` = 舵机 PWM 偏移(±10000)
- **安全**: 任一输出达 ±10000 则中止（机械限位保护）

---

## Work Objectives

### Core Objective
将 `servo_control_table()` 从线性映射升级为带参数的四杆机构逆运动学计算骨架。

### Concrete Deliverables
- `servo_kinematics.h` — 新增 5 个力学参数宏 + 内部辅助函数声明
- `servo_kinematics.c` — 替换为余弦定理解算 + servo_angle_to_pwm 转换

### Definition of Done
- [ ] `servo_kinematics.h` 含 5 个力学参数 `#define`，均标注 `[TODO: MATLAB仿真后填入]`
- [ ] `servo_kinematics.c` 中 `servo_control_table()` 使用余弦定理计算舵机角度
- [ ] `servo_angle_to_pwm()` 辅助函数：角度 → PWM 偏移
- [ ] 所有参数在代码中引用（非硬编码），便于后期统一修改
- [ ] 输出范围验证：pwm_ph1/pwm_ph4 在 ±10000 内

### Must Have
- 杆长参数: L1(上连杆), L2(下连杆), d(舵机轴间距) — 单位 mm
- 角度范围: SERVO_ANGLE_MAX — 舵机最大机械角度(deg)
- PWM 比例: PWM_PER_DEG — 每度对应的 PWM 偏移量
- `servo_angle_to_pwm()` 函数：角度→PWM，含限幅

### Must NOT Have
- 不填入实际参数值（全部为 0 或 1 占位 + TODO）
- 不实现雅可比矩阵或动力学方程
- 不修改 control.c
- 不添加新的 .c/.h 文件（在现有文件中扩展）

---

## Verification Strategy

> 嵌入式无测试框架。验证: IAR 编译 + 输出范围检查。

### QA Policy
- 编译验证：servo_kinematics.c 在 IAR 中 0 error
- 功能验证：调用 `servo_control_table(5.5, 0, &p1, &p4)` 验证输出在 int16 范围内

---

## Execution Strategy

```
Wave 1 (唯一任务):
└── Task 1: 更新 servo_kinematics.h/c — 力学参数 + 余弦定理 IK 骨架
```

---

## TODOs

- [x] 1. 更新 `servo_kinematics.h` + `servo_kinematics.c` — 力学参数框架

  **What to do**:

  ### servo_kinematics.h 追加内容（在现有 SERVO 别名和死区常量之后，函数声明之前）

  ```c
  //===== 四杆机构力学参数 =====
  // [TODO: 以下参数由 MATLAB 仿真和物理学解算后填入实际值]

  #define L1_MM              (0)    // 上连杆长度 (mm)
  #define L2_MM              (0)    // 下连杆长度 (mm)
  #define D_MM               (0)    // 上下舵机轴间距 (mm)

  // 舵机参数
  #define SERVO_ANGLE_MAX    (0)    // 舵机最大机械角度 (deg) — 单向行程
  #define PWM_PER_DEG        (0)    // 每度对应的 PWM 偏移量 (PWM/deg)
  //   PWM_PER_DEG 计算参考: 舵机 1000us-2000us 对应 ±90°，PWM 范围 ±10000
  //   PWM_PER_DEG = (10000 / 90) ≈ 111.11 (待实测校准)

  // 内部辅助
  void servo_angle_to_pwm(float angle_deg, int16 *out_pwm);
  ```

  ### servo_kinematics.c 替换内容

  **servo_angle_to_pwm()** — 舵机角度 → PWM 偏移:
  ```c
  void servo_angle_to_pwm(float angle_deg, int16 *out_pwm) {
      float pwm_f = angle_deg * PWM_PER_DEG;
      *out_pwm = (int16)clip2((int16)pwm_f, 10000);
  }
  ```

  **servo_control_table()** — 四杆机构逆运动学:
  ```c
  void servo_control_table(float p, float angle, int16 *out_ph1, int16 *out_ph4) {
      // [TODO: 四杆机构逆运动学解算]
      // 输入: p = 腿端伸缩量(mm), angle = 腿端偏转角(deg)
      // 输出: out_ph1 = 上舵机 PWM, out_ph4 = 下舵机 PWM
      //
      // 机构参数（待 MATLAB 仿真后填入）:
      //   L1 = L1_MM (上连杆), L2 = L2_MM (下连杆), d = D_MM (轴距)
      //
      // 解算步骤（余弦定理）:
      //   1. 腿端点坐标: x = p * cos(angle), y = p * sin(angle)
      //   2. 上舵机→端点距离: R1 = sqrt(x² + (y - d/2)²)
      //   3. 下舵机→端点距离: R2 = sqrt(x² + (y + d/2)²)
      //   4. 余弦定理求角: 
      //      cos_theta1 = (L1² + R1² - L2²) / (2*L1*R1)
      //      cos_theta2 = (L2² + R2² - L1²) / (2*L2*R2)
      //   5. theta1 = acos(cos_theta1) * 180/PI (弧度转度)
      //      theta2 = acos(cos_theta2) * 180/PI

      float x, y, R1, R2, cos_theta1, cos_theta2;
      float theta1_deg, theta2_deg;

      // 步骤1: 腿端点坐标（世界坐标系）
      float angle_rad = angle / DEG_TO_RAD;       // 角度转弧度
      x = p * cosf(angle_rad);
      y = p * sinf(angle_rad);

      // 步骤2: 各舵机轴到端点的距离
      R1 = sqrtf(x*x + (y - D_MM/2.0f)*(y - D_MM/2.0f));
      R2 = sqrtf(x*x + (y + D_MM/2.0f)*(y + D_MM/2.0f));

      // 步骤3: 余弦定理求舵机转角
      cos_theta1 = (L1_MM*L1_MM + R1*R1 - L2_MM*L2_MM) / (2.0f * L1_MM * R1);
      cos_theta2 = (L2_MM*L2_MM + R2*R2 - L1_MM*L1_MM) / (2.0f * L2_MM * R2);

      // 步骤4: 限幅到 [-1, 1] 防 acos 定义域溢出
      if (cos_theta1 > 1.0f) cos_theta1 = 1.0f;
      if (cos_theta1 < -1.0f) cos_theta1 = -1.0f;
      if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
      if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;

      theta1_deg = acosf(cos_theta1) * 57.29577951308232f;  // rad→deg
      theta2_deg = acosf(cos_theta2) * 57.29577951308232f;

      // 步骤5: 角度限幅到机械范围
      theta1_deg = func_limit(theta1_deg, SERVO_ANGLE_MAX);
      theta2_deg = func_limit(theta2_deg, SERVO_ANGLE_MAX);

      // 步骤6: 角度 → PWM 偏移
      servo_angle_to_pwm(theta2_deg, out_ph1);  // 下舵机角度→PWM1
      servo_angle_to_pwm(theta1_deg, out_ph4);  // 上舵机角度→PWM4
  }
  ```

  **Must NOT do**:
  - 不填入实际参数值（L1/L2/d/SERVO_ANGLE_MAX/PWM_PER_DEG 全部 0 占位）
  - 不删除现有 SERVO 别名和死区常量
  - 不修改函数签名（保持 `void servo_control_table(float p, float angle, int16 *out_ph1, int16 *out_ph4)`）
  - 不引入新的 .h/.c 文件

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: `[]` — 标准 C 数学（cosf/sinf/sqrtf/acosf），无额外依赖

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 1（唯一任务）
  - **Blocks**: 无后续任务

  **References**:
  - `project/code/servo_kinematics.h:1-37` — 现有内容（保留并追加）
  - `project/code/servo_kinematics.c:1-32` — 现有内容（替换 servo_control_table）
  - `project/code/control.c:368-408` — 调用模式（left/right_leg_control 的输入输出约定）
  - `libraries/zf_common/zf_common_function.h` — `func_limit()` 宏供角度限幅使用
  - `libraries/zf_common/zf_common_headfile.h` — 通过 `#include "servo_kinematics.h"` 已间接引入 `math.h` (cosf/sinf/sqrtf/acosf 可用)

  **QA Scenarios**:
  ```
  Scenario: servo_control_table 输出范围验证
    Tool: Bash (IAR 编译)
    Steps:
      1. 编译 servo_kinematics.c — 验证 0 error
      2. 代码审查：确认 L1/L2/d 等参数全部为 `(0)` + TODO
      3. 代码审查：确认 cos_theta 限幅到 [-1,1] 防 acos 溢出
      4. 代码审查：确认输出使用 func_limit 和 clip2 限幅
    Expected Result: 编译通过，所有参数为 0 占位，acosh 安全限幅存在
    Evidence: .sisyphus/evidence/task-mechanics-compile.txt
  ```

  **Commit**: YES
  - Message: `feat(kinematics): replace linear mapping with four-bar IK skeleton and mechanical parameter framework`
  - Files: `project/code/servo_kinematics.h`, `project/code/servo_kinematics.c`

---

## Final Verification Wave

- [x] F1. **Code Review** — `deep`
  Verify:
  - L1_MM/L2_MM/D_MM/SERVO_ANGLE_MAX/PWM_PER_DEG 全部为 0 + TODO
  - cos_theta 限幅到 [-1,1] 存在
  - 角度输出使用 func_limit 限幅
  - 未删除现有 SERVO 别名和死区常量
  Output: `Params [PASS/FAIL] | Safety [PASS/FAIL] | Regression [PASS/FAIL] | VERDICT: APPROVE/REJECT`

---

## Commit Strategy

- **1**: `feat(kinematics): replace linear mapping with four-bar IK skeleton and mechanical parameter framework`

---

## Known Limitations

| 项目 | 状态 | 后续 |
|------|------|------|
| L1/L2/d 杆长 | 0 占位 | MATLAB 仿真后填入 |
| SERVO_ANGLE_MAX | 0 占位 | 舵机规格书或实测 |
| PWM_PER_DEG | 0 占位 | 舵机标定（10000/90≈111） |
| 死区校准值 | 0 占位 | 硬件标定 |
| 运动学模型 | 二连杆简化假设 | 实际四杆机构可能需要修正公式 |
| cosf/sinf/sqrtf | 需要 `#include <math.h>` | 通过 zf_common_headfile.h 间接引入 |
