# 模块 1：位置环 PID 标定

> **文档信息**
> - 适用平台：Infineon Traveo II CYT4BB (Cortex-M7, 250MHz, float32 only)
> - 参考机型：两轮自平衡智能车（四轮独立转向, X 型悬挂）
> - 现有级联：角速度环(1ms) → 角度环(5ms) → 速度环(20ms)
> - 新增层级：**位置环**（级联最外层，20-50Hz）

---

## 目录

1. [现有 PID 基础设施](#1-现有-pid-基础设施)
2. [现有增益表与约束](#2-现有增益表与约束)
3. [级联调参方法论](#3-级联调参方法论)
4. [位置环 PID 设计与插桩](#4-位置环-pid-设计与插桩)
5. [标定流程](#5-标定流程)
6. [故障模式与对策](#6-故障模式与对策)
7. [运行时调参方案](#7-运行时调参方案)
8. [相关常数修正](#8-相关常数修正)

---

## 1. 现有 PID 基础设施

### 1.1 控制级联结构

当前系统在 `pit_call_back()`（1ms PIT ISR，`Body_ctrl.c:556-606`）中运行三级级联 PID：

```
target_speed (全局 extern int16)
       │
       ▼
  ┌─ speed_cycle PID ─────────── 20ms, P=5.0, I=0, D=0
  │    │ 反馈: car_speed (来自电机编码器 RPM)
  │    │ 输出: 期望角度偏移
  │    ▼
  ┌─ angle_cycle PID ──────────── 5ms, P=700, I=1.0, D=50, 内层
  │    │ target = -mechanical_zero = +6.0°
  │    │ 反馈: -posture_value.pit
  │    │ 输出: 期望角速度
  │    ▼
  ┌─ angular_speed_cycle PID ─── 1ms, P=1.1, I=0, D=0, 最内层
  │    │ 反馈: imu660rb_gyro_y
  │    │ 输出: 电机 duty
  │    ▼
  ┌─ motor clamp (±3000) + gyro Z differential
       │
       ▼
  CYT2_D_motor_ctrl() → UART → 外部电机驱动器
```

**关键发现：没有位置闭环。** `car_distance` 在 `Body_ctrl.c:524` 做开环积分，但从未反馈给任何 PID。这是要填补的空缺。

### 1.2 PID 数据结构

```c
// Imu.h:66-78
typedef struct {
    float p;                          // 比例增益
    float i;                          // 积分增益
    float d;                          // 微分增益
    float p_value_last;               // 上一次误差（用于微分）
    float i_value;                    // 积分累加器
    float i_value_pro;                // 积分累加速度系数 (0-1)
    float i_value_max;                // 积分抗饱和上限
    float out;                        // 控制器输出
    float out_max;                    // 输出饱和值
    float incremental_data[2];        // 增量式 PID 历史
} pid_cycle_struct;

// Imu.h:80-101
typedef struct {
    quaternion_module quaternion;                    // 四元数引擎
    cascade_common_value_struct posture_value;       // rol/pit/yaw, mechanical_zero
    pid_cycle_struct angular_speed_cycle;            // 角速度环
    pid_cycle_struct angle_cycle;                    // 角度环
    pid_cycle_struct speed_cycle;                    // 速度环
    pid_cycle_struct track_cycle;                    // 循迹环（已存在，未充分利用）
} cascade_value_struct;
```

### 1.3 pid_control() 实现细节

`Imu.c:254-273` — 位置式 PID，含积分钳位：

```c
void pid_control(pid_cycle_struct *pid_cycle, float target, float real)
{
    float proportion_value   = target - real;
    float differential_value = proportion_value - pid_cycle->p_value_last;

    pid_cycle->i_value += (proportion_value * pid_cycle->i_value_pro);
    pid_cycle->i_value  = func_limit_ab(pid_cycle->i_value,
                            -pid_cycle->i_value_max, pid_cycle->i_value_max);

    pid_cycle->out = (pid_cycle->p * proportion_value
                    + pid_cycle->i * pid_cycle->i_value
                    + pid_cycle->d * differential_value);
    pid_cycle->out = func_limit_ab(pid_cycle->out,
                            -pid_cycle->out_max, pid_cycle->out_max);

    pid_cycle->p_value_last = proportion_value;
}
```

**需注意的设计特征：**
- 微分基于**误差**（不是基于测量值）→ 目标突变时产生"微分冲击"
- 抗饱和方式：**条件钳位**（不包含反算）→ 对位置环的大行程运动，需强化
- `i_value_pro` 不是 Ki——而是**积分累加速度系数**（0-1），最终积分贡献 = `i × i_value`

### 1.4 软启动机制

`Body_ctrl.c:206-221` — `car_state_calculate()` 在 500ms 内将 P 增益从 20% 线性升至 100%：

```c
if (sys_times < 500) {
    float ramp = 0.2f + (float)sys_times / 500.0f * 0.8f;
    roll_balance_cascade.angle_cycle.p *= ramp;
    roll_balance_cascade.speed_cycle.p *= ramp;
    roll_balance_cascade.angle_cycle.i_value = 0;   // 禁止积分
}
```

位置环应同此规律：**加入软启动，且对应接入 `sys_times` 触发复位**。

---

## 2. 现有增益表与约束

### 2.1 前向（Roll）级联——**唯一活跃方向**

| 回路 | P | I | D | i_value_max | i_value_pro | out_max | 频率 |
|------|---|---|---|:-----------:|:-----------:|:-------:|:----:|
| 角速度环 | 1.1 | 0 | 0 | 1000 | 0.1 | 10000 | 1ms |
| 角度环 | 700 | 1.0 | 50 | 1000 | 2.0 | 10000 | 5ms |
| 速度环 | 5.0 | 0 | 0 | 500 | 0.005 | 2000 | 20ms |
| 循迹环 | 10 | 0 | 0 | — | — | — | 20ms (仅 fuxian=1) |

**定义位置**：`Imu.c:335-417`，`balance_cascade_init()`。

### 2.2 物理约束

| 约束 | 值 | 来源 | 含义 |
|------|-----|------|------|
| mechanical_zero | -6.0° | Imu.c:339 | 平衡所需的前倾角 |
| 电机 duty 上限 | ±3000 (30% full-scale) | Common_peripherals.h:31-32 | 正常平衡模式的力矩约束 |
| 安全倾斜范围 | ~±15-20°（tilt guard at 45°） | Body_ctrl.c:186 | 位置环输出必须在安全倾斜范围内 |
| 轮径 | 6.4 cm | Common_peripherals.h:69 | ⚠️ 名称 bug：`WHEEL_CIRCUMFERENCE` 实际是直径 |
| 导航采样间隔 | 5 cm | Flash.h:27 (Nag_Set_mileage) | 路径记录的最小空间分辨率 |
| 速度反馈更新率 | 50 Hz (20ms) | Body_ctrl.c:587 | 位置环不能快过此频率 |

### 2.3 跳台状态下的 PID 增益缩放

| 阶段 | PID 缩放系数 | 来源 |
|------|:----------:|------|
| PREPARE | 0.6 | Body_ctrl.c:230 |
| CHARGE | 0.4 | Body_ctrl.c:234 |
| LAUNCH | 0.2 | Body_ctrl.c:238 |
| AIRBORNE | 0.15 | Body_ctrl.c:241 |
| LANDING | 0.3 | Body_ctrl.c:244 |
| RECOVER | 0.3→1.0 线性恢复 | Body_ctrl.c:249-253 |

**位置环在跳台时必须暂停或大幅降权**——跳台期间 `STOP_FALG=0` 会切断电机输出。

---

## 3. 级联调参方法论

### 3.1 核心原则：从内到外逐级调参

**这是硬性规则。** 级联 PID 中，内环是外环的"执行器"——内环不稳，外环无从调起。

```
调参顺序：角速度环 → 角度环 → 速度环 → 位置环（最后）
```

### 3.2 为什么不用 Ziegler-Nichols

Z-N 在机器人上表现很差。来自实际机器人调参数据（Shubham Kulkarni 巡线车）：

| 方法 | Kp | Ki | Kd | 表现 |
|------|----|----|-----|------|
| Z-N 公式 | 10.2 | **68** | 0.38 | 轻微漂移 |
| 手动微调 | 10 | **0.05** | 28 | ✅ 平稳 |

Z-N 算出的 Ki 高了 **1300 倍**。原因：
- Z-N 假设一阶主导滞后 → 自平衡是二阶系统
- Z-N 不包含前馈项
- 机器人 PID 的增益范围与工业过程完全不同

### 3.3 推荐的手动调参流程

**使用 SeekFree PC Assistant 的 8 通道示波器 (`seekfree_assistant_oscilloscope_send()`)** 记录每一步。

**已调好的内环保持不变**——只调位置环：

```
Step 1: 设 Ki_pos=0, Kd_pos=0。逐步增加 Kp_pos，直到车开始向目标位置移动，无过冲。
        → 起始值建议 Kp_pos = 0.5~1.0

Step 2: 如果存在稳态误差（车停在离目标点 1-2cm 处不动），缓慢增加 Ki_pos。
        → 起始值建议 Ki_pos = 0.01 × Kp_pos

Step 3: 位置外环通常不需要 Kd（位置是速度的积分，本身就有阻尼）。
        → 如果有多余过冲：1. 降低 Kp_pos；2. 或者微小的 Kd_pos

Step 4: 在实际负载、不同地面条件下重复测试。
```

### 3.4 关键约束规则

1. **位置环输出必须硬钳位到安全倾斜角**（±5°~±8°）——这是自平衡车的红线
2. **外环频率 5-10× 低于内环**：速度环 50Hz → 位置环 20-50Hz 合适
3. **大行程时使用增益调度**：远方激进，近处保守
4. **绝对不要在角度环加 I 项**——平衡前倾时角度误差始终存在，积分会累积到失控

---

## 4. 位置环 PID 设计与插桩

### 4.1 架构设计

位置环 PID 位于现有三级级联的**最外层**，输出作为 `target_speed` 的贡献项：

```
position_error = target_pos_cm - current_pos_cm
       │
       ▼
  [Position PID]          ← 新增，20-50 Hz
  Kp=0.5~2.0, Ki=0.01~0.1, Kd=0
  out_max = ±5.0°         ← 安全倾斜限幅
       │
       ▼ desired_tilt_offset
  [Angle PID]              ← 现有，5ms
       │
       ▼
  [AngularSpeed PID]       ← 现有，1ms
       │
       ▼
  [Speed PID]              ← 现有，20ms
```

**设计决策：**
- 位置 PID 的输出是**额外倾斜角**，加到 `angle_cycle` 的目标上（叠加到 `-mechanical_zero`）
- **不是**替代速度环——速度环照常运行，位置环输出只是改变了"期望倾斜角"
- 这样位置控制和速度控制可以同时存在（复合控制）

### 4.2 代码插桩点

#### 4.2.1 新增数据结构 (`Imu.h`)

在 `cascade_value_struct` 中添加 `position_cycle`（参照现有四个 PID cycle）：

```c
typedef struct {
    // ... 现有字段 ...
    pid_cycle_struct position_cycle;   // 新增：位置环 PID
} cascade_value_struct;
```

新增全局变量（`Imu.h` / `Body_ctrl.h`）：

```c
extern float position_target_cm;   // 目标位置（cm）
extern float position_current_cm;  // 当前位置（cm）
extern int   position_control_enable;  // 位置控制使能
```

#### 4.2.2 初始化 (`Imu.c` balance_cascade_init())

参照现有四个 PID 的初始化风格，在函数末尾添加：

```c
// 位置环 PID 初始值
roll_balance_cascade.position_cycle.p            = 0.8f;
roll_balance_cascade.position_cycle.i            = 0.02f;
roll_balance_cascade.position_cycle.d            = 0.0f;
roll_balance_cascade.position_cycle.i_value_max  = 100.0f;   // 积分上限
roll_balance_cascade.position_cycle.i_value_pro  = 0.01f;    // 积分累加速度
roll_balance_cascade.position_cycle.out_max      = 5.0f;     // 输出上限：5°倾斜角
```

#### 4.2.3 控制计算 (`Body_ctrl.c` pit_call_back())

在 20ms 分支（`sys_times % 20 == 0`）中，**速度环 PID 之前**插入：

```c
// ==== 20ms 分支 ====
if (sys_times % 20 == 0)
{
    // (现有代码: 更新 car_speed)
    car_speed = (motor_value.receive_left_speed_data
               - motor_value.receive_right_speed_data) / 2;

    // ---- 新增: 位置环 ----
    if (position_control_enable)
    {
        pid_control(&roll_balance_cascade.position_cycle,
                    position_target_cm,
                    position_current_cm);

        // 位置环输出叠加到角度环目标
        // 原始 target = -mechanical_zero, 叠加位置输出
        float pos_offset = roll_balance_cascade.position_cycle.out;
        // 限制在安全范围内
        pos_offset = func_limit_ab(pos_offset, -5.0f, 5.0f);
        // 写入速度目标（正偏移=前倾=前进）
        target_speed = (int16)(pos_offset * 100.0f);  // 比例因子需实车标定
    }

    // (现有代码: 速度环 PID)
    pid_control(&roll_balance_cascade.speed_cycle,
                (float)target_speed, (float)car_speed);
    // ...
}
```

**注意**：`pos_offset * 100.0f` 中的 100.0 是一个**待标定的比例因子**，将倾斜角度映射到 `target_speed` 的范围（50-700）。这个值必须通过实验确定。

#### 4.2.4 软启动集成 (`Body_ctrl.c` car_state_calculate())

在现有的 500ms 软启动代码中，同样处理位置环：

```c
if (sys_times < 500)
{
    float ramp = 0.2f + (float)sys_times / 500.0f * 0.8f;
    roll_balance_cascade.position_cycle.p *= ramp;
    roll_balance_cascade.position_cycle.i_value = 0;
}
```

#### 4.2.5 Menu UI 集成

参照现有 Menu.c 的路径记录/回放功能，在 page 2 增加：

- 一个 "Position Ctrl: ON/OFF" 切换项（设置 `position_control_enable`）
- 一个 "Set Target: ±XXX cm" 数值输入项（设置 `position_target_cm`）
- 一个实时位置显示项

---

## 5. 标定流程

### 5.1 前置条件检查

- [ ] 机械零点 (`mechanical_zero`) 正确。将车放在水平面上，读取 `posture_value.pit`，其负值即为 mechanical_zero
- [ ] 平衡稳定：直立放置时不抖动、不漂移
- [ ] 速度环已标定：给定 `target_speed=200`，车应匀速前进，速度稳定
- [ ] 里程计准确：推动车前进 1m，`car_distance` 读数应接近 100 cm
- [ ] 轮径常数修正（见第 8 节）

### 5.2 标定步骤

#### 步骤 0：接通 SeekFree PC Assistant

通过 UART 连接 PC，8 通道示波器实时记录以下信号：

```
通道 1: position_target_cm      （目标位置 cm）
通道 2: position_current_cm     （当前位置 cm，= car_distance）
通道 3: position_cycle.out      （位置 PID 输出，期望倾斜角°）
通道 4: posture_value.pit       （实际倾斜角°）
通道 5: target_speed            （期望速度 RPM）
通道 6: car_speed               （实际速度 RPM）
```

周期：50Hz（每 20ms 发送一次）。

#### 步骤 1：P 增益标定（Kp_pos）

```
目标：找到使车开始移动但不出现过冲的最大 P 值

1. Ki_pos = 0, Kd_pos = 0
2. Kp_pos = 0.3
3. 设 position_target_cm = 50.0（前进 50cm）
4. 启动，观察：
   - 车是否向目标移动？
   - 到达后是否过冲超过 5cm？
   - 稳态停车位置距目标多远？
5. 如果不动 → Kp_pos += 0.2，重复
6. 如果过冲 > 5cm → Kp_pos -= 0.1，重复
7. 记录最后的 Kp_pos 值
```

**预期范围**：Kp_pos = 0.5 ~ 2.0

**判据**：
- 车应能到达目标位置 ± 3cm 以内
- 无振荡（倾斜角不应来回摆动）
- 允许有 1-2cm 稳态误差（下一步用 I 消除）

#### 步骤 2：I 增益标定（Ki_pos）

```
目标：消除稳态误差，不引起振荡

1. 保持步骤 1 确定的 Kp_pos
2. Ki_pos = 0.01 * Kp_pos
3. 设 position_target_cm = 50.0，重复 3 次，观察：
   - 稳态误差是否减小？
   - 到达目标后是否来回振荡？
4. 如果稳态误差仍然存在 → Ki_pos *= 1.5
5. 如果开始振荡 → Ki_pos *= 0.5
6. 记录最终的 Ki_pos
```

**预期范围**：Ki_pos = 0.005 ~ 0.05

**关键警告**：Ki_pos 过大是位置环最常见的故障来源。对一个行进 1m 的任务，位置误差在大部分时间是最大值（1m），如果积分累加速度过快，到达目标后积分会"过充"，导致大幅过冲。

#### 步骤 3：大行程测试

```
目标：验证长途行进的稳定性

1. position_target_cm = 200.0（2m 前进）
2. 观察全过程：
   - 加速度是否平缓？（倾斜角不应突变）
   - 匀速段速度是否稳定？
   - 减速/停车是否平稳？（不应有急刹车式的后仰）
3. 如果加速度阶段车身抖动 → 降低 Kp_pos，或加入 S 曲线速度规划
4. 如果减速阶段后仰过大 → 降低 Kp_pos，或降低 out_max
```

#### 步骤 4：抗扰动测试

```
目标：验证鲁棒性

1. 车向 50cm 目标行进时，轻推车体
2. 观察车是否能恢复并继续向目标移动
3. 停车后，轻推车体使其偏离目标 10-20cm
4. 观察车是否能重新回到目标位置
```

#### 步骤 5：实地面测试

在不同地面上重复步骤 3-4：
- 光滑瓷砖（低摩擦）
- 短毛地毯（中摩擦）
- 如果场地有坡面，测试上坡/下坡表现

### 5.3 标定结果记录模板

```
=== 位置环 PID 标定记录 ===
日期：
操作人：
地面类型：

机械零点: mechanical_zero = ____°  (Imu.c:339)
轮径: WHEEL_DIAMETER = ____ cm     (建议重命名为 WHEEL_DIAMETER)

标定结果:
  Kp_pos = ____
  Ki_pos = ____
  Kd_pos = ____
  i_value_max  = ____
  i_value_pro  = ____
  out_max      = ____°  (安全倾斜角上限)
  角度→速度映射系数: ____

性能指标:
  50cm 阶跃:
    过冲: ____ cm (____%)
    稳态误差: ____ cm
    调节时间: ____ ms

  200cm 阶跃:
    过冲: ____ cm
    稳态误差: ____ cm
    匀速段速度: ____ RPM

  抗扰动恢复:
    10cm 扰动恢复时间: ____ ms
    20cm 扰动恢复时间: ____ ms
```

---

## 6. 故障模式与对策

### 故障 1：持续稳态误差（车停在目标前不动）

**原因**：纯 P 控制缺乏积分。摩擦力/齿轮间隙形成死区。

**对策**：增加 Ki_pos。起始值 = 0.01 × Kp_pos。

### 故障 2：位置过冲/振荡（车冲过目标再倒回来）

**原因**：Kp_pos 太高。位置环命令的倾斜角太大，内环来不及响应。

**对策**：
1. 降低 Kp_pos
2. 降低 out_max（例如从 5.0° 降到 3.0°）
3. 检查是否应加入 Kd_pos

### 故障 3：积分过冲（长途行进后大幅过冲）

**原因**：长途行进中位置误差一直很大，积分持续累积到巨大值。到达目标后，过大的积分推动车继续前进。

**对策**：
1. 降低 `i_value_pro`（积分累加速度）
2. 降低 `i_value_max`（积分上限）
3. 加入增益调度：远方激进，近处保守

### 故障 4：位置环导致平衡不稳

**原因**：位置环输出的倾斜角超出了车体能承受的稳定范围。

**对策**：
1. **硬钳位 out_max = 5.0°**（不要超过 8.0°，这是安全红线）
2. 确保 `position_cycle.out_max` 在 `balance_cascade_init()` 中正确设置
3. 在位置环输出后加低通滤波：`pos_offset_filtered = 0.8*pos_offset_filtered + 0.2*pos_offset`

### 故障 5：跳台后位置环行为异常

**原因**：跳台期间 `STOP_FALG=0` 切断电机，但位置环仍在后台运行，积分可能累积。

**对策**：
1. 在跳台 FSM 进入 JUMP_PREPARE 时，将位置环积分清零
2. 跳台期间禁用位置环（`position_control_enable = 0`）
3. 跳台 RECOVER 阶段结束后重新使能（配合软启动）

### 故障 6：编码器分辨率不足

**原因**：低速时编码器速度数据噪声大 → 里程计不准确 → 位置环无法精确定位。

**对策**：
1. 在 `position_error < 2cm` 时，冻结位置环输出（死区）
2. 或者：低于某速度阈值时，切换到纯 P 控制（禁用积分）

可以参考 [fastbot_advanced](https://github.com/search?q=fastbot_advanced+MIN_VALID_VEL) 的零速检测模式。

---

## 7. 运行时调参方案

### 7.1 SeekFree PC Assistant 调参通道

库层已提供 8 个调参通道（`seekfree_assistant.h:54`）：

```c
extern float  seekfree_assistant_parameter[8];
extern vuint8 seekfree_assistant_parameter_update_flag[8];
```

**PC 端可通过 UART 下发 float 值到这些通道，但目前没有任何代码读取它们。** 需要在 `pit_call_back()` 或 `car_state_calculate()` 中添加映射：

```
通道分配建议：
  通道 0: Kp_pos
  通道 1: Ki_pos
  通道 2: Kd_pos
  通道 3: position_cycle.out_max
  通道 4: position_cycle.i_value_max
  通道 5: position_target_cm
  通道 6: position_control_enable
  通道 7: (保留)
```

**实现方式**：在每个位置环 PID 计算前，检查 `update_flag[i]` 是否置 1，若是则更新对应参数并清零标志。

### 7.2 增益调度（自适应 PID）

针对长途 vs 短途的不同需求，使用距离分段增益：

```c
float dist_to_target = fabsf(position_target_cm - position_current_cm);

if (dist_to_target > 50.0f)
{
    // 远方：激进加速
    position_cycle.p = Kp_pos * 1.5f;
    position_cycle.i_value_pro = 0.005f;  // 慢积分
}
else if (dist_to_target > 10.0f)
{
    // 中距离：正常
    position_cycle.p = Kp_pos;
    position_cycle.i_value_pro = 0.01f;
}
else
{
    // 近处：保守，防过冲
    position_cycle.p = Kp_pos * 0.5f;
    position_cycle.i_value_pro = 0.02f;   // 快积分消除残差
    position_cycle.out_max = 2.0f;        // 降低倾斜限幅
}
```

---

## 8. 相关常数修正

### 8.1 轮径命名 bug 修复

当前代码中有两个同名不同义（但等值）的常数：

| 宏名 | 值 | 位置 | 实际含义 |
|------|-----|------|---------|
| `WHEEL_CIRCUMFERENCE` | 6.4f | Body_ctrl.c:3 | ⚠️ 实际是**直径**（不是周长） |
| `wheel_diameter` | 6.4f | Common_peripherals.h:69 | 正确命名：直径 |

两者在里程计算中的用法都是 `× π` 来得到真实周长：
```c
// Body_ctrl.c:524
car_distance += ((float)car_speed / 60.0f * WHEEL_CIRCUMFERENCE * PI * 0.001f);
// 实际含义: (RPM/60) * 直径 * π * 0.001 = 真实距离增量

// Common_peripherals.c:144
Car.mileage += ((float)Car.speed / 60.0f * wheel_diameter * PI * 0.005f);
// 同样: (RPM/60) * 直径 * π * 0.005
```

**建议统一**：将 `Body_ctrl.c:3` 的 `WHEEL_CIRCUMFERENCE` 重命名为 `WHEEL_DIAMETER`，并在 `Common_peripherals.h` 中统一定义。

### 8.2 轮径标定

理论轮径 6.4 cm 可能因胎压、磨损、负载而变化。实际标定方法：

1. 在地上标记 1m 精确距离
2. 推车前进 1m，记录 `car_distance` 的变化值
3. 修正系数 = 100.0 / (distance_end - distance_start)
4. 将修正系数应用到所有里程计算

---

## 附录 A：相关文件索引

| 文件 | 关键内容 |
|------|---------|
| `project/code/Imu.c:335-417` | `balance_cascade_init()` — 所有 PID 增益定义 |
| `project/code/Imu.c:254-273` | `pid_control()` — 位置式 PID 实现 |
| `project/code/Imu.c:282-300` | `pid_control_incremental()` — 增量式 PID（⚠️ 有 bug, 未使用）|
| `project/code/Imu.h:66-101` | PID 数据结构定义 |
| `project/code/Body_ctrl.c:556-606` | `pit_call_back()` — 1ms 控制心跳 |
| `project/code/Body_ctrl.c:184-267` | `car_state_calculate()` — 软启动 + 跳台 PID 管理 |
| `project/code/Body_ctrl.c:524` | `car_distance` 累加 |
| `project/code/Body_ctrl.c:587` | `car_speed` 更新 |
| `project/code/Common_peripherals.c:141-148` | `CYT2_get_distance()` — `Car.mileage` 累加 |
| `project/code/Common_peripherals.h:31-32` | M_MAX/M_MIN 电机限幅 |
| `project/code/Common_peripherals.h:69` | `wheel_diameter = 6.4f` |
| `project/code/Flash.h:27` | `Nag_Set_mileage = 5` (5cm 导航分辨率) |
| `libraries/zf_components/seekfree_assistant.h:54` | 8 通道 PC 调参接口 |
| `project/user/main_cm7_0.c:72-73` | PIT 定时器配置 (1ms + 5ms) |
