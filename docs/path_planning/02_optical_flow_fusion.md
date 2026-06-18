# 模块 2：光流（PMW3901）与轮式里程计融合

> **文档信息**
> - 适用平台：Infineon Traveo II CYT4BB (Cortex-M7, 250MHz, float32 only)
> - 参考机型：两轮自平衡智能车（四轮独立转向, X 型悬挂）
> - 传感器：PMW3901 光流 (SPI_0, 2MHz) + 电机编码器 (UART2, 460800 bps)
> - 融合目标：互补滤波器（初级）+ 标量 EKF（进阶）→ 鲁棒 2D 位姿估计

---

## 目录

1. [PMW3901 硬件接口与现状](#1-pmw3901-硬件接口与现状)
2. [像素到距离的尺度转换](#2-像素到距离的尺度转换)
3. [现有轮式里程计分析](#3-现有轮式里程计分析)
4. [互补滤波器设计（推荐起点）](#4-互补滤波器设计推荐起点)
5. [标量 EKF 设计（进阶方案）](#5-标量-ekf-设计进阶方案)
6. [异常值处理策略](#6-异常值处理策略)
7. [标定流程](#7-标定流程)
8. [代码集成方案](#8-代码集成方案)

---

## 1. PMW3901 硬件接口与现状

### 1.1 驱动文件

| 文件 | 说明 |
|------|------|
| `libraries/zf_device/zf_device_pmw3901.h` | 寄存器定义、SPI 引脚、全局数据声明 |
| `libraries/zf_device/zf_device_pmw3901.c` | 初始化函数（~60 个寄存器配置）、运动读取函数 |

### 1.2 硬件连接

```c
// zf_device_pmw3901.h:52-59
#define PMW3901_SPI         (SPI_0)             // SPI 外设 0
#define PMW3901_SPI_SPEED   (2 * 1000 * 1000)   // 2 MHz
#define PMW3901_SCLK_PIN    (SPI0_CLK_P02_2)    // SCK  = P02_2
#define PMW3901_MOSI_PIN    (SPI0_MOSI_P02_1)   // MOSI = P02_1
#define PMW3901_MISO_PIN    (SPI0_MISO_P02_0)   // MISO = P02_0
#define PMW3901_CS_PIN      (P02_3)             // CS   = P02_3 (GPIO 手动控制)
#define PMW3901_CS(x)       ((x) ? (gpio_high(PMW3901_CS_PIN)) : (gpio_low(PMW3901_CS_PIN)))
```

SPI 模式 0 (CPOL=0, CPHA=0)。

### 1.3 关键 API

```c
// zf_device_pmw3901.c:159-283
uint8 pmw3901_init(void);
    // 返回 0 = 成功, 1 = 失败
    // 执行: SPI 初始化 → 上电复位 → 自检(读 PRODUCT_ID=0x49) →
    //       写入 ~60 个配置寄存器 → 验证(读 OBSERVATION=0xBF)

// zf_device_pmw3901.c:137-150
void pmw3901_get_motion(void);
    // 突发读取 6 字节 (从寄存器 0x16):
    //   buf[0]=MOTION, buf[1]=OBSERVATION,
    //   buf[2]=DELTA_X_L, buf[3]=DELTA_X_H,
    //   buf[4]=DELTA_Y_L, buf[5]=DELTA_Y_H
    //
    // 更新全局数据:
    //   pmw3901_delta_x   = (int16)((buf[3] << 8) | buf[2]);
    //   pmw3901_delta_y   = (int16)((buf[5] << 8) | buf[4]);
    //   pmw3901_delta_x_i += pmw3901_delta_x;   // 积分累加
    //   pmw3901_delta_y_i += pmw3901_delta_y;
```

### 1.4 全局数据

```c
// zf_device_pmw3901.h:80-82
extern int16 pmw3901_delta_x;      // 本次读取的 X 方向像素位移
extern int16 pmw3901_delta_y;      // 本次读取的 Y 方向像素位移
extern int32 pmw3901_delta_x_i;    // X 方向累积像素位移
extern int32 pmw3901_delta_y_i;    // Y 方向累积像素位移
```

### 1.5 当前集成状态：**未初始化**

`main_cm7_0.c:48-96` 的 `main()` 中没有 `pmw3901_init()` 调用。SPI_0 总线上目前没有活动的设备（WiFi SPI 也未启用）。

**其他 SPI 总线分配（无冲突）：**

| SPI 总线 | 设备 | 状态 |
|----------|------|------|
| SPI_0 | **PMW3901**（或 WiFi） | **空闲，可用** |
| SPI_1 | IPS200 显示屏 | 已初始化 |
| SPI_2 | IMU660RB IMU | 已初始化 |
| SPI_3 | — | 空闲 |

### 1.6 使用注意事项（来自驱动注释）

> 1. 调用周期至少 20ms
> 2. 两次读取间实际移动距离至少 5cm
> 3. 保证充足光照

**这意味着不能直接在 1ms PIT ISR 中调用** `pmw3901_get_motion()`。合适的调用位置：
- `pit_call_back()` 的 20ms 分支（`sys_times % 20 == 0`）
- 或 5ms 分支（`sys_times % 5 == 0`），此时满足 ≥20ms 条件

---

## 2. 像素到距离的尺度转换

### 2.1 三种标定方法

PMW3901 驱动返回的是**原始像素计数**，无任何物理单位转换。需要自行标定或使用经验公式。

#### 方法 A：几何模型（已知高度）

来自 [adityakamath/optical_flow_equation.py](https://gist.github.com/adityakamath/b88d36209b5ebf08d484bc9898362812)：

```c
// h    = 传感器距地面高度 (m)
// fov  = 42° = 0.733 rad (传感器视场角)
// res  = 有效分辨率 (约 30 px)
// scaler = 经验修正系数 (通常 ~5)

float conversion_factor_m_per_px = h * 2.0f * tanf(0.733f / 2.0f) / (30.0f * 5.0f);

float dist_x_m = conversion_factor_m_per_px * (float)pmw3901_delta_x;
float dist_y_m = conversion_factor_m_per_px * (float)pmw3901_delta_y;
```

**优点**：不需要实车标定，仅需高度参数。
**缺点**：精度依赖高度测量准确性和 scaler 经验值，地面纹理影响大。

#### 方法 B：弧度/像素（PX4 经验值）

来自 [PX4 PMW3901 驱动](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/optical_flow/pmw3901/PMW3901.cpp#L136)：

```c
// PX4 使用 ÷385 将像素转换为弧度
float flow_x_rad = (float)pmw3901_delta_x / 385.0f;
float flow_y_rad = (float)pmw3901_delta_y / 385.0f;

// 角分辨率转为线速度：乘以高度
float vel_x_mps = -flow_y_rad * height_m / dt;
float vel_y_mps =  flow_x_rad * height_m / dt;
```

来自 [Paparazzi 光流驱动](https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/modules/sensors/opticflow_pmw3901.c#L30-L34)：

```c
// Crazyflie 标定值: 0.002443389 rad/px
#define PMW3901_RAD_PER_PX  0.002443389f
```

#### 方法 C：实车标定（最推荐）

在地上精确标记 1m 直线距离，推动车行进，记录 PMW3901 的累积像素值：

```c
// 标定步骤:
// 1. 清零: pmw3901_delta_x_i = 0, pmw3901_delta_y_i = 0
// 2. 推车前进 1m（直行）
// 3. 记录累积值 total_px = pmw3901_delta_x_i
//
// 计算: px_per_cm = total_px / 100.0f
//       cm_per_px = 100.0f / total_px
```

**这是最准确的方法**，因为直接在你的地面表面上标定。

### 2.2 坐标系与转换

PMW3901 的 **物理安装方向** 决定了 X/Y 的含义。典型的安装方式（传感器朝下）：

```
       ┌──────────┐
       │ PMW3901  │  ← 传感器朝下
       └────┬─────┘
            │
       ┌────▼─────┐
       │ 镜头/孔  │
       └──────────┘
        地面表面

传感器 X → 机器人前方（前进方向）
传感器 Y → 机器人右侧
```

**速度转换（车体坐标系）**：

```c
// dt = 两次 pmw3901_get_motion() 之间的时间 (秒)
// cm_per_px = 标定值

// 车体前向速度 (cm/s): delta_y 的正方向 = 前进
float flow_vx_cmps =  pmw3901_delta_y * cm_per_px / dt;

// 车体侧向速度 (cm/s): delta_x 的正方向 = 右侧移动
float flow_vy_cmps = -pmw3901_delta_x * cm_per_px / dt;

// 对于两轮自平衡车，主要关注前向速度 (flow_vx)
```

**⚠️ 注意符号约定**：上述映射取决于 PMW3901 的安装方向。需要在实际硬件上验证：将车向前推，观察 `pmw3901_delta_x` 和 `pmw3901_delta_y` 的符号变化，然后确定正确的转换关系。

---

## 3. 现有轮式里程计分析

### 3.1 速度数据来源

电机编码器数据通过 **UART2 (460800 bps)** 从外部电机驱动器回传：

```c
// small_driver_uart_control.h:25-27
int16 receive_left_speed_data;   // 左轮 RPM
int16 receive_right_speed_data;  // 右轮 RPM

// uart_control_callback() (small_driver_uart_control.c:44-46) 解析 7 字节帧
// 帧格式: [0xA5, cmd, L_hi, L_lo, R_hi, R_lo, checksum]
```

### 3.2 速度计算

```c
// Body_ctrl.c:587 (每 20ms 更新)
car_speed = (motor_value.receive_left_speed_data
           - motor_value.receive_right_speed_data) / 2;
// car_speed 类型: int16, 单位: RPM

// Common_peripherals.c:128-137
Car.speed   = (left - right) / 2;   // float, RPM
Car.speed_L = left;                  // 左轮 RPM
Car.speed_R = -right;                // 右轮 RPM (符号翻转)
```

**注意**：右轮符号翻转是因为 `CYT2_D_motor_ctrl()` 中左轮取反（`small_driver_set_duty(-L_SPEED, R_SPEED)`），所以右轮反馈的符号需要翻转以得到正确的正值。

### 3.3 距离积分

```c
// Common_peripherals.c:144 (每 5ms 调用，通过 CYT2_get_distance())
Car.mileage += ((float)Car.speed / 60.0f * wheel_diameter * PI * 0.005f);
// 公式: (RPM / 60) × 直径(cm) × π × 0.005s = 5ms 内的行进距离(cm)

// Body_ctrl.c:524 (每 1ms 调用)
car_distance += ((float)car_speed / 60.0f * WHEEL_CIRCUMFERENCE * PI * 0.001f);
// 同样公式，但用 0.001s 乘数
```

**⚠️ 命名 bug**：`WHEEL_CIRCUMFERENCE = 6.4f`（实际是**直径**，不是周长）和 `wheel_diameter = 6.4f`（正确命名为直径）。两者值相同（6.4 cm），但 `WHEEL_CIRCUMFERENCE` 的名字有误导性。

**实际周长** = π × 6.4 = 20.1 cm。

### 3.4 里程计误差来源

| 误差源 | 量级 | 影响 |
|--------|------|------|
| 轮径标定不准 | ±3-5% | 系统性尺度误差 |
| 轮子打滑（加速/减速时） | ±10-30% 瞬时 | 短距离漂移 |
| 侧滑（转弯时） | ±5-15% | 航向估计误差 |
| 编码器分辨率 | ±1 RPM | 低速死区 |
| 外部电机驱动 UART 延迟 | ~0.5-1ms | 反馈滞后 |

**光流的核心价值**：不受轮子打滑影响，可以独立验证轮式里程计的瞬时精度。

---

## 4. 互补滤波器设计（推荐起点）

### 4.1 设计原理

互补滤波器利用了两种传感器的**频域互补特性**：

| 传感器 | 优势 | 劣势 |
|--------|------|------|
| PMW3901 光流 | 高频响应好，不受轮滑影响 | 低频漂移（积分噪声），受地面纹理影响 |
| 轮式里程计 | 长期稳定，无积分漂移 | 高频受轮滑影响，瞬时不准 |

**融合公式**（一阶互补滤波）：

```
v_fused = α × v_flow + (1 - α) × v_odom

其中 α ∈ [0, 1] 是信任权重：
  α → 1: 更信任光流
  α → 0: 更信任里程计
```

### 4.2 自适应权重

将 PMW3901 的表面质量（SQUAL，寄存器 0x07, 0-255）纳入权重计算：

```c
// 读取 SQUAL (可扩展到 pmw3901_get_motion 中)
// 当前驱动不读取 SQUAL → 需要修改 burst read 或单独读取

// 自适应 alpha: 高质量表面 → 更信任光流
float squal_norm = (float)squal / 255.0f;    // 归一化到 [0, 1]
float alpha = 0.90f * squal_norm;             // 最大 0.9, 最小 0
float beta  = 1.0f - alpha;                   // 里程计权重
```

**设计理由**：
- 光滑瓷砖（SQUAL≈50）：alpha ≈ 0.18 → 主要信任里程计
- 纹理地毯（SQUAL≈200）：alpha ≈ 0.70 → 主要信任光流
- 无纹理表面（SQUAL<10）：alpha ≈ 0 → 完全依赖里程计

### 4.3 完整 C 实现（兼容 float32 only）

```c
// ============================================================
// 互补滤波器: PMW3901 光流 + 轮式里程计
// 调用频率: 50Hz (每 20ms)
// 兼容性: 无矩阵运算, float32 only, 无外部库依赖
// ============================================================

// 标定参数 (通过实车标定确定)
static float cm_per_px    = 0.0f;   // 像素 → cm 转换系数
static float px_per_cm    = 0.0f;   // cm → 像素转换系数 (可选)

// 滤波器状态
static float fused_vx_cmps   = 0.0f;   // 融合后车体前向速度 (cm/s)
static float fused_vy_cmps   = 0.0f;   // 融合后车体侧向速度 (cm/s)
static float fused_x_cm      = 0.0f;   // 融合后全局 X 位置 (cm)
static float fused_y_cm      = 0.0f;   // 融合后全局 Y 位置 (cm)

// 光流信任权重 (动态)
static float alpha_flow      = 0.5f;   // [0, 1], 默认 50%

// ============================================================
// 主融合函数 — 在每个控制周期调用
// ============================================================
void sensor_fusion_update(float dt_s,      // 距上次调用的时间 (s)
                          float height_cm)  // 传感器距地面高度 (cm)
{
    // ---- 步骤 1: 光流 → 速度 (cm/s) ----
    // 需要高度信息。如果没有高度传感器，使用固定高度值
    float flow_vx = 0.0f, flow_vy = 0.0f;

    if (cm_per_px > 0.0f && dt_s > 0.001f)
    {
        // 直接使用标定的 cm_per_px
        flow_vx = (float)pmw3901_delta_y * cm_per_px / dt_s;
        flow_vy = (float)pmw3901_delta_x * cm_per_px / dt_s * (-1.0f);

        // 如果已知高度，可引入高度校正:
        // float height_factor = height_cm / NOMINAL_HEIGHT_CM;
        // flow_vx *= height_factor;
        // flow_vy *= height_factor;
    }

    // ---- 步骤 2: 轮式里程计 → 速度 (cm/s) ----
    // 现有代码中 data: Car.speed (RPM)
    float odom_vx = (float)Car.speed / 60.0f * wheel_diameter * PI;
    // Car.speed 是 RPM → 除以 60 得到 RPS → × 轮径 × π = cm/s

    // 侧向速度: 两轮差速车通常为 0 (忽略侧滑)
    float odom_vy = 0.0f;

    // ---- 步骤 3: 自适应权重 ----
    // 读取 SQUAL (需要修改 pmw3901_get_motion 来返回 squal)
    // 暂时使用固定或简单启发式:
    float squal_norm = 0.5f;  // 默认 0.5
    // TODO: 从 PMW3901 读取实际 SQUAL

    // 异常检测: 如果光流值过大(传感器故障/强光干扰), 降低权重
    if (func_abs(pmw3901_delta_x) > 200 || func_abs(pmw3901_delta_y) > 200)
    {
        squal_norm = 0.0f;   // 拒信光流
    }

    alpha_flow = 0.90f * squal_norm;

    // ---- 步骤 4: 互补融合 ----
    float beta_odom = 1.0f - alpha_flow;

    fused_vx_cmps = alpha_flow * flow_vx + beta_odom * odom_vx;
    fused_vy_cmps = alpha_flow * flow_vy + beta_odom * odom_vy;

    // ---- 步骤 5: 死推算 (Dead Reckoning) ----
    // 使用当前航向角 (来自 IMU 四元数)
    float yaw_rad = roll_balance_cascade.posture_value.yaw * (PI / 180.0f);
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    // 将车体系速度转换到世界系
    float vx_world = fused_vx_cmps * cos_yaw - fused_vy_cmps * sin_yaw;
    float vy_world = fused_vx_cmps * sin_yaw + fused_vy_cmps * cos_yaw;

    // 积分位置
    fused_x_cm += vx_world * dt_s;
    fused_y_cm += vy_world * dt_s;

    // ============================================================
    // 与现有系统同步: 将融合后的位置写入 Car 结构或全局变量
    // ============================================================
    // 可选: 将 Car.mileage (纯里程计) 与 fused 进行比较
    // float odom_mileage = Car.mileage;  // 用于调试对比
}
```

### 4.4 融合位置与纯里程计的对比

互补滤波器产出的 `fused_x_cm` 和 `fused_y_cm` 可以直接替代或补充现有的 `car_distance`（纯里程计开环积分），用于位置环 PID 的反馈输入。

**对比测试方法**：
1. 同时记录 `fused_x_cm` 和 `car_distance`
2. 让车走一段已知距离（如 200cm）
3. 比较两者与实际距离的误差
4. 在有轮滑的场景（加速起步、急停）中，融合值应明显优于纯里程计

---

## 5. 标量 EKF 设计（进阶方案）

互补滤波器在大多数场景下足够。EKF 适用于以下场景：
- 需要协方差估计（用于下游规划的不确定性传播）
- 融合 3+ 个传感器（如加入磁力计或 GNSS）
- 需要正式的异常值检验（马氏距离门控）

### 5.1 状态向量

```c
// 5 状态: [x, y, θ, v, ω]
// x, y: 全局位置 (cm)
// θ:   航向角 (rad)
// v:   车体前向速度 (cm/s)
// ω:   偏航角速度 (rad/s)
```

### 5.2 预测步骤（差速运动模型）

```c
void ekf_predict(float dt)
{
    float s = sinf(x[2]), c = cosf(x[2]);

    // 状态传播
    x[0] += x[3] * c * dt;   // px
    x[1] += x[3] * s * dt;   // py
    x[2] += x[4] * dt;       // theta

    // 稀疏雅可比 P = F * P * F^T
    // F 的稀疏部分: ∂px/∂θ = -v*sin(θ)*dt = Ja
    //               ∂px/∂v =  cos(θ)*dt    = Jb
    //               ∂py/∂θ =  v*cos(θ)*dt  = Jc
    //               ∂py/∂v =  sin(θ)*dt    = Jd
    float Ja = -x[3] * s * dt;
    float Jb =  c * dt;
    float Jc =  x[3] * c * dt;
    float Jd =  s * dt;

    // 5×5 稀疏矩阵乘法 (P = F·P·F^T)
    // 对于嵌入式应用，全展开比通用矩阵乘法快 10-50×
    // ... (见 FouadRobotics/embedded-ekf-cortex-m0 的实现参考)

    // 添加过程噪声 Q
    P[0][0] += Q_pos  * dt;  P[1][1] += Q_pos  * dt;
    P[2][2] += Q_head * dt;  P[3][3] += Q_vel  * dt;
    P[4][4] += Q_omg  * dt;
}
```

### 5.3 标量测量更新

EKF 的核心优势：当每个测量只更新状态的一个分量时，**不需要矩阵求逆**。这称为标量更新（scalar update）。

来自 [FouadRobotics/embedded-ekf-cortex-m0](https://github.com/FouadRobotics/embedded-ekf-cortex-m0) 的核心思想（在 Cortex-M0+ 上仅需 ~50µs）：

```c
// 标量更新: 当测量 z 只涉及状态 x[k] 时
// H = e_k^T (第 k 个标准基向量)
//
// 创新协方差 (1×1): S = P[k][k] + R
// 卡尔曼增益:        K[i] = P[i][k] / S   (无矩阵求逆!)
// 状态更新:          x[i] += K[i] * (z - x[k])
// 协方差更新:        P -= K * (P 的第 k 行)

static void ekf_update_scalar(int k, float z, float R)
{
    float S = P[k][k] + R;       // 创新协方差 (标量)
    if (S <= 0.0f) return;

    // 马氏距离门控: 如果 innovation² / S > gate², 拒绝测量
    float innovation = z - x[k];
    if (innovation * innovation > MAHA_GATE_SQ * S) return;

    float inv_S = 1.0f / S;

    // 计算卡尔曼增益 (5×1 向量)
    float K[5];
    for (int i = 0; i < 5; i++) K[i] = P[i][k] * inv_S;

    // 状态更新
    for (int i = 0; i < 5; i++) x[i] += K[i] * innovation;

    // 协方差更新: P = P - K * row_k
    for (int i = 0; i < 5; i++) {
        float ki = K[i];
        for (int j = 0; j < 5; j++) P[i][j] -= ki * P[k][j];
    }
}
```

### 5.4 测量源映射

| 测量 | 对应状态 | 测量噪声 R | 来源 |
|------|---------|-----------|------|
| 光流前向速度 | `x[3]` (v) | 大 (10-50) | `pmw3901_delta_y` → cm/s |
| 里程计前向速度 | `x[3]` (v) | 中 (5-20) | `Car.speed` → cm/s |
| IMU 偏航角速度 | `x[4]` (ω) | 小 (0.1-1) | `imu660rb_gyro_z` → rad/s |
| IMU 偏航角(姿态) | `x[2]` (θ) | 小 (0.05-0.2) | `posture_value.yaw` → rad |

**内存预算**：
- 状态向量 `x[5]`：5 × 4 = 20 bytes
- 协方差矩阵 `P[5][5]`：25 × 4 = 100 bytes
- 临时存储：~40 bytes
- **总计**：~160 bytes（远小于 100KB RAM 预算）

### 5.5 互补滤波 vs EKF 选择

| 特性 | 互补滤波 | 标量 EKF |
|------|---------|---------|
| 代码复杂度 | ~50 行 | ~200 行 |
| RAM | ~20 bytes | ~160 bytes |
| CPU (每周期) | ~30 cycles | ~600 cycles |
| 协方差估计 | ❌ | ✅ |
| 异常值门控 | 手动 if | ✅ 马氏距离 |
| 多传感器(>2) | 需要嵌套 | ✅ 统一框架 |
| 推荐场景 | **初始实现** | 精度要求高，3+ 传感器 |

**推荐路径：先实现互补滤波器，验证光流可用性，再评估是否需要 EKF。**

---

## 6. 异常值处理策略

### 6.1 光流 SQUAL 阈值

PMW3901 的 SQUAL 寄存器 (0x07) 报告表面跟踪质量 (0-255)：

```c
#define SQUAL_MIN_ACCEPTABLE  20    // 低于此值 → 拒信光流

if (squal < SQUAL_MIN_ACCEPTABLE) {
    alpha_flow = 0.0f;              // 完全信任里程计
}
```

**注意**：当前驱动 `pmw3901_get_motion()` 不读取 SQUAL。需要修改驱动，在 burst read 中增加 SQUAL 字段，或单独读取寄存器 0x07。

### 6.2 光流幅值异常检测

PX4 驱动的做法：如果单次读取的位移像素超过 240，判定为数据异常：

```c
#define FLOW_DELTA_MAX  240   // 单次读取的像素上限

if (func_abs(pmw3901_delta_x) > FLOW_DELTA_MAX ||
    func_abs(pmw3901_delta_y) > FLOW_DELTA_MAX) {
    // 可能是传感器故障、强光干扰或运动过快
    alpha_flow = 0.0f;
}
```

### 6.3 光流-里程计不一致检测

当光流和里程计报告的速度差异很大时，很可能有一方出错：

```c
#define FLOW_ODOM_MAX_DIFF  0.5f   // m/s, 超过此差异 → 怀疑光流

float flow_speed_mps = fabsf(flow_vx_cmps) * 0.01f;
float odom_speed_mps = fabsf((float)Car.speed / 60.0f * wheel_diameter * PI * 0.01f);

if (fabsf(flow_speed_mps - odom_speed_mps) > FLOW_ODOM_MAX_DIFF) {
    // 速度差异过大
    // 如果是轮滑场景(加速时里程计偏高) → 信任光流
    // 如果是无纹理地面(光流失效) → 信任里程计
    //
    // 简单策略: 检查 SQUAL
    if (squal > 100) {
        alpha_flow = 0.8f;  // 表面好 → 可能是轮滑, 信光流
    } else {
        alpha_flow = 0.0f;  // 表面差 → 信里程计
    }
}
```

### 6.4 高度有效性检查

如果没有高度传感器，使用固定的标称高度。如果有 ToF (DL1A/DL1B)：

```c
#define HEIGHT_MIN_CM  8.0f    // PMW3901 最小工作距离 80mm
#define HEIGHT_MAX_CM  200.0f  // 实用上限

if (height_cm < HEIGHT_MIN_CM || height_cm > HEIGHT_MAX_CM) {
    alpha_flow = 0.0f;   // 超出传感器有效范围
}
```

### 6.5 光流长期不更新检测

如果 PMW3901 通信中断或传感器挂死：

```c
static uint32 last_flow_update_ms = 0;

// 每次成功读取 PMW3901 后:
last_flow_update_ms = sys_times;

// 在融合前检查:
if ((sys_times - last_flow_update_ms) > 200) {   // 200ms 无更新
    alpha_flow = 0.0f;   // 退化为纯里程计
}
```

---

## 7. 标定流程

### 7.1 像素-厘米转换因子标定（方法 C）

**器材**：卷尺/直尺，平坦地面，标记笔

**步骤**：

1. 将车放在起点，确保直线行进
2. 清零光流积分：`pmw3901_delta_x_i = 0; pmw3901_delta_y_i = 0;`
3. 开始以 **~20Hz** 调用 `pmw3901_get_motion()`
4. 推动车**匀速直行**前进 100cm
5. 停止读取，记录 `pmw3901_delta_y_i` (Y 向累积像素)
6. 计算：`cm_per_px = 100.0f / (float)pmw3901_delta_y_i`
7. **重复 5 次，取平均值**
8. 同样的方法标定侧向：`cm_per_px_lateral = 100.0f / (float)pmw3901_delta_x_i`

**记录表**：

| 次数 | 行进距离 | delta_y_i | cm_per_px |
|------|---------|-----------|-----------|
| 1 | 100cm | ____ | ____ |
| 2 | 100cm | ____ | ____ |
| 3 | 100cm | ____ | ____ |
| 4 | 100cm | ____ | ____ |
| 5 | 100cm | ____ | ____ |
| **平均** | | | ____ |

**在不同地面上分别标定**：
- 光滑瓷砖
- 短毛地毯
- 木地板

如果不同地面的 cm_per_px 差异 > 10%，需要在融合时根据 SQUAL 动态调整。

### 7.2 轮式里程计标定（轮径修正）

使用 UMBmark 方法的简化版：

1. 在地面上标记 2m 直线距离
2. 推车前进 2m，记录 `Car.mileage` 的变化值
3. 正确的轮径 = 当前轮径 × (200cm / Car.mileage变化值)

```c
// 标定结果示例
#define CALIBRATED_WHEEL_DIAMETER  6.52f   // 修正后的轮径 (cm)
```

### 7.3 融合验证测试

在标定完成后，执行以下验证：

1. **直线测试**：前进 200cm，对比 `fused_x_cm` 与地面真值
2. **回程测试**：前进 100cm → 后退 100cm，检查位置是否归零
3. **打滑测试**：在光滑地面上急加速 50cm，对比融合值与纯里程计（融合值应显著更准）
4. **转弯测试**：走一个 1m×1m 正方形，检查回到起点的位置误差

---

## 8. 代码集成方案

### 8.1 修改文件清单

| 文件 | 修改内容 |
|------|---------|
| `project/user/main_cm7_0.c` | 在 `main()` 中添加 `pmw3901_init()` |
| `project/code/Body_ctrl.c` | 在 `pit_call_back()` 20ms 分支中添加融合调用 |
| `project/code/Imu.h` | 新增融合数据结构（如 `fused_odom_struct`）|
| `libraries/zf_device/zf_device_pmw3901.c` | 可选：修改 `pmw3901_get_motion()` 增加 SQUAL 读取 |
| `project/code/Common_peripherals.h` | 新增 `WHEEL_DIAMETER` 统一命名 |
| **新建** `project/code/Sensor_fusion.c` | 互补滤波器 / EKF 实现 |
| **新建** `project/code/Sensor_fusion.h` | 融合 API 声明 |

### 8.2 `main()` 中的初始化

```c
// main_cm7_0.c 约第 60 行，在 imu660rb_init() 之后:
pmw3901_init();          // 新增：初始化光流传感器
```

### 8.3 `pit_call_back()` 中的调用位置

```c
// Body_ctrl.c pit_call_back(), 20ms 分支 (约第 584 行):
if (sys_times % 20 == 0)
{
    // 读取光流 (≥20ms 周期)
    pmw3901_get_motion();

    // 执行传感器融合
    sensor_fusion_update(0.02f, NOMINAL_HEIGHT_CM);

    // 更新 car_speed 和 速度环 PID (原有代码)
    car_speed = ...;
    pid_control(&roll_balance_cascade.speed_cycle, ...);
}
```

### 8.4 与位置环 PID 对接

融合后的位置 `fused_x_cm` 直接作为位置环 PID 的反馈输入：

```c
position_current_cm = fused_x_cm;   // 替代原来的 car_distance
pid_control(&roll_balance_cascade.position_cycle,
            position_target_cm, position_current_cm);
```

---

## 附录 A：相关文件索引

| 文件 | 关键内容 |
|------|---------|
| `libraries/zf_device/zf_device_pmw3901.h` | PMW3901 寄存器定义、SPI 引脚、全局数据 |
| `libraries/zf_device/zf_device_pmw3901.c` | `pmw3901_init()`, `pmw3901_get_motion()` |
| `project/code/small_driver_uart_control.c/h` | 电机 UART 协议, `receive_left/right_speed_data` |
| `project/code/Common_peripherals.c:128-148` | `CYT2_get_speed()`, `CYT2_get_distance()` |
| `project/code/Common_peripherals.h:69` | `wheel_diameter = 6.4f` |
| `project/code/Body_ctrl.c:524` | `car_distance` 累加 |
| `project/code/Body_ctrl.c:587` | `car_speed` 更新 |
| `project/code/Body_ctrl.c:556-606` | `pit_call_back()` 控制心跳 |
| `project/user/main_cm7_0.c:48-96` | `main()` 初始化序列 |
| `libraries/zf_device/zf_device_dl1a.h` | ToF 距离传感器 (可选,用于高度测量) |

## 附录 B：外部参考

| 资源 | 说明 |
|------|------|
| [PX4 PMW3901 驱动](https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/optical_flow/pmw3901/PMW3901.cpp) | 生产级 SPI 驱动, ÷385 缩放, 质量门控 |
| [Paparazzi PMW3901 驱动](https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/modules/sensors/opticflow_pmw3901.c) | 角速率去旋转, RAD_PER_PX=0.002443389 |
| [PX4 EKF 光流融合](https://github.com/PX4/ecl/blob/master/EKF/optflow_fusion.cpp) | 24 状态 EKF, 创新门控, 质量加权噪声 |
| [embedded-ekf-cortex-m0](https://github.com/FouadRobotics/embedded-ekf-cortex-m0) | 标量测量 EKF, M0+ 上 50µs, 无矩阵库 |
| [adityakamath 光流标定](https://gist.github.com/adityakamath/b88d36209b5ebf08d484bc9898362812) | Python→C 可移植的转换因子公式 |
| [UMBmark 论文](https://johnloomis.org/ece445/topics/odometry/borenstein/paper60.pdf) | 差速驱动机器人里程计标定的金标准 |

## 附录 C：SQUAL 扩展读取的驱动修改

当前 `pmw3901_get_motion()` 突发读取 6 字节 (从 0x16)，这个突发读取中 `buf[1]` 已经是 OBSERVATION 寄存器（包含 SQUAL 的高 3 位）。要获得完整 8 位 SQUAL，需要额外读取寄存器 0x07：

```c
// 在 pmw3901_get_motion() 末尾添加:
uint8 squal_raw;
PMW3901_CS(0);
spi_read_8bit_registers(PMW3901_SPI, PMW3901_SQUAL, &squal_raw, 1);
PMW3901_CS(1);

// 暴露给外部
extern uint8 pmw3901_squal;
pmw3901_squal = squal_raw;
```
