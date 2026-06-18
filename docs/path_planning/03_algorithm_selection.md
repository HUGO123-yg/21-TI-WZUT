# 模块 3：路径规划算法选型

> **文档信息**
> - 适用平台：Infineon Traveo II CYT4BB (Cortex-M7, 250MHz, float32 only, ~100KB RAM)
> - 参考机型：两轮自平衡智能车（四轮独立转向, X 型悬挂, 差速驱动模型）
> - 竞争约束：无 OS, 无动态内存分配, 无矩阵库, 浮点仅单精度
> - 控制架构：1ms PIT ISR (姿态+平衡), 5ms ISR (按键), 超级循环 (Menu + 异步任务)

---

## 目录

1. [机器人运动学约束](#1-机器人运动学约束)
2. [算法对比矩阵](#2-算法对比矩阵)
3. [候选算法详细评估](#3-候选算法详细评估)
4. [推荐架构](#4-推荐架构)
5. [全局路径规划：A*](#5-全局路径规划a)
6. [局部路径跟随：Pure Pursuit](#6-局部路径跟随pure-pursuit)
7. [局部避障：Bug2 / VFH+](#7-局部避障bug2--vfh)
8. [速度规划：S 曲线](#8-速度规划s-曲线)
9. [与现有系统集成](#9-与现有系统集成)

---

## 1. 机器人运动学约束

### 1.1 驱动模型

这是一个**两轮差速驱动**的自平衡倒立摆，配有四轮独立转向伺服。

| 参数 | 值 | 说明 |
|------|-----|------|
| 驱动方式 | 差速驱动 (differential drive) | 不是阿克曼模型 |
| 控制输入 | v (前进速度), ω (偏航角速度) | 单轮模型 (unicycle) |
| 轮径 | 6.4 cm | 实际直径，⌀20.1 cm 周长 |
| 电机 duty 范围 | ±3000 (30% full-scale) | 正常平衡模式限幅 |
| 用户速度设定范围 | 50 - 700 | Flash.c:318-320 |

### 1.2 伺服转向约束

| 参数 | 值 | 说明 |
|------|-----|------|
| 伺服频率 | 300 Hz | PWM 驱动 |
| 中心位置 | steer1/2=4400, steer3=4700, steer4=4200 | duty 单位 |
| 最大转向速率 | ±10 duty/ms | 每 1ms 周期的增量限幅 |
| 全量程时间 | ~1 秒 | 从一端到另一端 |
| X 型悬挂 | 对角同步 | 跳台机构复用 |

**关键限制**：转向伺服**不能瞬时响应**。转向命令的带宽约为 10-20 Hz，快于这个频率的命令会被限幅器截断。因此路径规划生成的转向命令频率**不应超过 50 Hz**。

### 1.3 传感器反馈约束

| 传感器 | 更新率 | 延迟 | 精度 |
|--------|--------|------|------|
| IMU 姿态 (yaw/pitch/roll) | 1 kHz | ~12 µs (四元数) | 漂移 ~1°/min |
| 电机编码器速度 | 50 Hz (20ms) | ~1ms (UART) | ±1 RPM 分辨率 |
| 里程计距离 | 1 kHz 积分 | — | 受轮滑影响 |
| PMW3901 光流 | ≥20 ms 间隔 | ~0.5ms (SPI) | 需实车标定 |
| GNSS (可选) | 1-10 Hz | 大 | ⚠️ 与电机 UART 共享 UART_2 |

**关键限制**：速度反馈回路只有 **50 Hz**。路径跟随控制的频率不应超过这个更新率。

### 1.4 跳台中断

7 态跳台 FSM 在跳台期间（~500ms）会：
- 逐步降低 PID 增益到 0.15×（AIRBORNE 阶段）
- 在 LAUNCH 阶段电机加力（+500 boost）
- 伺服快照到跳台专用位置
- 设置 `STOP_FALG=0` 切断正常控制

**路径规划必须在跳台期间暂停或降权**，跳台结束后恢复。

### 1.5 计算预算

| 资源 | 预算 | 说明 |
|------|------|------|
| 1ms ISR 可用时间 | ~980 µs (98% idle) | 实际控制占用 ~19 µs |
| 超级循环可用时间 | ~980 µs / iteration | Menu() 阻塞式，空闲时可用 |
| RAM 总量 | ~100 KB | 平衡/IMU/Menu 等已占用部分 |
| 可供路径规划的 RAM | ~50-70 KB | 保守估计 |
| FPU | 单精度，1 cycle/乘加 | 250 MHz → 约 40M ops/s |

**关键发现**：计算能力远不是瓶颈（1ms ISR 仅用 2%）。真正的瓶颈是传感器反馈带宽（50Hz）和伺服转向速率（~10°/s 量级）。

---

## 2. 算法对比矩阵

### 2.1 全局规划

| 算法 | CPU/周期 | RAM | 完备性 | 最优性 | 适用性 |
|------|----------|-----|--------|--------|--------|
| **A\*** | 10-200ms (取决于网格大小) | 10-40 KB | ✅ 完备 | ✅ 最优 | ⭐ 推荐 |
| Dijkstra | > A\* (无启发式) | 同 A\* | ✅ | ✅ | A\* 启发式有效时不需要 |
| RRT | 5-50ms | 5-15 KB | 概率完备 | 次优 | 高维空间, 此处无优势 |
| Flood Fill | 增量更新, <1ms/步 | 2-10 KB | ✅ | 加权图中最优 | 迷宫场景, 不适用于空地 |

### 2.2 局部跟随

| 算法 | CPU/周期 | RAM | 平滑度 | 适用性 |
|------|----------|-----|--------|--------|
| **Pure Pursuit** | <0.5ms | ~200 B | 好 (四轮转向优势) | ⭐ 推荐 |
| Regulated Pure Pursuit | <0.5ms | ~200 B | 更好 (自适应速度) | ⭐ 推荐 |
| Stanley Controller | <0.5ms | ~200 B | 好 | 需要前轮转角模型 |
| MPC | 5-50ms | 300-400 KB (仅求解器) | 最好 | ❌ 内存不可行 |

### 2.3 局部避障

| 算法 | CPU/周期 | RAM | 适用性 |
|------|----------|-----|--------|
| **Bug2** | <0.5ms/步 | ~100 B | ⭐ 简单场景推荐 |
| **VFH+** | 1-5ms | 5-15 KB | ⭐ 复杂场景推荐 |
| DWA (最简版) | 5-15ms | 2-5 KB | 需调参, 可选 |
| DWA (完整版) | 10-30ms | 5-10 KB | ❌ 100Hz 不可行 |
| VFH\* | 5-20ms | 15-30 KB | A\* 前向搜索冗余 |

### 2.4 速度规划

| 算法 | CPU/周期 | 实现复杂度 | 平滑度 | 适用性 |
|------|----------|-----------|--------|--------|
| 梯形速度曲线 | <0.1ms | 简单 | 一般 (加速度突变) | 可用, 但会扰动平衡 |
| **S 曲线 (3 段)** | <0.2ms | 中等 | ⭐ 好 | ⭐ 推荐 |
| S 曲线 (7 段) | <0.3ms | 较复杂 | 最好 | 对自平衡车过度 |

---

## 3. 候选算法详细评估

### 3.1 Pure Pursuit — ⭐ 强烈推荐

**原理**：在参考路径上找一个前视点（lookahead point），用几何关系计算到达该点所需的曲率，映射到转向命令。

**复杂度**：O(1) 每周期。仅需一次路径搜索（找前视点）+ 一次反正切。

**计算量**：<0.5ms on Cortex-M7。

**对本车的特殊优势**：四轮独立转向可实现 **crab-walk（平移转向）** 模式，Pure Pursuit 的几何转向可以直接映射到伺服偏转角度，比差速驱动实现更精确。

**参数**：
- `lookahead_distance`：前视距离 (cm)，通常 20-50cm
  - 越大 → 路径平滑但跟踪精度低
  - 越小 → 跟踪紧密但可能振荡
- 自适应前视距离：`L = max(L_min, k × v)`，高速时看得更远

**局限**：
- 在曲率变化剧烈的路径（急弯）上会切角
- 纯几何方法不考虑动力学约束

### 3.2 A\* 栅格搜索 — ⭐ 强烈推荐

**原理**：在占据栅格上，用启发式函数 (通常曼哈顿距离或欧式距离) 引导的图搜索。

**复杂度**：O(N log N)，N = 探索的节点数。对于 100×100 栅格（5m×5m @ 5cm），典型探索 10-30% 的节点。

**计算量**：
| 网格 | 运行时 | RAM (典型) |
|------|--------|------------|
| 50×50 (2.5m×2.5m @ 5cm) | 5-30ms | ~6 KB |
| 100×100 (5m×5m @ 5cm) | 10-80ms | ~25 KB |
| 50×50 (5m×5m @ 10cm) | 3-15ms | ~3 KB |

**对本车的优化建议**：
1. 使用 **2 bits/cell** 的 Akers 编码存储占据栅格 → 100×100 = 2.5 KB
2. 使用 **位图** 追踪 open/closed 状态 → 额外 2.5 KB
3. A\* 运行在**超级循环**中（非 ISR），按需触发
4. 如果 RAM 吃紧，降采样到 10cm 分辨率 → 50×50 = 2500 cells → 总共 ~6 KB

**启发式选择**：
- 空地场景：欧式距离 `sqrt(dx² + dy²)`
- 迷宫/走廊：曼哈顿距离 `|dx| + |dy|`

### 3.3 DWA（动态窗口法）— ⚠️ 可选

**原理**：在速度空间 (v, ω) 中采样，对每个采样预测短时轨迹，用代价函数（目标对齐 + 避障 + 速度最大化）评分，选择最优速度对。

**计算量**：
- ROS 默认实现：600 iterations → **不可行**
- 优化版（20-30 速度对，10 步预测）：5-15ms → 50Hz 勉强可行
- 梯度下降 DWA（[hal.science](https://hal.science/hal-04041336/document)）：~1.33 iterations 平均 → **可行**

**对本车的适用性**：
- ✅ 直接输出速度命令 (v, ω)，与差速驱动模型完美匹配
- ✅ 考虑加速度/减速度约束
- ❌ 需要障碍物距离传感器（超声波/ToF/摄像头）
- ❌ 参数调优复杂（5 个权重 + 2 个分辨率 + 预测步数）

**决策**：仅在有可靠障碍物检测传感器、且需要动态避障时考虑。

### 3.4 Bug2 算法 — ⭐ 简单场景推荐

**原理**：沿起点→目标的 M-line 行进。遇障碍时，沿障碍边界绕行，直到重新碰到 M-line 的更靠近目标的位置。

**复杂度**：O(1) per step。仅需当前位置 + 目标方位 + 障碍物传感器读数。

**优势**：
- 计算几乎为零（<100 µs/步）
- 内存几乎为零（~100 bytes）
- 实现极简（~50 行 C 代码）

**对本车的适用性**：
- ✅ 适合静态障碍物且传感器覆盖 360° 的场景
- ⚠️ 严重依赖位置估计精度（光流融合可缓解）
- ❌ 绕障碍的路径不平滑，可能频繁前进/后退

### 3.5 VFH+ （向量场直方图）— ⭐ 复杂场景推荐

**原理**：将激光/声纳数据投影到极坐标直方图，找最接近目标方向的无障碍扇区。

**计算量**：窗口 33×33 时 ~2,400 次向量运算 → 1-3ms on Cortex-M7。

**对本车的优势**：
- 产出的航向角直接可用于转向命令
- 天然平滑（直方图低通滤波）
- 内存可控（10-15 KB for 33×33 window）

**局限**：
- 需要 2D 障碍物分布数据（栅格地图或直接传感器读数）
- 原始 VFH 不能处理 C 形陷阱（需要 VFH* 的 A* 前向搜索）

### 3.6 MPC（模型预测控制）— ❌ 不可行

**原因**：
- 求解器内存需求 300-400 KB（仅求解器本身，超过 100 KB 总预算）
- ACADO/HPMPC 在 Cortex-M3 (84MHz) 上的基准：N=5 horizon 需 300-900 µs，但**内存限制阻止了更大问题规模**
- 当前硬件平台的理论计算能力够，但 **RAM 完全不够**

---

## 4. 推荐架构

基于上述分析，推荐的路径规划系统分为三个层级：

```
┌──────────────────────────────────────────────────────────────────┐
│  Layer 3: 全局路径规划 (Super-loop, 按需异步)                      │
│                                                                  │
│   • A* 栅格搜索 (100×100 @ 5cm or 50×50 @ 10cm)                  │
│   • 触发条件: 新目标设定 / 路径被堵 / 地图更新                      │
│   • 输出: 全局路点序列 [(x1,y1), (x2,y2), ...]                     │
│   • 预算: 10-80ms, ~25KB RAM                                     │
└───────────────────────────┬──────────────────────────────────────┘
                            │ 路点序列
                            ▼
┌──────────────────────────────────────────────────────────────────┐
│  Layer 2: 局部路径跟随 (20ms PIT 分支, 确定性实时)                  │
│                                                                  │
│   • Pure Pursuit / RPP (前视点跟踪)                                │
│   • S 曲线速度规划                                                 │
│   • Bug2 / VFH+ 障碍物绕行 (可选, 需传感器)                         │
│   • 输出: target_speed, steer_command                             │
│   • 预算: <1ms, ~500B RAM                                        │
└───────────────────────────┬──────────────────────────────────────┘
                            │ target_speed, steer_command
                            ▼
┌──────────────────────────────────────────────────────────────────┐
│  Layer 1: 平衡+位置控制 (1ms PIT ISR, 硬实时)                      │
│                                                                  │
│   • 位置环 PID (20ms) → 期望倾斜角                                 │
│   • 角度环 PID (5ms) → 期望角速度                                  │
│   • 角速度环 PID (1ms) → 电机 duty                                │
│   • 现有代码, 不改动                                                │
└──────────────────────────────────────────────────────────────────┘
```

**为什么这样分层**：
- **Layer 1** 是硬实时安全关键层（平衡），不可阻塞
- **Layer 2** 是软实时跟随层，20ms 预算充足，卡在传感器带宽之内
- **Layer 3** 是尽力而为的规划层，在超级循环空闲中运行，不阻塞控制

---

## 5. 全局路径规划：A\*

### 5.1 占据栅格地图

**推荐的存储方案**：使用 Akers 编码（2 bits/cell）。

```c
// 100×100 grid × 2 bits = 2,500 bytes = 2.44 KB
// 0b00 = 未知, 0b01 = 空闲, 0b10 = 占据, 0b11 = (保留)

#define GRID_SIZE   100        // 5m × 5m @ 5cm = 100×100
#define CELL_CM     5.0f      // 每格 5cm
#define GRID_BYTES  ((GRID_SIZE * GRID_SIZE * 2 + 7) / 8)

static uint8 occupancy_grid[GRID_BYTES];

// 读写宏
#define CELL_GET(g, x, y)  (((g)[((y)*GRID_SIZE+(x))*2/8] >> (((y)*GRID_SIZE+(x))*2%8)) & 0x03)
#define CELL_SET(g, x, y, v) do { \
    int idx = ((y)*GRID_SIZE+(x))*2/8; \
    int bit = ((y)*GRID_SIZE+(x))*2%8; \
    (g)[idx] = ((g)[idx] & ~(0x03 << bit)) | ((v) << bit); \
} while(0)
```

### 5.2 地图构建

根据可用传感器，有三种建图策略：

**策略 A：预知地图**（最简单）
- 地图由 PC 端工具生成，通过 UART 或 Flash 加载
- 无需在线建图 → 零 CPU 开销

**策略 B：增量建图**（需要测距传感器）
- 用 ToF (DL1A/DL1B) 或超声波检测障碍物
- 每次传感器读取后更新对应栅格
- 参考 micromouse 的 Modified Flood Fill 方式：只更新已知区域的增量
- 预算：<100 µs per sensor reading

**策略 C：SLAM**（最复杂，通常不适用）
- 需要摄像头或激光雷达
- 对 Cortex-M7 单核 + 100KB RAM 不现实
- 不在本文档范围内

### 5.3 A\* 实现要点

```c
// 关键数据结构
typedef struct {
    int16 x, y;       // 栅格坐标
    int16 parent_x, parent_y;  // 父节点（路径回溯用）
    uint16 g_cost;    // 从起点到当前的代价
    uint16 f_cost;    // g_cost + h_cost (用于优先队列排序)
} astar_node_t;

// 优先队列: 用二叉堆实现 (O(log n) 插入/提取)
// 或者: 对于 100×100 格, 可以直接用数组扫描找最小 f (O(n) 但常数小)
```

**优化建议**：
1. **避免浮点**：欧式距离启发式用平方距离 `dx² + dy²`（整数运算，无需 sqrt），或直接用曼哈顿距离 `|dx| + |dy|`
2. **避免动态分配**：预分配固定大小的 open/closed 数组
3. **使用位图**追踪 open/closed 状态（每个节点 1 bit）
4. **路径平滑**：A\* 产出的路径是离散栅格的折线，需用线性插值或贝塞尔曲线平滑

### 5.4 路径平滑

A\* 的原始输出是栅格序列，需要平滑为适合 Pure Pursuit 跟踪的连续路径：

```c
// 简单方法: 共线点删减 + 分段线性插值
// 输入: raw_path[K] (栅格序列)
// 输出: smoothed_path[M], M ≤ K (路点序列, 世界坐标 cm)

void path_smooth(int16 raw[][2], int raw_len,
                 float smooth[][2], int *smooth_len)
{
    // 1. 共线过滤: 如果 A→B→C 三点共线, 删除 B
    // 2. 距离重采样: 以均匀间隔 (如 10cm) 重新采样路径
    // 3. 可选: 贝塞尔曲线平滑 (三次贝塞尔, 仅需 4 个控制点)
}
```

---

## 6. 局部路径跟随：Pure Pursuit

### 6.1 算法

```
输入: path[] (全局路点序列), robot_pose (x, y, yaw), lookahead_L
输出: target_curvature (曲率, 映射到转向命令)

步骤:
1. 找前视点: 从当前位置开始, 沿路径搜索距离 = lookahead_L 的点
2. 计算前视点在车体系中的坐标 (lx, ly)
3. 曲率 = 2 * lx / (lookahead_L²)
4. 转向角 = arctan(曲率 × 轴距)
```

**对本车的映射**：转向角 → 伺服 target_offset, 映射关系需通过实验标定。

### 6.2 自适应前视距离

```c
float compute_lookahead(float speed_cmps)
{
    float L_min = 15.0f;   // 最小前视距离 (cm)
    float L_max = 50.0f;   // 最大前视距离 (cm)
    float k     = 0.8f;    // 速度系数

    float L = k * speed_cmps;
    if (L < L_min) L = L_min;
    if (L > L_max) L = L_max;
    return L;
}
```

### 6.3 与现有 steering 的映射

由于四轮独立转向，Pure Pursuit 的曲率输出可以映射为：

- **Crab 模式**（推荐）：所有四轮同向偏转，车体保持航向，直接侧向平移
  - `steer_target_offset[0..3] = ±curvature_to_duty(curvature)`
- **差速模式**（备选）：保持伺服中位，用左右轮差速实现转向
  - `motor_differential = curvature_to_motor_diff(curvature)`

### 6.4 在现有 ISR 中的调用位置

```c
// Body_ctrl.c pit_call_back(), 20ms 分支:
if (sys_times % 20 == 0)
{
    // 更新里程计
    car_speed = ...;

    // ---- Pure Pursuit (新) ----
    if (path_follow_enable)
    {
        float curvature = pure_pursuit_step(
            global_path, &path_index,
            fused_x_cm, fused_y_cm, fused_yaw_rad,
            compute_lookahead(fused_vx_cmps));

        // 曲率 → 伺服偏移
        int16 steer_offset = (int16)(curvature * STEER_GAIN);
        // 写入全局转向变量 (被 car_steer_control 消费)
    }

    // ---- 速度环 PID (原有) ----
    target_speed = compute_s_curve_target(path_index, path_progress, max_speed);
    pid_control(&roll_balance_cascade.speed_cycle, target_speed, car_speed);
}
```

---

## 7. 局部避障：Bug2 / VFH+

### 7.1 Bug2（推荐作为起步方案）

**仅需传感器**：至少 3 个 ToF/超声波（前/左前/右前），或 1 个扫描式传感器。

**触发条件**：传感器检测到路径前方有障碍物，且 Pure Pursuit 的前视点在障碍物内。

```c
// Bug2 简化实现
typedef enum { BUG_M_LINE, BUG_WALL_FOLLOW } bug_state_t;

bug_state_t bug2_step(float robot_x, float robot_y,
                       float goal_x, float goal_y,
                       float *obstacle_distances, int n_sensors)
{
    // M-line: 当前位置 → 目标 的直线
    // 沿 M-line 朝向目标
    // 遇到障碍 → 切换为沿墙模式 (跟随左侧/右侧边界)
    // 碰到 M-line 且离目标更近 → 切换回 M-line 模式
}
```

**调用频率**：与 Pure Pursuit 同频（20-50 Hz）。

### 7.2 VFH+（推荐作为升级方案）

**需要传感器**：能够提供 2D 扇区障碍物信息的传感器（如 8 个环布的 ToF、或 360° 激光雷达、或从占据栅格中提取的局部窗口）。

**内存**：33×33 窗口的占据栅格 + 72 扇区直方图 ≈ 10-15 KB。

**计算**：
1. 从全局占据栅格提取 33×33 局部窗口
2. 对每个占据单元格计算障碍物向量（大小 = 1/d²）
3. 投影到 72 扇区极坐标直方图
4. 低通滤波直方图
5. 选择最接近目标方向且无障碍的扇区

**总预算**：~1-5 ms on Cortex-M7，完全在 20ms 窗口内。

---

## 8. 速度规划：S 曲线

### 8.1 为什么用 S 曲线

自平衡车对**加速度突变（jerk）极度敏感**：
- 梯形速度曲线的加速度阶跃会直接导致车身前后倾 → 角度环 PID 必须剧烈响应 → 可能失稳
- S 曲线的加速度变化率（jerk）有界且连续 → 车身倾斜平缓 → PID 在线性范围内工作

### 8.2 三段式 S 曲线

简化版（3 段，而非完整 7 段），更适合嵌入式实现：

```
阶段 1: 加速段 (S 型加速)
  加速度从 0 平滑上升至 a_max，再平滑降回 0
  v(t) = 三次样条插值

阶段 2: 匀速段
  v(t) = v_max

阶段 3: 减速段 (S 型减速)
  加速度从 0 平滑降至 -a_max，再平滑升回 0
```

### 8.3 实现框架

```c
typedef enum { PROFILE_ACCEL, PROFILE_CRUISE, PROFILE_DECEL, PROFILE_DONE } profile_phase_t;

typedef struct {
    profile_phase_t phase;

    float v_start;       // 起始速度 (cm/s)
    float v_max;         // 最大速度 (cm/s)
    float v_end;         // 终点速度 (cm/s, 通常为 0)
    float a_max;         // 最大加速度 (cm/s²)
    float j_max;         // 最大 jerk (cm/s³)

    float total_dist;    // 总路程 (cm)
    float traveled;      // 已走路程 (cm)
    float t_phase;       // 当前阶段内的时间 (s)

    // 预计算的阶段切换点
    float dist_accel;    // 加速段路程
    float dist_decel;    // 减速段路程
    float dist_cruise;   // 匀速段路程
    float T_accel;       // 加速段时间
    float T_decel;       // 减速段时间
} s_curve_profile_t;

// 每周期调用, 返回当前期望速度
float s_curve_update(s_curve_profile_t *p, float dt)
{
    switch (p->phase) {
        case PROFILE_ACCEL:
            // v(t) = v_start + j_max * t² / 2  (jerk 上升段)
            // 然后用三次多项式计算
            return compute_accel_phase_velocity(p, p->t_phase);
        case PROFILE_CRUISE:
            return p->v_max;
        case PROFILE_DECEL:
            return compute_decel_phase_velocity(p, p->t_phase);
        default:
            return 0.0f;
    }
}
```

**计算预算**：每 20ms 调用一次，每次 ~20 float ops → <1µs on Cortex-M7。

### 8.4 与速度环的对接

S 曲线输出的期望速度 `v_desired` 直接写入 `target_speed`：

```c
// 在 20ms 分支中
float v_desired_cmps = s_curve_update(&speed_profile, 0.02f);
target_speed = (int16)(v_desired_cmps / (PI * wheel_diameter) * 60.0f);
// cm/s → RPM: v / (π×D) × 60

pid_control(&roll_balance_cascade.speed_cycle, (float)target_speed, (float)car_speed);
```

---

## 9. 与现有系统集成

### 9.1 修改文件清单

| 文件 | 修改内容 |
|------|---------|
| **新建** `project/code/Path_planning.h` | 路径规划 API 声明、数据结构 |
| **新建** `project/code/Path_planning.c` | A\*, Pure Pursuit, Bug2, S 曲线实现 |
| **新建** `project/code/Occupancy_grid.h` | 占据栅格数据结构与读写宏 |
| **新建** `project/code/Occupancy_grid.c` | 栅格地图初始化和更新函数 |
| `project/code/Body_ctrl.c` | 在 `pit_call_back()` 20ms 分支中调用 Pure Pursuit + S 曲线 |
| `project/user/main_cm7_0.c` | 在 `while(1)` 超级循环中调用 A\* 重规划 + 地图更新 |
| `project/code/Menu.c` | 新增路径规划菜单项 (Page 2 扩展) |
| `project/code/Imu.h` | 新增 `path_target_speed`, `path_steer_command` 全局变量 |

### 9.2 超级循环中的集成

```c
// main_cm7_0.c 的 while(1) 超级循环:
while (1)
{
    // ---- 路径规划 (尽力而为, 异步) ----
    if (path_replan_requested)
    {
        astar_plan(occupancy_grid,
                   start_x, start_y,
                   goal_x, goal_y,
                   global_path, &path_length);
        path_replan_requested = 0;
    }

    // ---- 地图更新 (如果有新传感器数据) ----
    if (sensor_data_ready)
    {
        occupancy_grid_update(latest_sensor_readings);
        sensor_data_ready = 0;
    }

    // ---- 原有 Menu ----
    Menu();
}
```

### 9.3 菜单集成

在现有 Menu.c 的 page 2（路径记录/回放旁边）增加：

```
Page 2 子菜单:
  • Record Path 1/2/3       (现有, 不变)
  • Replay Path 1/2/3       (现有, 不变)
  • Clear Path 1/2/3         (现有, 不变)
  ─────────────────────────
  • Path Plan: ON/OFF        (新增: 使能路径规划)
  • Set Goal: X=__ Y=__      (新增: 设定目标坐标)
  • Show Map                  (新增: 显示占据栅格)
  • Avoid Obstacle: ON/OFF    (新增: 使能局部避障)
```

### 9.4 与跳台系统的互斥

```c
// 在每个控制周期检查:
if (jump_cfg.state != JUMP_IDLE)
{
    // 跳台进行中 → 暂停路径规划
    path_follow_enable = 0;
    s_curve_reset(&speed_profile);
}
else if (path_follow_enable == 0 && path_was_active)
{
    // 跳台结束 → 恢复路径规划
    path_follow_enable = 1;
    // 重新计算到目标的路径 (当前位置可能已偏离)
    path_replan_requested = 1;
}
```

### 9.5 完整的同步信号流

```
1ms PIT ISR (pit_call_back):
  ┌─ IMU 读取 + 四元数融合
  ┌─ angular_speed PID (1ms)
  ┌─ 角度 PID (5ms)
  ├─ 【位置 PID (20ms) ← 新增】
  ├─ 【Pure Pursuit (20ms) ← 新增】
  ├─ 【S 曲线速度更新 (20ms) ← 新增】
  ├─ 速度 PID (20ms)
  ├─ 电机/伺服输出
  └─ car_state_calculate (跳台管理)

5ms PIT ISR:
  └─ 按键扫描

超级循环:
  ├─ 【A* 全局重规划 ← 新增】
  ├─ 【占据栅格地图更新 ← 新增】
  └─ Menu 交互
```

---

## 附录 A：内存预算分解

| 组件 | RAM | 说明 |
|------|-----|------|
| 占据栅格 (100×100, 2bit/cell) | 2.5 KB | Akers 编码 |
| A\* open/closed 位图 | 2.5 KB | 位图追踪 |
| A\* 优先队列 (30% 探索) | ~10 KB | 典型值 |
| 全局路点缓冲区 (max 200 点) | 1.6 KB | 每点 2×float32 + 1×int16 |
| Pure Pursuit 状态 | 0.2 KB | 前视点, 当前索引 |
| Bug2 状态 | 0.1 KB | M-line + 状态 |
| S 曲线规划器 | 0.2 KB | 阶段状态 + 预计算 |
| PID 级联 (现有, 不变) | ~2 KB | 5 个 pid_cycle_struct |
| 平衡/IMU/Menu (现有) | ~15-30 KB | 估计 |
| **路径规划子系统** | **~17 KB** | |
| **系统总计** | **~40-55 KB** | 100 KB budget 内充裕 |

---

## 附录 B：算法选择决策树

```
开始
 │
 ├─ 是否需要全局路径规划？ (大到需要 A→B 的路线)
 │   ├─ YES → A* 栅格搜索
 │   │        │
 │   │        ├─ 地图已知？ → 预加载占据栅格
 │   │        └─ 地图未知？ → 增量建图 (需传感器)
 │   │
 │   └─ NO  → 直接跳到局部跟随
 │
 ├─ 路径如何跟随？
 │   ├─ Pure Pursuit（推荐）
 │   │   参数: L = max(15, 0.8 × v), 自适应
 │   │
 │   └─ Regulated Pure Pursuit (如果需要自适应减速)
 │
 ├─ 速度如何规划？
 │   ├─ S 曲线（推荐）
 │   │   理由: 自平衡车对 jerk 敏感
 │   │
 │   └─ 梯形（不推荐）
 │
 ├─ 需要局部避障吗？
 │   ├─ YES + 传感器 ≤ 3 个方向 → Bug2
 │   ├─ YES + 传感器 ≥ 6 个方向 → VFH+
 │   ├─ YES + 需要速度空间的平滑避障 → DWA (最简版)
 │   └─ NO  → 不需要
 │
 └─ 完成
```

---

## 附录 C：相关文件索引

| 文件 | 关键内容 |
|------|---------|
| `project/code/Body_ctrl.c:273-517` | `car_steer_control()` — 四轮转向 + 跳台状态机 |
| `project/code/Body_ctrl.c:556-606` | `pit_call_back()` — 控制心跳 |
| `project/code/Body_ctrl.c:184-221` | `car_state_calculate()` — 软启动 + PID 缩放 |
| `project/code/Body_ctrl.h:5-14` | 跳台状态枚举 |
| `project/code/Common_peripherals.h:35-53` | 伺服引脚/DIR/CENTER 定义 |
| `project/code/Common_peripherals.h:31-32` | M_MAX/M_MIN 电机限幅 |
| `project/code/Common_peripherals.h:69` | `wheel_diameter = 6.4f` |
| `project/code/Flash.c:234-295` | `Nag_System()` — 现有导航框架 |
| `project/code/Flash.h:27` | `Nag_Set_mileage = 5` (5cm 导航分辨率) |
| `project/code/Menu.c:16-932` | 100-entry 菜单表 |
| `project/user/main_cm7_0.c:48-96` | `main()` 初始化序列 + 超级循环 |
| `project/user/cm7_0_isr.c:47-117` | 未使用的 PIT ISR (ch2-ch20 可用) |

## 附录 D：外部参考

| 资源 | 说明 |
|------|------|
| [STM32F103 RRT 路径规划](https://www.mdpi.com/2079-9292/14/24/4901) | Cortex-M3 (72MHz, 20KB SRAM, 无 FPU) 上的 RRT 实现 |
| [NUCLEO-F446RE A\* 导航](https://github.com/StarDust-XCHH/NoeticMaze) | Cortex-M4F 上的完整导航栈 (ICP+A\*+平滑) |
| [micromouse flood fill](https://github.com/SuleymanEmreErdem/modified-flood-fill) | 增量式栅格更新, MCU 优化 |
| [A\* 零分配 Go 实现](https://www.quasilyte.dev/blog/post/pathfinding/) | 2 bits/cell, 60×60 栅格仅需 900 bytes |
| [DWA 梯度下降优化](https://hal.science/hal-04041336/document) | 平均 1.33 iterations (vs ROS 默认 600) |
| [VFH+ 原始论文](http://www.cs.cmu.edu/~iwan/papers/vfh+.pdf) | 4 阶段数据降维, 表驱动实现 |
| [ROS2 运动控制器白皮书](https://nobleo-technology.nl/wp-content/uploads/2023/01/White-Paper-ROS2-Motion-Controllers.pdf) | Pure Pursuit/RPP/DWB/TEB 对比 |
