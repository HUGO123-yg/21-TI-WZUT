# Multi-Core Motion & Vision Architecture — CYT4BB7

## TL;DR

> **Quick Summary**: Distribute workload across Infineon CYT4BB7 dual Cortex-M7 cores: CM7_0 runs full motion pipeline (IMU→Euler→LQR→PID→FOC→servos) at 1kHz bare-metal; CM7_1 runs TSL1401 CCD acquisition + 三值化 (ternarization) and sends processed vision data unidirectionally to CM7_0 via IPC PIPE.

> **Deliverables**:
> - CM7_0: 6 unlinked .c files added to IAR project, safe servo kinematics, 1kHz PIT-driven control ISR, state estimator, IPC receive callback
> - CM7_1: Vision application (vision.c/h), CCD ternarization, IPC send in main loop, cleaned ISR file
> - IPC: Activated zf_driver_ipc between cores, defined data format, ISR-safe (no IPC calls in ISR context)
> - ISR cleanup: Remove CCD handler from CM7_0, remove non-CCD sensor handlers from CM7_1

> **Estimated Effort**: Large
> **Parallel Execution**: YES — 4 waves
> **Critical Path**: T1 (div-by-zero guard) → T11 (state estimator) → T13 (1kHz ISR) → T16 (control integration) → F1-F4

---

  - [x] 8. **IPC init on CM7_0 — register callback that updates volatile image_error**

  **What to do**:
  - In `project/user/main_cm7_0.c`, after `clock_init()` and `debug_init()`, add:
    ```c
    ipc_communicate_init(IPC_PORT_1, ipc_cm7_0_callback);
    ```
  - Implement the callback function in `main_cm7_0.c` (or a new `project/code/ipc_handler_cm7_0.c`):
    ```c
    void ipc_cm7_0_callback(void)
    {
        // 从IPC接收视觉数据
        communicate_data_t rx_data;
        // 注意: zf_driver_ipc 在回调触发时数据已在全局缓冲区中
        // 需要读取 ipc_received_data (在 zf_driver_ipc.c 中定义的全局变量)
        extern communicate_data_t ipc_received_data;
        
        if (ipc_received_data.clientId == IPC_MSG_TYPE_VISION)
        {
            uint32 packed = ipc_received_data.data;
            image_error = IPC_VISION_UNPACK_ERROR(packed);
            // quality = IPC_VISION_UNPACK_QUALITY(packed);  // 可用于数据有效性判断
        }
    }
    ```
  - The callback runs in background/main-loop context (zf_driver_ipc triggers it after receiving), NOT in ISR context — this is safe
  - If `image_error` is declared `volatile` (from T6), the ISR that reads it will always see the latest value
  - Include `zf_driver_ipc.h` and `ipc_protocol.h` at the top of the callback file
  - Add the callback .c file to CM7_0 .ewp if created as separate file

  **Must NOT do**:
  - Do NOT call `ipc_send_data()` from the IPC callback (it's a receive callback)
  - Do NOT call IPC functions from any ISR on CM7_0
  - Do NOT block or spin-wait in the callback — it must be fast

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Single IPC init call + callback function implementation (existing wrapper)
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with T9-T12)
  - **Blocks**: T15 (control pipeline reads image_error from IPC)
  - **Blocked By**: T4 (need zf_driver_ipc compiled), T6 (need volatile globals), T7 (need IPC_VISION_UNPACK_ERROR macro)

  **References**:
  - `libraries/zf_driver/zf_driver_ipc.h` — `ipc_communicate_init()`, `IPC_PORT_1` enum, `communicate_data_t`
  - `libraries/zf_driver/zf_driver_ipc.c` — `ipc_received_data` global, callback invocation pattern
  - `project/user/main_cm7_0.c` — Current init sequence: `clock_init()` → `debug_init()` → `gpio_init()` → empty `while(true)`
  - `project/code/ipc_protocol.h` — `IPC_VISION_UNPACK_ERROR()` macro (from T7)
  - `project/code/control.h` — `extern volatile float image_error` declaration (from T6)
  - **WHY**: IPC must be initialized BEFORE any data transfer. CM7_0 as EP0 (IPC_PORT_1) is the receiver. The callback is the bridge that converts raw IPC uint32 → float image_error for the control pipeline.

  **Acceptance Criteria**:
  - [ ] `grep -c "ipc_communicate_init.*IPC_PORT_1" project/user/main_cm7_0.c` returns ≥1
  - [ ] `grep -c "IPC_VISION_UNPACK_ERROR" project/user/main_cm7_0.c` returns ≥1 (or ≥1 in ipc_handler file)
  - [ ] `grep -c "ipc_send_data\|ipc_communicate" project/user/cm7_0_isr.c` returns 0

  **QA Scenarios**:

  ```
  Scenario: IPC initialized on CM7_0 with callback using vision unpack macro
    Tool: Bash (grep)
    Preconditions: main_cm7_0.c edited
    Steps:
      1. grep -c "ipc_communicate_init" project/user/main_cm7_0.c
      2. grep -c "IPC_VISION_UNPACK_ERROR" project/user/main_cm7_0.c
      3. grep -c "ipc_send_data" project/user/cm7_0_isr.c
    Expected Result: Step 1 ≥1 (init called), Step 2 ≥1 (unpack macro used), Step 3 = 0 (no IPC in ISR).
    Failure Indicators: Init not called, unpack macro not imported, or IPC found in ISR.
    Evidence: .sisyphus/evidence/task-8-ipc-cm7_0.log
  ```

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 9. **IPC init on CM7_1 — setup for send in main loop**

  **What to do**:
  - In `project/user/main_cm7_1.c`, after `clock_init()` and `vision_init()`, add:
    ```c
    ipc_communicate_init(IPC_PORT_2, ipc_cm7_1_callback);
    ```
  - Implement the callback:
    ```c
    void ipc_cm7_1_callback(void)
    {
        // CM7_1 is primarily a sender; callback may be used for heartbeat/ack from CM7_0
        // 当前仅用于确认发送完成，无需额外处理
    }
    ```
  - In `vision_send_to_cm7_0()` (in vision.c), implement the IPC send:
    ```c
    void vision_send_to_cm7_0(void)
    {
        uint32 packed = IPC_VISION_PACK(image_error_cm7_1, 255);  // quality=255 (最佳)
        ipc_send_data(packed);
    }
    ```
  - Place the `vision_send_to_cm7_0()` call in `main_cm7_1.c`'s `while(true)` loop — AFTER ternarization and line error calculation, NOT in the PIT ISR
  - Include `zf_driver_ipc.h` and `ipc_protocol.h` where needed

  **Must NOT do**:
  - Do NOT call `vision_send_to_cm7_0()` from `pit0_ch21_isr()` — IPC blocks for up to 5ms
  - Do NOT spin-wait for IPC completion in ISR context

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Single IPC init call + send wrapper function
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with T8, T10-T12)
  - **Blocks**: T14 (CM7_1 main loop calls vision_send_to_cm7_0)
  - **Blocked By**: T4 (need zf_driver_ipc compiled), T7 (need IPC_VISION_PACK macro)

  **References**:
  - `libraries/zf_driver/zf_driver_ipc.h` — `ipc_communicate_init()`, `IPC_PORT_2` enum, `ipc_send_data()`
  - `project/user/main_cm7_1.c` — Current init sequence + while loop (to be expanded)
  - `project/code/vision.h` — `vision_send_to_cm7_0()` declaration (from T5)
  - `project/code/vision.c` — Implementation location for `vision_send_to_cm7_0()`
  - `project/code/ipc_protocol.h` — `IPC_VISION_PACK()` macro (from T7)
  - **WHY**: CM7_1 as EP1 (IPC_PORT_2) is the sender. IPC must init before first send. The send call MUST be in main loop, not ISR.

  **Acceptance Criteria**:
  - [ ] `grep -c "ipc_communicate_init.*IPC_PORT_2" project/user/main_cm7_1.c` returns ≥1
  - [ ] `grep -c "IPC_VISION_PACK" project/code/vision.c` returns ≥1
  - [ ] `grep -c "ipc_send_data\|ipc_communicate" project/user/cm7_1_isr.c` returns 0

  **QA Scenarios**:

  ```
  Scenario: IPC initialized on CM7_1, send in vision.c (not ISR)
    Tool: Bash (grep)
    Preconditions: main_cm7_1.c and vision.c edited
    Steps:
      1. grep -c "ipc_communicate_init" project/user/main_cm7_1.c
      2. grep -c "IPC_VISION_PACK" project/code/vision.c
      3. grep -c "ipc_send_data" project/user/cm7_1_isr.c
    Expected Result: Step 1 ≥1 (init called), Step 2 ≥1 (pack macro used), Step 3 = 0 (no IPC in ISR).
    Failure Indicators: Init not called, pack macro not used, or IPC found in ISR.
    Evidence: .sisyphus/evidence/task-9-ipc-cm7_1.log
  ```

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 10. **Implement CCD ternarization algorithm in vision.c**

  **What to do**:
  - In `project/code/vision.c`, implement `vision_ternarize()`:
    ```c
    void vision_ternarize(void)
    {
        uint8 *raw = tsl1401_get_buffer();  // 获取128像素原始灰度值
        
        for (uint8 i = 0; i < 128; i++)
        {
            if (raw[i] > CCD_THRESHOLD_HIGH)
                ccd_ternary[i] = CCD_VALUE_WHITE;    // 白
            else if (raw[i] < CCD_THRESHOLD_LOW)
                ccd_ternary[i] = CCD_VALUE_BLACK;    // 黑
            else
                ccd_ternary[i] = CCD_VALUE_EDGE;     // 边缘
        }
    }
    ```
  - Implement `vision_calc_line_error()`:
    ```c
    float vision_calc_line_error(void)
    {
        float sum_pos = 0.0f;
        uint16 count = 0;
        
        // 计算白色/边缘像素的加权中心位置
        for (uint8 i = 0; i < 128; i++)
        {
            if (ccd_ternary[i] >= CCD_VALUE_EDGE)  // 边缘或白
            {
                sum_pos += (float)i;
                count++;
            }
        }
        
        if (count == 0) return 0.0f;  // 无有效数据
        
        float center = sum_pos / (float)count;
        // image_error: 正值=线偏右, 负值=线偏左, 单位: 像素
        image_error_cm7_1 = center - 64.0f;  // CCD中心为64
        return image_error_cm7_1;
    }
    ```
  - Wire the processing sequence in `main_cm7_1.c` while loop:
    ```c
    while(1)
    {
        if (tsl1401_finish_flag)
        {
            vision_ternarize();
            image_error_cm7_1 = vision_calc_line_error();
            vision_send_to_cm7_0();   // IPC send (NOT in ISR)
            tsl1401_finish_flag = 0;  // 清除标志
        }
    }
    ```
  - Adjust `CCD_THRESHOLD_HIGH` and `CCD_THRESHOLD_LOW` thresholds based on expected lighting conditions (180/60 are initial values — may need tuning on hardware)

  **Must NOT do**:
  - Do NOT call IPC from within `vision_ternarize()` or `vision_calc_line_error()` — these are pure computation
  - Do NOT implement line-following logic — only ternarization + error calculation
  - Do NOT use floating-point in the ternarization inner loop (integer comparisons only, float only in calc_line_error)

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Algorithm implementation (dual-threshold classification, weighted centroid calculation) + integration with CCD driver
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed for embedded C algorithm

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with T8-T9, T11-T12)
  - **Blocks**: T14 (CM7_1 main loop integration)
  - **Blocked By**: T5 (vision.h defines thresholds and function signatures)

  **References**:
  - `project/code/vision.h` — `CCD_THRESHOLD_HIGH`, `CCD_THRESHOLD_LOW`, `ccd_ternary[]`, function declarations (from T5)
  - `libraries/zf_device/zf_device_tsl1401.h` — `tsl1401_get_buffer()` returns pointer to 128-byte raw pixel array
  - `libraries/zf_device/zf_device_tsl1401.c` — CCD driver: exposure control, ADC sampling, buffer management
  - `project/user/main_cm7_1.c` — While loop where processing sequence is wired
  - `libraries/zf_common/zf_common_typedef.h` — Types: `uint8`, `uint16`, `float32`
  - **WHY**: Ternarization reduces 256-level grayscale to 3 values, massively simplifying downstream line/obstacle detection. The weighted centroid gives a sub-pixel line position estimate.

  **Acceptance Criteria**:
  - [ ] `vision_ternarize()` in vision.c iterates over 128 pixels with dual-threshold comparison
  - [ ] `vision_calc_line_error()` computes weighted centroid and returns offset from center (64)
  - [ ] Processing sequence in `main_cm7_1.c` while loop: check flag → ternarize → calc error → IPC send → clear flag
  - [ ] No `ipc_send_data` call inside `vision_ternarize()` or `vision_calc_line_error()` (grep on vision.c)

  **QA Scenarios**:

  ```
  Scenario: Ternarization function structure correct
    Tool: Bash (grep)
    Preconditions: vision.c has ternarization implemented
    Steps:
      1. grep -c "CCD_THRESHOLD_HIGH\|CCD_THRESHOLD_LOW" project/code/vision.c
      2. grep -c "ccd_ternary\[" project/code/vision.c
      3. grep -c "for.*128" project/code/vision.c
    Expected Result: All three return ≥1. Thresholds used, ternary array written, 128-pixel loop present.
    Failure Indicators: Missing thresholds, no ternary array access, or incorrect loop count.
    Evidence: .sisyphus/evidence/task-10-ternarization.log

  Scenario: Line error calculation uses centroid method
    Tool: Bash (grep)
    Preconditions: vision_calc_line_error implemented
    Steps:
      1. grep -c "sum_pos\|center.*64\|64\.0" project/code/vision.c
    Expected Result: Returns ≥1 for centroid calculation and center offset (64).
    Failure Indicators: No centroid logic found.
    Evidence: .sisyphus/evidence/task-10-line-error.log
  ```

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 11. **Implement state estimator — complementary filter for v_hat/x_hat**

  **What to do**:
  - Create `project/code/state_estimator.h`:
    ```c
    #ifndef _state_estimator_h_
    #define _state_estimator_h_
    #include "zf_common_headfile.h"
    
    void state_estimator_init(void);
    void state_estimator_update(float pitch_angle, float motor_accel, float dt);
    
    #endif
    ```
  - Create `project/code/state_estimator.c`:
    ```c
    #include "state_estimator.h"
    #include "control.h"  // for v_hat, x_hat, image_error
    
    void state_estimator_init(void)
    {
        v_hat = 0.0f;
        x_hat = 0.0f;
    }
    
    void state_estimator_update(float pitch_angle, float motor_accel, float dt)
    {
        // 互补滤波器: 融合IMU倾角 + 电机加速度估计速度/位置
        // v_hat: 速度估计 (m/s)
        // x_hat: 位置估计 (m)
        
        // 电机加速度积分 → 速度预测
        float v_pred = v_hat + motor_accel * dt;
        
        // IMU倾角作为速度修正项 (比例系数需标定)
        // pitch_angle > 0 表示前倾 → 期望向前速度
        float v_correction = pitch_angle * 0.05f;  // TODO: 标定此增益
        
        // 互补滤波融合
        v_hat = 0.95f * v_pred + 0.05f * v_correction;
        
        // 位置积分
        x_hat += v_hat * dt;
    }
    ```
  - Add `state_estimator.c` to CM7_0 .ewp project file
  - Call `state_estimator_init()` in CM7_0's `main()` after clock init
  - Call `state_estimator_update(pitch_angle, motor_accel, dt)` in the 1kHz ISR (T13)
  - Add TODO comment: gains (0.05, 0.95/0.05 filter ratio) need calibration on hardware

  **Must NOT do**:
  - Do NOT implement Kalman filter — complementary filter is sufficient for initial bring-up
  - Do NOT read sensors inside `state_estimator_update()` — inputs are passed as parameters
  - Do NOT call IPC or blocking functions from this module

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: New module (2 files), algorithm design (complementary filter), IAR project registration
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with T8-T10, T12)
  - **Blocks**: T13 (1kHz ISR calls state_estimator_update)
  - **Blocked By**: T6 (v_hat/x_hat defined as volatile)

  **References**:
  - `project/code/control.h:78-79` — `extern volatile float v_hat; extern volatile float x_hat;` (from T6)
  - `project/code/control.c:100-150` — `LQR_control()` reads v_hat and x_hat for state feedback
  - `libraries/zf_device/zf_device_imu660ra.h` — IMU API: `imu660ra_get_gyro()`, `imu660ra_get_acc()`
  - `project/user/main_cm7_0.c` — Init sequence where state_estimator_init() should be called
  - `libraries/zf_common/zf_common_typedef.h` — `float32` type
  - **WHY**: `LQR_control()` in control.c reads v_hat and x_hat — without state estimation, LQR state feedback produces garbage output. Complementary filter provides a simple, tunable velocity/position estimate.

  **Acceptance Criteria**:
  - [ ] `project/code/state_estimator.h` exists with `state_estimator_init()` and `state_estimator_update()` declarations
  - [ ] `project/code/state_estimator.c` exists with complementary filter implementation (v_pred + v_correction fusion)
  - [ ] `grep -c "state_estimator.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp` returns ≥1
  - [ ] `grep -c "state_estimator_init" project/user/main_cm7_0.c` returns ≥1

  **QA Scenarios**:

  ```
  Scenario: State estimator files created and added to CM7_0 project
    Tool: Bash (ls + grep)
    Preconditions: state_estimator.c/h created and registered
    Steps:
      1. ls -la project/code/state_estimator.c project/code/state_estimator.h
      2. grep -c "state_estimator.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp
      3. grep -c "state_estimator_init" project/user/main_cm7_0.c
    Expected Result: Both files exist, registered in .ewp, init called in main.
    Failure Indicators: Missing file, missing .ewp entry, init not called.
    Evidence: .sisyphus/evidence/task-11-state-estimator.log

  Scenario: Complementary filter uses v_pred + v_correction fusion
    Tool: Bash (grep)
    Preconditions: state_estimator.c implemented
    Steps:
      1. grep -c "v_pred\|v_correction\|0\.95\|0\.05" project/code/state_estimator.c
    Expected Result: Returns ≥1 for complementary filter structure (prediction + correction with fusion weights).
    Failure Indicators: No filter fusion logic found.
    Evidence: .sisyphus/evidence/task-11-filter-structure.log
  ```

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 12. **Implement IMU→Euler fusion — raw gyro data to Euler angles**

  **What to do**:
  - In `project/code/euler.c` (currently only has `euler_angle = {0,0,0}`), implement gyro integration:
    ```c
    #include "euler.h"
    #include "control.h"
    #include "zf_device_imu660ra.h"
    
    // euler_angle 定义在 euler.h 中: extern euler_angle_struct euler_angle;
    euler_angle_struct euler_angle = {0.0f, 0.0f, 0.0f};
    
    static float gyro_offset_x = 0.0f;  // 陀螺仪零偏
    static float gyro_offset_y = 0.0f;
    static float gyro_offset_z = 0.0f;
    
    void euler_init(void)
    {
        // 初始化IMU
        imu660ra_init();
        
        // 简单零偏校准: 采样100次取平均
        float sum_x = 0, sum_y = 0, sum_z = 0;
        for (int i = 0; i < 100; i++)
        {
            imu660ra_get_gyro();  // 触发读取
            sum_x += imu660ra_gyro_x;
            sum_y += imu660ra_gyro_y;
            sum_z += imu660ra_gyro_z;
            system_delay_us(1000);
        }
        gyro_offset_x = sum_x / 100.0f;
        gyro_offset_y = sum_y / 100.0f;
        gyro_offset_z = sum_z / 100.0f;
    }
    
    void euler_update(float dt)
    {
        // 读取陀螺仪角速度 (deg/s)
        imu660ra_get_gyro();
        
        float gx = imu660ra_gyro_x - gyro_offset_x;
        float gy = imu660ra_gyro_y - gyro_offset_y;
        float gz = imu660ra_gyro_z - gyro_offset_z;
        
        // 欧拉角积分 (度)
        euler_angle.pitch += gx * dt;  // X轴 → pitch
        euler_angle.roll  += gy * dt;  // Y轴 → roll
        euler_angle.yaw   += gz * dt;  // Z轴 → yaw
        
        // TODO: 添加加速度计修正 (Mahony/Madgwick 滤波器)
        // 当前为纯陀螺仪积分, 长时间会有漂移
    }
    ```
  - Call `euler_init()` in CM7_0's `main()` after clock init (before state_estimator_init)
  - Call `euler_update(dt)` in the 1kHz ISR (T13)
  - The existing `euler_angle_struct` is defined in `euler.h` — verify it contains `pitch`, `roll`, `yaw` as `float`

  **Must NOT do**:
  - Do NOT implement full Mahony/Madgwick filter — pure gyro integration with offset calibration is sufficient for initial bring-up
  - Do NOT call `euler_update()` from main loop — must be called at fixed rate (1kHz ISR)

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Sensor fusion algorithm (gyro offset calibration + integration), IMU driver integration
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2 (with T8-T11)
  - **Blocks**: T13 (1kHz ISR calls euler_update)
  - **Blocked By**: T4 (euler.c compiled into project)

  **References**:
  - `project/code/euler.c` — Currently just `euler_angle = {0,0,0}` — replace with full implementation
  - `project/code/euler.h` — `euler_angle_struct` typedef, `extern euler_angle_struct euler_angle`
  - `project/code/control.c:120` — `LQR_control()` reads `euler_angle.pitch` and `euler_angle.roll`
  - `libraries/zf_device/zf_device_imu660ra.h` — `imu660ra_init()`, `imu660ra_get_gyro()`, `imu660ra_gyro_x/y/z` globals
  - `libraries/zf_driver/zf_driver_delay.h` — `system_delay_us()` for calibration loop
  - `libraries/zf_common/zf_common_typedef.h` — `float32` type
  - **WHY**: control.c's `LQR_control()` reads `euler_angle` which is currently always zero. Without gyro integration, the robot has no attitude feedback — LQR balance control cannot function.

  **Acceptance Criteria**:
  - [ ] `euler_init()` in euler.c performs gyro offset calibration (100-sample average)
  - [ ] `euler_update(dt)` integrates gyro data into pitch/roll/yaw
  - [ ] `grep -c "euler_init" project/user/main_cm7_0.c` returns ≥1
  - [ ] `grep -c "imu660ra_get_gyro" project/code/euler.c` returns ≥1

  **QA Scenarios**:

  ```
  Scenario: Euler module performs gyro calibration and integration
    Tool: Bash (grep)
    Preconditions: euler.c rewritten with full implementation
    Steps:
      1. grep -c "gyro_offset\|100.*sum\|imu660ra_get_gyro" project/code/euler.c
      2. grep -c "euler_init" project/user/main_cm7_0.c
    Expected Result: Step 1 ≥2 (offset calibration + IMU read), Step 2 ≥1 (init called in main).
    Failure Indicators: No calibration logic, no IMU API calls, init not called.
    Evidence: .sisyphus/evidence/task-12-euler-fusion.log
  ```

  **Commit**: NO (groups with Wave 2 batch)


### Original Request
将运动解算全部交给CM7_0核心，视觉解算交给CM7_1核心，同时实现跨核心通讯。CM7_1先实现数据三值化（TSL1401 CCD），降低后期巡线/过台阶/识别障碍的MCU压力。

### Interview Summary
**Key Decisions**:
- **CM7_0 = ALL motion**: Full pipeline IMU→Euler→LQR→PID→FOC→servo kinematics, 1kHz PIT-driven, bare-metal
- **CM7_1 = ALL vision**: TSL1401 CCD acquisition + 三值化 (ternarization: grayscale→3-value black/edge/white), bare-metal
- **IPC**: Unidirectional CM7_1→CM7_0 via existing `zf_driver_ipc` PIPE API (channels 6-7)
- **OS**: Bare-metal both cores — no FreeRTOS
- **Sensors**: CM7_0 owns all (IMU, TOF, GNSS, wireless, encoders); CM7_1 owns only CCD
- **State estimator**: Include v_hat/x_hat computation (complementary filter)
- **CM0+**: Out of scope (512K flash + 128K SRAM unused)

**Research Findings**:
- 6 unlinked application .c files on disk but NOT in IAR project: control.c, foc.c, euler.c, pid.c, timer_control.c, servo_kinematics.c
- Both ISR files (cm7_0_isr.c, cm7_1_isr.c) are byte-for-byte IDENTICAL — all handlers on both cores
- IPC available but NEVER called; `zf_driver_ipc.c` compiled into both projects
- CM7_1 IAR project has ZERO application .c files — completely empty main loop
- servo_kinematics.c has TODO parameters: L1_MM, L2_MM, D_MM, SERVO_ANGLE_MAX, PWM_PER_DEG all zero → **division by zero at runtime**
- TSL1401 CCD PIT_CH21 handler exists in BOTH ISR files → hardware conflict if both cores initialize
- `ipc_send_data()` blocks for up to 5ms → CANNOT be called from 1kHz ISR (period = 1ms)

### Metis Review
**Identified Gaps** (addressed in plan):
- **BLOCKER**: servo_kinematics.c division-by-zero → Add runtime safety guard (T1)
- **BLOCKER**: IPC blocking 5ms vs 1kHz ISR → All IPC in main-loop context only (T8, T10)
- **BLOCKER**: Dual PIT_CH21 handler → Remove from CM7_0 ISR (T2), keep on CM7_1
- **Safety**: CM7_1 ISR has wireless/GNSS/remote handlers → Remove from CM7_1 (T3)
- **Missing**: v_hat/x_hat never computed → Complementary filter (T11)
- **Missing**: euler_angle static zeros → IMU→Euler fusion in 1kHz ISR (T12)
- **Undefined**: IPC data format → Define packed struct with image_error + flags (T7)
- **Rate mismatch**: CCD 100Hz vs control 1000Hz → Zero-order hold on last valid data (T16)

---

## Work Objectives

### Core Objective
Activate dual-core bare-metal architecture where CM7_0 runs the complete 1kHz motion control pipeline and CM7_1 runs CCD ternarization with unidirectional IPC data flow.

### Concrete Deliverables
- CM7_0 IAR project compiles with all 6 previously unlinked .c files
- CM7_1 IAR project compiles with new vision.c/h and ipc_handler files
- IPC PIPE activated: `ipc_communicate_init()` called on both cores
- Servo kinematics runtime-safe (no div-by-zero)
- ISR files cleaned: CCD handler only on CM7_1, sensor handlers only on CM7_0
- State estimator computes v_hat/x_hat via complementary filter
- 1kHz PIT ISR drives full motion pipeline on CM7_0
- CM7_1 main loop: CCD trigger → ternarization → IPC send
- Both IAR projects build successfully (Debug config)

### Definition of Done
- [ ] `IarBuild.exe` succeeds for both `cyt4bb7_cm_7_0.ewp` and `cyt4bb7_cm_7_1.ewp` (Debug)
- [ ] `grep -c "tsl1401_collect_pit_handler" project/user/cm7_0_isr.c` returns 0
- [ ] `grep -c "tsl1401_collect_pit_handler" project/user/cm7_1_isr.c` returns 1
- [ ] `grep -c "ipc_send_data\|ipc_communicate" project/user/cm7_0_isr.c` returns 0 (no IPC in ISR)
- [ ] `grep -c "ipc_send_data\|ipc_communicate" project/user/cm7_1_isr.c` returns 0 (no IPC in ISR)
- [ ] CM7_0 `.map` contains `image_error`, `v_hat`, `x_hat` as defined symbols (not unresolved)
- [ ] CM7_1 `.map` contains `tsl1401_finish_flag`, `ipc_send_data` as defined symbols
- [ ] `servo_control_table()` has early-return guard for zero kinematics params

### Must Have
- No IPC function calls from any ISR context on either core
- Only CM7_1 initializes TSL1401 hardware (PIT_CH21, ADC, GPIO)
- CM7_0 ISR does NOT contain `tsl1401_collect_pit_handler`
- CM7_1 ISR does NOT contain wireless/GNSS/remote-control UART handlers
- Div-by-zero guard in servo_kinematics.c before kinematic computation
- `image_error`, `v_hat`, `x_hat` declared as `volatile` for cross-context access

### Must NOT Have (Guardrails)
- **NO FreeRTOS**: No RTOS tasks, queues, semaphores — stays bare-metal
- **NO CM0+ changes**: CM0+ boot core untouched — out of scope
- **NO IPC in ISR**: No `ipc_send_data()`, `ipc_communicate_init()`, or any IPC call inside any ISR handler
- **NO dual CCD init**: CM7_0 must NOT call `tsl1401_init()` or configure PIT_CH21
- **NO vision algorithms beyond ternarization**: Line following, obstacle detection, step crossing deferred
- **NO modifying Infineon SDK files**: Only Seekfree/user code changed
- **NO automatic servo output with zero kinematics**: Guard prevents PWM output when parameters unconfigured

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** — ALL verification is agent-executed via IAR build + grep checks. No hardware-in-the-loop testing required for plan acceptance (hardware test is deferred to user's physical bring-up).

### Test Decision
- **Infrastructure exists**: NO — no test framework in this project
- **Automated tests**: None — hardware-in-the-loop only via IAR C-SPY
- **Framework**: N/A
- **Agent QA**: Build verification + code-level grep checks

### QA Policy
Every task includes agent-executed QA scenarios:
- **Build verification**: Run IAR build command, check exit code, verify .out/.map exist
- **Code checks**: grep for forbidden patterns (IPC in ISR, duplicate handlers, missing symbols)
- **Structure checks**: Verify file existence, IAR .ewp entries, header includes

Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.log`.

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — safety + cleanup + scaffolding, MAX PARALLEL):
├── Task 1: Div-by-zero guard in servo_kinematics.c [quick]
├── Task 2: Clean CM7_0 ISR — remove CCD handler [quick]
├── Task 3: Clean CM7_1 ISR — remove non-CCD sensor handlers [quick]
├── Task 4: Add 6 unlinked .c files to CM7_0 IAR project [quick]
├── Task 5: Create CM7_1 vision application skeleton + add to CM7_1 IAR [quick]
├── Task 6: Declare volatile globals (image_error, v_hat, x_hat) + init [quick]
└── Task 7: Define IPC data format (shared header) [quick]

Wave 2 (After Wave 1 — core logic, MAX PARALLEL):
├── Task 8: IPC init on CM7_0 — callback that updates volatile image_error [quick]
├── Task 9: IPC init on CM7_1 — setup for send in main loop [quick]
├── Task 10: CM7_1 CCD ternarization algorithm in vision.c [deep]
├── Task 11: State estimator — complementary filter for v_hat/x_hat [deep]
└── Task 12: IMU→Euler fusion — raw gyro to Euler angles [deep]

Wave 3 (After Wave 2 — integration):
├── Task 13: Wire 1kHz PIT-driven motion control ISR on CM7_0 [deep]
├── Task 14: Wire CM7_1 main loop — CCD trigger → ternarization → IPC send [deep]
├── Task 15: Integrate IPC image_error into control pipeline (volatile read) [deep]
└── Task 16: Wire servo kinematics into control loop output [deep]

Wave FINAL (After ALL tasks — 4 parallel verification checks):
├── Task F1: Build CM7_0 — IAR compile, check .out/.map [quick]
├── Task F2: Build CM7_1 — IAR compile, check .out/.map [quick]
├── Task F3: ISR cleanup verification — grep for forbidden handlers [quick]
└── Task F4: IPC safety + symbol verification — grep for forbidden patterns [quick]

Critical Path: T1 → T11 → T13 → T16 → F1-F4
Parallel Speedup: ~60% faster than sequential
Max Concurrent: 7 (Wave 1), 5 (Wave 2)
```

### Dependency Matrix

| Task | Depends On | Blocks | Wave |
|------|-----------|--------|------|
| T1 | None | T13, T16 | 1 |
| T2 | None | F3 | 1 |
| T3 | None | F3 | 1 |
| T4 | None | T8, T9, T13 | 1 |
| T5 | None | T10, T14 | 1 |
| T6 | None | T8, T11, T13 | 1 |
| T7 | None | T8, T9, T15 | 1 |
| T8 | T4, T6, T7 | T15 | 2 |
| T9 | T4, T7 | T14 | 2 |
| T10 | T5 | T14 | 2 |
| T11 | T6 | T13 | 2 |
| T12 | T4 | T13 | 2 |
| T13 | T1, T4, T6, T11, T12 | T16 | 3 |
| T14 | T5, T9, T10 | T15 | 3 |
| T15 | T7, T8, T14 | T16 | 3 |
| T16 | T1, T13, T15 | F1-F4 | 3 |

### Agent Dispatch Summary

- **Wave 1**: **7** — T1-T7 → all `quick`
- **Wave 2**: **5** — T8-T9 → `quick`, T10-T12 → `deep`
- **Wave 3**: **4** — T13-T16 → `deep`
- **Wave FINAL**: **4** — F1-F4 → `quick`

---

## TODOs

  - [x] 1. **Add div-by-zero safety guard to servo_kinematics.c**

  **What to do**:
  - In `servo_control_table()` at `project/code/servo_kinematics.c`, add an early-return check at the top of the function:
    ```c
    if (L1_MM == 0.0f || L2_MM == 0.0f || D_MM == 0.0f) {
        return;  // Kinematics parameters not configured — skip PWM output
    }
    ```
  - Also add a `static uint8 param_warned = 0;` flag to print a one-time debug warning via `printf("servo_kinematics: params unconfigured, PWM disabled\n")` on first invocation
  - Add a `// TODO: Replace with real kinematics parameters from MATLAB simulation` comment above the guard

  **Must NOT do**:
  - Do NOT set arbitrary non-zero values for L1_MM/L2_MM/D_MM — these must come from physical measurement
  - Do NOT remove the TODO parameter markers — keep them as documentation

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Single-file, single-function safety guard — trivial scope
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed for single-file C edit

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with T2-T7)
  - **Blocks**: T13 (1kHz ISR calls servo_control_table), T16 (servo kinematics integration)
  - **Blocked By**: None

  **References**:
  - `project/code/servo_kinematics.c:65-75` — The `servo_control_table()` function with division by L1_MM/R1 at lines 68-69
  - `project/code/servo_kinematics.h` — Header declaring `servo_control_table()`, `servo_angle_to_pwm()`, and macro parameters L1_MM/L2_MM/D_MM/SERVO_ANGLE_MAX/PWM_PER_DEG (all zero)
  - `project/code/control.c:274` — `servo_control_table()` call site in `right_leg_control()`
  - `libraries/zf_common/zf_common_typedef.h` — Use `uint8` type (not `uint8_t`) for the warning flag
  - **WHY**: servo_kinematics.c divides by L1_MM and R1 (derived from L1_MM/L2_MM/D_MM) — all zero → NaN/Inf → undefined behavior. Guard prevents crash on first invocation.

  **Acceptance Criteria**:
  - [ ] `servo_control_table()` contains early-return guard checking `L1_MM == 0.0f || L2_MM == 0.0f || D_MM == 0.0f`
  - [ ] Guard is placed BEFORE any division/math operations

  **QA Scenarios**:

  ```
  Scenario: Div-by-zero guarded — params all zero
    Tool: Bash (grep)
    Preconditions: servo_kinematics.c has the guard inserted
    Steps:
      1. grep -n "L1_MM == 0.0f" project/code/servo_kinematics.c
      2. grep -n "L2_MM == 0.0f" project/code/servo_kinematics.c
      3. grep -n "return;" project/code/servo_kinematics.c | head -5
    Expected Result: Lines found for L1_MM check AND L2_MM check. A return statement exists within the first 20 lines of servo_control_table().
    Failure Indicators: grep returns empty for the guard checks.
    Evidence: .sisyphus/evidence/task-1-guard-check.log

  Scenario: All division operations are after the guard
    Tool: Bash (grep)
    Preconditions: Guard inserted in servo_control_table()
    Steps:
      1. grep -n "L1_MM \*\|/ L1_MM\|/ R1\|/ (" project/code/servo_kinematics.c
    Expected Result: All line numbers for division operations are GREATER than the line number of the guard's return statement.
    Failure Indicators: A division line number ≤ guard line number.
    Evidence: .sisyphus/evidence/task-1-division-order.log
  ```

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 5. **Create CM7_1 vision application skeleton + add to CM7_1 IAR project**

  **What to do**:
  - Create `project/code/vision.h`:
    ```c
    #ifndef _vision_h_
    #define _vision_h_
    #include "zf_common_headfile.h"
    
    // TSL1401 CCD 三值化阈值
    #define CCD_THRESHOLD_HIGH   180   // 白阈值 (高于此值为白)
    #define CCD_THRESHOLD_LOW     60   // 黑阈值 (低于此值为黑)
    // 中间值为边缘
    
    // 三值化结果：0=黑, 1=边缘, 2=白
    #define CCD_VALUE_BLACK   0
    #define CCD_VALUE_EDGE    1
    #define CCD_VALUE_WHITE   2
    
    extern volatile uint8 tsl1401_finish_flag;   // CCD采集完成标志
    extern uint8 ccd_ternary[128];               // 三值化结果数组
    extern float image_error_cm7_1;              // CM7_1计算出的线偏移量
    
    void vision_init(void);
    void vision_ternarize(void);                 // 三值化处理
    float vision_calc_line_error(void);          // 计算线位置误差
    void vision_send_to_cm7_0(void);             // 通过IPC发送到CM7_0
    
    #endif
    ```
  - Create `project/code/vision.c`:
    - `vision_init()`: Call `tsl1401_init()`, configure PIT_CH21 for CCD exposure timing
    - `vision_ternarize()`: Read raw CCD pixel array from `tsl1401_get_buffer()`, apply dual-threshold to produce 3-value output into `ccd_ternary[]`
    - `vision_calc_line_error()`: Find center of mass of edge/white pixels, compute offset from CCD center (64)
    - `vision_send_to_cm7_0()`: Call `ipc_send_data()` with float→uint32 reinterpret-cast (via union) — THIS IS IN MAIN LOOP, NOT ISR
  - Set `tsl1401_finish_flag = 1` in the PIT_CH21 ISR (cm7_1_isr.c), clear it in main loop after processing
  - Edit `project/iar/project_config/cyt4bb7_cm_7_1.ewp` — add `vision.c` to the code group
  - In `project/user/main_cm7_1.c`:
    - Call `vision_init()` after clock init
    - In `while(true)`: check `tsl1401_finish_flag` → if set: `vision_ternarize()` → `vision_calc_line_error()` → `vision_send_to_cm7_0()` → clear flag

  **Must NOT do**:
  - Do NOT implement line-following, obstacle detection, or step-crossing algorithms — only ternarization
  - Do NOT call IPC functions from `pit0_ch21_isr()` — only set the flag there
  - Do NOT add any files to CM7_0 project from this task

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Multi-file creation (vision.c, vision.h) + IAR project editing + main loop wiring — moderate complexity with CCD driver integration
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed for embedded C

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with T1-T4, T6-T7)
  - **Blocks**: T10 (ternarization algorithm), T14 (CM7_1 main loop)
  - **Blocked By**: None

  **References**:
  - `project/user/main_cm7_1.c` — Current empty main loop, clock init, debug init pattern to follow
  - `project/user/cm7_1_isr.c` — `pit0_ch21_isr()` where `tsl1401_finish_flag` must be set
  - `libraries/zf_device/zf_device_tsl1401.h` — `tsl1401_init()`, `tsl1401_get_buffer()`, `tsl1401_collect_pit_handler()` APIs
  - `libraries/zf_device/zf_device_tsl1401.c` — CCD driver implementation (128-pixel array, exposure control)
  - `libraries/zf_driver/zf_driver_ipc.h` — `ipc_send_data()`, `ipc_communicate_init()`, `communicate_data_t` struct
  - `libraries/zf_common/zf_common_typedef.h` — Use `uint8`, `float32` types
  - `project/iar/project_config/cyt4bb7_cm_7_1.ewp` — IAR project to add vision.c
  - **WHY**: CM7_1 currently has zero application code. vision.c/h provides the minimal skeleton for CCD acquisition and ternarization with IPC send capability.

  **Acceptance Criteria**:
  - [ ] `project/code/vision.h` exists with ternarization threshold macros and function declarations
  - [ ] `project/code/vision.c` exists with `vision_init()`, `vision_ternarize()`, `vision_calc_line_error()`, `vision_send_to_cm7_0()` implemented
  - [ ] `grep -c "vision.c" project/iar/project_config/cyt4bb7_cm_7_1.ewp` returns ≥1
  - [ ] `main_cm7_1.c` while loop calls vision functions (grep for `vision_ternarize` in main_cm7_1.c)
  - [ ] `tsl1401_finish_flag` set in `pit0_ch21_isr` in cm7_1_isr.c
  - [ ] No `ipc_send_data` call in `cm7_1_isr.c` (grep returns 0)

  **QA Scenarios**:

  ```
  Scenario: Vision files created and added to CM7_1 project
    Tool: Bash (ls + grep)
    Preconditions: vision.c/h created, .ewp edited, main loop wired
    Steps:
      1. ls -la project/code/vision.c project/code/vision.h
      2. grep -c "vision.c" project/iar/project_config/cyt4bb7_cm_7_1.ewp
      3. grep -n "vision_ternarize\|vision_init\|tsl1401_finish_flag" project/user/main_cm7_1.c
    Expected Result: Both files exist. vision.c found in CM7_1 .ewp. Main loop references vision functions.
    Failure Indicators: Missing file, missing .ewp entry, no vision calls in main.
    Evidence: .sisyphus/evidence/task-5-vision-skeleton.log

  Scenario: No IPC calls in CM7_1 ISR
    Tool: Bash (grep)
    Preconditions: cm7_1_isr.c has flag set, not IPC
    Steps:
      1. grep -c "ipc_send_data\|ipc_communicate" project/user/cm7_1_isr.c
      2. grep -c "tsl1401_finish_flag" project/user/cm7_1_isr.c
    Expected Result: Step 1 returns 0. Step 2 returns ≥1 (flag set, but no IPC call).
    Failure Indicators: Step 1 returns >0 (IPC call in ISR — VIOLATION).
    Evidence: .sisyphus/evidence/task-5-no-ipc-in-isr.log
  ```

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 6. **Declare volatile globals for cross-core data + state estimator variables**

  **What to do**:
  - In `project/code/control.h`, update the extern declarations:
    ```c
    extern volatile float image_error;   // 视觉模块 — updated by IPC callback on CM7_0
    extern volatile float v_hat;         // 状态估计器 — velocity estimate
    extern volatile float x_hat;         // 状态估计器 — position estimate
    ```
  - In `project/code/control.c` (or a new `project/code/state_estimator.c` if preferred), add definitions:
    ```c
    volatile float image_error = 0.0f;
    volatile float v_hat = 0.0f;
    volatile float x_hat = 0.0f;
    ```
  - The `volatile` qualifier is CRITICAL: `image_error` is written by IPC callback (background context) and read by 1kHz ISR (interrupt context). Without `volatile`, the compiler may optimize away the read.
  - If v_hat/x_hat are computed in ISR context and read in main loop (or vice versa), they also need `volatile`.
  - Ensure these definitions exist in exactly ONE .c file to avoid linker "multiple definition" errors
  - If creating a new `state_estimator.c` file, add it to CM7_0 .ewp if not already present

  **Must NOT do**:
  - Do NOT define these variables in a header file (would cause multiple definitions)
  - Do NOT forget the `volatile` qualifier — compiler optimization will silently break cross-context data sharing

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Add `volatile` to 3 declarations + ensure single definition point
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with T1-T5, T7)
  - **Blocks**: T8 (IPC callback writes image_error), T11 (state estimator writes v_hat/x_hat), T13 (1kHz ISR reads all three)
  - **Blocked By**: None

  **References**:
  - `project/code/control.h:76-79` — Current extern declarations (non-volatile, with TODO comments)
  - `project/code/control.c:141` — `turn_control(float image_error)` shadows the extern via parameter — check if this is intentional
  - `libraries/zf_common/zf_common_typedef.h` — Type system: use `float` (single precision) for these variables
  - **WHY**: `volatile` is mandatory for variables shared between ISR and main-loop contexts in bare-metal C. Without it, the optimizer may cache stale values in registers.

  **Acceptance Criteria**:
  - [ ] `grep "volatile.*image_error" project/code/control.h` returns a match
  - [ ] `grep "volatile.*v_hat" project/code/control.h` returns a match
  - [ ] `grep "volatile.*x_hat" project/code/control.h` returns a match
  - [ ] Exactly ONE .c file defines each variable (check: `grep -rl "float.*image_error\s*=" project/code/` returns ≤1 file)

  **QA Scenarios**:

  ```
  Scenario: All three globals declared volatile in header
    Tool: Bash (grep)
    Preconditions: control.h edited
    Steps:
      1. grep -n "volatile" project/code/control.h
    Expected Result: At least 3 volatile declarations found, including image_error, v_hat, x_hat.
    Failure Indicators: Fewer than 3 volatile declarations, or 'volatile' missing on any of the three.
    Evidence: .sisyphus/evidence/task-6-volatile-globals.log

  Scenario: Single definition point per variable
    Tool: Bash (grep)
    Preconditions: control.c or state_estimator.c has definitions
    Steps:
      1. grep -rn "float.*image_error\s*=" project/code/*.c
      2. grep -rn "float.*v_hat\s*=" project/code/*.c
      3. grep -rn "float.*x_hat\s*=" project/code/*.c
    Expected Result: Each variable defined in exactly 1 file. No header file definitions.
    Failure Indicators: 0 definitions (undefined symbol) or >1 definition (linker error).
    Evidence: .sisyphus/evidence/task-6-single-def.log
  ```

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 7. **Define IPC data format — shared header for cross-core communication**

  **What to do**:
  - Create `project/code/ipc_protocol.h` (shared header included by both cores):
    ```c
    #ifndef _ipc_protocol_h_
    #define _ipc_protocol_h_
    #include "zf_common_headfile.h"
    
    // IPC消息类型定义
    #define IPC_MSG_TYPE_VISION      0x01   // 视觉数据包
    #define IPC_MSG_TYPE_HEARTBEAT   0x02   // 心跳/存活检测
    
    // 视觉数据包 (通过 ipc_send_data 发送的 uint32 编码)
    // 使用 union 实现 float ↔ uint32 安全转换
    typedef union {
        float    f_value;
        uint32   u_value;
    } ipc_float_converter_t;
    
    // 视觉IPC消息结构 (通过ipc_send_data发送，clientId=IPC_MSG_TYPE_VISION)
    // communicate_data_t: { clientId, data }
    // data字段打包格式:
    //   [31:24] 保留
    //   [23:16] 数据质量 (0-255, 255=最佳)
    //   [15:0]  image_error 的定点表示 (乘1000取整, 单位: 千分之一像素)
    #define IPC_VISION_PACK(error_float, quality) \
        ((uint32)(((uint32)((uint8)(quality)) << 16) | \
                  ((uint32)((int16)((error_float) * 1000.0f)) & 0xFFFF)))
    
    #define IPC_VISION_UNPACK_ERROR(packed)  ((float)((int16)((packed) & 0xFFFF)) / 1000.0f)
    #define IPC_VISION_UNPACK_QUALITY(packed) ((uint8)(((packed) >> 16) & 0xFF))
    
    #endif
    ```
  - Include `ipc_protocol.h` from `vision.h` (CM7_1) and `control.h` (CM7_0)
  - Document the encoding: `image_error` is a float in pixel units, packed as 16-bit signed fixed-point (×1000) with 8-bit quality in upper byte

  **Must NOT do**:
  - Do NOT use `memcpy` or pointer casting for float→uint32 — use the union (strict aliasing safe)
  - Do NOT define variables in this header — only macros and type definitions

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Single header file with macros and a union typedef
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with T1-T6)
  - **Blocks**: T8 (IPC callback unpacks data), T9 (IPC send packs data), T15 (control pipeline reads image_error)
  - **Blocked By**: None

  **References**:
  - `libraries/zf_driver/zf_driver_ipc.h` — `communicate_data_t` struct: `{ uint32 clientId; uint32 data; }`
  - `libraries/zf_driver/zf_driver_ipc.c` — `ipc_send_data()` sends `communicate_data_t` to other core
  - `libraries/zf_common/zf_common_typedef.h` — `uint8`, `uint16`, `uint32`, `int16` types
  - **WHY**: `ipc_send_data()` sends one 32-bit word. CM7_1 needs to encode `image_error` (float) + quality (uint8) into a single uint32. The union avoids undefined behavior from pointer aliasing.

  **Acceptance Criteria**:
  - [ ] `project/code/ipc_protocol.h` exists with `IPC_VISION_PACK` and `IPC_VISION_UNPACK_ERROR` macros
  - [ ] `grep -c "ipc_protocol.h" project/code/vision.h` returns ≥1 (included by CM7_1)
  - [ ] `grep -c "ipc_protocol.h" project/code/control.h` returns ≥1 (included by CM7_0)

  **QA Scenarios**:

  ```
  Scenario: IPC protocol header exists and defines packing macros
    Tool: Bash (grep)
    Preconditions: ipc_protocol.h created
    Steps:
      1. grep -c "IPC_VISION_PACK" project/code/ipc_protocol.h
      2. grep -c "IPC_VISION_UNPACK_ERROR" project/code/ipc_protocol.h
      3. grep -c "ipc_float_converter_t" project/code/ipc_protocol.h
    Expected Result: All three return ≥1.
    Failure Indicators: Any return 0 (missing macro/typedef).
    Evidence: .sisyphus/evidence/task-7-ipc-protocol.log

  Scenario: Header included by both cores
    Tool: Bash (grep)
    Preconditions: Includes added to vision.h and control.h
    Steps:
      1. grep -c "ipc_protocol.h" project/code/vision.h
      2. grep -c "ipc_protocol.h" project/code/control.h
    Expected Result: Both return ≥1.
    Failure Indicators: Either returns 0.
    Evidence: .sisyphus/evidence/task-7-ipc-includes.log
  ```

  **Commit**: NO (groups with Wave 1 batch)

---


---

  - [x] 2. **Clean CM7_0 ISR — remove TSL1401 CCD handler**

  **What to do**:
  - In `project/user/cm7_0_isr.c`, find `pit0_ch21_isr()` function
  - Remove the call to `tsl1401_collect_pit_handler()` from the ISR body
  - Replace with a comment: `// PIT_CH21 owned by CM7_1 for CCD — handler removed`
  - Keep the ISR function stub (do NOT delete the function — it's referenced by the interrupt vector table)
  - The function should become:
    ```c
    void pit0_ch21_isr(void)
    {
        // PIT_CH21 owned by CM7_1 for CCD — handler removed from CM7_0
        pit_clear_flag(PIT_CH21);
        __DSB();
    }
    ```

  **Must NOT do**:
  - Do NOT delete the `pit0_ch21_isr()` function entirely — it's in the vector table
  - Do NOT modify any other PIT ISR handlers
  - Do NOT touch `cm7_1_isr.c`

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Single-function cleanup in one file
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with T1, T3-T7)
  - **Blocks**: F3 (ISR cleanup verification), T13 (runs on CM7_0)
  - **Blocked By**: None

  **References**:
  - `project/user/cm7_0_isr.c` — Lines containing `pit0_ch21_isr` and `tsl1401_collect_pit_handler`
  - `libraries/zf_device/zf_device_tsl1401.h` — `tsl1401_collect_pit_handler()` declaration
  - `libraries/sdk/common/src/drivers/pit/cy_pit.h` — `pit_clear_flag()` API
  - **WHY**: Both cores have identical ISR files. Only CM7_1 should own CCD hardware. CM7_0 must NOT call TSL1401 functions to avoid peripheral access conflict.

  **Acceptance Criteria**:
  - [ ] `grep -c "tsl1401_collect_pit_handler" project/user/cm7_0_isr.c` returns 0
  - [ ] `grep -c "pit0_ch21_isr" project/user/cm7_0_isr.c` returns ≥1 (function still exists)

  **QA Scenarios**:

  ```
  Scenario: CCD handler removed from CM7_0 ISR
    Tool: Bash (grep)
    Preconditions: cm7_0_isr.c edited
    Steps:
      1. grep -c "tsl1401_collect_pit_handler" project/user/cm7_0_isr.c
      2. grep -n "pit0_ch21_isr" project/user/cm7_0_isr.c
    Expected Result: Step 1 returns 0. Step 2 finds the function with only pit_clear_flag + __DSB inside.
    Failure Indicators: Step 1 returns >0 (CCD handler still present).
    Evidence: .sisyphus/evidence/task-2-cm7_0_isr-clean.log
  ```

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 3. **Clean CM7_1 ISR — remove non-CCD sensor handlers**

  **What to do**:
  - In `project/user/cm7_1_isr.c`, remove the following handlers (they belong to CM7_0):
    1. `uart1_isr()` — remove `wireless_module_uart_handler()` call → keep `uart_clear_flag(UART_1); __DSB();`
    2. `uart2_isr()` — remove `gnss_uart_callback()` call → keep `uart_clear_flag(UART_2); __DSB();`
    3. `uart4_isr()` — remove `uart_receiver_handler()` call → keep `uart_clear_flag(UART_4); __DSB();`
  - Keep `uart0_isr()` for debug UART (both cores may use it)
  - Keep `uart3_isr()`, `uart5_isr()`, `uart6_isr()` (empty, no harm)
  - Add a comment in each cleaned ISR: `// Handler removed — peripheral owned by CM7_0`
  - Keep ALL PIT ISRs — CM7_1 needs PIT_CH21 for CCD

  **Must NOT do**:
  - Do NOT remove the ISR functions entirely — they're in the vector table
  - Do NOT modify `uart0_isr()` (debug)
  - Do NOT touch `cm7_0_isr.c`

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Remove 3 function calls across 3 ISR handlers in one file
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with T1-T2, T4-T7)
  - **Blocks**: F3 (ISR cleanup verification)
  - **Blocked By**: None

  **References**:
  - `project/user/cm7_1_isr.c` — Lines containing `uart1_isr`, `uart2_isr`, `uart4_isr`
  - `libraries/zf_device/zf_device_wireless_uart.h` — `wireless_module_uart_handler()` (belongs on CM7_0)
  - `libraries/zf_device/zf_device_gnss.h` — `gnss_uart_callback()` (belongs on CM7_0)
  - `libraries/zf_device/zf_device_uart_receiver.h` — `uart_receiver_handler()` (belongs on CM7_0)
  - `libraries/zf_driver/zf_driver_uart.h` — `uart_clear_flag()` API
  - **WHY**: CM7_1 must not accidentally service interrupts for peripherals owned by CM7_0. If both cores service the same UART interrupt, data corruption results.

  **Acceptance Criteria**:
  - [ ] `grep -c "wireless_module_uart_handler" project/user/cm7_1_isr.c` returns 0
  - [ ] `grep -c "gnss_uart_callback" project/user/cm7_1_isr.c` returns 0
  - [ ] `grep -c "uart_receiver_handler" project/user/cm7_1_isr.c` returns 0
  - [ ] `grep -c "uart1_isr\|uart2_isr\|uart4_isr" project/user/cm7_1_isr.c` returns ≥3 (functions still exist)

  **QA Scenarios**:

  ```
  Scenario: Non-CCD sensor handlers removed from CM7_1 ISR
    Tool: Bash (grep)
    Preconditions: cm7_1_isr.c edited
    Steps:
      1. grep -c "wireless_module_uart_handler" project/user/cm7_1_isr.c
      2. grep -c "gnss_uart_callback" project/user/cm7_1_isr.c
      3. grep -c "uart_receiver_handler" project/user/cm7_1_isr.c
    Expected Result: All three return 0. UART ISR functions still exist with only clear_flag + __DSB.
    Failure Indicators: Any grep returns >0 (sensor handler still present on CM7_1).
    Evidence: .sisyphus/evidence/task-3-cm7_1_isr-clean.log
  ```

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 4. **Add 6 unlinked .c files to CM7_0 IAR project**

  **What to do**:
  - Edit `project/iar/project_config/cyt4bb7_cm_7_0.ewp` (XML format)
  - Add the following files to the `<group><name>code</name>` section:
    ```xml
    <file><name>$PROJ_DIR$\..\..\code\control.c</name></file>
    <file><name>$PROJ_DIR$\..\..\code\foc.c</name></file>
    <file><name>$PROJ_DIR$\..\..\code\euler.c</name></file>
    <file><name>$PROJ_DIR$\..\..\code\pid.c</name></file>
    <file><name>$PROJ_DIR$\..\..\code\timer_control.c</name></file>
    <file><name>$PROJ_DIR$\..\..\code\servo_kinematics.c</name></file>
    ```
  - Verify the paths relative to the .ewp location: `$PROJ_DIR$` resolves to `project/iar/project_config/`, so `..\..\code\` resolves to `project/code/`
  - Check existing entries (servo.c, small_driver_uart_control.c) for correct XML indentation and format

  **Must NOT do**:
  - Do NOT add these files to the CM7_1 .ewp — they belong exclusively to CM7_0
  - Do NOT change the group structure or add new groups

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: XML file editing — trivial insertions
  - **Skills**: []
  - **Skills Evaluated but Omitted**: None needed

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with T1-T3, T5-T7)
  - **Blocks**: T8, T9 (IPC init needs these compiled), T13 (1kHz ISR uses control.c/pid.c/euler.c)
  - **Blocked By**: None

  **References**:
  - `project/iar/project_config/cyt4bb7_cm_7_0.ewp` — IAR project XML, find `<group><name>code</name>` section
  - Existing entries: `<file><name>$PROJ_DIR$\..\..\code\servo.c</name></file>` and `<file><name>$PROJ_DIR$\..\..\code\small_driver_uart_control.c</name></file>` — copy exact format
  - `project/code/control.c`, `project/code/foc.c`, `project/code/euler.c`, `project/code/pid.c`, `project/code/timer_control.c`, `project/code/servo_kinematics.c` — verify these files exist on disk
  - **WHY**: These 6 files are written but not compiled. Without .ewp entries, the linker won't find their symbols and the build fails.

  **Acceptance Criteria**:
  - [ ] `grep -c "control.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp` returns ≥1
  - [ ] `grep -c "foc.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp` returns ≥1
  - [ ] `grep -c "euler.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp` returns ≥1
  - [ ] `grep -c "pid.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp` returns ≥1
  - [ ] `grep -c "timer_control.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp` returns ≥1
  - [ ] `grep -c "servo_kinematics.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp` returns ≥1

  **QA Scenarios**:

  ```
  Scenario: All 6 files added to CM7_0 .ewp
    Tool: Bash (grep)
    Preconditions: cyt4bb7_cm_7_0.ewp edited
    Steps:
      1. for f in control.c foc.c euler.c pid.c timer_control.c servo_kinematics.c; do
           grep -c "$f" project/iar/project_config/cyt4bb7_cm_7_0.ewp
         done
    Expected Result: Each file returns count ≥1. All files found in the code group section.
    Failure Indicators: Any file returns 0.
    Evidence: .sisyphus/evidence/task-4-ewp-files.log

  Scenario: Files NOT in CM7_1 .ewp
    Tool: Bash (grep)
    Preconditions: cyt4bb7_cm_7_1.ewp unmodified
    Steps:
      1. grep -c "control.c\|foc.c\|servo_kinematics.c" project/iar/project_config/cyt4bb7_cm_7_1.ewp
    Expected Result: Returns 0 for each (these files must NOT be in CM7_1 project).
    Failure Indicators: Any motion file found in CM7_1 .ewp.
     Evidence: .sisyphus/evidence/task-4-cm7_1-no-motion.log
  ```

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 5. **Create CM7_1 vision application skeleton + add to CM7_1 IAR project**

  **What to do**:
  - Create `project/code/vision.h` with ternarization threshold macros, function declarations, and global variables (`tsl1401_finish_flag`, `ccd_ternary[128]`, `image_error_cm7_1`)
  - Create `project/code/vision.c` with `vision_init()`, `vision_ternarize()`, `vision_calc_line_error()`, `vision_send_to_cm7_0()` stubs
  - Set `tsl1401_finish_flag = 1` in `pit0_ch21_isr` in `cm7_1_isr.c`, clear it in main loop after processing
  - Add `vision.c` to `cyt4bb7_cm_7_1.ewp` code group
  - In `main_cm7_1.c`: call `vision_init()` after clock init; in `while(true)`: check flag → process → IPC send → clear flag

  **Must NOT do**: No IPC calls from PIT ISR, no vision algorithms beyond ternarization

  **Recommended Agent Profile**: `deep` — Multi-file creation + IAR project + ISR wiring
  **Parallelization**: Wave 1 (with T1-T4, T6-T7). Blocks: T10, T14. Blocked by: None.

  **References**: `project/user/main_cm7_1.c`, `project/user/cm7_1_isr.c`, `libraries/zf_device/zf_device_tsl1401.h`, `libraries/zf_driver/zf_driver_ipc.h`, `project/iar/project_config/cyt4bb7_cm_7_1.ewp`

  **Acceptance Criteria**:
  - [ ] `project/code/vision.h` and `vision.c` exist
  - [ ] `grep -c "vision.c" project/iar/project_config/cyt4bb7_cm_7_1.ewp` returns ≥1
  - [ ] `grep -c "ipc_send_data" project/user/cm7_1_isr.c` returns 0 (no IPC in ISR)

  **QA**: grep vision files + .ewp entry + no IPC in ISR. Evidence: `.sisyphus/evidence/task-5-vision-skeleton.log`

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 6. **Declare volatile globals for cross-core data + state estimator variables**

  **What to do**:
  - In `control.h`, add `volatile` to `image_error`, `v_hat`, `x_hat` extern declarations
  - In `control.c`, define all three as `volatile float ... = 0.0f` — exactly ONE definition point
  - `volatile` is CRITICAL: `image_error` is written by IPC callback (background) and read by 1kHz ISR (interrupt context)

  **Must NOT do**: No header file definitions (multiple definition linker error), no missing `volatile`

  **Recommended Agent Profile**: `quick` — Add qualifier + ensure single definition
  **Parallelization**: Wave 1 (with T1-T5, T7). Blocks: T8, T11, T13. Blocked by: None.

  **References**: `project/code/control.h:76-79`, `project/code/control.c`

  **Acceptance Criteria**:
  - [ ] `grep "volatile.*image_error" project/code/control.h` returns match
  - [ ] Exactly one .c file defines each variable

  **QA**: grep volatile declarations in control.h. Evidence: `.sisyphus/evidence/task-6-volatile-globals.log`

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 7. **Define IPC data format — shared header for cross-core communication**

  **What to do**:
  - Create `project/code/ipc_protocol.h` with:
    - `IPC_MSG_TYPE_VISION` / `IPC_MSG_TYPE_HEARTBEAT` macros
    - `ipc_float_converter_t` union for safe float↔uint32 conversion
    - `IPC_VISION_PACK(error_float, quality)` macro: packs float image_error (×1000, 16-bit signed) + quality (8-bit) into uint32
    - `IPC_VISION_UNPACK_ERROR(packed)` and `IPC_VISION_UNPACK_QUALITY(packed)` unpack macros
  - Include `ipc_protocol.h` from both `vision.h` (CM7_1) and `control.h` (CM7_0)

  **Must NOT do**: No variable definitions in header, no pointer-casting for float↔uint32 (use union)

  **Recommended Agent Profile**: `quick` — Single header with macros and union typedef
  **Parallelization**: Wave 1 (with T1-T6). Blocks: T8, T9, T15. Blocked by: None.

  **References**: `libraries/zf_driver/zf_driver_ipc.h` (communicate_data_t struct), `libraries/zf_common/zf_common_typedef.h` (types)

  **Acceptance Criteria**:
  - [ ] `project/code/ipc_protocol.h` exists with `IPC_VISION_PACK` and `IPC_VISION_UNPACK_ERROR` macros
  - [ ] Included by both `vision.h` and `control.h`

  **QA**: grep for packing macros. Evidence: `.sisyphus/evidence/task-7-ipc-protocol.log`

  **Commit**: NO (groups with Wave 1 batch)

---

  - [x] 8. **IPC init on CM7_0 — register callback that updates volatile image_error**

  **What to do**:
  - In `main_cm7_0.c`, call `ipc_communicate_init(IPC_PORT_1, ipc_cm7_0_callback)` after clock init
  - Implement callback: unpack received uint32 via `IPC_VISION_UNPACK_ERROR()`, write to `volatile float image_error`
  - Callback runs in background context (not ISR) — zf_driver_ipc triggers it after receiving

  **Must NOT do**: No `ipc_send_data()` in callback, no IPC in any CM7_0 ISR

  **Recommended Agent Profile**: `quick` — IPC init call + callback implementation
  **Parallelization**: Wave 2 (with T9-T12). Blocks: T15. Blocked by: T4, T6, T7.

  **References**: `libraries/zf_driver/zf_driver_ipc.h`, `project/user/main_cm7_0.c`, `project/code/ipc_protocol.h`, `project/code/control.h`

  **Acceptance Criteria**:
  - [ ] `grep -c "ipc_communicate_init.*IPC_PORT_1" project/user/main_cm7_0.c` returns ≥1
  - [ ] `grep -c "IPC_VISION_UNPACK_ERROR" project/user/main_cm7_0.c` returns ≥1
  - [ ] `grep -c "ipc_send_data" project/user/cm7_0_isr.c` returns 0

  **QA**: grep ipc_communicate_init + unpack macro, verify no IPC in ISR. Evidence: `.sisyphus/evidence/task-8-ipc-cm7_0.log`

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 9. **IPC init on CM7_1 — setup for send in main loop**

  **What to do**:
  - In `main_cm7_1.c`, call `ipc_communicate_init(IPC_PORT_2, ipc_cm7_1_callback)` after clock init
  - Implement `vision_send_to_cm7_0()` in vision.c: `ipc_send_data(IPC_VISION_PACK(image_error_cm7_1, 255))`
  - Called from main loop (NOT from PIT ISR)

  **Must NOT do**: No IPC calls from PIT ISR

  **Recommended Agent Profile**: `quick` — IPC init + send wrapper
  **Parallelization**: Wave 2 (with T8, T10-T12). Blocks: T14. Blocked by: T4, T7.

  **References**: `libraries/zf_driver/zf_driver_ipc.h`, `project/user/main_cm7_1.c`, `project/code/ipc_protocol.h`

  **Acceptance Criteria**:
  - [ ] `grep -c "ipc_communicate_init.*IPC_PORT_2" project/user/main_cm7_1.c` returns ≥1
  - [ ] `grep -c "IPC_VISION_PACK" project/code/vision.c` returns ≥1
  - [ ] `grep -c "ipc_send_data" project/user/cm7_1_isr.c` returns 0

  **QA**: grep ipc init + pack macro, verify no IPC in ISR. Evidence: `.sisyphus/evidence/task-9-ipc-cm7_1.log`

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 10. **Implement CCD ternarization algorithm in vision.c**

  **What to do**:
  - `vision_ternarize()`: Read 128-pixel raw array from `tsl1401_get_buffer()`, apply dual-threshold: >CCD_THRESHOLD_HIGH→WHITE, <CCD_THRESHOLD_LOW→BLACK, else→EDGE
  - `vision_calc_line_error()`: Weighted centroid of edge/white pixels, compute offset from CCD center (64), return as `image_error_cm7_1` in pixel units
  - Wire in `main_cm7_1.c` while loop: check flag → ternarize → calc error → IPC send → clear flag

  **Must NOT do**: No IPC in computation functions, no line-following logic

  **Recommended Agent Profile**: `deep` — Algorithm implementation (dual-threshold classification + weighted centroid)
  **Parallelization**: Wave 2 (with T8-T9, T11-T12). Blocks: T14. Blocked by: T5.

  **References**: `project/code/vision.h`, `libraries/zf_device/zf_device_tsl1401.h`, `project/user/main_cm7_1.c`

  **Acceptance Criteria**:
  - [ ] `vision_ternarize()` iterates 128 pixels with dual-threshold
  - [ ] `vision_calc_line_error()` computes centroid offset from center
  - [ ] Processing sequence in main loop: flag → ternarize → calc → send → clear

  **QA**: grep for CCD_THRESHOLD + 128-pixel loop + centroid logic. Evidence: `.sisyphus/evidence/task-10-ternarization.log`

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 11. **Implement state estimator — complementary filter for v_hat/x_hat**

  **What to do**:
  - Create `state_estimator.h/c`: `state_estimator_init()` zeros v_hat/x_hat; `state_estimator_update(pitch, motor_accel, dt)` runs complementary filter: `v_pred = v_hat + motor_accel*dt; v_correction = pitch*GAIN; v_hat = 0.95*v_pred + 0.05*v_correction; x_hat += v_hat*dt`
  - Add `state_estimator.c` to CM7_0 .ewp
  - Call `state_estimator_init()` in CM7_0 main; call `state_estimator_update()` in 1kHz ISR
  - Mark gains as TODO (need hardware calibration)

  **Must NOT do**: No Kalman filter, no sensor reads inside update function

  **Recommended Agent Profile**: `deep` — New module (2 files) + algorithm + IAR registration
  **Parallelization**: Wave 2 (with T8-T10, T12). Blocks: T13. Blocked by: T6.

  **References**: `project/code/control.h` (v_hat/x_hat), `libraries/zf_device/zf_device_imu660ra.h`

  **Acceptance Criteria**:
  - [ ] `state_estimator.h/c` exist with complementary filter
  - [ ] Added to CM7_0 .ewp, init called in main

  **QA**: grep for v_pred + v_correction + fusion weights. Evidence: `.sisyphus/evidence/task-11-state-estimator.log`

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 12. **Implement IMU→Euler fusion — raw gyro data to Euler angles**

  **What to do**:
  - Rewrite `euler.c`: `euler_init()` performs 100-sample gyro offset calibration then calls `imu660ra_init()`; `euler_update(dt)` reads gyro, subtracts offset, integrates into `euler_angle.pitch/roll/yaw`
  - Call `euler_init()` in CM7_0 main; call `euler_update(dt)` in 1kHz ISR
  - Mark as TODO: add accelerometer correction (Mahony/Madgwick) to fix long-term drift

  **Must NOT do**: No full Mahony/Madgwick filter yet — pure gyro integration with offset calibration only

  **Recommended Agent Profile**: `deep` — Sensor fusion algorithm + IMU driver integration
  **Parallelization**: Wave 2 (with T8-T11). Blocks: T13. Blocked by: T4.

  **References**: `project/code/euler.c`, `project/code/euler.h`, `libraries/zf_device/zf_device_imu660ra.h`

  **Acceptance Criteria**:
  - [ ] `euler_init()` does 100-sample gyro offset calibration
  - [ ] `euler_update(dt)` integrates gyro into euler_angle
  - [ ] `grep -c "euler_init" project/user/main_cm7_0.c` returns ≥1

  **QA**: grep gyro_offset + 100-sample loop + imu660ra_get_gyro. Evidence: `.sisyphus/evidence/task-12-euler-fusion.log`

  **Commit**: NO (groups with Wave 2 batch)

---

  - [x] 13. **Wire 1kHz PIT-driven motion control ISR on CM7_0**

  **What to do**:
  - In `main_cm7_0.c`: `pit_init(PIT_CH0, 1000)` — 1kHz timer (use PIT_CH0, NOT PIT_CH21 owned by CM7_1)
  - In `cm7_0_isr.c` `pit0_ch0_isr`: 8-step pipeline at dt=0.001s:
    1. `euler_update(dt)` — IMU→Euler
    2. `small_driver_get_angle/speed()` — motor feedback
    3. `state_estimator_update(pitch, motor_accel, dt)` — state estimation
    4. `LQR_control()` — LQR state feedback (reads volatile image_error internally)
    5. `pid_ctrl_Run()` — PID cascade
    6. `foc_set_duty()` — motor FOC output
    7. `left_leg_control()` / `right_leg_control()` — servo kinematics
    8. `pit_clear_flag(PIT_CH0); __DSB();`
  - ISR must complete within 1ms (1000µs)

  **Must NOT do**: No IPC calls in ISR, no printf/system_delay, do NOT use PIT_CH21

  **Recommended Agent Profile**: `deep` — Complex 8-step ISR wiring
  **Parallelization**: Wave 3 (sequential). Blocks: T16. Blocked by: T1, T4, T6, T11, T12.

  **References**: `project/user/cm7_0_isr.c`, `project/user/main_cm7_0.c`, `libraries/sdk/common/src/drivers/pit/cy_pit.h`, `project/code/control.c`, `project/code/euler.h`, `project/code/state_estimator.h`

  **Acceptance Criteria**:
  - [ ] `pit_init(PIT_CH0, 1000)` in main_cm7_0.c
  - [ ] `pit0_ch0_isr` has all 8 pipeline steps
  - [ ] `grep -c "ipc_send_data" project/user/cm7_0_isr.c` returns 0

  **QA**: grep pit0_ch0_isr for all pipeline functions, verify no IPC in ISR. Evidence: `.sisyphus/evidence/task-13-1khz-isr.log`

  **Commit**: NO (groups with Wave 3 batch)

---

  - [x] 14. **Wire CM7_1 main loop — CCD trigger → ternarization → IPC send**

  **What to do**:
  - Complete `main_cm7_1.c` while loop: poll `tsl1401_finish_flag` → `vision_ternarize()` → `vision_calc_line_error()` → `vision_send_to_cm7_0()` → clear flag
  - CCD ~100Hz frame rate → loop spins idle 99% of time (no CPU load concern)
  - Includes: `vision.h`, `zf_driver_ipc.h`, `ipc_protocol.h`

  **Must NOT do**: No IPC send from PIT ISR, no blocking delays in while loop

  **Recommended Agent Profile**: `deep` — End-to-end vision pipeline wiring
  **Parallelization**: Wave 3 (sequential). Blocks: T15. Blocked by: T5, T9, T10.

  **References**: `project/user/main_cm7_1.c`, `project/user/cm7_1_isr.c`, `project/code/vision.h`

  **Acceptance Criteria**:
  - [ ] while loop: check flag → ternarize → calc error → IPC send → clear flag (correct order)
  - [ ] `grep -c "vision_send_to_cm7_0" project/user/cm7_1_isr.c` returns 0

  **QA**: grep while loop for pipeline order, verify IPC only in main loop. Evidence: `.sisyphus/evidence/task-14-cm7_1-main-loop.log`

  **Commit**: NO (groups with Wave 3 batch)

---

  - [x] 15. **Integrate IPC image_error into control pipeline (volatile read)**

  **What to do**:
  - In `control.c`: `LQR_control()` reads `volatile float image_error` directly
  - FIX `turn_control(float image_error)` at line 141: parameter shadows global. **Remove parameter** → `float turn_control(void)`, use global `image_error` internally
  - Add zero-load fallback: `if (fabsf(image_error) < 0.001f) return 0.0f;` — no valid vision data = straight line

  **Must NOT do**: No IPC calls in control functions, no mutex (volatile float read is atomic on CM7)

  **Recommended Agent Profile**: `deep` — Critical integration point with refactoring (remove shadowed parameter)
  **Parallelization**: Wave 3 (sequential). Blocks: T16. Blocked by: T7, T8, T14.

  **References**: `project/code/control.c:100-150` (LQR_control), `project/code/control.c:141` (turn_control param), `project/code/control.h:76`

  **Acceptance Criteria**:
  - [ ] `turn_control()` takes no parameter (uses global volatile image_error)
  - [ ] `image_error ≈ 0` produces no turn output
  - [ ] No IPC calls in control.c

  **QA**: grep turn_control signature + image_error usage in LQR_control. Evidence: `.sisyphus/evidence/task-15-lqr-integration.log`

  **Commit**: NO (groups with Wave 3 batch)

---

  - [x] 16. **Wire servo kinematics into control loop output**

  **What to do**:
  - Verify in `pit0_ch0_isr`: `foc_set_duty()` → `left_leg_control()` → `right_leg_control()` (motor first, then servos)
  - Confirm div-by-zero guard from T1 is in servo_kinematics.c
  - Verify FPU enabled (Startup_Init sets CPACR)

  **Must NOT do**: No servo calls from main loop, no calls without T1 guard

  **Recommended Agent Profile**: `deep` — Final integration verification
  **Parallelization**: Wave 3 (sequential). Blocks: F1-F4. Blocked by: T1, T13, T15.

  **References**: `project/user/cm7_0_isr.c`, `project/code/control.h`, `project/code/servo_kinematics.c`

  **Acceptance Criteria**:
  - [ ] Servo calls present in ISR after foc_set_duty
  - [ ] Div-by-zero guard verified in servo_kinematics.c

  **QA**: grep ISR for foc_set_duty → left_leg_control order. Evidence: `.sisyphus/evidence/task-16-servo-order.log`

  **Commit**: NO (groups with Wave 3 batch)

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 verification tasks run in PARALLEL. ALL must PASS.

  - [x] F1. **Build CM7_0 — IAR compile + link**

  **What to do**: Run IAR build for CM7_0 project. Verify exit code 0. Check `.out` and `.map` files exist.
  **Agent**: `unspecified-low`
  **QA**:
  ```
  Scenario: CM7_0 project builds successfully
    Tool: Bash (IarBuild.exe or file existence check)
    Steps:
      1. If IarBuild available: IarBuild.exe project/iar/project_config/cyt4bb7_cm_7_0.ewp -build Debug
         Else: verify .out and .map files exist in build output directory
      2. Check exit code or file existence
    Expected: Build succeeds OR .out/.map files present at expected paths
    Evidence: .sisyphus/evidence/f1-cm7_0-build.log
  ```

  - [x] F2. **Build CM7_1 — IAR compile + link**

  **What to do**: Run IAR build for CM7_1 project. Verify exit code 0. Check `.out` and `.map` files exist.
  **Agent**: `unspecified-low`
  **QA**:
  ```
  Scenario: CM7_1 project builds successfully
    Tool: Bash
    Steps: Same as F1 but for cyt4bb7_cm_7_1.ewp
    Expected: Build succeeds OR .out/.map files present
    Evidence: .sisyphus/evidence/f2-cm7_1-build.log
  ```

  - [x] F3. **ISR cleanup + IPC safety verification**

  **What to do**: Run all grep checks:
  - `grep -c "tsl1401_collect_pit_handler" project/user/cm7_0_isr.c` → must be 0
  - `grep -c "tsl1401_collect_pit_handler" project/user/cm7_1_isr.c` → must be 1
  - `grep -c "wireless_module_uart_handler\|gnss_uart_callback\|uart_receiver_handler" project/user/cm7_1_isr.c` → all must be 0
  - `grep -c "ipc_send_data\|ipc_communicate" project/user/cm7_0_isr.c` → must be 0
  - `grep -c "ipc_send_data\|ipc_communicate" project/user/cm7_1_isr.c` → must be 0
  **Agent**: `quick`
  **QA**: Run all grep commands, verify each returns expected value. Evidence: `.sisyphus/evidence/f3-isr-cleanup.log`

  - [x] F4. **Symbol + structure verification**

  **What to do**: Run checks:
  - CM7_0 .map: `image_error`, `v_hat`, `x_hat` defined (not "unresolved external")
  - CM7_1 .map: `tsl1401_finish_flag`, `ipc_send_data`, `vision_ternarize` defined
  - All 6 motion .c files in CM7_0 .ewp; none in CM7_1 .ewp
  - `vision.c` in CM7_1 .ewp
  - Div-by-zero guard present in servo_kinematics.c
  - `volatile` on all 3 cross-core globals
  **Agent**: `quick`
  **QA**: Run all grep/map checks. Evidence: `.sisyphus/evidence/f4-symbols.log`

---

## Commit Strategy

| Wave | Files | Message |
|------|-------|---------|
| 1 | servo_kinematics.c, cm7_0_isr.c, cm7_1_isr.c, cyt4bb7_cm_7_0.ewp, vision.c/h, control.h/c, ipc_protocol.h | `feat(multicore): add safety guards, ISR cleanup, vision skeleton, IPC protocol` |
| 2 | main_cm7_0.c, main_cm7_1.c, vision.c, state_estimator.c/h, euler.c | `feat(multicore): activate IPC, CCD ternarization, state estimator, IMU fusion` |
| 3 | cm7_0_isr.c, main_cm7_1.c, control.c | `feat(multicore): wire 1kHz ISR, vision main loop, IPC integration, servo output` |

---

## Success Criteria

### Verification Commands
```bash
# ISR cleanup
grep -c "tsl1401_collect_pit_handler" project/user/cm7_0_isr.c  # Expected: 0
grep -c "tsl1401_collect_pit_handler" project/user/cm7_1_isr.c  # Expected: 1
grep -c "ipc_send_data" project/user/cm7_0_isr.c                # Expected: 0
grep -c "ipc_send_data" project/user/cm7_1_isr.c                # Expected: 0

# .ewp entries
grep -c "control.c" project/iar/project_config/cyt4bb7_cm_7_0.ewp   # Expected: ≥1
grep -c "vision.c" project/iar/project_config/cyt4bb7_cm_7_1.ewp    # Expected: ≥1

# Safety guard
grep -c "L1_MM == 0.0f" project/code/servo_kinematics.c             # Expected: ≥1

# volatile globals
grep -c "volatile.*image_error" project/code/control.h               # Expected: ≥1
```

### Final Checklist
- [ ] All "Must Have" present (IPC not in ISR, div-by-zero guard, ISR cleanup, volatile globals)
- [ ] All "Must NOT Have" absent (no FreeRTOS, no CM0+ changes, no dual CCD init)
- [ ] Both IAR projects build without errors
- [ ] IPC data path: CM7_1 main loop → ipc_send_data → IPC PIPE → CM7_0 callback → volatile image_error → 1kHz ISR LQR_control

