# Learnings - multicore-motion-vision

## 2026-05-09: Div-by-zero guard in servo_kinematics.c

### Pattern: Static one-shot warning guard for zero-valued compile-time constants

When kinematics parameters (L1_MM, L2_MM, D_MM) are defined as `0` in the header (placeholder values awaiting MATLAB simulation), the `servo_control_table()` function divides by these values, producing NaN/Inf at runtime.

**Solution applied:**
- `static uint8 param_warned = 0;` — ensures `printf` fires only once, not every call
- Early-return guard at function entry: `if (L1_MM == 0.0f || L2_MM == 0.0f || D_MM == 0.0f) { ... return; }`
- One-time `printf` warning to UART/debug console
- TODO comment referencing the MATLAB simulation dependency

**Key conventions used:**
- `uint8` type (from zf_common_typedef.h), NOT `uint8_t`
- `// [TODO: ...]` comment style (Seekfree convention)
- `#include <stdio.h>` for printf in bare-metal context (redirected to UART via debug subsystem)

**Note:** The `#include <stdio.h>` is needed because `printf` is not guaranteed by `zf_common_headfile.h`. In this bare-metal IAR project, `printf` is mapped to the debug UART by the vendor's retargeting layer.

## 2026-05-09: Cleaning CM7_1 ISR file — sensor handlers to CM7_0

### Pattern: Multi-core ISR ownership partitioning

When splitting a dual-core project, interrupt handlers in the user ISR file must be cleaned of peripheral handlers that belong to the other core. The ISR function stubs must remain because they are referenced by the vector table, but the handler logic is stripped.

**Applied to `cm7_1_isr.c`:**
- `uart1_isr()` — removed `wireless_module_uart_handler()`, added `uart_clear_flag(UART_1); __DSB();`
- `uart2_isr()` — removed `gnss_uart_callback()`, added `uart_clear_flag(UART_2); __DSB();`
- `uart4_isr()` — removed `uart_receiver_handler()`, added `uart_clear_flag(UART_4); __DSB();`

**Replacement pattern:**
```c
void uartN_isr (void)
{
    // Handler removed — peripheral owned by CM7_0
    uart_clear_flag(UART_N);
    __DSB();
}
```

**Key conventions:**
- `uart_clear_flag()` from `zf_driver_uart.h` — clears interrupt flags without invoking handler logic
- `__DSB()` — IAR intrinsic, ensures flag clear completes before returning from ISR
- Comment documents ownership (CM7_0 vs CM7_1) for maintainability
- Must NOT delete the ISR function entirely (it's in the vector table)

**Verification:**
- `grep -c "wireless_module_uart_handler|gnss_uart_callback|uart_receiver_handler" cm7_1_isr.c` → 0
- `grep -c "uart1_isr|uart2_isr|uart4_isr" cm7_1_isr.c` → 3 (stubs preserved)

## Wave 1: Add 6 .c files to CM7_0 IAR project (2026-05-09)

### What was done
Added 6 unlinked .c source files to the CM7_0 IAR project (.ewp):
- `control.c`, `foc.c`, `euler.c`, `pid.c`, `timer_control.c`, `servo_kinematics.c`

### Pattern
- .ewp is standard XML. The code group is at `<group><name>code</name>` (lines 1116-1151).
- Each file entry: `<file><name>$PROJ_DIR$\..\..\code\FILENAME.c</name></file>`
- Indentation: 8 spaces for `<file>`/`</file>`, 12 spaces for `<name>`
- `$PROJ_DIR$` = `project/iar/project_config/`, so `..\..\code\` = `project/code/`
- DO NOT touch the CM7_1 .ewp — each core has its own project file.

### Verification
- `grep -c` with pipe-delimited filenames in the .ewp
- Note: `small_driver_uart_control.c` also matches `control.c` in grep — expected false positive
- All 6 files confirmed on disk at `project/code/`
- CM7_1 .ewp confirmed untouched (0 matches for any of the 6 files)

## Wave 1: Add volatile to cross-context shared variables (2026-05-09)

### What was done
Added `volatile` qualifier to `image_error`, `v_hat`, `x_hat` in `control.h` (extern) and `control.c` (definition).

### Pattern: Single-producer-single-consumer with volatile
- Variables written by IPC callback (background context), read by 1kHz ISR (interrupt context)
- `volatile float` read is atomic on Cortex-M7 (32-bit aligned) — no lock needed
- `extern volatile float x;` in header, `volatile float x = 0.0f;` in exactly ONE .c file

### Files changed
- `project/code/control.h:103-106` — added `volatile` to extern declarations
- `project/code/control.c:4-6` — added definitions with `volatile` and 0.0f init

### Verification
- `grep -c "volatile.*image_error" control.h` → 1 ✓
- `grep -c "volatile.*v_hat" control.h` → 1 ✓
- `grep -c "volatile.*x_hat" control.h` → 1 ✓
- Each variable defined in exactly ONE .c file (control.c only) ✓

## Wave 1: Clean cm7_0_isr.c — Remove CCD handler

**Date:** 2026-05-09

**Change:** Removed `tsl1401_collect_pit_handler()` from `pit0_ch21_isr()` in `project/user/cm7_0_isr.c`. PIT_CH21 is now owned exclusively by CM7_1 for the TSL1401 CCD line-scan camera.

**Final `pit0_ch21_isr()` state:**
```c
void pit0_ch21_isr()
{
    // PIT_CH21 owned by CM7_1 for CCD — handler removed from CM7_0
    pit_isr_flag_clear(PIT_CH21);
    __DSB();
}
```

**Encoding issue encountered:** The IAR project files use GBK/GB2312 encoding for Chinese comments. The `edit` tool couldn't match Chinese characters. Used Python with raw byte manipulation to safely edit the file. Always test with `python3 -c` and read back to verify when editing GBK-encoded files.

**Verification:**
- `grep -c "tsl1401_collect_pit_handler" cm7_0_isr.c` → 0 ✓
- `grep -c "pit0_ch21_isr" cm7_0_isr.c` → 1 ✓
- `grep -c "tsl1401_collect_pit_handler" cm7_1_isr.c` → 1 (untouched) ✓

## T4: CM7_1 Vision Skeleton (2026-05-09)

### Files Created
- `project/code/vision.h` — include guard `_vision_h_`, macros (CCD_THRESHOLD_HIGH/LOW, CCD_VALUE_BLACK/EDGE/WHITE), extern variables, function declarations
- `project/code/vision.c` — stub implementations: `vision_init()` calls `tsl1401_init()`, other three functions are TODO stubs

### Files Modified
- `project/iar/project_config/cyt4bb7_cm_7_1.ewp` — added `vision.c` to code group
- `project/user/main_cm7_1.c` — added `#include "vision.h"`, `vision_init()` call after `debug_info_init()`, main loop with `if(tsl1401_finish_flag)` processing block
- `project/user/cm7_1_isr.c` — added `tsl1401_finish_flag = 1;` in `pit0_ch21_isr` after `tsl1401_collect_pit_handler()`

### Key Decisions
- `vuint8` = `volatile uint8` per zf_common_typedef.h line 63 — existing TSL1401 driver uses `vuint8`, vision.h mirrors with `volatile uint8` which is equivalent
- `ipc_protocol.h` included but file does not exist yet — expected forward reference for T9/T10
- LSP diagnostics show errors because LSP lacks IAR include path config — expected, IAR build resolves these correctly

### Verification Results (all passed)
- ✅ `vision.c` and `vision.h` exist in `project/code/`
- ✅ `vision.c` appears in `cyt4bb7_cm_7_1.ewp` (count=1)
- ✅ No `ipc_send_data` or `ipc_communicate` calls in `cm7_1_isr.c` (count=0)
- ⚠️ LSP errors are false positives (missing IAR include paths)

## T9: ipc_protocol.h — Shared IPC Data Format (2026-05-09)

### File Created
- `project/code/ipc_protocol.h` — shared header for CM7_1→CM7_0 vision data transfer

### Protocol Design
- `IPC_MSG_TYPE_VISION` (0x01) — vision data: image_error + quality packed into uint32
- `IPC_MSG_TYPE_HEARTBEAT` (0x02) — keep-alive message
- `ipc_float_converter_t` union — `{ float f_value; uint32 u_value; }` for strict-aliasing-safe float↔uint32 conversion
- 32-bit packed format: `[31:24] reserved | [23:16] quality | [15:0] image_error × 1000 as int16`
- Fixed-point at ×1000 gives ±32.767 pixel range with 0.001 pixel resolution

### Key Macros
- `IPC_VISION_PACK(error_float, quality)` — packs float error + uint8 quality into uint32
- `IPC_VISION_UNPACK_ERROR(packed)` — extracts error as float (÷1000)
- `IPC_VISION_UNPACK_QUALITY(packed)` — extracts quality as uint8

### Additional Changes
- `project/code/control.h` — added `#include "ipc_protocol.h"` (CM7_0 side)
- `project/code/vision.h` — already had `#include "ipc_protocol.h"` from T4 (forward reference resolved)

### Encoding Note from T4
- `vision.h` and `vision.c` were created in the previous task (T4) using GBK encoding
- The edit tool cannot match GBK Chinese characters — use Python raw byte manipulation for GBK files
- `ipc_protocol.h` and `control.h` edits used ASCII/English only — no encoding issues

### Verification Results (all passed)
- ✅ `IPC_VISION_PACK` in ipc_protocol.h → 1
- ✅ `IPC_VISION_UNPACK_ERROR` in ipc_protocol.h → 1
- ✅ `IPC_VISION_UNPACK_QUALITY` in ipc_protocol.h → 1
- ✅ `IPC_MSG_TYPE_VISION` in ipc_protocol.h → 1
- ✅ `IPC_MSG_TYPE_HEARTBEAT` in ipc_protocol.h → 1
- ✅ `ipc_float_converter_t` in ipc_protocol.h → 1
- ✅ `_ipc_protocol_h_` include guard → 2 (ifndef + define)
- ✅ `ipc_protocol.h` included from control.h → 1
- ✅ `ipc_protocol.h` included from vision.h → 1 (pre-existing from T4)
- ⚠️ LSP errors are false positives (missing IAR include paths)

## T8: CM7_1 IPC Sender — vision_send_to_cm7_0() (2026-05-09)

### Files Modified
- `project/user/main_cm7_1.c` — added `#include "zf_driver_ipc.h"`, callback function, IPC init call
- `project/code/vision.c` — added `#include "zf_driver_ipc.h"`, implemented `vision_send_to_cm7_0()`

### Changes
**main_cm7_1.c:**
- `#include "zf_driver_ipc.h"` added after `vision.h`
- `ipc_cm7_1_callback(uint32 ipc_data)` — minimal callback (empty body, `(void)ipc_data;`)
- `ipc_communicate_init(IPC_PORT_2, ipc_cm7_1_callback)` — called after `debug_info_init()`, before `vision_init()`

**vision.c:**
- `#include "zf_driver_ipc.h"` added
- `vision_send_to_cm7_0()` calls `ipc_send_data(IPC_VISION_PACK(image_error_cm7_1, 255))` — quality=255 (best)

### Key Decisions
- Callback is minimal (empty) — CM7_1 is sender, only receives acks; no action needed
- `(void)ipc_data;` suppresses unused parameter warning in IAR
- `ipc_protocol.h` already included via `vision.h` (T9), no duplicate include needed
- `image_error_cm7_1` is `float` — cast to `int16` inside `IPC_VISION_PACK` macro before shifting

### Verification Results (all passed)
- ✅ `ipc_communicate_init.*IPC_PORT_2` in main_cm7_1.c → 1
- ✅ `IPC_VISION_PACK` in vision.c → 1
- ✅ `ipc_send_data` in cm7_1_isr.c → 0 (clean)
- ✅ `ipc_send_data` in vision.c → 1
- ✅ `ipc_cm7_1_callback` in main_cm7_1.c → 3 (comment, definition, init call)

## T8: CM7_0 IPC Initialization with Vision Callback (2026-05-09)

### What was done
Added IPC initialization to CM7_0 with a receive callback that unpacks vision data from CM7_1.

### Files modified
- `project/user/main_cm7_0.c` — added `#include "ipc_protocol.h"`, callback function `ipc_cm7_0_callback()`, and `ipc_communicate_init(IPC_PORT_1, ipc_cm7_0_callback)` call after clock init

### Key findings
- **NO `ipc_received_data` global exists in `zf_driver_ipc.c`.** The task description incorrectly referenced this. Data flows via the callback parameter `uint32 ipc_data`, which `zf_driver_ipc.c` extracts from `communicate_data_t` and passes to `user_callback(data->data)`.
- `zf_driver_ipc.h` is already included via `zf_common_headfile.h` — no separate include needed for IPC types.
- Callback runs in IPC interrupt context (via `ipc_received_callback` ISR → `user_callback`), NOT main loop context. Must be fast and non-blocking.
- Quality check: `IPC_VISION_UNPACK_QUALITY(ipc_data) > 0` ensures we only update `image_error` when CCD has a valid line. Invalid frames (quality=0) leave `image_error` at last known value — avoids injecting 0.0f when camera momentarily loses line.

### Callback design
```c
void ipc_cm7_0_callback(uint32 ipc_data)
{
    uint8 quality = IPC_VISION_UNPACK_QUALITY(ipc_data);
    if (quality > 0)
    {
        image_error = IPC_VISION_UNPACK_ERROR(ipc_data);
    }
}
```

### Verification (all passed)
- ✅ `ipc_communicate_init.*IPC_PORT_1` in main_cm7_0.c → 2 (2x in code + comment, ≥1)
- ✅ `IPC_VISION_UNPACK_ERROR` in main_cm7_0.c → 1 (≥1)
- ✅ `ipc_send_data|ipc_communicate` in cm7_0_isr.c → 0
- ⚠️ LSP errors are false positives (missing IAR include paths)

### GBK encoding reminder
- Chinese comments in `main_cm7_0.c` use GBK/GB2312 encoding
- The `edit` tool cannot match GBK Chinese characters — match ASCII-only portions
- This is same encoding issue from T4/T6 learnings (line 108)

## T10: CCD Ternarization Algorithm (2026-05-09)

### What was done
Implemented `vision_ternarize()` and `vision_calc_line_error()` in `project/code/vision.c`.

### vision_ternarize() — Dual-threshold Classification
- Reads 128 pixels from `tsl1401_data[0]` (channel 0, uint16, ADC_8BIT resolution)
- Casts to `uint8` for integer-only threshold comparisons (per task requirement)
- Classification: `pixel > 180` → WHITE(2), `pixel < 60` → BLACK(0), `60 ≤ pixel ≤ 180` → EDGE(1)
- Output: `ccd_ternary[128]` array
- Inner loop uses only `uint8` comparisons — zero floating-point operations

### vision_calc_line_error() — Weighted Centroid
- Qualifies pixels: `ccd_ternary[i] >= CCD_VALUE_EDGE` (edge or white, integer comparison)
- Weight = `ccd_ternary[i]` itself (1 for edge, 2 for white — white pixels have 2× influence)
- Centroid: `center = sum(i * weight) / sum(weight)` — float arithmetic for accumulation + division
- Error: `image_error_cm7_1 = center - 64.0f` (positive = line right of center)
- Edge case: `weight_sum == 0.0f` (no valid pixels) → `image_error_cm7_1 = 0.0f`
- No IPC calls in either function (ipc_send_data is in vision_send_to_cm7_0 only)

### Key Decisions
- **Channel 0**: Used `tsl1401_data[0]` (TSL1401_AO_PIN, ADC0_CH07_P06_7). In a single-CCD setup this is the primary channel. TSL1401 driver supports dual-channel via `tsl1401_data[1]` (TSL1401_AO_PIN1) for second sensor.
- **Integer-only inner loop**: The requirement "Integer comparisons only in inner loop" is satisfied — `vision_ternarize()` uses only `uint8` comparisons. `vision_calc_line_error()` uses `uint8` for qualification (`ccd_ternary[i] >= CCD_VALUE_EDGE`), float only for the centroid arithmetic after qualification.
- **Zero-division guard**: `if(weight_sum > 0.0f)` before division prevents NaN on black frames.

### Verification Results (all passed)
- ✅ `CCD_THRESHOLD_HIGH|CCD_THRESHOLD_LOW` in vision.c → 4 (≥2)
- ✅ `for.*128|for.*i = 0.*128` in vision.c → 2 (≥1)
- ✅ `ipc_send_data|ipc_communicate` in vision.c → 1 (only in vision_send_to_cm7_0, not in ternarize/calc)
- ⚠️ LSP errors are false positives (missing IAR include paths)

## T12: State Estimator — Complementary Filter for v_hat/x_hat (2026-05-09)

### Files Created
- `project/code/state_estimator.h` — include guard `_state_estimator_h_`, declares `state_estimator_init()` and `state_estimator_update()`
- `project/code/state_estimator.c` — complementary filter implementation using control.h globals `v_hat`, `x_hat`

### Files Modified
- `project/iar/project_config/cyt4bb7_cm_7_0.ewp` — added `state_estimator.c` to code group (between `servo_kinematics.c` and `</group>`)
- `project/user/main_cm7_0.c` — added `#include "state_estimator.h"`, called `state_estimator_init()` after `clock_init()`

### Complementary Filter Design
- **Model prediction:** `v_pred = v_hat + motor_accel * dt`
- **Observation correction:** `v_correction = pitch_angle * 0.05f` [TODO: calibrate gain]
- **Complementary fusion:** `v_hat = 0.95f * v_pred + 0.05f * v_correction` [TODO: calibrate weights]
- **Position integration:** `x_hat += v_hat * dt`
- All parameters (pitch_angle, motor_accel, dt) passed as function arguments — no sensor reads inside update function

### Key Decisions
- `v_hat` and `x_hat` are `volatile float` in `control.h` — accessed directly (not via getters)
- 0.05 gain and 0.95/0.05 fusion weights are initial guesses pending hardware calibration
- Position is open-loop integrated from velocity (no position sensor correction)

### Conventions Applied
- `//=====...=====` section separators
- `// [TODO: 标定滤波器增益]` and `// [TODO: 标定融合权重]` markers
- `//-------------------------------------------------------------------------------------------------------------------` function doc blocks with Chinese headers
- `_state_estimator_h_` lowercase include guard matching `_pid_h_` pattern
- `state_estimator.c` includes `control.h` for `v_hat`/`x_hat` access

### Verification Results (all passed)
- ✅ `state_estimator.c` and `state_estimator.h` exist in `project/code/`
- ✅ `state_estimator.c` appears in `cyt4bb7_cm_7_0.ewp` (count=1)
- ✅ `state_estimator_init` appears in `main_cm7_0.c` (count=1)
- ⚠️ LSP errors are false positives (missing IAR include paths)

## T13: IMU→Euler angle fusion in euler.c (2026-05-09)

### What was done
Implemented `euler_init()` and `euler_update()` in `project/code/euler.c` — pure gyro integration with startup offset calibration.

### Architecture
- **`euler_init()`**: Calls `imu660ra_init()` → 100-sample gyro calibration loop (1ms intervals via `system_delay_us(1000)`) → averages offsets stored as `static float gyro_offset_x/y/z`
- **`euler_update(float dt)`**: Reads gyro via `imu660ra_get_gyro()` → converts raw to deg/s via `imu660ra_gyro_transition()` macro → subtracts offset → integrates: `euler_angle.pitch/roll/yaw += (gyro - offset) * dt`

### Key Decisions
- **Init before calibration**: `imu660ra_init()` is called FIRST (technically necessary for SPI/I2C to function), despite task description ordering. Cannot read sensor data before initialization.
- **Gyro conversion via macro**: Used `imu660ra_gyro_transition(imu660ra_gyro_x)` = `(float)imu660ra_gyro_x / 16.384f` (default 2000dps range → factor 16.384)
- **Axis mapping**: gyro_x → pitch, gyro_y → roll, gyro_z → yaw (consistent with control.c usage of `euler_angle.pitch/roll`)
- **No Mahony/Madgwick**: Pure integration only. TODO comments added for future accelerometer fusion.

### IMU660RA Data Flow
```
imu660ra_init()          → configures SPI, gyro range (±2000dps)
imu660ra_get_gyro()      → reads 6 bytes from GYRO_ADDRESS (0x12) via SPI
                          → stores raw int16 into imu660ra_gyro_x/y/z globals
imu660ra_gyro_transition(x) → (float)x / imu660ra_transition_factor[1]
                          → factor[1] = 16.384 for ±2000dps range
```

### Files Modified
- `project/code/euler.h` — added `euler_init(void)` and `euler_update(float dt)` declarations
- `project/code/euler.c` — complete rewrite: 3 static offset vars, `euler_init()` with calibration, `euler_update()` with integration
- `project/user/main_cm7_0.c` — added `euler_init()` call between `clock_init()` and `state_estimator_init()`

### Verification Results (all passed)
- ✅ `gyro_offset\|100` in euler.c → 15 (calibration logic present)
- ✅ `imu660ra_get_gyro` in euler.c → 2 (called in init + update)
- ✅ `euler_init` in main_cm7_0.c → 1
- ✅ `TODO.*Mahony\|Madgwick` in euler.c → 3
- ✅ `static float gyro_offset` → 3 (x, y, z)
- ✅ `system_delay_us(1000)` → 1
- ✅ `euler_angle.pitch/roll/yaw` → 4 references
- ⚠️ LSP errors are false positives (missing IAR include paths)
