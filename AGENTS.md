# PROJECT KNOWLEDGE BASE

**Generated:** 2026-06-18
**Commit:** no commits yet
**Branch:** main
**SDK:** SEEKFREE ZF Open Source Library V3.10.1 (GPLv3)
**IDE:** IAR Embedded Workbench for ARM 9.40.1

## OVERVIEW

Self-balancing smart car firmware for **Infineon Traveo II CYT4BB** (dual Cortex-M7 + CM0+). Bare-metal super-loop with interrupt-driven callbacks. Built exclusively with IAR EWARM — no Makefile, CMake, or CLI build path.

## STRUCTURE

```
test3/
├── libraries/
│   ├── sdk/              # Infineon/Cypress TVII-B-H-4M vendor SDK (DO NOT MODIFY)
│   ├── zf_common/        # Core shared hub — typedefs, debug, clock, fifo, interrupts
│   ├── zf_driver/        # MCU peripheral abstraction — GPIO, UART, SPI, PWM, ADC, etc.
│   ├── zf_device/        # External device drivers — IMUs, displays, cameras, wireless
│   ├── zf_components/    # Higher-level components (SeekFree assistant protocol)
│   └── doc/              # Version history, GPLv3 license
└── project/
    ├── code/             # Application logic — balance control, IMU fusion, menu, flash
    ├── user/             # Entry points — main() for CM7_0/CM7_1, ISRs
    └── iar/              # IAR IDE config — .ewp/.eww, linker script (.icf), debug XML
```

## ENTRY POINTS

### Boot Flow
```
POR → Reset_Handler (startup_cm7.s)
  ├─ Unlock PPB, set VTOR=ROM, enable ITCM/DTCM, ECC init
  ├─ Startup_Init()   → FPU, ECC for SRAM, VTOR→RAM, I/D-cache
  ├─ __iar_data_init3() → BSS zero, .data copy
  └─ main()
```

### Main Functions
| Core | File | Role |
|------|------|------|
| CM7_0 | `project/user/main_cm7_0.c:48` | **Primary** — clock→debug→IMU→flash→balance→PIT(1ms+5ms)→super-loop(`Menu()`) |
| CM7_1 | `project/user/main_cm7_1.c:47` | **Stub** — spins idle. Wired-up but unused. |

### Critical ISR Time Budget
| ISR | Period | File | Role |
|-----|--------|------|------|
| `pit0_ch0_isr` | **1ms** | `main_cm7_0.c:98` | Balance cascade: IMU→quaternion→PID→motor output |
| `pit0_ch1_isr` | **5ms** | `main_cm7_0.c:110` | Key scan |

### Interrupt Dispatch Chain
```
NVIC → CpuUserInt2_Handler (PIT)       → Cy_SystemIrqUserTable[] → pit0_chX_isr (15 ch)
NVIC → CpuUserInt3_Handler (UART)      → Cy_SystemIrqUserTable[] → uartX_isr (7 ch)
NVIC → IOSS GPIO ACT handler           → gpio_X_exti_isr (24 ch)
```
ISRs registered at runtime via `interrupt_init()` → `Cy_SysInt_SetSystemIrqVector()` — not statically in vector table.

### Application Callbacks (driven by ISRs)
| Callback | Called From | File | Role |
|----------|-------------|------|------|
| `pit_call_back()` | 1ms PIT ISR | `Body_ctrl.c:556` | Core control loop: IMU read, quaternion fusion, cascaded PID, motor/steer output, jump FSM, nav recording |
| `key_scan()` | 5ms PIT ISR | `Common_peripherals.c:50` | Debounced button GPIO read |
| `Menu()` | super-loop | `Menu.c:99` | 100-entry hierarchical UI via function-pointer table |

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add/modify application logic | `project/code/` | All `.c` files include `zf_common_headfile.h` |
| Change MCU peripheral init | `libraries/zf_driver/` | Wraps SDK APIs into simpler interfaces |
| Add a new sensor/device | `libraries/zf_device/` | Follow existing `zf_device_*.c/h` pattern |
| Change typedefs or common utils | `libraries/zf_common/` | Central hub — changes affect EVERY layer |
| Open project in IAR | `project/iar/cyt4bb7.eww` | Workspace links both CM7_0 + CM7_1 |
| Understand memory layout | `project/iar/icf/linker_directives_tviibh.icf` | 3-core layout: CM0+, CM7_0, CM7_1 |
| Version history | `libraries/doc/version.txt` | Chinese changelog v3.0.0 → v3.10.1 |
| License text | `libraries/doc/GPL3_permission_statement.txt` | ⚠️ Not at repo root |
| Boot sequence | `libraries/sdk/common/src/startup/iar/startup_cm7.s` → `startup.c` → `main()` | |
| Interrupt handlers | `project/user/cm7_0_isr.c` | PIT, UART, EXTI ISRs (IAR links by name) |
| Interrupt dispatch (SDK) | `libraries/sdk/tviibh4m/src/interrupts/cy_interrupt_map.c` | NVIC → CpuUserInt → Cy_SystemIrqUserTable dispatch |
| Runtime ISR registration | `libraries/zf_common/zf_common_interrupt.c` | `interrupt_init()` binds handlers at runtime |
| IAR project workspace | `project/iar/cyt4bb7.eww` | Dual-core workspace (CM7_0 + CM7_1) |
| Linker script | `project/iar/icf/linker_directives_tviibh.icf` | 3-core memory layout (CM0+/CM7_0/CM7_1) |

## CONVENTIONS

### Include Strategy
Every `.c` file starts with `#include "zf_common_headfile.h"` — a single monolithic header that pulls in SDK → zf_common → zf_driver → zf_device → zf_components → project/code in order.

### Type System (zf_common_typedef.h)
Use `uint8`, `uint16`, `uint32`, `int8`, `int16`, `int32` — NOT `uint8_t`/`int16_t`.
```c
#define ZF_ENABLE   (1)     // boolean macros
#define ZF_DISABLE  (0)
#define ZF_WEAK     __attribute__((weak))
```

### Naming
| Element | Convention | Example |
|---------|-----------|---------|
| Files | `zf_<layer>_<name>.c/h` (lib); `<Name>.c/h` (app) | `zf_driver_gpio.c`, `Body_ctrl.c` |
| Functions | `snake_case()` | `balance_cascade_init()`, `pit_call_back()` |
| Macros | `CAPITAL_SNAKE_CASE` | `BUZZER_PIN`, `M_MAX` |
| Types | `snake_case_struct` suffix | `steer_control_struct`, `pid_cycle_struct` |
| ISRs | `pitX_chY_isr()`, `uartX_isr()` | `pit0_ch0_isr()` |

### Helper Macros (zf_common_function.h)
```c
#define func_abs(x)             ((x) >= 0 ? (x): -(x))
#define func_limit(x, y)        ((x) > (y) ? (y) : ((x) < -(y) ? -(y) : (x)))
#define func_limit_ab(x, a, b)  ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))
```

### Float Literals
Always `f` suffix: `1.0f`, `0.001f`.

### Hardware Pin Definitions
Via `#define` in `Common_peripherals.h`: `#define BUZZER_PIN P19_4`

### Function Documentation
Chinese-structured comment blocks with: 函数功能 / 输入参数 / 返回值 / 使用示例 / 注意事项.

### Layer Dependency
```
project/user → project/code → zf_components → zf_device → zf_driver → zf_common → sdk
```
All flows upward. No circular dependencies. `zf_common_headfile.h` is the sole include path for app code.

## ANTI-PATTERNS (THIS PROJECT)

- **No LICENSE at root** — GPLv3 text is buried in `libraries/doc/`. Add `LICENSE` to repo root.
- **Chinese filename** — `project/code/日志.txt` breaks cross-platform tooling. Rename.
- **Stale header** — `zf_device_config.h` still references MM32F327X-G9P in its license block. Correct to CYT4BB.
- **Precompiled blobs** — `zf_device_config.a`/`.lib` in `libraries/zf_device/` with no build source. GPLv3 requires source.
- **Commented-out code** — Large dead code blocks in `Body_ctrl.c`, `main_cm7_0.c`, `Imu.c`. Delete or extract.
- **Magic numbers** — Raw values in servo/balance calculations. Use named constants.
- **`int` for booleans** — `int jump_flag`, `int run_state` instead of `bool`. stdbool.h is available.
- **No `const` on read-only params** — Pointer parameters that should be `const` are almost never marked.
- **No centralized `config.h`** — Compile-time settings scattered across multiple headers.
- **Libraries depend on project code** — `zf_common_headfile.h` (in `libraries/`) includes `Common_peripherals.h`, `Imu.h`, etc. (from `project/code/`). Reverse dependency violation.
- **Duplicated constant** — `WHEEL_CIRCUMFERENCE (6.4f)` in `Body_ctrl.c` vs `wheel_diameter (6.4f)` in `Common_peripherals.h`. Same value, different names — change one, miss the other.
- **`double` in float-only project** — `angle_plan()` in `Flash.c` returns `double`. FPU is single-precision only. Use `float`.
- **`uint8_t` inconsistency** — `Menu.c` uses `uint8_t` (via stdint.h) in 12 places instead of the project-standard `uint8` from `zf_common_typedef.h`.

## COMPLEXITY HOTSPOTS

| Hotspot | File | Lines | Why |
|---------|------|-------|-----|
| 7-state jump FSM | `Body_ctrl.c:356-516` | ~160 | IDLE→PREPARE→CHARGE→LAUNCH→AIRBORNE→LANDING→RECOVER with gyro detection |
| 100-entry menu table | `Menu.c:10-932` | 932 | 60+ near-identical callback functions, 3-level tree navigation |
| Cascaded PID | `Imu.c + Body_ctrl.c` | spread | Angle→AngularSpeed→Speed 3-stage PID with soft-start ramp |
| Flash paging FSM | `Flash.c` | 397 | 3-path save/replay with meta-page tracking, 5cm resolution |
| Quaternion fusion | `Imu.c:86-270` | ~185 | Adaptive filter (static/motion), fast initial convergence (KP=100) |

## FILES BY LINE COUNT (application code only)

| File | Lines | Domain |
|------|-------|--------|
| `Menu.c` | 932 | UI state machine |
| `Body_ctrl.c` | 607 | Balance + jump |
| `Imu.c` | 419 | Sensor fusion + PID |
| `Flash.c` | 397 | Navigation storage |
| `cm7_1_isr.c` | 440 | ISR stubs (unused) |
| `cm7_0_isr.c` | 432 | ISR implementations |
| `Common_peripherals.c` | 264 | HW abstraction |
| `small_driver_uart_control.c` | 184 | Motor UART protocol |

## COMMANDS

```bash
# Open in IAR (Windows only, IAR EWARM 9.40.1 required)
# Double-click: project/iar/cyt4bb7.eww

# Clean build artifacts
# Run: project/iar/删除临时文件IAR.bat  (Windows batch file)

# No CLI build available — IAR IDE only.
# For CI (Windows self-hosted runner):
# "C:\Program Files\IAR Systems\...\common\bin\iarbuild.exe" project\iar\project_config\cyt4bb7_cm_7_0.ewp -build Debug
```

## NOTES

- **Dual-core**: CM7_0 runs all application logic (balance, IMU, display, menu). CM7_1 is a wired-up stub — `main_cm7_1.c` spins idle. CM0+ runs from ROM only (no source in this project).
- **No RTOS** — Bare-metal super-loop with PIT timer interrupts (1ms + 5ms) driving the control cascade.
- **No test framework** — Testing is manual on-target via `zf_assert()` + debug UART (115200 baud). Assert prints file/line then halts.
- **IAR path hardcodes**: The auto-generated `.cspy.bat` contains absolute Windows paths. Ignore for portability.
- **Chinese comments**: All documentation and comments are in Chinese. Code identifiers are in English.
