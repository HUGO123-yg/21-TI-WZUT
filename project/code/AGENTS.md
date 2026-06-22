# APPLICATION CODE LAYER

**Generated:** 2026-06-18
**Scope:** `project/code/` — balance control, IMU fusion, menu UI, flash storage, peripherals.

## STRUCTURE

```
project/code/
├── Body_ctrl.c/h           # Balance cascade: PIT callback, car_state_calculate, steer control
├── Imu.c/h                 # Quaternion fusion, PID controllers (position/incremental), arctan2/arcsin
├── Menu.c/h                # 100-entry menu state machine via function-pointer key_table[]
├── Flash.c/h               # Navigation path recording/playback (3 paths, flash paging)
├── Common_peripherals.c/h  # BUZZER, keys, steer servos (4 channels), Car_param_t speed/mileage
├── small_driver_uart_control.c/h  # UART motor driver protocol (460800 baud, 7-byte frames)
├── Rotation.c/h             # In-place rotation: differential + servo counter-steer, elapsed decay
├── Stair_test.c/h           # Stair climbing: 3 modes (flat jump, single step, sequential 3-step)
└── 日志.txt                # Chinese-named dev log (cross-platform breakage risk)
```

## FILES BY LINE COUNT

| File | Lines | Domain |
|------|-------|--------|
| `Menu.c` | 932 | Menu UI — largest module |
| `Body_ctrl.c` | 607 | Balance + jump FSM |
| `Imu.c` | 419 | Quaternion fusion + PID |
| `Flash.c` | 397 | Navigation flash storage |
| `Common_peripherals.c` | 264 | HW abstraction |
| `small_driver_uart_control.c` | 184 | Motor UART protocol |
| `Stair_test.c` | 353 | Stair climbing test |
| `Rotation.c` | 86 | In-place rotation control |

## DEPENDENCY GRAPH

```
small_driver_uart_control.c/h      # Leaf module — only zf_common + zf_driver
  └──← Common_peripherals.c/h
Common_peripherals.c/h              # HW abstraction
  └──← Body_ctrl.c, Flash.c, Menu.c
Imu.c/h                             # Sensor fusion + control math
  └──← Body_ctrl.c, Flash.c (via Nag_Yaw macro), Menu.c
Flash.c/h                           # Navigation storage
  └──← Body_ctrl.c, Menu.c
Body_ctrl.c/h                       # Balance loop + jump FSM (CENTRAL HUB)
  └──← Menu.c (jump UI)
Menu.c/h                            # Top-level UI — depends on ALL others
```

## COMPLEXITY HOTSPOTS

| Hotspot | File | Lines | Why |
|---------|------|-------|-----|
| 7-state jump FSM | `Body_ctrl.c:356-516` | ~160 | IDLE→PREPARE→CHARGE→LAUNCH→AIRBORNE→LANDING→RECOVER with gyro detection |
| 100-entry menu table | `Menu.c:10-932` | 932 | 60+ near-identical callback functions, 3-level tree navigation |
| Cascaded PID | `Imu.c + Body_ctrl.c` | spread | Angle→AngularSpeed→Speed 3-stage PID with soft-start ramp |
| Flash paging FSM | `Flash.c` | 397 | 3-path save/replay with meta-page tracking, 5cm resolution |
| Quaternion fusion | `Imu.c:86-270` | ~185 | Adaptive filter (static/motion), fast initial convergence (KP=100) |

## WHERE TO LOOK

| File | Key Contents |
|------|-------------|
| `Body_ctrl.c/h` | `pit_call_back()` — 1ms/5ms control cascade. `car_state_calculate()` — tilt guard, PID soft-start. Steer `steer_control()` calls. Externs: `target_speed`, `left_motor_duty`, `right_motor_duty`, `STOP_FALG` |
| `Imu.c/h` | `quaternion_module_calculate()` — gyro+accel fusion → rotation matrix → roll/pitch/yaw. Static `arctan2()`, `arcsin()`, `arctan1()` helpers. `pid_control()` (position) + `pid_control_incremental()`. |
| `Menu.c/h` | `key_table table[100]` — each entry: current/up/down/enter + function pointer. ~60+ callback functions (`fun_a1`..`fun_e35`). `Menu()` dispatcher reads key events. |
| `Flash.c/h` | 3-path navigation storage (pages 3-95). `Nag` struct with mileage, angle, save index. `Nag_System()` — run, save, or GPS replay. |
| `Common_peripherals.c/h` | Pin defs (`BUZZER_PIN`, `KEY1`-`KEY4`, `STEER_1`-`4_PWM`). `Car_param_t` — speed_L/R, mileage_L/R. `steer_control_struct` per servo. |
| `small_driver_uart_control.c/h` | UART2 at 460800. `small_driver_set_duty()` / `small_driver_get_speed()` — 7-byte protocol to external motor driver. |
| `Rotation.c/h` | `rotation_start(dir, max_duty, duration_ms)` — initiates rotation. `rotation_run()` — called every 1ms from `pit_call_back`, drives differential motor duty + servo counter-steer with elapsed-based decay. States: ROT_IDLE, ROT_RUNNING, ROT_DONE. |
| `Stair_test.c/h` | Three test modes via centralized `#define STAIR_*` tunable parameters. `stair_jump_start/run/is_done()` — flat ground jump. `stair_single_start/run/is_done()` — single step. `stair_seq_start/run/is_done()` — sequential 3-step. `stair_abort()` — emergency stop all modes. |

## CONVENTIONS

- Every `.c` starts with `#include "zf_common_headfile.h"`
- Documentation via Chinese comment blocks: 函数功能 / 输入参数 / 返回值 / 使用示例 / 注意事项
- All modules coupled through global externs (no local encapsulation)
- `int` used for booleans (`jump_flag`, `run_state`) — no `stdbool.h`
- PID state held in `pid_cycle_struct` (p, i, d, i_value, out, incremental_data[2])

## NOTES

- **IMU selection**: `Imu.h` defaults `IMU_MODE` to `1` when the build does not define it, then gates declarations and implementation via `#if IMU_MODE`. Current active: imu660rb. Alternatives (commented out): imu660ra / imu963ra with per-sensor scale factors.
- **Duplicated constant**: `Body_ctrl.c` defines `#define WHEEL_CIRCUMFERENCE (6.4f)` — same value as `wheel_diameter` in `Common_peripherals.h`. Change one, miss the other.
- **`日志.txt`**: Chinese-named dev log file. Rename to `devlog.txt` for cross-platform git compatibility.
- **No test infra**: All testing is on-target via debug UART and `zf_assert()`.
- **Global coupling**: All modules coupled through global externs (no local encapsulation). Typical for bare-metal embedded but be aware when refactoring.
