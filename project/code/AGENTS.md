# APPLICATION LAYER KNOWLEDGE BASE

## OVERVIEW
Robot application logic for a balance / smart car: attitude estimation, cascade PID, motor control, navigation path recording, and an LCD menu.

## STRUCTURE
```
project/code/
├── Body_ctrl.c/h                 — motor/servo control, balance cascade, PID loop
├── Common_peripherals.c/h        — hardware pin map, key/servo/display macros
├── Flash.c/h                     — on-chip flash parameters, navigation path storage
├── Imu.c/h                       — IMU conversion, quaternion attitude estimation
├── Menu.c/h                      — multi-level LCD menu state machine
├── small_driver_uart_control.c/h — UART stepper motor driver
└── 日志.txt                       — developer log (GBK Chinese)
```

## WHERE TO LOOK
| Task | File | Key Symbols |
|------|------|-------------|
| Add new robot behavior | `Body_ctrl.c` | `pit_call_back()`, `target_speed`, `STOP_FALG` |
| Tune balance/steering PID | `Imu.c` / `Imu.h` | `roll_balance_cascade`, `track_cascade`, `pid_control()` |
| Change servo pins or limits | `Common_peripherals.h` | `STEER_1_PWM`, `M_MAX`, `Car` |
| Record / replay a path | `Flash.c` / `Flash.h` | `N`, `Nag_PathSelect`, `Run_Nag_GPS()` |
| Add a menu page | `Menu.c` | `table[]`, `fun_a1()`...`fun_e35()` |
| Stepper motor driver | `small_driver_uart_control.c/h` | `motor_value`, `small_driver_set_duty()` |

## CONVENTIONS
- Application headers use `PascalCase.h` or `snake_case.h` (e.g., `Body_ctrl.h`, `Common_peripherals.h`).
- `cascade_value_struct` holds PID + quaternion state; `quaternion_module_calculate()` must be called before reading `posture_value`.
- `pit_call_back()` is the de-facto real-time loop; everything timing-critical lives there.

## ANTI-PATTERNS (THIS DIRECTORY)
1. **Monolithic ISR**: `pit_call_back()` in `Body_ctrl.c` couples attitude estimation, navigation flash I/O, and all motor PID into one PIT interrupt.
2. **Global cascade structs**: `roll_balance_cascade`, `pitch_balance_cascade`, and `track_cascade` are mutable globals shared across modules.
3. **Typo-prone flag**: `STOP_FALG` (missing 'G') is used throughout `Body_ctrl.c` and external callers.
4. **Flash wear during motion**: `Run_Nag_Save()` writes flash pages from the control loop; stalls or corruptions affect real-time response.
5. **Menu explosion**: `Menu.c` declares 50+ near-identical display functions and a hardcoded 100-entry jump table.
