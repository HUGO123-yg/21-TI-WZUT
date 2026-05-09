# PROJECT KNOWLEDGE BASE

## OVERVIEW

Dual-core user application for CYT4BB. CM7_0 is primary (sensors, control), CM7_1 is secondary. Application code goes here; library code stays in `../libraries/`.

## STRUCTURE

```
project/
├── code/                  # Application logic modules (auto-included by IAR project)
│   ├── servo.c/h          # Servo motor control
│   └── small_driver_uart_control.c/h  # UART control helper
├── user/                  # Entry points + interrupt handlers
│   ├── main_cm7_0.c       # CM7_0 main() — primary application
│   ├── main_cm7_1.c       # CM7_1 main() — secondary application
│   ├── cm7_0_isr.c        # CM7_0 user interrupt handlers
│   └── cm7_1_isr.c        # CM7_1 user interrupt handlers
└── iar/                   # IAR IDE configuration
    ├── cyt4bb7.eww        # Top-level workspace (double-click to open)
    ├── icf/
    │   └── linker_directives_tviibh.icf  # Memory layout: flash, SRAM, ITCM/DTCM per core
    └── project_config/    # Per-core IAR projects + debug settings
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add new application module | `project/code/` | Create `.c`/`.h` pair — auto-included in IAR build |
| Modify CM7_0 entry | `project/user/main_cm7_0.c` | `clock_init()` → `debug_init()` → user loop |
| Modify CM7_1 entry | `project/user/main_cm7_1.c` | `clock_init()` → `debug_info_init()` → user loop |
| Add CM7_0 interrupt handler | `project/user/cm7_0_isr.c` | PIT, UART, EXTI GPIO handlers |
| Add CM7_1 interrupt handler | `project/user/cm7_1_isr.c` | Same structure as CM7_0 |
| Change memory partitioning | `project/iar/icf/linker_directives_tviibh.icf` | Per-core flash/SRAM/ITCM/DTCM regions |
| Open in IDE | `project/iar/cyt4bb7.eww` | Double-click or File → Open Workspace |
| Multi-core debug | `cm7_0_cm7_1_debug.xml` | Loads both cores simultaneously |

## IAR BUILD

- **Workspace**: `project/iar/cyt4bb7.eww` (contains both CM7_0 + CM7_1 projects)
- **Build**: F7 (Make) — each core builds independently
- **Clean**: Run `project/iar/删除临时文件IAR.bat` or delete `Debug_m7_0/` and `Debug_m7_1/`
- **Target defines**: `CY_CORE_CM7_0` or `CY_CORE_CM7_1`, `tviibh4m`, `CYT4BB7CEE`
- **Debug**: Use `cm7_0_cm7_1_debug.xml` session to load both cores in C-SPY

## MEMORY MAP (per core)

| Region | CM7_0 | CM7_1 |
|--------|-------|-------|
| CODE_FLASH | 2MB @ 0x10080000 | ~1.6MB @ 0x10280000 |
| SRAM | 384KB @ 0x28020000 | 256KB @ 0x28080000 |
| ITCM | 16KB @ 0xA0000000 | 16KB @ 0xA0100000 |
| DTCM | 16KB @ 0xA0010000 | 16KB @ 0xA0110000 |

## CONVENTIONS

- User code naming is free — no `zf_` or `cy_` prefix required
- Include library headers via `#include "zf_common_headfile.h"` (aggregates all)
- Application modules in `code/` are independent; no interdependencies assumed
- Interrupt handlers go in `user/cm7_<n>_isr.c`, NOT in application modules
- Each core has its own `main()` — they share libraries but run independently
