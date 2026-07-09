# PROJECT KNOWLEDGE BASE

**Generated:** 2026-07-02
**Project:** SEEKFREE CYT4BB Open-Source Library (smart-car/electronics competition firmware)
**MCU:** Infineon CYT4BB7 (Traveo II), dual Cortex-M7 (CM7_0 + CM7_1) + CM0+
**IDE:** IAR Embedded Workbench for ARM 9.40.1
**License:** GPL-3.0

## OVERVIEW
This is a Chinese SEEKFREE (逐飞科技) open-source embedded C library for the Infineon CYT4BB7 dual-core MCU. It wraps the Cypress/Infineon SDK with simplified drivers (`zf_*`), adds device drivers for common competition sensors/modules, and contains a small application layer for a balance/smart-car robot. The project is built entirely with IAR EWARM and flashed via C-SPY/J-Link.

## STRUCTURE
```
software_zyl/
├── libraries/          # Reusable library code (read-only for most changes)
│   ├── sdk/            # Infineon/Cypress Traveo II vendor SDK (vendor code)
│   ├── zf_common/      # Common utilities + master umbrella header
│   ├── zf_driver/      # MCU peripheral abstraction drivers
│   ├── zf_device/      # External sensor/module device drivers
│   ├── zf_components/  # SeekFree Assistant (PC debugging tool protocol)
│   └── doc/            # License + version changelog
│
└── project/            # Application-specific code (edit here)
    ├── user/           # Entry points and ISR handlers per core
    ├── code/           # Application logic modules
    └── iar/            # IAR Embedded Workbench project files
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Add application logic | `project/code/` | New modules go here; add header to `zf_common_headfile.h` |
| Wire a new ISR | `project/user/cm7_0_isr.c` or `cm7_1_isr.c` | Same ISR name exists in both files per core |
| Change clock/peripheral init | `project/user/main_cm7_0.c` | Primary core; `main_cm7_1.c` is mostly empty |
| Add a new external device | `libraries/zf_device/` or `project/code/` | Prefer `zf_device` if reusable |
| Modify pin definitions | `project/code/Common_peripherals.h` | Hardware pin map |
| Change build/linker settings | `project/iar/project_config/*.ewp` | Core-specific projects |
| Change memory layout | `project/iar/icf/linker_directives_tviibh.icf` | Shared CM7 linker script |
| Debug/flash | `project/iar/cyt4bb7.eww` | IAR workspace with multi-core debug session |

## CONVENTIONS
- **Single umbrella header**: Every `.c` file includes `#include "zf_common_headfile.h"` which pulls in SDK, all `zf_*` libraries, and all application headers.
- **Non-standard integer types**: `uint8`/`uint16`/`uint32` (not `uint8_t`) from `zf_common_typedef.h`.
- **File prefixes**: `zf_common_*`, `zf_driver_*`, `zf_device_*` for library files; `seekfree_*` for PC-assistant component; PascalCase/snake_case for project files.
- **Function naming**: `zf_*` library functions use module-based `snake_case` WITHOUT a `zf_` prefix (e.g., `imu660ra_init()`, `ips200_show_string()`).
- **ISR naming**: `pit0_ch0_isr()`, `uart0_isr()`, `gpio_0_exti_isr()` — weak overrides in `project/user/`.
- **CRLF line endings** and **Chinese (GBK) comments** throughout source; IAR formatter has no multibyte support.
- **Include guards**: `_zf_<module>_h_` pattern (with one inconsistency in `zf_device_uart_receiver.h`).

## ANTI-PATTERNS (THIS PROJECT)
1. **Heavy PID/control code in ISR**: `pit_call_back()` in `Body_ctrl.c` runs attitude estimation, navigation, and PID inside a PIT ISR.
2. **Magic numbers everywhere**: Thresholds, scaling factors, and timing constants are hardcoded in `Body_ctrl.c` and `Imu.c` without symbolic names.
3. **Pervasive global state**: `STOP_FALG` (typo), `run_state`, `sys_times`, `left_motor_duty`, etc. are file-scope globals shared across modules.
4. **Extreme duplication**: `Menu.c` is ~900 lines of nearly identical display functions; `Menu.h` declares 50+ extern functions.
5. **Hardware rules embedded in comments**: PMW3901 requires ≥20 ms call period, ≥5 cm distance, and adequate light; wireless UART auto-baud requires RTS pin and module v2.0+; IMU660RC quaternion read requires INT2 external interrupt.
6. **Flash + camera concurrency**: Version history notes repeated crashes when using camera and writing flash simultaneously.
7. **D-cache coherency**: Camera/DMA buffers must be invalidated after use; D-cache not cleaned caused image tearing in earlier versions.
8. **No automated tests**: Validation is manual via IAR C-SPY and runtime `zf_assert()` assertions only.

## UNIQUE STYLES
- **Dual ISR files**: `cm7_0_isr.c` and `cm7_1_isr.c` both define the same ISR names (e.g., `pit0_ch0_isr`) for their respective cores — edits must be made in the correct file for the target core.
- **SeekFree Assistant protocol**: A custom binary UART protocol in `libraries/zf_components/` for PC-side oscilloscope plotting, parameter tuning, and camera streaming.
- **Precompiled binary blobs**: `libraries/zf_device/zf_device_config.a` / `.lib` are checked-in precompiled libraries.
- **No RTOS**: Bare-metal dual-core project using PIT timer interrupts and UART/GPIO ISRs for scheduling, despite SDK including FreeRTOS.
- **No CM0+ application**: IAR workspace only contains CM7_0 and CM7_1 projects; CM0+ boot is handled by factory ROM firmware.
- **Chinese-first codebase**: Comments, docs, and changelogs are in Chinese (GBK); English only in the GPL statement file.

## COMMANDS
```bash
# Open workspace in IAR (Windows only)
IarIdePm.exe project\iar\cyt4bb7.eww

# Build CM7_0 from command line (IarBuild.exe on Windows)
IarBuild.exe project\iar\project_config\cyt4bb7_cm_7_0.ewp -build Debug -log info
IarBuild.exe project\iar\project_config\cyt4bb7_cm_7_1.ewp -build Debug -log info

# Clean temporary IAR files (run from project/iar/)
.\删除临时文件IAR.bat
```

## NOTES
- **Build tool**: IAR EWARM is Windows-only; no Makefile, CMake, or cross-platform build exists.
- **Output path gotcha**: `.hex`/`.out` are written to `project/iar/project_config/Debug_m7_0/Exe/` (relative to `.ewp`), while Obj/BrowseInfo/List are in `project/iar/Debug_m7_0/`.
- **Core selection**: `CY_CORE_CM7_0` / `CY_CORE_CM7_1` preprocessor defines select interrupt maps and linker regions; do not change these manually.
- **License rule**: Any modification to `zf_*` library files must retain the SEEKFREE copyright statement per the GPL3.0 header.
