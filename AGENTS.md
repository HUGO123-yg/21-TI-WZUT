# PROJECT KNOWLEDGE BASE

**Generated:** 2026-05-09
**MCU:** Infineon CYT4BB (TRAVEO T2G, TVIIBH4M) — dual Cortex-M7 + CM0+ boot core
**Toolchain:** IAR EWARM v9.40.1 (only — no Make/CMake)
**Version:** V3.10.2 (see `libraries/doc/version.txt`)
**License:** GPL v3

## OVERVIEW

Seekfree CYT4BB Open Source Library — bare-metal multi-core firmware for the Infineon Traveo II platform.
Three code layers: Infineon SDK (`libraries/sdk/`) → Seekfree abstraction (`libraries/zf_*`) → User application (`project/`).

## STRUCTURE

```
.
├── libraries/
│   ├── doc/                   # Version history, GPL3 statement
│   ├── sdk/                   # Infineon official SDK (DO NOT MODIFY)
│   │   ├── common/            # Cross-chip: CMSIS, FreeRTOS, startup, drivers
│   │   └── tviibh4m/          # Chip-specific: device hdr, sysclk, interrupts
│   ├── zf_common/             # Seekfree common: clock, debug, fifo, font, typedef
│   ├── zf_driver/             # Seekfree HAL: GPIO, UART, SPI, I2C, PWM, ADC, DMA
│   ├── zf_device/             # Seekfree device drivers: IMU, camera, display, wireless
│   └── zf_components/         # High-level: Seekfree Assistant protocol
└── project/
    ├── code/                  # User application logic
    ├── user/                  # Entry points (main_cm7_0.c, main_cm7_1.c) + ISRs
    └── iar/                   # IAR workspace, projects, linker script
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add peripheral driver | `libraries/zf_driver/` | Follow `zf_driver_<periph>.h` naming |
| Add device/sensor driver | `libraries/zf_device/` | Follow `zf_device_<model>.h` naming |
| Change system clock | `libraries/zf_common/zf_common_clock.h` | `SYSTEM_CLOCK_250M` |
| Add user application code | `project/code/` | Auto-included by IAR project |
| Modify entry point | `project/user/main_cm7_0.c` or `main_cm7_1.c` | One per CM7 core |
| Add interrupt handler | `project/user/cm7_0_isr.c` or `cm7_1_isr.c` | User ISRs only |
| Change memory layout | `project/iar/icf/linker_directives_tviibh.icf` | IAR linker script |
| Find type definitions | `libraries/zf_common/zf_common_typedef.h` | `uint8`, `uint16`, `uint32`, etc. |
| Master include | `libraries/zf_common/zf_common_headfile.h` | Aggregates all SDK + library headers |
| Clock init chain | `zf_common_clock.c` → `system_tviibh4m_cm7.c` | See entry-point doc below |
| FreeRTOS config | `libraries/sdk/common/src/rtos/` | Tasks, queues, semaphores, timers |
| Ethernet stack | `libraries/sdk/tviibh4m/src/drivers/ethernet/` | DP83867 PHY |

## BOOT FLOW

```
CM0+ Boot Core (system ROM / not in this repo)
  └── Releases CM7_0 + CM7_1 from reset

CM7_0 / CM7_1 Reset_Handler (startup_cm7.s)
  └── startup.c: Startup_Init()  → FPU, cache, ECC, vector table → RAM
      └── main()  →  clock_init(SYSTEM_CLOCK_250M)
          ├── SystemInit()        (system_tviibh4m_cm7.c — cache coherence + clock update)
          ├── system_delay_init() (zf_driver_delay.c — TCPWM or SysTick)
          └── interrupt_global_enable(0)
      → debug_init() → user init → while(true) { loop }
```

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `main()` | fn | `project/user/main_cm7_0.c` | CM7_0 application entry |
| `main()` | fn | `project/user/main_cm7_1.c` | CM7_1 application entry |
| `clock_init()` | fn | `libraries/zf_common/zf_common_clock.c` | System clock setup wrapper |
| `SystemInit()` | fn | `libraries/sdk/tviibh4m/src/system/system_tviibh4m_cm7.c` | Cache + clock register update |
| `Startup_Init()` | fn | `libraries/sdk/common/src/startup/startup.c` | C early-init (FPU, ECC, cache) |
| `Reset_Handler` | asm | `libraries/sdk/common/src/startup/iar/startup_cm7.s` | Vector table + BSS/DATA init |
| `Cy_SCB_UART_Init()` | fn | SDK | Infineon UART driver (Cy_ prefix) |
| `uart_init()` | fn | `libraries/zf_driver/zf_driver_uart.c` | Seekfree UART wrapper |
| `zf_common_typedef.h` | hdr | `libraries/zf_common/` | All custom types (`uint8`, `uint32` etc.) |
| `zf_common_headfile.h` | hdr | `libraries/zf_common/` | Master include — start here |

## CONVENTIONS

These apply to **Seekfree-authored code** only (libraries/zf_*, project/). SDK code follows Infineon conventions.

### Types
- Use `uint8`, `uint16`, `uint32`, `int8`, `int16`, `int32` (NOT stdint `_t` variants)
- Typedef enums use `_enum` suffix: `typedef enum { ... } gpio_dir_enum;`
- Typedef structs use `_struct` suffix: `typedef struct { ... } fifo_struct;`
- Enum members: `SCREAMING_SNAKE_CASE`
- Callback typedefs: `typedef void (*callback_function)(void);`

### Naming
- Files: `zf_<layer>_<module>.h` (e.g., `zf_driver_uart.h`, `zf_device_icm20602.h`)
- Functions: `lowercase_snake_case` — `{module}_{verb}_{noun}`: `uart_write_byte()`, `gpio_set_level()`
- Macros (constants): `SCREAMING_SNAKE_CASE` with module prefix: `PWM_DUTY_MAX`, `ICM20602_DEV_ADDR`
- Function-like macros: `lowercase_snake` — always wrap params in `()`: `#define func_limit(x, y) ((x) > (y) ? ...)`
- Include guards: `#ifndef _filename_h_` (lowercase, leading + trailing underscore)

### Comments
- Use `//` section blocks (NOT Doxygen `/** */`):
  ```
  //-------------------------------------------------------------------------------------------------------------------
  // 函数名称       uart_write_byte
  // 功能说明       ...
  // 输入参数       dat       需要发送的字节
  // 返回参数       void
  //-------------------------------------------------------------------------------------------------------------------
  ```
- Copyright header: `/* */` block with file name, modification log (dates + author)
- Section separators: `//=====...=====`

### Three-Layer Awareness

| Layer | Prefix | Function style | Comments | Modify? |
|-------|--------|---------------|----------|---------|
| Infineon SDK | `cy_` / `Cy_` | `Cy_PascalCase()` | Doxygen `/** */` | **NO** |
| Seekfree Library | `zf_` | `lowercase_snake()` | `//` blocks | Yes (follow zf_ conventions) |
| User Project | free | mixed | free | Yes |

## ANTI-PATTERNS (THIS PROJECT)

### Hard Rules
- **DO NOT** edit `libraries/sdk/common/src/startup/` — vendor startup code with non-ANSI C context
- **DO NOT** edit `libraries/sdk/tviibh4m/src/interrupts/cy_interrupt_map.c` directly — use `cy_interrupt_map_<core>.h`
- **DO NOT** edit auto-generated files: `cedi.h` (Ethernet MAC registers)
- **DO NOT** call FreeRTOS internal functions (`prv*`, `xTaskResumeAll()`, `vTaskSwitchContext()`) from application
- **DO NOT** call blocking functions from `vApplicationIdleHook()`
- **DO NOT** use `ARRAY_SIZE()` on pointer parameters — only on array declarations
- `#include <stdint.h>` — **DO NOT** use `uint8_t` style types, use `uint8` from `zf_common_typedef.h`
- `as any`, `@ts-ignore`, `@ts-expect-error` — type safety suppression (not applicable in C, but no `#pragma` warning disables without documented reason)

### Startup Context Gotcha
`Startup_Init()` runs **before C runtime init**. Global/static initializers are NOT valid yet. `.bss` not cleared, `.data` not copied.

### Deep Sleep I2C/EZI2C
When using Deep Sleep with SCB I2C or EZI2C, you **must insert manual code** to disable/enable the peripheral clock. The driver leaves `/* Insert code */` placeholders — these are NOT optional.

### Flash Voltage Constraints
At 0.9V core voltage, Flash is **read-only**. Match Flash wait states to core voltage/frequency. See `cy_syspm.c:1518`.

## COMMANDS

```bash
# Build: Open in IAR Embedded Workbench
open project/iar/cyt4bb7.eww    # macOS — or double-click on Windows
# Then: Project → Make (F7) for Debug config

# Clean build artifacts
./project/iar/删除临时文件IAR.bat    # Windows batch — removes Debug_*/, .crun, .dbgdt, .ps1 files

# Multi-core debug: use cm7_0_cm7_1_debug.xml session (loads both cores simultaneously)
```

## NOTES

- **No CI/CD, test framework, or `.gitignore`.** Build artifacts tracked. Testing is hardware-in-the-loop via IAR C-SPY.
- **Chinese-named files.** Team is 成都寻芯科技 (Seekfree). Project overview in `libraries/doc/version.txt` (Chinese), not README.md.
- **YOTS markers** in SDK code = known incomplete vendor code (TCPWM buffered registers, SMIF CRC, MCWDT CPU select). Verify before relying.
- **Dual-core:** CM7_0 and CM7_1 have separate IAR projects, shared libraries, independent entry points/ISRs. Build and flash both.
- **GCC/DIAB/GHS startup files** in `libraries/sdk/common/src/startup/` are vendor-provided for multi-toolchain support — **not used** by this IAR-only project.
- **Precompiled libraries:** `libcrypto_server_cm0plus_iar.a`, `zf_device_config.a` — rebuild requires vendor tooling.
