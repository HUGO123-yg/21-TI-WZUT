# zf_common — Shared Utilities Layer

## OVERVIEW
Foundation layer for the SeekFree ZF library. 14 files (7 .c/.h pairs). Type aliases, clock init, debug I/O, ring buffers, font data, math helpers, interrupt wrappers. All other layers depend on this one.

## WHERE TO LOOK

| Module | Files | Purpose |
|--------|-------|---------|
| typedef | `zf_common_typedef.h` | Custom types (uint8, int16, etc.), ZF_ENABLE/DISABLE/TRUE/FALSE, ZF_WEAK |
| clock | `zf_common_clock.c/h` | System clock init (default 250 MHz), `system_clock` external |
| debug | `zf_common_debug.c/h` | zf_assert(), zf_log(), printf/scanf redirect to UART (115200 baud) |
| fifo | `zf_common_fifo.c/h` | Ring buffers — 8/16/32-bit modes, read/write/clear, nested-IRQ safe. State enum tracks pending operations to detect conflicts |
| font | `zf_common_font.c/h` | ASCII 8x16/6x8 bitmaps, OLED 16x16 Chinese glyphs, Seekfree logo (38400 bytes), RGB565 color enum (16 named colors) |
| function | `zf_common_function.c/h` | func_abs/func_limit/func_limit_ab macros, string-to-number parsers, sprintf wrapper, sin amplitude table generator |
| interrupt | `zf_common_interrupt.c/h` | Global enable/disable with nesting counter, NVIC priority set, Cy_SysInt-based interrupt_init() for callback registration |

## CONVENTIONS
- **Master include**: `zf_common_headfile.h` — every .c in the project starts here. Pulls SDK headers, then zf_common/*, zf_driver/*, zf_device/*, zf_components/*, project/code/* in order. Changing it triggers a full rebuild.
- **USE_ZF_TYPEDEF** (default 1): Enables custom type aliases. Set to 0 to fall back to raw stdint.h types.
- **Volatile variants**: Typedef also provides vuint8/vint8/vuint16/etc. for volatile-qualified registers.
- **ZF_WEAK**: `__attribute__((weak))` — IAR-specific, allows function overriding for test mocking or stubbing.
- **COMPATIBLE_WITH_OLDER_VERSIONS**: Commented-out define in typedef header — enables legacy API compat if uncommented.
- **Float suffix**: Always `1.0f`, never bare doubles.
- **Doc blocks**: Chinese — 函数功能 / 输入参数 / 返回值 / 使用示例 / 注意事项.
## NOTES

- **Central hub**: Changes here ripple through ALL layers (zf_driver, zf_device, zf_components, project). Highest-impact layer in the codebase.
- **Internal coupling — KNOWN VIOLATIONS**:
  - `debug.c` includes `zf_driver_uart.h` + `zf_driver_delay.h` — lower layer depending on upper layer
  - `interrupt.c` includes `zf_driver_pwm.h` — same violation (for assert handler PWM shutdown)
  - `zf_common_headfile.h` includes `Common_peripherals.h`, `Imu.h`, `Flash.h`, `Body_ctrl.h`, `Menu.h` from `project/code/` — **libraries depending on project code** (reverse dependency)
- **Debug output struct**: `debug_output_struct` holds function pointers for UART or screen display. Callers can bind custom output backends.
- **zf_assert() HALTS**: Prints "Assert error" + file/line via debug UART, then locks in `while(1)`. Has a reentrancy guard — nested asserts cause immediate infinite loop.
- **interrupt.c** includes `zf_driver_pwm.h`. The `assert_interrupt_config()` function disables global IRQs and closes all PWM channels on assert.
- **FIFO state machine**: Tracks 5 operations (reset, clear, write, read, idle) to prevent IRQ-nesting corruption. Returns enum error codes instead of silently failing.
- **No RTOS**: All FIFO and interrupt code runs bare-metal. No mutexes, no scheduler.
- **Startup order**: clock_init() must run first. debug_init() depends on UART init, which depends on clock. Asserts before debug_init() will spin silently.
