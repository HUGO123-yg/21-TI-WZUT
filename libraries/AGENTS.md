# LIBRARIES KNOWLEDGE BASE

## OVERVIEW

Three-tier C library stack: Infineon SDK (lowest) → Seekfree abstraction layers → user includes via `zf_common_headfile.h`.

## STRUCTURE

```
libraries/
├── doc/                   # GPL3 statement, version changelog (V3.10.2)
├── sdk/                   # ⚠️  DO NOT MODIFY — Infineon official SDK
│   ├── common/            # Cross-chip: CMSIS+DSP, FreeRTOS, startup, SCB/GPIO/TCPWM drivers
│   └── tviibh4m/          # Chip-specific: sysclk, interrupts, Ethernet, SDHC, SMIF, CAN-FD
├── zf_common/             # Seekfree utilities: clock, debug, fifo, font, function, typedef
├── zf_driver/             # Seekfree MCU peripheral HAL: GPIO, UART, SPI, I2C, PWM, ADC, DMA, etc.
├── zf_device/             # Seekfree device drivers: IMU, camera, display, wireless, GNSS
└── zf_components/         # High-level protocol: Seekfree Assistant
```

## WHERE TO LOOK

| Task | Location | Pattern |
|------|----------|---------|
| Write a new MCU peripheral driver | `zf_driver/` | `zf_driver_<periph>.h/c` |
| Write a new sensor/device driver | `zf_device/` | `zf_device_<model>.h/c` |
| Add common utility | `zf_common/` | `zf_common_<feature>.h/c` |
| Understand how X peripheral works | Find `zf_driver_<periph>.c` → trace calls into `cy_*` SDK functions |
| System clock init chain | `zf_common_clock.c` → `system_tviibh4m_cm7.c` → `cy_sysclk.c` |
| FreeRTOS API | `sdk/common/src/rtos/` — use `xTaskCreate`, `xQueueSend`, not internal `prv*` |
| Interrupt mapping | `sdk/tviibh4m/src/interrupts/cy_interrupt_map_cm7_<n>.h` — NOT the .c table directly |
| CMSIS-DSP functions | `sdk/common/hdr/cmsis/dsp/arm_math.h` |

## CONVENTIONS (Seekfree-authored only)

### Type System
- Integers: `uint8`, `uint16`, `uint32`, `int8`, `int16`, `int32` — from `zf_common_typedef.h`
- Do NOT use `<stdint.h>` types (`uint8_t`, etc.)
- `typedef enum { ... } name_enum;` — always `_enum` suffix
- `typedef struct { ... } name_struct;` — always `_struct` suffix
- Enum members: `SCREAMING_SNAKE_CASE`

### Function Naming
`{module}_{verb}_{noun}` in `lowercase_snake_case`:
- `gpio_set_level()`, `uart_write_byte()`, `adc_init()`, `pwm_set_duty()`
- `ips200_clear()`, `icm20602_init()`, `oled_show_string()`
- Static helpers: `get_uart_config()`, `get_scb_module()` (file-local)
- Never `CamelCase` or `Cy_PascalCase` in zf_* code

### Macros
- Constants: `SCREAMING_SNAKE` with module prefix: `PWM_DUTY_MAX`, `DEBUG_UART_BAUDRATE`
- Function-like: `lowercase_snake`, EVERY param in `()`: `#define func_abs(x) ((x) >= 0 ? (x): -(x))`
- Pin definitions: `DEVICE_PIN_FUNCTION`: `#define ICM20602_CS_PIN (P15_3)`

### Comments
```
//-------------------------------------------------------------------------------------------------------------------
// 函数名称       function_name
// 功能说明       <description>
// 输入参数       param     <description>
// 返回参数       void
//-------------------------------------------------------------------------------------------------------------------
```
- Section separator: `//=====<section name>=====`
- Copyright: `/* */` block with modification log (dates + author)

### Include Guards
`#ifndef _filename_h_` (lowercase, leading + trailing underscore)

## ANTI-PATTERNS

- **DO NOT** edit anything under `sdk/` — vendor code, different conventions (Cy_ prefix, Doxygen `/** */`)
- **DO NOT** bypass zf_driver to call Cy_ functions directly in application code
- **DO NOT** mix `uint8_t` (stdint) with `uint8` (zf_common) — pick one, use `uint8`
- **DO NOT** use `#pragma` to suppress warnings without documenting why
- SDK startup code runs before C runtime — do not rely on globals in `Startup_Init()`
- YOTS-marked TCPWM/SMIF functions are known-incomplete — verify before use
