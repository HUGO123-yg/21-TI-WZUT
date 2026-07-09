# ZF_COMMON KNOWLEDGE BASE

## OVERVIEW
The common utility layer and master umbrella header. This is where every compilation unit enters the include graph.

## STRUCTURE
```
zf_common/
├── zf_common_headfile.h    — Pulls in SDK, all zf_driver / zf_device / zf_components, plus project headers
├── zf_common_typedef.h     — uint8/uint16/uint32/vuint32 and ZF_ENABLE/ZF_WEAK macros
├── zf_common_clock.c/h     — clock_init(), system_delay_*()
├── zf_common_debug.c/h     — Debug UART, zf_assert(), zf_log()
├── zf_common_fifo.c/h      — Generic FIFO (8/16/32-bit element size)
├── zf_common_font.c/h      — LCD font bitmap data
├── zf_common_function.c/h  — Misc utilities
└── zf_common_interrupt.c/h — NVIC priority configuration
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Register a new project header | `zf_common_headfile.h` | Append `#include "YourHeader.h"` at the bottom, after the existing project blocks |
| Toggle custom typedefs | `zf_common_typedef.h` | `USE_ZF_TYPEDEF` controls whether `uint8`, `vuint32`, etc. are defined |
| Change debug UART settings | `zf_common_debug.h` | `DEBUG_UART_INDEX`, `BAUDRATE`, `TX_PIN`, `RX_PIN` are all hardcoded macros |
| Use FIFO buffers | `zf_common_fifo.c/h` | Supports 8/16/32-bit element width; `fifo_init()` requires a pre-allocated buffer |

## ANTI-PATTERNS (THIS DIRECTORY)
1. **Missing header registration**: Adding a module to `project/code/` without including it in `zf_common_headfile.h` silently breaks other files that rely on the umbrella header.
2. **`uint8_t` mixed with `uint8`**: The SDK and `arm_math.h` use standard `stdint` names, while `zf_common_typedef.h` provides `uint8`. Pick one style per file.
3. **Assert in ISRs or fast loops**: `zf_assert()` prints file and line over debug UART. Calling it inside a PIT ISR or tight control loop will stall or miss deadlines.
4. **FIFO without concurrency guard**: The FIFO tracks an `execution` flag to catch nested access on a single core, but it has no atomic primitives. Do not share a FIFO between CM7_0 and CM7_1 without external locking.
5. **Debug UART pin change desync**: If you edit `DEBUG_UART_INDEX` or the pin macros in `zf_common_debug.h`, you must also wire the matching ISR in `project/user/cm7_0_isr.c` or the handler will never fire.
