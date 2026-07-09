# PROJECT KNOWLEDGE BASE — project/user/

**Scope:** Core-specific entry points and ISR overrides for CM7_0 and CM7_1.

## OVERVIEW
CM7_0 runs the full application; CM7_1 boots to a minimal loop. ISR stubs are weak overrides, so unused channels can stay empty but must exist in the correct core's file.

## STRUCTURE
```
project/user/
├── main_cm7_0.c   # Primary core: clock, drivers, control init, Menu() loop
├── main_cm7_1.c   # Secondary core: placeholder main (clock + debug only)
├── cm7_0_isr.c    # CM7_0 ISR overrides: PIT 2/10-21, UART 0-6, GPIO EXTI 0-23
└── cm7_1_isr.c    # CM7_1 ISR overrides: same names, independent vector table
```

## WHERE TO LOOK
| Task | File | Notes |
|------|------|-------|
| Add startup / peripheral init | `main_cm7_0.c` | Clock, debug, buzzer, IMU, flash, control init, then `Menu()` loop |
| Offload work to second core | `main_cm7_1.c` | Currently placeholder; only `clock_init` and `debug_info_init` |
| Override PIT timer ISR | `cm7_0_isr.c` or `cm7_1_isr.c` | Channels 2, 10–21 are here. **Channels 0 and 1 live in `main_cm7_0.c`** |
| Override UART ISR | `cm7_0_isr.c` or `cm7_1_isr.c` | Check `uart_isr_mask()` branch for RX vs TX |
| Override GPIO EXTI ISR | `cm7_0_isr.c` or `cm7_1_isr.c` | Groups 0–23; use `exti_flag_get(Pxx_x)` to identify source pin |
| Handle line-scan camera timing | `cm7_0_isr.c` / `cm7_1_isr.c` | `pit0_ch21_isr()` calls `tsl1401_collect_pit_handler()` in both files |

## CONVENTIONS
- **ISR placement matters**: The same function name in `cm7_0_isr.c` binds to CM7_0's vector table; the copy in `cm7_1_isr.c` binds to CM7_1's. They are independent.
- **ISR/RX branching**: UART ISRs split with `uart_isr_mask(UART_x)` to handle TX-empty vs RX-not-empty. Place user logic in the correct branch.

## ANTI-PATTERNS (THIS DIRECTORY)
1. **Defining `pit0_ch0_isr` in `cm7_0_isr.c`**: It won't compile or link correctly because the strong definition already lives in `main_cm7_0.c`. Same for `pit0_ch1_isr`.
2. **Editing the wrong core's ISR file**: `cm7_1_isr.c` and `cm7_0_isr.c` are near-identical. It's easy to add logic to CM7_1 and wonder why CM7_0 never fires it.
3. **Diverging UART2 callbacks**: `cm7_0_isr.c` calls `uart_control_callback()` on UART2 RX; `cm7_1_isr.c` calls `gnss_uart_callback()`. If you build both cores, ensure the active image matches your hardware wiring.
4. **Empty ISR bodies without flag clear**: Every stub already calls `pit_isr_flag_clear()` or `uart_isr_mask()`. Don't remove these or the interrupt will re-trigger instantly.
5. **Heavy work in `main_cm7_0.c` ISRs**: `pit0_ch0_isr` calls `pit_call_back()` which runs attitude estimation, navigation, and PID. Adding more code here worsens jitter for the 1 ms key-scan ISR on channel 1.
