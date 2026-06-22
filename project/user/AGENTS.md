# ENTRY POINTS & ISR LAYER

**Generated:** 2026-06-19
**Scope:** `project/user/` — `main()` entry points, PIT/UART/EXTI ISR handlers.

## OVERVIEW

4 files (1077 lines). Dual-core (CM7_0 + CM7_1) entry points. ISR functions are declared here and linked by IAR via name matching — NOT registered via `interrupt_init()` at runtime (that's for application callbacks only).

## STRUCTURE

```
project/user/
├── main_cm7_0.c       # CM7_0 main() — init sequence + super-loop (Menu)
├── main_cm7_1.c       # CM7_1 main() — idle stub
├── cm7_0_isr.c        # PIT (ch0-14), UART (ch0-6), EXTI (ch0-23) ISR bodies
└── cm7_1_isr.c        # CM7_1 ISR stubs — wired but unused
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| CM7_0 boot/init | `main_cm7_0.c:48` | clock→debug→IMU→flash→balance→PIT→super-loop |
| CM7_1 idle stub | `main_cm7_1.c:47` | Wired-up, spins idle — available for offload |
| PIT ISR dispatch | `cm7_0_isr.c:45-300` | 15 channels: ch0=1ms balance, ch1=5ms key scan, ch2-14=stubs |
| UART ISR dispatch | `cm7_0_isr.c:300-380` | 7 channels: UART0-6, motor UART on ch2 |
| GPIO EXTI dispatch | `cm7_0_isr.c:380-432` | 24 channels: external pin interrupts |

## ISR NAMING

| Prefix | Channels | Example | Driven By |
|--------|----------|---------|-----------|
| `pit0_ch` | 0-14 | `pit0_ch0_isr()` | PIT timer |
| `uart` | 0-6 | `uart2_isr()` | UART peripheral |
| `gpio_` | 0-23 | `gpio_5_exti_isr()` | IOSS GPIO ACT |

ISR names match the IAR vector table. Each ISR clears its flag, then dispatches to an application callback via `pit_isr_func[]` / `uart_isr_func[]` / `exti_isr_func[]` function pointer arrays (defined in `zf_driver` layer).

## CRITICAL ISRs

| ISR | Period | Calls | Notes |
|-----|--------|-------|-------|
| `pit0_ch0_isr` | 1ms | `pit_call_back()` | Balance cascade — must complete <<1ms |
| `pit0_ch1_isr` | 5ms | `key_scan()` | Key debounce |
| `uart2_isr` | async | `uart_control_callback()` | Motor speed feedback |

## CONVENTIONS

- ISRs declared `void pit0_ch0_isr()` — no params, no return
- Each ISR calls `pit_isr_flag_clear(PIT_CHn)` first
- ISR bodies are thin dispatchers — heavy logic in `project/code/`
- `cm7_1_isr.c` is a near-empty stub (440 lines of empty ISR shells)
- Both `main()` functions include `zf_common_headfile.h`

## NOTES

- **CM7_1 is unused**: `main_cm7_1.c` spins idle. All application runs on CM7_0. CM7_1 available for compute offload.
- **CM0+ runs from ROM**: No source in this project. Boots before CM7 cores.
- **No dynamic ISR registration**: ISRs are statically named for IAR linker. Application callbacks (e.g., `pit_call_back`) are registered via `interrupt_init()` in `zf_common_interrupt.c`.
- **PIT ch2-14**: All stub ISRs with flag clear only — available for new timers.
- **UART ch3-6**: Available for additional serial peripherals.
- **GCC build**: Project root `Makefile` builds CM7_0 via ARM GNU GCC 15.2 (macOS/Linux). Output: `build/cm7_0.elf/.hex/.bin`. Linker: `project/gcc/linker_cm7_0.ld`. ISR/entry point structure unchanged — build flexibility only.
