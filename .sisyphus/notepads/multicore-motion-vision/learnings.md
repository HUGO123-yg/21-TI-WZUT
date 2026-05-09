# Learnings - multicore-motion-vision

## 2026-05-09: Div-by-zero guard in servo_kinematics.c

### Pattern: Static one-shot warning guard for zero-valued compile-time constants

When kinematics parameters (L1_MM, L2_MM, D_MM) are defined as `0` in the header (placeholder values awaiting MATLAB simulation), the `servo_control_table()` function divides by these values, producing NaN/Inf at runtime.

**Solution applied:**
- `static uint8 param_warned = 0;` — ensures `printf` fires only once, not every call
- Early-return guard at function entry: `if (L1_MM == 0.0f || L2_MM == 0.0f || D_MM == 0.0f) { ... return; }`
- One-time `printf` warning to UART/debug console
- TODO comment referencing the MATLAB simulation dependency

**Key conventions used:**
- `uint8` type (from zf_common_typedef.h), NOT `uint8_t`
- `// [TODO: ...]` comment style (Seekfree convention)
- `#include <stdio.h>` for printf in bare-metal context (redirected to UART via debug subsystem)

**Note:** The `#include <stdio.h>` is needed because `printf` is not guaranteed by `zf_common_headfile.h`. In this bare-metal IAR project, `printf` is mapped to the debug UART by the vendor's retargeting layer.

## 2026-05-09: Cleaning CM7_1 ISR file — sensor handlers to CM7_0

### Pattern: Multi-core ISR ownership partitioning

When splitting a dual-core project, interrupt handlers in the user ISR file must be cleaned of peripheral handlers that belong to the other core. The ISR function stubs must remain because they are referenced by the vector table, but the handler logic is stripped.

**Applied to `cm7_1_isr.c`:**
- `uart1_isr()` — removed `wireless_module_uart_handler()`, added `uart_clear_flag(UART_1); __DSB();`
- `uart2_isr()` — removed `gnss_uart_callback()`, added `uart_clear_flag(UART_2); __DSB();`
- `uart4_isr()` — removed `uart_receiver_handler()`, added `uart_clear_flag(UART_4); __DSB();`

**Replacement pattern:**
```c
void uartN_isr (void)
{
    // Handler removed — peripheral owned by CM7_0
    uart_clear_flag(UART_N);
    __DSB();
}
```

**Key conventions:**
- `uart_clear_flag()` from `zf_driver_uart.h` — clears interrupt flags without invoking handler logic
- `__DSB()` — IAR intrinsic, ensures flag clear completes before returning from ISR
- Comment documents ownership (CM7_0 vs CM7_1) for maintainability
- Must NOT delete the ISR function entirely (it's in the vector table)

**Verification:**
- `grep -c "wireless_module_uart_handler|gnss_uart_callback|uart_receiver_handler" cm7_1_isr.c` → 0
- `grep -c "uart1_isr|uart2_isr|uart4_isr" cm7_1_isr.c` → 3 (stubs preserved)

## Wave 1: Add 6 .c files to CM7_0 IAR project (2026-05-09)

### What was done
Added 6 unlinked .c source files to the CM7_0 IAR project (.ewp):
- `control.c`, `foc.c`, `euler.c`, `pid.c`, `timer_control.c`, `servo_kinematics.c`

### Pattern
- .ewp is standard XML. The code group is at `<group><name>code</name>` (lines 1116-1151).
- Each file entry: `<file><name>$PROJ_DIR$\..\..\code\FILENAME.c</name></file>`
- Indentation: 8 spaces for `<file>`/`</file>`, 12 spaces for `<name>`
- `$PROJ_DIR$` = `project/iar/project_config/`, so `..\..\code\` = `project/code/`
- DO NOT touch the CM7_1 .ewp — each core has its own project file.

### Verification
- `grep -c` with pipe-delimited filenames in the .ewp
- Note: `small_driver_uart_control.c` also matches `control.c` in grep — expected false positive
- All 6 files confirmed on disk at `project/code/`
- CM7_1 .ewp confirmed untouched (0 matches for any of the 6 files)

## Wave 1: Clean cm7_0_isr.c — Remove CCD handler

**Date:** 2026-05-09

**Change:** Removed `tsl1401_collect_pit_handler()` from `pit0_ch21_isr()` in `project/user/cm7_0_isr.c`. PIT_CH21 is now owned exclusively by CM7_1 for the TSL1401 CCD line-scan camera.

**Final `pit0_ch21_isr()` state:**
```c
void pit0_ch21_isr()
{
    // PIT_CH21 owned by CM7_1 for CCD — handler removed from CM7_0
    pit_isr_flag_clear(PIT_CH21);
    __DSB();
}
```

**Encoding issue encountered:** The IAR project files use GBK/GB2312 encoding for Chinese comments. The `edit` tool couldn't match Chinese characters. Used Python with raw byte manipulation to safely edit the file. Always test with `python3 -c` and read back to verify when editing GBK-encoded files.

**Verification:**
- `grep -c "tsl1401_collect_pit_handler" cm7_0_isr.c` → 0 ✓
- `grep -c "pit0_ch21_isr" cm7_0_isr.c` → 1 ✓
- `grep -c "tsl1401_collect_pit_handler" cm7_1_isr.c` → 1 (untouched) ✓
