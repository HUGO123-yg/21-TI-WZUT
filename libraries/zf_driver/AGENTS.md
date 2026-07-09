# ZF_DRIVER KNOWLEDGE BASE

**Scope:** MCU peripheral abstraction layer wrapping the Cypress/Infineon Traveo II SDK.

## OVERVIEW
Each `zf_driver_*` pair maps a CYT4BB7 peripheral to a simplified `snake_case` API. Drivers consume SDK headers under `libraries/sdk/` and expose weak ISR stubs that are overridden in `project/user/cm7_*_isr.c`.

## WHERE TO LOOK
| Task | File(s) | Notes |
|------|---------|-------|
| Add a new peripheral driver | `zf_driver_<periph>.c/h` | Include SDK headers; expose `xxx_init()` + data functions; add header to `zf_common_headfile.h` |
| Fix SPI/UART clock conflict | `zf_driver_spi.c`, `zf_driver_uart.c` | V3.6.1 fixed SPI_0 / UART_4 conflict; UART_5 shares clock with SPI0, UART_6 with SPI3. Check `version.txt` before mixing instances. |
| CM7_0 ↔ CM7_1 messaging | `zf_driver_ipc.c/h` | Wraps `cy_ipc_pipe.h`. CM7_0 uses `IPC_PORT_1`, CM7_1 uses `IPC_PORT_2`. Sends one `uint32`; blocks up to 5 ms until release callback. |
| Change PIT/Encoder/PWM timer | `zf_driver_pit.c`, `zf_driver_encoder.c`, `zf_driver_pwm.c` | All built on TCPWM (`cy_tcpwm_pwm.h`). PIT channels 0-12 are available. |
| ADC sampling config | `zf_driver_adc.c` | Uses `PASS0_SAR0/1/2` via `cy_adc.h`. Max sample freq is ~13.34 MHz. |
| Hardware UART init | `zf_driver_uart.c` | Wraps SCB UART (`cy_scb_uart.h`). SBUS mode has a dedicated `uart_sbus_init()`. |
| Hardware SPI init | `zf_driver_spi.c` | Supports SPI 0-3, modes 0-3, 8/16-bit transfers. CS can be hardware or `SPI_CS_NULL` for software control. |
| Software IIC / SPI | `zf_driver_soft_iic.c`, `zf_driver_soft_spi.c` | Bit-banged implementations for devices without a free hardware peripheral. |
| GPIO external interrupt | `zf_driver_exti.c/h` | Maps to `cy_gpio.h` EXTI lines. |
| On-chip flash write | `zf_driver_flash.c/h` | Wraps SDK flash driver. Avoid concurrent camera/DMA access during writes. |

## CONVENTIONS (zf_driver-specific)
- **Weak ISR declarations**: Most drivers declare `__WEAK` ISR prototypes (e.g., `pit0_ch0_isr()`, `uart0_isr()`). The real implementation lives in `project/user/cm7_0_isr.c` or `cm7_1_isr.c` for the target core.
- **Pin enums**: Every driver exposes pin-selection enums (e.g., `spi_clk_pin_enum`, `uart_tx_pin_enum`). Not all pins are routable; check the enum definitions before assigning.

## ANTI-PATTERNS (zf_driver-specific)
1. **DMA driver is mostly a stub**: `zf_driver_dma.c` contains commented-out `cyhal_dma` code and a no-op `dma_disable()`. Do not rely on it for production DMA transfers; use SDK DMA directly or fix the driver first.
2. **Blocking IPC from ISRs**: `ipc_send_data()` spins with `system_delay_us(10)` inside a 5 ms timeout loop. Calling it from a high-frequency ISR will jitter timing.
3. **SPI clock source regressions**: Multiple version-history entries (V3.0.1, V3.6.1, V3.7.2) show recurring SPI/UART clock-source conflicts. When adding a new SPI or UART instance, verify clock-tree independence in `cy_sysclk.h`.
4. **Flash driver vs camera concurrency**: On-chip flash writes stall the bus. The application layer already hits this; do not add flash-erase logic inside camera/DMA-critical paths.
