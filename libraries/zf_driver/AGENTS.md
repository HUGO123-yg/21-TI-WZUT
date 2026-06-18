# ZF_DRIVER — MCU PERIPHERAL ABSTRACTION

**Files:** 15 driver pairs (`zf_driver_*.c/.h`) — 30 total.
**Role:** Wraps raw Infineon SDK register API into simpler interfaces for app code.
**Depends on:** `zf_common` (typedefs, debug, interrupt). **Depended by:** `zf_device`, `zf_components`, `project/code`.

## DRIVERS

| # | Driver | Init function | Notes |
|---|--------|---------------|-------|
| 1 | `adc` | `adc_init()` | SAR ADC conversion |
| 2 | `delay` | `system_delay_init()` | Busy-wait µs/ms |
| 3 | `dma` | `dma_init()` | Stub — depends on `zf_driver_exti` internally |
| 4 | `encoder` | `encoder_quad_init()` / `encoder_dir_init()` | Quadrature or direction/count |
| 5 | `exti` | `exti_init()` | GPIO interrupt (24 channels, ERU-based) |
| 6 | `flash` | `flash_init()` | Internal flash erase/write |
| 7 | `gpio` | `gpio_init()` | Pin direction, mode, pull |
| 8 | `ipc` | `ipc_communicate_init()` | Inter-core messaging |
| 9 | `pit` | `pit_init()` | Timer for 1ms/5ms control loop (critical) |
| 10 | `pwm` | `pwm_init()` | Servo/motor PWM output |
| 11 | `soft_iic` | `soft_iic_init()` | Bit-bang I2C — no HW peripheral |
| 12 | `soft_spi` | `soft_spi_init()` | Bit-bang SPI — no HW peripheral |
| 13 | `spi` | `spi_init()` | HW SPI master/slave |
| 14 | `timer` | `timer_init()` | General-purpose HW timer |
| 15 | `uart` | `uart_init()` / `uart_sbus_init()` | Async serial + SBUS |

All init functions are `<name>_init()` (no `zf_driver_` prefix). File name prefix is `zf_driver_`.

## INIT PATTERN

Every driver provides an init function taking a pin/channel enum + config params. Most call `Cy_*` SDK functions internally. Param validation via `zf_assert()` — 80+ assert calls across the layer.

## ISR PATTERN

Interrupt-driven drivers (uart, pit, exti) define handler callbacks as function pointers linked by IAR via vector table name matching:

- **PIT (15 channels):** `pit0_ch0_isr` through `pit0_ch21_isr` — `pit0_ch0_isr` drives 1ms tick, `pit0_ch1_isr` drives 5ms tick
- **UART (7 channels):** `uart0_isr` through `uart6_isr`
- **EXTI (24 channels):** `gpio_0_exti_isr` through `gpio_23_exti_isr`

ISRs are declared forward, stored in function pointer arrays (`pit_isr_func[15]`, `uart_isr_func[7]`, `exti_isr_func[24]`), then dispatched through a single hardware IRQ handler.

## CROSS-DEPENDENCIES

- `zf_driver_dma` includes `zf_driver_exti.h` — DMA trigger config uses EXTI pin/trigger enums
- No other cross-driver includes. All other drivers are independent.

## NOTES

- Bit-bang drivers (`soft_iic`, `soft_spi`) toggle GPIOs in software — no hardware I2C/SPI peripheral used.
- `zf_driver_dma.c` is a stub (empty `dma_init()`, disabled `dma_enable()`/`dma_disable()`) — not production-ready.
- `zf_assert()` halts on failure. Prints file + line via debug UART.
- App code must call `<name>_init()` before using a peripheral. No lazy init.
