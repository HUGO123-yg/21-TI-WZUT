# DEVICE DRIVER LAYER

## OVERVIEW

54 files: 27 independent driver pairs (`zf_device_<name>.c/.h`) plus type/config files for the SeekFree external IC library.

## WHERE TO LOOK

| Device | Files | Bus |
|--------|-------|-----|
| IMU | `imu660ra/rb/rc`, `imu963ra`, `icm20602` | SPI/I2C |
| Display | `ips114`, `ips200`, `ips200pro`, `tft180`, `oled` | SPI |
| Camera | `mt9v03x`, `tsl1401` | DVP/SPI |
| Wireless | `wifi_uart`, `wifi_spi`, `ble6a20`, `lora3a22`, `wireless_uart` | UART/SPI |
| ToF | `dl1a`, `dl1b` | I2C |
| Positioning | `gnss`, `pmw3901` | UART/SPI |
| Input | `key`, `uart_receiver`(SBUS), `menc15a` | GPIO/UART |
| Config/Type | `zf_device_config.h/.a/.lib`, `zf_device_type.h/.c` | -- |

## CONVENTIONS

- **Init pattern**: Each device exposes `zf_device_<name>_init(...)`. Data is accessed via `extern` globals declared in the header (e.g., `extern volatile int16 imu660ra_acc_x`).
- **Callback registration**: `set_wireless_type()` / `set_tof_type()` in `zf_device_type.h` binds interrupt handlers using function pointer enums (`wireless_type_enum`, `tof_type_enum`).
- **Dependencies only on zf_driver + zf_common**. No cross-device coupling.
- **Naming**: `zf_device_<name>.c/.h`. Each pair is a self-contained module.
- **Include**: All headers are reachable through `zf_common_headfile.h` (single monolithic include).

## ANTI-PATTERNS

- **Stale MCU name**: `zf_device_config.h` license block still reads MM32F327X-G9P. Should be CYT4BB.
- **Precompiled blobs**: `zf_device_config.a` and `zf_device_config.lib` ship without source. GPLv3 requires source distribution. Risk if redistributing.
- **Garbled docs**: `外设文件说明.txt` is encoded in GBK and renders as mojibake on UTF-8 systems. Rename or re-encode.
- **No const qualifiers**: Read-only pointer params in driver functions are almost never marked `const`.

## NOTES

- All .c files include `zf_common_headfile.h` first.
- Interrupt service routines live in `project/user/cm7_0_isr.c`, not in device drivers. Callbacks bridge the gap via `zf_device_type.h` function pointers.
- `zf_device_type.h` defines `callback_function` typedef, wireless/ToF enums, and extern handler pointers.
- `zf_device_config.h` contains compile-time pin/parameter `#define`s for each supported device.
- No test infrastructure. Validation is manual on-target via debug UART.
