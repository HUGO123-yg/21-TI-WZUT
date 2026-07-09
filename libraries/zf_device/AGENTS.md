# ZF_DEVICE KNOWLEDGE BASE

**Directory:** `libraries/zf_device/` — External sensor and module drivers for competition hardware.

## OVERVIEW
Each file pair (`zf_device_<module>.c` / `.h`) is a self-contained driver for one external device. Headers define hardware pins, SPI/UART/IIC parameters, and register addresses; source files implement init and data read functions.

## WHERE TO LOOK
| Task | File(s) | Notes |
|------|---------|-------|
| IMU (accel/gyro) | `imu660ra`, `imu660rb`, `imu660rc`, `imu963ra`, `icm20602` | `imu660rc` supports quaternion fusion; `imu963ra` adds magnetometer |
| TOF distance | `dl1a`, `dl1b` | DL1A ~33 Hz, 1.2 m max; DL1B ~100 Hz, 1.4 m max |
| Display | `ips114`, `ips200`, `ips200pro`, `tft180`, `oled` | IPS200 is 240x320; OLED is 128x64 |
| Camera | `mt9v03x` | MT9V03X: width <= 188, height <= 120, total image size must be <= 65535 |
| Wireless | `ble6a20`, `wifi_spi`, `wifi_uart`, `wireless_uart`, `lora3a22` | `wireless_uart` auto-baud needs RTS pin and module v2.0+ |
| Optical flow | `pmw3901` | Requires >= 20 ms call period, >= 5 cm height, and adequate light |
| Encoder | `menc15a` | 15-bit absolute magnetic encoder |
| Remote / SBUS | `uart_receiver` | SBUS protocol receiver |
| GPS / GNSS | `gnss` | GPS + RTK dual-frequency positioning |
| Key input | `key` | Simple GPIO key scanning |
| Hardware pins | Any `zf_device_*.h` | Pin macros are at the top of each header; edit there to rewire |
| Device catalog | `外设文件说明.txt` | Chinese (GBK) listing of all modules and specs |

## CONVENTIONS
- **Pin macros live in headers**: Every `zf_device_*.h` defines `MODULE_PIN` macros at the top for SCL, SDA, CS, UART index, etc. These are the only place to change wiring.
- **SPI speed constants**: Named `MODULE_SPI_SPEED` and typically set to 1-10 MHz.
- **Global state exposed**: Most drivers declare extern variables (e.g., `imu660rc_gyro_x`, `pmw3901_delta_x`) for polled data access.

## ANTI-PATTERNS (THIS DIRECTORY)
1. **Do not delete precompiled blobs**: `zf_device_config.a` and `zf_device_config.lib` are checked-in binaries required by the IAR linker. Removing them breaks the build.
2. **Hardware constraints only in comments**: Device-specific rules (e.g., PMW3901 light/height, wireless UART RTS, IMU660RC INT2) are documented in headers but not enforced by the drivers. Validate wiring and timing before relying on default init.
3. **Camera resolution unchecked at compile time**: `MT9V03X_W` and `MT9V03X_H` are `#define`d with comments stating max 188x120 and total size <= 65535, but there is no static assertion.
4. **Copyright must be preserved**: Any modification to `zf_device_*` files must retain the SEEKFREE copyright comment block per the GPL3.0 header. Do not strip the Chinese license text.
