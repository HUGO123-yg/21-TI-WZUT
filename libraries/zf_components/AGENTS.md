# zf_components — SeekFree PC Assistant Protocol

## OVERVIEW
4 files (2 `.c`/`.h` pairs). Bridges the embedded system to the SeekFree PC debug assistant via UART/WiFi/BLE. Three sub-protocols: oscilloscope (8-channel float streaming), camera image upload, and bidirectional parameter tuning.

## STRUCTURE

```
libraries/zf_components/
├── seekfree_assistant.h/c              # Protocol encode/decode: osc data, camera frames, parameter packets
└── seekfree_assistant_interface.h/c    # Transport abstraction layer binding to UART/WiFi/BLE/etc.
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Oscilloscope data send | `seekfree_assistant.c` — `seekfree_assistant_oscilloscope_send()` | 8-channel float streaming with packet framing |
| Camera image upload | `seekfree_assistant.c` — `seekfree_assistant_camera_send()` | Supports binary/gray/RGB565 with boundary overlay |
| Parameter tuning | `seekfree_assistant.c` — `seekfree_assistant_data_analysis()` | Bidirectional: PC sends params, MCU reads/writes |
| Transport binding | `seekfree_assistant_interface.h` — `seekfree_assistant_transfer_device_enum` | 7 options: UART, BLE6A20, CH9141, WIFI_UART, WIFI_SPI, RECEIVER_UART, LORA3A22_UART |
| Receive callback | `seekfree_assistant_interface.h` — `seekfree_assistant_receive_callback_function` | Per-transport receive handler registration |

## PROTOCOL TYPES

| Struct | Fields | Use |
|--------|--------|-----|
| `seekfree_assistant_oscilloscope_struct` | head, channel_num, check_sum, length, data[8] | 8-channel oscilloscope packet |
| `seekfree_assistant_camera_struct` | 7 fields: head, mode, width, height, type, count, check_sum | Camera frame header |
| `seekfree_assistant_camera_dot_struct` | 8 fields: x/y start/end + type/flags | Boundary line overlay |
| `seekfree_assistant_camera_buffer_struct` | image buffer + boundary pointer arrays | In-progress frame assembly |
| `seekfree_assistant_parameter_struct` | head, func, channel, data | Parameter read/write packet |

## CONVENTIONS

- **Header-first framing**: All protocol packets use `#define` magic bytes at packet head for sync.
- **Transport enum dispatch**: `seekfree_assistant_transfer_device_enum` selects send/receive functions. Each transport registers a `callback_function` send handler + `receive_callback_function` receive handler.
- **Extern globals**: `extern oscilloscope_data[8]`, `extern parameter[4]`, `extern image_update_flag` — accessed by `project/code/` for UI display.
- **Single include**: Both `.c` files include `zf_common_headfile.h`.

## NOTES

- Built for the SeekFree PC debugging tool — not a general-purpose protocol.
- Transport abstraction allows swapping UART/WiFi/BLE without changing protocol logic.
- Image streaming uses boundary detection (X/Y/XY/NO_BOUNDARY) for camera-based line following.
- No RTOS: all protocol logic is callback-driven or polled from the super-loop.
- Protocol documentation is in Chinese (SeekFree PC assistant spec) — no English reference available.
