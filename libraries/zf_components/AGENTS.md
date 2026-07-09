# ZF_COMPONENTS KNOWLEDGE BASE

**Directory:** `libraries/zf_components/`  
**Scope:** SeekFree Assistant (PC debugging tool) protocol implementation

## OVERVIEW
Binary UART protocol layer for streaming oscilloscope data, camera frames, and receiving tunable parameters from the PC-side SeekFree Assistant. Transport-agnostic: callbacks are wired to UART, BLE, WiFi, or SPI in `seekfree_assistant_interface_init()`.

## STRUCTURE
```
libraries/zf_components/
├── seekfree_assistant.c/h           — Protocol encoder/decoder + packet structs
└── seekfree_assistant_interface.c/h — Transport callback registration
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Wire transport (UART/WiFi/BLE) | `seekfree_assistant_interface.c` | Call `seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART)` after the device init |
| Stream oscilloscope channels | `seekfree_assistant.h` | Fill `seekfree_assistant_oscilloscope_data.data[]`, then call `seekfree_assistant_oscilloscope_send()` |
| Stream camera + boundary overlay | `seekfree_assistant.h` | `seekfree_assistant_camera_information_config()` then `seekfree_assistant_camera_boundary_config()`, then `seekfree_assistant_camera_send()` |
| Receive tuned parameters from PC | `seekfree_assistant.h` | Poll `seekfree_assistant_data_analysis()` in a PIT ISR or main loop; read `seekfree_assistant_parameter[]` and clear `seekfree_assistant_parameter_update_flag[]` |
| Use custom transport | `seekfree_assistant_interface.c` | Select `SEEKFREE_ASSISTANT_CUSTOM` and override the weak `seekfree_assistant_transfer()` / `seekfree_assistant_receive()` |

## CONVENTIONS
- Frame headers: `0xAA` for MCU-to-PC, `0x55` for PC-to-MCU.
- Camera types are hardcoded enums: `SEEKFREE_ASSISTANT_OV7725_BIN`, `SEEKFREE_ASSISTANT_MT9V03X`, `SEEKFREE_ASSISTANT_SCC8660`.
- `seekfree_assistant_transfer_callback` is a function pointer swapped at runtime by `interface_init()`; it is NOT a `zf_driver` abstraction.

## ANTI-PATTERNS (THIS DIRECTORY)
1. **Forgetting to poll `seekfree_assistant_data_analysis()`**: Parameter tuning stalls if this is not called regularly. Place it in the same PIT ISR as the oscilloscope send, or in the main loop.
2. **Calling boundary config before camera info config**: `seekfree_assistant_camera_boundary_config()` asserts if `camera_information_config()` was not called first.
3. **Not clearing update flags**: `seekfree_assistant_parameter_update_flag[channel]` stays set until manually zeroed; code that only checks truthiness will think every loop is a new update.
4. **Blocking transport in camera send**: `seekfree_assistant_camera_send()` can emit tens of kilobytes. If the `transfer_callback` blocks (e.g., polling UART FIFO), it will eat ISR time. Prefer DMA-backed UART or move camera streaming to a lower-priority context.
