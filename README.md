# Software_v0.2

## CMake / macOS build

The IAR workspace remains the reference project. The CMake build reads the active
source files from its two `.ewp` files, so adding a C source to an IAR CM7
project is automatically reflected at the next CMake configure.

Install CMake, Ninja, and the official Arm GNU Toolchain for macOS. The
Homebrew `arm-none-eabi-gcc` formula is not sufficient by itself because it
does not ship the bare-metal C library headers or newlib libraries required by
this firmware. Download and unpack Arm's `darwin-arm64-arm-none-eabi` package,
then point CMake to its compiler prefix:

```sh
brew install cmake ninja
cmake -S . -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake \
  -DARM_NONE_EABI_PREFIX=/path/to/arm-gnu-toolchain/bin/arm-none-eabi
cmake --build build
```

This produces `build/cm7_0.hex` and `build/cm7_1.hex`. The two files must be
programmed together: CM7_0 is linked at `0x10080000`; CM7_1 at `0x10280000`.

The board's IAR debug configuration uses CMSIS-DAP plus an IAR-specific
`FlashCYT4_CF4M_WF256K` loader. CMake cannot reuse that proprietary loader.
Set `T2G_FLASH_COMMAND` to the command for the actual macOS-capable programmer
used with the board; its final two arguments receive the CM7_0 and CM7_1 HEX
paths. For example, after installing and validating a suitable J-Link or
Infineon command-line programmer:

```sh
cmake -S . -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake \
  '-DT2G_FLASH_COMMAND=/path/to/programmer --device CYT4BB7CEE --write'
cmake --build build --target flash
```

The exact programmer command is deliberately not guessed: probe firmware,
board wiring, security lifecycle, and the flash algorithm must match the
physical board. Record the confirmed command in this README when the probe
model is known.
