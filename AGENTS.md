# Repository Guidelines

## Project Structure & Module Organization

- `project/user/` contains application entry points and interrupt handlers for the two Cortex-M7 cores: `main_cm7_0.c`, `main_cm7_1.c`, and their ISR files.
- `project/code/` is reserved for project-specific application and control modules. Keep a feature's `.c` and `.h` files together here.
- `libraries/zf_common/`, `libraries/zf_driver/`, `libraries/zf_device/`, and `libraries/zf_components/` provide the Seekfree platform layer. Treat these as shared infrastructure; avoid unrelated changes to them.
- `libraries/sdk/` is the Cypress/Infineon TVII-B-H SDK and CMSIS support. Do not hand-edit generated/vendor content unless a hardware fix requires it.
- `project/iar/` holds the IAR workspace, core-specific projects, linker directives, and debugger settings. Open `project/iar/cyt4bb7.eww` for the complete workspace.

## Build, Test, and Development Commands

This repository has no Makefile, CMake project, or automated unit-test runner. Build with IAR Embedded Workbench (the source headers identify IAR 9.40.1):

1. Open `project/iar/cyt4bb7.eww`.
2. Select the `Debug` configuration and rebuild both CM7 projects.
3. Download the image to the CYT4BB target and exercise initialization, periodic interrupts, and affected peripherals.

On Windows, `project/iar/删除临时文件IAR.bat` removes IAR temporary files. Do not commit `Debug*/`, `settings/`, or debugger/session artifacts; they are intentionally ignored.

## Coding Style & Naming Conventions

Use the existing embedded C style: four-space indentation, braces on their own lines, and a space before control-flow parentheses (`if (...)`). Keep functions and variables in `lower_snake_case`; use uppercase names for macros, constants, and hardware identifiers (for example, `PIT_CH0`). Match existing public APIs with a paired header and source file. Include the project umbrella header (`zf_common_headfile.h`) where its platform declarations are needed. Keep ISR bodies short: clear the interrupt flag first and defer substantial work to callbacks or the main loop.

## Testing Guidelines

There is no repository-level coverage target. For every firmware change, rebuild the affected core(s), confirm the linker succeeds, and validate on hardware with the relevant sensor/actuator path. State the board, configuration, and observed behavior in the change description. Changes to clocks, interrupts, DMA, flash, PWM, or IPC require explicit on-target regression checks.

## Commit & Pull Request Guidelines

Use short, imperative commit subjects consistent with the history, such as `Remove obsolete control code` or `Fix PIT callback timing`. Keep each commit focused. Pull requests should explain the hardware behavior changed, list modified core/project files, identify the IAR configuration used, and record build plus on-target test results. Include serial logs or screenshots when they clarify a peripheral or debug-display change.
