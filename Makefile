################################################################################
# Makefile for CYT4BB CM7_0 — Self-Balancing Smart Car Firmware
#
# Replaces: IAR EWARM 9.40.1 → ARM GNU Toolchain (GCC)
# Target:   Infineon Traveo II CYT4BB7CEE, Cortex-M7, bare-metal
#
# Prerequisites:
#   ARM GNU Toolchain 15.2+ from developer.arm.com (includes newlib + CMSIS-DSP)
#   Install to: /Users/$(USER)/arm-gnu-toolchain/ or set ARM_TC_PATH
#
# Usage:
#   make              Build CM7_0 firmware → build/cm7_0.elf / .hex / .bin
#   make clean        Remove build artifacts
#   make flash        Flash via J-Link / OpenOCD (customize CFLASH_CMD)
#   make size         Print section sizes
################################################################################

# ─── Toolchain ────────────────────────────────────────────────────────────────

ARM_TC_PATH ?= /Users/hugo/arm-gnu-toolchain/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi
CROSS       ?= $(ARM_TC_PATH)/bin/arm-none-eabi-
CC         = $(CROSS)gcc
CXX        = $(CROSS)g++
AS         = $(CROSS)gcc -x assembler-with-cpp
OBJCOPY    = $(CROSS)objcopy
OBJDUMP    = $(CROSS)objdump
SIZE       = $(CROSS)size

# ─── Directories ──────────────────────────────────────────────────────────────

TOP        = $(realpath .)
BUILD_DIR  = $(TOP)/build/cm7_0
LDSCRIPT   = $(TOP)/project/gcc/linker_cm7_0.ld

# ─── Target ───────────────────────────────────────────────────────────────────

TARGET     = $(BUILD_DIR)/cm7_0
TARGET_ELF = $(TARGET).elf
TARGET_HEX = $(TARGET).hex
TARGET_BIN = $(TARGET).bin

# ─── CPU / FPU Flags ──────────────────────────────────────────────────────────

CPU     = -mcpu=cortex-m7
FPU     = -mfpu=fpv5-d16
FLOAT   = -mfloat-abi=hard
M_ARCH  = -mthumb

# ─── Compiler Flags ───────────────────────────────────────────────────────────

# Preprocessor defines (match IAR project)
DEFINES = \
	-Dtviibh4m \
	-DCYT4BB7CEE \
	-DCY_USE_PSVP=0 \
	-DCY_MCU_rev_b \
	-DCPU_BOARD_REVB \
	-DCY_CORE_CM7_0 \
	-DCY_CPU_CORTEX_M7 \
	-D__ICACHE_PRESENT=1 \
	-D__DCACHE_PRESENT=1 \
	-DARM_MATH_CM7 \
	-D__FPU_PRESENT=1 \
	-DIMU_MODE=1 \
	-DUSE_ZF_TYPEDEF=1 \
	-D__no_init= \
	-D__ramfunc= \
	-D__root=

# Include paths (from IAR .ewp + GCC startup paths)
INCLUDES = \
	-I$(TOP)/libraries/sdk/common/hdr \
	-I$(TOP)/libraries/sdk/common/hdr/cmsis/include \
	-I$(TOP)/libraries/sdk/common/hdr/cmsis/dsp \
	-I$(TOP)/libraries/sdk/tviibh4m/hdr \
	-I$(TOP)/libraries/sdk/tviibh4m/hdr/ip \
	-I$(TOP)/libraries/sdk/tviibh4m/hdr/mcureg \
	-I$(TOP)/libraries/sdk/tviibh4m/src \
	-I$(TOP)/libraries/sdk/tviibh4m/src/interrupts \
	-I$(TOP)/libraries/sdk/tviibh4m/src/system \
	-I$(TOP)/libraries/sdk/tviibh4m/src/drivers \
	-I$(TOP)/libraries/sdk/tviibh4m/src/drivers/audioss \
	-I$(TOP)/libraries/sdk/tviibh4m/src/drivers/cpu \
	-I$(TOP)/libraries/sdk/tviibh4m/src/drivers/ethernet \
	-I$(TOP)/libraries/sdk/tviibh4m/src/drivers/sd_host \
	-I$(TOP)/libraries/sdk/tviibh4m/src/drivers/sysclk \
	-I$(TOP)/libraries/sdk/tviibh4m/src/drivers/sysflt \
	-I$(TOP)/libraries/sdk/tviibh4m/src/drivers/sysreghc \
	-I$(TOP)/libraries/sdk/tviibh4m/src/mw \
	-I$(TOP)/libraries/sdk/tviibh4m/src/mw/eth_phy \
	-I$(TOP)/libraries/sdk/tviibh4m/src/mw/power \
	-I$(TOP)/libraries/sdk/tviibh4m/src/mw/smif_mem \
	-I$(TOP)/libraries/sdk/common/src/drivers \
	-I$(TOP)/libraries/sdk/common/src/drivers/adc \
	-I$(TOP)/libraries/sdk/common/src/drivers/canfd \
	-I$(TOP)/libraries/sdk/common/src/drivers/crypto \
	-I$(TOP)/libraries/sdk/common/src/drivers/cxpi \
	-I$(TOP)/libraries/sdk/common/src/drivers/dma \
	-I$(TOP)/libraries/sdk/common/src/drivers/evtgen \
	-I$(TOP)/libraries/sdk/common/src/drivers/flash \
	-I$(TOP)/libraries/sdk/common/src/drivers/gpio \
	-I$(TOP)/libraries/sdk/common/src/drivers/ipc \
	-I$(TOP)/libraries/sdk/common/src/drivers/lin \
	-I$(TOP)/libraries/sdk/common/src/drivers/lvd \
	-I$(TOP)/libraries/sdk/common/src/drivers/mcwdt \
	-I$(TOP)/libraries/sdk/common/src/drivers/mpu \
	-I$(TOP)/libraries/sdk/common/src/drivers/prot \
	-I$(TOP)/libraries/sdk/common/src/drivers/scb \
	-I$(TOP)/libraries/sdk/common/src/drivers/smartio \
	-I$(TOP)/libraries/sdk/common/src/drivers/smif/common \
	-I$(TOP)/libraries/sdk/common/src/drivers/smif/ver2 \
	-I$(TOP)/libraries/sdk/common/src/drivers/srom \
	-I$(TOP)/libraries/sdk/common/src/drivers/sysflt \
	-I$(TOP)/libraries/sdk/common/src/drivers/sysint \
	-I$(TOP)/libraries/sdk/common/src/drivers/syslib \
	-I$(TOP)/libraries/sdk/common/src/drivers/syspm \
	-I$(TOP)/libraries/sdk/common/src/drivers/sysreset \
	-I$(TOP)/libraries/sdk/common/src/drivers/sysrtc \
	-I$(TOP)/libraries/sdk/common/src/drivers/systick \
	-I$(TOP)/libraries/sdk/common/src/drivers/syswdt \
	-I$(TOP)/libraries/sdk/common/src/drivers/tcpwm \
	-I$(TOP)/libraries/sdk/common/src/drivers/trigmux \
	-I$(TOP)/libraries/sdk/common/src/startup \
	-I$(TOP)/libraries/sdk/common/src/mw \
	-I$(TOP)/libraries/sdk/common/src/mw/button \
	-I$(TOP)/libraries/sdk/common/src/mw/flash \
	-I$(TOP)/libraries/sdk/common/src/mw/semihosting \
	-I$(TOP)/libraries/sdk/common/src/mw/sw_timer \
	-I$(TOP)/libraries/zf_common \
	-I$(TOP)/libraries/zf_driver \
	-I$(TOP)/libraries/zf_device \
	-I$(TOP)/libraries/zf_components \
	-I$(TOP)/project/code \
	-I$(TOP)/project/user

CFLAGS   = $(CPU) $(FPU) $(FLOAT) $(M_ARCH) $(DEFINES) $(INCLUDES)
CFLAGS  += -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
CFLAGS  += -Wno-incompatible-pointer-types -Wno-memset-elt-size
CFLAGS  += -Wno-maybe-uninitialized
CFLAGS  += -Wno-builtin-macro-redefined
CFLAGS  += -fdata-sections -ffunction-sections
CFLAGS  += -fno-common -fno-strict-aliasing
CFLAGS  += -std=c11
CFLAGS  += -g3 -gdwarf-2
CFLAGS  += -Os

AFLAGS   = $(CPU) $(FPU) $(FLOAT) $(M_ARCH) $(DEFINES) $(INCLUDES)

LDFLAGS  = $(CPU) $(FPU) $(FLOAT) $(M_ARCH)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -nostartfiles
LDFLAGS += -specs=nosys.specs
LDFLAGS += -specs=nano.specs
LDFLAGS += -u _start

# ─── Source Files ─────────────────────────────────────────────────────────────

# GCC Assembly (startup + syslib helpers)
ASM_SRCS  = \
	libraries/sdk/common/src/startup/gcc/startup_cm7.S \
	libraries/sdk/common/src/drivers/syslib/gcc/cy_syslib_gcc.S

# SDK — startup
SDK_STARTUP_SRCS = \
	libraries/sdk/common/src/startup/startup.c

# SDK — system
SDK_SYSTEM_SRCS = \
	libraries/sdk/tviibh4m/src/system/system_tviibh4m_cm7.c

# SDK — interrupts
SDK_INT_SRCS = \
	libraries/sdk/tviibh4m/src/interrupts/cy_interrupt_map.c

# SDK — device-specific drivers (tviibh4m)
SDK_DEV_SRCS = \
	libraries/sdk/tviibh4m/src/drivers/audioss/cy_i2s.c \
	libraries/sdk/tviibh4m/src/drivers/cpu/cy_cpu.c \
	libraries/sdk/tviibh4m/src/drivers/ethernet/cy_ethif.c \
	libraries/sdk/tviibh4m/src/drivers/ethernet/edd.c \
	libraries/sdk/tviibh4m/src/drivers/ethernet/edd_rx.c \
	libraries/sdk/tviibh4m/src/drivers/ethernet/edd_tx.c \
	libraries/sdk/tviibh4m/src/drivers/sd_host/cy_sd_host.c \
	libraries/sdk/tviibh4m/src/drivers/sysclk/cy_sysclk.c \
	libraries/sdk/tviibh4m/src/drivers/sysreghc/cy_sysreghc.c

# SDK — common drivers
SDK_COM_SRCS = \
	libraries/sdk/common/src/drivers/adc/cy_adc.c \
	libraries/sdk/common/src/drivers/canfd/cy_canfd.c \
	libraries/sdk/common/src/drivers/crypto/cy_crypto_config.c \
	libraries/sdk/common/src/drivers/dma/cy_mdma.c \
	libraries/sdk/common/src/drivers/dma/cy_pdma.c \
	libraries/sdk/common/src/drivers/evtgen/cy_evtgen.c \
	libraries/sdk/common/src/drivers/flash/cy_flash.c \
	libraries/sdk/common/src/drivers/gpio/cy_gpio.c \
	libraries/sdk/common/src/drivers/ipc/cy_ipc_drv.c \
	libraries/sdk/common/src/drivers/ipc/cy_ipc_pipe.c \
	libraries/sdk/common/src/drivers/ipc/cy_ipc_sema.c \
	libraries/sdk/common/src/drivers/lin/cy_lin.c \
	libraries/sdk/common/src/drivers/lvd/cy_lvd.c \
	libraries/sdk/common/src/drivers/mcwdt/cy_mcwdt.c \
	libraries/sdk/common/src/drivers/mpu/cy_mpu.c \
	libraries/sdk/common/src/drivers/prot/cy_prot.c \
	libraries/sdk/common/src/drivers/scb/cy_scb_common.c \
	libraries/sdk/common/src/drivers/scb/cy_scb_ezi2c.c \
	libraries/sdk/common/src/drivers/scb/cy_scb_i2c.c \
	libraries/sdk/common/src/drivers/scb/cy_scb_spi.c \
	libraries/sdk/common/src/drivers/scb/cy_scb_uart.c \
	libraries/sdk/common/src/drivers/smartio/cy_smartio.c \
	libraries/sdk/common/src/drivers/smif/common/cy_smif.c \
	libraries/sdk/common/src/drivers/smif/common/cy_smif_memslot.c \
	libraries/sdk/common/src/drivers/smif/ver2/cy_smif_ver_specific.c \
	libraries/sdk/common/src/drivers/srom/cy_srom.c \
	libraries/sdk/common/src/drivers/sysflt/cy_sysflt.c \
	libraries/sdk/common/src/drivers/sysint/cy_sysint.c \
	libraries/sdk/common/src/drivers/syslib/cy_syslib.c \
	libraries/sdk/common/src/drivers/syspm/cy_syspm.c \
	libraries/sdk/common/src/drivers/sysreset/cy_sysreset.c \
	libraries/sdk/common/src/drivers/sysrtc/cy_sysrtc.c \
	libraries/sdk/common/src/drivers/systick/cy_systick.c \
	libraries/sdk/common/src/drivers/syswdt/cy_syswdt.c \
	libraries/sdk/common/src/drivers/tcpwm/cy_tcpwm.c \
	libraries/sdk/common/src/drivers/tcpwm/cy_tcpwm_counter.c \
	libraries/sdk/common/src/drivers/tcpwm/cy_tcpwm_pwm.c \
	libraries/sdk/common/src/drivers/tcpwm/cy_tcpwm_quaddec.c \
	libraries/sdk/common/src/drivers/tcpwm/cy_tcpwm_sr.c \
	libraries/sdk/common/src/drivers/trigmux/cy_trigmux.c

# SDK — middleware
SDK_MW_SRCS = \
	libraries/sdk/tviibh4m/src/mw/eth_phy/cy_dp83867.c \
	libraries/sdk/tviibh4m/src/mw/power/cy_power.c \
	libraries/sdk/tviibh4m/src/mw/smif_mem/cy_smif_device_common.c \
	libraries/sdk/tviibh4m/src/mw/smif_mem/cy_smif_hb_flash.c \
	libraries/sdk/tviibh4m/src/mw/smif_mem/cy_smif_s25fl.c \
	libraries/sdk/tviibh4m/src/mw/smif_mem/cy_smif_s25fs.c \
	libraries/sdk/tviibh4m/src/mw/smif_mem/cy_smif_semp.c \
	libraries/sdk/common/src/mw/button/cy_button.c \
	libraries/sdk/common/src/mw/flash/cy_mw_flash.c \
	libraries/sdk/common/src/mw/semihosting/cy_semihosting.c \
	libraries/sdk/common/src/mw/sw_timer/cy_sw_tmr.c

# ZF Common
ZF_COMMON_SRCS = \
	libraries/zf_common/zf_common_clock.c \
	libraries/zf_common/zf_common_debug.c \
	libraries/zf_common/zf_common_fifo.c \
	libraries/zf_common/zf_common_font.c \
	libraries/zf_common/zf_common_function.c \
	libraries/zf_common/zf_common_interrupt.c

# ZF Driver
ZF_DRIVER_SRCS = \
	libraries/zf_driver/zf_driver_adc.c \
	libraries/zf_driver/zf_driver_delay.c \
	libraries/zf_driver/zf_driver_dma.c \
	libraries/zf_driver/zf_driver_encoder.c \
	libraries/zf_driver/zf_driver_exti.c \
	libraries/zf_driver/zf_driver_flash.c \
	libraries/zf_driver/zf_driver_gpio.c \
	libraries/zf_driver/zf_driver_ipc.c \
	libraries/zf_driver/zf_driver_pit.c \
	libraries/zf_driver/zf_driver_pwm.c \
	libraries/zf_driver/zf_driver_soft_iic.c \
	libraries/zf_driver/zf_driver_soft_spi.c \
	libraries/zf_driver/zf_driver_spi.c \
	libraries/zf_driver/zf_driver_timer.c \
	libraries/zf_driver/zf_driver_uart.c

# ZF Device
ZF_DEVICE_SRCS = \
	libraries/zf_device/zf_device_ble6a20.c \
	libraries/zf_device/zf_device_config.c \
	libraries/zf_device/zf_device_dl1a.c \
	libraries/zf_device/zf_device_dl1b.c \
	libraries/zf_device/zf_device_gnss.c \
	libraries/zf_device/zf_device_icm20602.c \
	libraries/zf_device/zf_device_imu660ra.c \
	libraries/zf_device/zf_device_imu660rb.c \
	libraries/zf_device/zf_device_imu660rc.c \
	libraries/zf_device/zf_device_imu963ra.c \
	libraries/zf_device/zf_device_ips114.c \
	libraries/zf_device/zf_device_ips200.c \
	libraries/zf_device/zf_device_ips200pro.c \
	libraries/zf_device/zf_device_key.c \
	libraries/zf_device/zf_device_lora3a22.c \
	libraries/zf_device/zf_device_menc15a.c \
	libraries/zf_device/zf_device_mt9v03x.c \
	libraries/zf_device/zf_device_oled.c \
	libraries/zf_device/zf_device_pmw3901.c \
	libraries/zf_device/zf_device_tft180.c \
	libraries/zf_device/zf_device_tsl1401.c \
	libraries/zf_device/zf_device_type.c \
	libraries/zf_device/zf_device_uart_receiver.c \
	libraries/zf_device/zf_device_wifi_spi.c \
	libraries/zf_device/zf_device_wifi_uart.c \
	libraries/zf_device/zf_device_wireless_uart.c

# ZF Components
ZF_COMP_SRCS = \
	libraries/zf_components/seekfree_assistant.c \
	libraries/zf_components/seekfree_assistant_interface.c

# Project — application code
PROJ_CODE_SRCS = \
	project/code/Body_ctrl.c \
	project/code/Common_peripherals.c \
	project/code/Flash.c \
	project/code/Imu.c \
	project/code/Menu.c \
	project/code/One_bridge.c \
	project/code/Remote_ctrl.c \
	project/code/Rotation.c \
	project/code/Stair_test.c \
	project/code/gnss_cache.c \
	project/code/small_driver_uart_control.c \
	project/code/yaw_fusion.c

# Project — user entry points
PROJ_USER_SRCS = \
	project/user/cm7_0_isr.c \
	project/user/main_cm7_0.c

# ─── Aggregate Sources ────────────────────────────────────────────────────────

C_SRCS   = \
	$(SDK_STARTUP_SRCS) \
	$(SDK_SYSTEM_SRCS) \
	$(SDK_INT_SRCS) \
	$(SDK_DEV_SRCS) \
	$(SDK_COM_SRCS) \
	$(SDK_MW_SRCS) \
	$(ZF_COMMON_SRCS) \
	$(ZF_DRIVER_SRCS) \
	$(ZF_DEVICE_SRCS) \
	$(ZF_COMP_SRCS) \
	$(PROJ_CODE_SRCS) \
	$(PROJ_USER_SRCS)

OBJS     = $(addprefix $(BUILD_DIR)/, $(C_SRCS:.c=.o) $(ASM_SRCS:.S=.o))
DEPS     = $(OBJS:.o=.d)

# ─── Libraries ────────────────────────────────────────────────────────────────

# CMSIS DSP (ARM GCC) — only needed if arm_math.h functions are actually called
# Uncomment if using DSP functions: -larm_cortexM7lfsp_math
DSP_LIB   =
# DSP_LIB   = $(shell $(CC) $(CPU) $(FPU) $(FLOAT) -print-file-name=libarm_cortexM7lfsp_math.a)

LIBS     = $(DSP_LIB) -lm

# ─── Rules ────────────────────────────────────────────────────────────────────

.PHONY: all clean flash size

all: $(TARGET_ELF) $(TARGET_HEX) $(TARGET_BIN)
	@echo ""
	@$(SIZE) $(TARGET_ELF)

$(TARGET_ELF): $(OBJS) $(LDSCRIPT)
	@mkdir -p $(dir $@)
	@echo "  LD    $@"
	$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(TARGET_HEX): $(TARGET_ELF)
	@echo "  HEX   $@"
	$(OBJCOPY) -O ihex $< $@

$(TARGET_BIN): $(TARGET_ELF)
	@echo "  BIN   $@"
	$(OBJCOPY) -O binary $< $@

# C compilation
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "  CC    $<"
	$(CC) $(CFLAGS) -MMD -MP -MF $(BUILD_DIR)/$*.d -c $< -o $@

# Assembly compilation
$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo "  AS    $<"
	$(AS) $(AFLAGS) -c $< -o $@

# ─── Utilities ────────────────────────────────────────────────────────────────

size: $(TARGET_ELF)
	$(SIZE) $<

clean:
	rm -rf $(BUILD_DIR)

flash: $(TARGET_HEX)
	@echo "  FLASH — configure CFLASH_CMD for your debug probe"
	@echo "  Example (J-Link):  JLinkExe -device CYT4BB7CEE -if SWD -speed 4000 -autoconnect 1 -CommanderScript flash.jlink"
	@echo "  Example (OpenOCD): openocd -f board/cyt4bb.cfg -c 'program $(TARGET_HEX) verify reset exit'"

# ─── Dependency Tracking ──────────────────────────────────────────────────────

-include $(DEPS)

# Help
help:
	@echo "CYT4BB CM7_0 Firmware Build"
	@echo ""
	@echo "Targets:"
	@echo "  make          Build firmware → build/cm7_0.{elf,hex,bin}"
	@echo "  make clean    Remove build artifacts"
	@echo "  make size     Print section sizes"
	@echo "  make flash    Flash to target (requires debug probe)"
	@echo ""
	@echo "Variables:"
	@echo "  CROSS=        Toolchain prefix (default: arm-none-eabi-)"
	@echo "  BUILD_DIR=    Build output directory (default: build/cm7_0)"
	@echo "  V=1           Verbose output"
