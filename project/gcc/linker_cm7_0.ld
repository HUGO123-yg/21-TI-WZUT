/******************************************************************************
 * GCC Linker Script for CYT4BB CM7_0
 *
 * Converted from IAR linker_directives_tviibh.icf
 * Target: Infineon Traveo II CYT4BB7CEE, Cortex-M7, silicon, flash build
 *
 * Memory Layout:
 *   CM0+ CODE_FLASH: 0x10000000 - 0x1007FFFF ( 512K)
 *   CM7_0 CODE_FLASH: 0x10080000 - 0x1027FFFF (2048K)
 *   CM7_1 CODE_FLASH: 0x10280000 - ...       (1600K reserved)
 *
 *   SROM private:    0x28000000 - 0x280007FF (   2K)
 *   CM0+ SRAM:       0x28000800 - 0x2801FFFF (~128K)
 *   CM7_0 SRAM:      0x28020000 - 0x2807FFFF ( 384K)
 *   CM7_1 SRAM:      0x28080000 - 0x280BFFFF ( 256K)
 *
 *   CM7_0 ITCM:      0xA0000000 - 0xA0003FFF (  16K)
 *   CM7_0 DTCM:      0xA0010000 - 0xA0013FFF (  16K)
 ******************************************************************************/

/* --------------------------------------------------------------------------
 * Memory Symbol Definitions (mirrors IAR .icf defines)
 * -------------------------------------------------------------------------- */
_CORE_cm7_0_ = 0;
_LINK_flash_ = 0;

sram_base_address        = 0x28000000;
code_flash_base_address  = 0x10000000;

cm0plus_sram_reserve      = 128K;
cm0plus_code_flash_reserve = 512K;
sram_private_for_srom     = 2K;

cm7_0_sram_reserve        = 384K;  /* 512K - 128K */
cm7_0_code_flash_reserve  = 2048K;

cm7_itcm_reserve  = 16K;
cm7_dtcm_reserve  = 16K;

stack_reserve = 8K;
heap_reserve  = 1K;

ecc_init_width = 8;

/* --------------------------------------------------------------------------
 * Memory Regions
 * -------------------------------------------------------------------------- */
MEMORY
{
    ITCM       (rx)  : ORIGIN = 0xA0000000, LENGTH = 16K
    DTCM       (rw)  : ORIGIN = 0xA0010000, LENGTH = 16K
    CODE_FLASH (rx)  : ORIGIN = code_flash_base_address + cm0plus_code_flash_reserve,
                       LENGTH = cm7_0_code_flash_reserve
    SRAM       (rwx) : ORIGIN = sram_base_address + sram_private_for_srom + cm0plus_sram_reserve,
                       LENGTH = cm7_0_sram_reserve
    WORK_FLASH (rx)  : ORIGIN = 0x14000000, LENGTH = 256K
}

/* --------------------------------------------------------------------------
 * Exported Symbols (used by startup.c / startup_customize.h)
 * -------------------------------------------------------------------------- */
__cm7_0_vector_base_linker_symbol = ORIGIN(CODE_FLASH);

__ecc_init_sram_start_address = ORIGIN(SRAM);
__ecc_init_sram_end_address   = ORIGIN(SRAM) + LENGTH(SRAM);

__ecc_init_dtcm_start_address = ORIGIN(DTCM);
__ecc_init_dtcm_end_address   = ORIGIN(DTCM) + LENGTH(DTCM);

__ecc_init_itcm_start_address = ORIGIN(ITCM);
__ecc_init_itcm_end_address   = ORIGIN(ITCM) + LENGTH(ITCM);

/* --------------------------------------------------------------------------
 * Section Layout
 * -------------------------------------------------------------------------- */
SECTIONS
{
    /* ---------- Code Flash Sections ---------- */

    /* Vector table: MUST be first in CODE_FLASH (128-byte aligned for VTOR) */
    .intvec : ALIGN(128)
    {
        KEEP(*(.intvec))
    } > CODE_FLASH

    .text :
    {
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        . = ALIGN(4);
        KEEP(*(.init))
        KEEP(*(.fini))
    } > CODE_FLASH

    .rodata :
    {
        *(.rodata*)
        . = ALIGN(4);
    } > CODE_FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > CODE_FLASH

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > CODE_FLASH

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > CODE_FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > CODE_FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > CODE_FLASH

    /* ---------- Data Sections ---------- */

    /* Load Memory Address for .data (in flash) */
    __data_lma = LOADADDR(.data);

    .data :
    {
        __data_start = .;
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    } > SRAM AT> CODE_FLASH

    /* BSS (zero-initialized data) */
    .bss :
    {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > SRAM

    /* RAM vector table copy area (same size as ROM vector table) */
    .intvec_ram (NOLOAD) : ALIGN(128)
    {
        __ramVectors = .;
        . = . + SIZEOF(.intvec);
    } > SRAM

    /* ---------- ITCM / DTCM Sections (optional, for performance-critical code/data) ---------- */

    .itcm_text :
    {
        . = ALIGN(4);
        *(.itcm_text*)
        *(.itcm_rodata*)
    } > ITCM AT> CODE_FLASH
    __itcm_lma = LOADADDR(.itcm_text);
    __itcm_start = ADDR(.itcm_text);
    __itcm_end = ADDR(.itcm_text) + SIZEOF(.itcm_text);

    .dtcm_data :
    {
        . = ALIGN(4);
        *(.dtcm_data*)
        . = ALIGN(4);
    } > DTCM AT> CODE_FLASH
    __dtcm_lma = LOADADDR(.dtcm_data);
    __dtcm_start = ADDR(.dtcm_data);
    __dtcm_end = ADDR(.dtcm_data) + SIZEOF(.dtcm_data);

    .dtcm_bss (NOLOAD) :
    {
        . = ALIGN(4);
        *(.dtcm_bss*)
        . = ALIGN(4);
    } > DTCM

    /* ---------- Heap and Stack (at end of SRAM) ---------- */

    .heap (NOLOAD) :
    {
        __heap_start = .;
        . = . + heap_reserve;
        . = ALIGN(8);
        __heap_end = .;
    } > SRAM

    /* newlib nano _sbrk expects 'end' symbol for heap start */
    end = __heap_start;
    _end = __heap_start;
    __end__ = __heap_start;

    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        __stack_start = .;
        . = . + stack_reserve;
        . = ALIGN(8);
        __stack_end = .;
    } > SRAM

    /* ---------- Work Flash Sections ---------- */
    .work_flash :
    {
        KEEP(*(.work_flash*))
    } > WORK_FLASH

    /* ---------- Debug / Discard ---------- */
    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
    }
}
