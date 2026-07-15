SECTIONS
{
    .intvec :
    {
        . = ALIGN(128);
        KEEP(*(.intvec))
    } > CODE_FLASH

    .text :
    {
        *(.text .text.*)
        *(.glue_7 .glue_7t)
        *(.rodata .rodata.*)
        *(.init)
        *(.fini)
        . = ALIGN(4);
        __etext = .;
    } > CODE_FLASH

    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > CODE_FLASH
    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > CODE_FLASH

    .work_flash : { KEEP(*(.work_flash)) } > WORK_FLASH
    .sflash_uf_rows : { KEEP(*(.sflash_uf_rows)) } > SFLASH_UF
    .sflash_nar : { KEEP(*(.sflash_nar)) } > SFLASH_NAR
    .sflash_pkey : { KEEP(*(.sflash_pkey)) } > SFLASH_PKEY
    .sflash_app_prot : { KEEP(*(.sflash_app_prot)) } > SFLASH_AP
    .sflash_toc2 : { KEEP(*(.sflash_toc2)) } > SFLASH_TOC2

    .data :
    {
        . = ALIGN(8);
        __data_start = .;
        *(.data .data.*)
        . = ALIGN(8);
        __data_end = .;
    } > SRAM AT > CODE_FLASH
    __data_lma = LOADADDR(.data);

    .intvec_ram (NOLOAD) :
    {
        . = ALIGN(128);
        *(.intvec_ram)
    } > SRAM

    .global_ram_data (NOLOAD) : { *(.global_ram_data) } > SRAM

    /* Keep the camera DMA buffer at the addresses used by the IAR build. */
    .mt9v03x_image_size 0x28006bf0 (NOLOAD) :
    {
        *(.mt9v03x_image_size)
    }
    .mt9v03x_image_temp 0x28026024 (NOLOAD) :
    {
        *(.mt9v03x_image_temp)
    }

    .bss MAX(ADDR(.intvec_ram) + SIZEOF(.intvec_ram), __cm7_bss_start) (NOLOAD) :
    {
        . = ALIGN(8);
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(8);
        __bss_end = .;
    } > SRAM

    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __heap_start = .;
        end = .;
        . += 1K;
        __heap_end = .;
    } > SRAM

    .stack ORIGIN(SRAM) + LENGTH(SRAM) - 8K (NOLOAD) :
    {
        . = ALIGN(8);
        __stack_start = .;
        . += 8K;
        __stack_end = .;
    } > SRAM

    __ecc_init_sram_start_address = ORIGIN(SRAM);
    __ecc_init_sram_end_address = ORIGIN(SRAM) + LENGTH(SRAM) - 1;
    __ecc_init_dtcm_start_address = ORIGIN(DTCM);
    __ecc_init_dtcm_end_address = ORIGIN(DTCM) + LENGTH(DTCM) - 1;
    __ecc_init_itcm_start_address = ORIGIN(ITCM);
    __ecc_init_itcm_end_address = ORIGIN(ITCM) + LENGTH(ITCM) - 1;

    /DISCARD/ : { *(.note.GNU-stack) *(.comment) }
}
