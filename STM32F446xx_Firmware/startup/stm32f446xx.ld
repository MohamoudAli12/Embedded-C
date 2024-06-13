ENTRY(Reset_Handler)

MEMORY
{
    FLASH(rx):ORIGIN = 0x08000000,LENGTH = 512K
    SRAM(rwx):ORIGIN = 0x20000000,LENGTH = 128K
}

SECTIONS
{
    .text : 
    {
        KEEP(*(.isr_vector))
        *(.text)
        *(.text*)
        *(.rodata)
        . = ALIGN(4);
        _etext =  .;

    }>FLASH

    
    .data : 
    {
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    
    } > SRAM AT> FLASH

    _la_data = LOADADDR(.data);

    .bss :
    {
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
        . = ALIGN(4);
        end = .;
        
    }>SRAM

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

}