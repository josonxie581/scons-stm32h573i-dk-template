/*
 ******************************************************************************
 **
 ** @file        : LinkerScript.ld
 ** @brief       : Linker script for STM32H573xx Device from STM32H5 series
 **                      2048Kbytes FLASH
 **                      640Kbytes RAM
 ******************************************************************************
 */

/* Entry Point */
ENTRY(Reset_Handler)

/* Specify memory regions */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 640K
  FLASH    (rx)    : ORIGIN = 0x8000000,   LENGTH = 2048K
}

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of "RAM" Ram type memory */

_Min_Heap_Size = 0x800;      /* required amount of heap: 2KB  */
_Min_Stack_Size = 0x800;     /* required amount of stack: 2KB */

/* Sections */
SECTIONS
{
  /* The startup code into "FLASH" Rom type memory */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH
  
  /* The program code and other data into "FLASH" Rom type memory */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))
    KEEP(*(.text._sbrk))  /* 显式保留 _sbrk */
    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data into "FLASH" Rom type memory */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into "RAM" Ram type memory */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* Uninitialized data section into "RAM" Ram type memory */
  . = ALIGN(4);
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
        _end = .;        /* 定义数据段结束标记 */
        __end__ = _end;
        PROVIDE(end = .);
    } >RAM

  /* 堆段定义 */
  .heap (NOLOAD) : ALIGN(8)
  {
    __heap_start = .;            /* 堆起始地址 */
    PROVIDE(__heap_start = .);   /* 提供全局可见的符号 */
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
    __heap_end = .;              /* 堆结束地址 */
    PROVIDE(__heap_end = .);     /* 提供全局可见的符号 */
    PROVIDE(end = .);            /* 提供 end 符号给 C 库使用 */
  } >RAM

  /* 栈段定义 */
  .stack (NOLOAD) : ALIGN(8)
  {
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
    PROVIDE(__stack = .);        /* 提供栈底符号 */
    _estack = .;                 /* 栈顶地址 */
  } >RAM

  /* 添加内存溢出检查 */
  _ram_end = ORIGIN(RAM) + LENGTH(RAM);
  ASSERT(_estack <= _ram_end, "Error: RAM overflow")

  /* Remove information from the compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}