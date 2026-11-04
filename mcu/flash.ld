ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : {
    *(.text*) *(.rodata*) *(.glue_7*)
    KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4);
  } > FLASH
  .ARM.extab : { *(.ARM.extab*) } > FLASH
  .ARM.exidx : {
    __exidx_start = .; *(.ARM.exidx*); __exidx_end = .;
  } > FLASH
  .preinit_array : {
    __preinit_array_start = .; KEEP(*(.preinit_array*)); __preinit_array_end = .;
  } > FLASH
  .init_array : {
    __init_array_start = .; KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array)); __init_array_end = .;
  } > FLASH
  .fini_array : {
    __fini_array_start = .; KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array)); __fini_array_end = .;
  } > FLASH
  .data : {
    . = ALIGN(4); _sdata = .; *(.data*); . = ALIGN(4); _edata = .;
  } > RAM AT > FLASH
  _sidata = LOADADDR(.data);
  .bss (NOLOAD) : {
    . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON); . = ALIGN(4); _ebss = .;
  } > RAM
  PROVIDE(end = _ebss); PROVIDE(_end = _ebss);
  .reserved_stack (NOLOAD) : { . = ALIGN(8); . += 0x1000; } > RAM
  /DISCARD/ : { *(.note.GNU-stack) }
}
ASSERT(_ebss + 0x1000 < _estack, "Not enough stack space")
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= 0x08008000, "Firmware must fit first 32KB")
