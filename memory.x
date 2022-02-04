/* Assuming https://github.com/adafruit/Adafruit_nRF52_Bootloader default bootloader with  softdevice S140 - i.e. https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather/hathach-memory-map*/
MEMORY 
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x26000, LENGTH = 796K
  /* FLASH : ORIGIN = 0x00000000, LENGTH = 1024K /* WITHOUT SOFT DEVICE */
  RAM : ORIGIN = 0x20000000, LENGTH = 255K
  /* RAM : ORIGIN = 0x20000000, LENGTH = 256K /* WITHOUT PANDUMP */
  /* RAM : ORIGIN = 0x20000000, LENGTH = 256K /* WITHOUT SOFT DEVICE */
  PANDUMP: ORIGIN = 0x2003FC00, LENGTH = 1K
}

_panic_dump_start = ORIGIN(PANDUMP);
_panic_dump_end   = ORIGIN(PANDUMP) + LENGTH(PANDUMP);


/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

/* You can use this symbol to customize the location of the .text section */
/* If omitted the .text section will be placed right after the .vector_table
   section */
/* This is required only on microcontrollers that store some configuration right
   after the vector table */
/* _stext = ORIGIN(FLASH) + 0x400; */

/* Size of the heap (in bytes) */
/* _heap_size = 1024; */
