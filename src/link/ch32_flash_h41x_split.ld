/*rotoflight ch32_flash_h41x_split.ld*/
/*
*****************************************************************************
**
**  File        : ch32_flash_h41x_split.ld
**
**  Abstract    : Common linker script for CH32 devices.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY( _start )

/* Highest address of the user mode stack */
_Hot_Reboot_Flags_Size = 16;
_estack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - _Hot_Reboot_Flags_Size;    /* end of RAM */

/* Base address where the config is stored. */
__config_start = ORIGIN(FLASH_CONFIG);
__config_end = ORIGIN(FLASH_CONFIG) + LENGTH(FLASH_CONFIG);

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 4096;  /* required amount of stack */

SECTIONS
{
    /* startup code & copy code to ITCM / SRAM */
    .init :
	{
		_sinit = .;
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init)))
        KEEP(*(SORT_NONE(.handle_reset)))
		. = ALIGN(4);
		_einit = .;
	} >FLASH AT>FLASH

    /* Critical program code goes into ITCM RAM */
   /* Copy specific fast-executing code to ITCM RAM */
    tcm_code = LOADADDR(.tcm_code);
    .tcm_code :
    {
        . = ALIGN(4);
        tcm_code_start = .;
		/* Code that must be placed in the ITCM area */		
		*(.vector);
		KEEP(*(SORT_NONE(.vector_handler)))
        *(.tcm_code)
        *(.tcm_code*)
		/* Prioritize placing code in ITCM area — CRITICAL FLIGHT PATH ONLY */
		/* Core flight control modules */
		obj/main/CH32H41*/flight/*.o(.text .text.*)
		obj/main/CH32H41*/fc/*.o(.text .text.*)
		obj/main/CH32H41*/config/*.o(.text .text.*)
		obj/main/CH32H41*/pg/*.o(.text .text.*) 
		obj/main/CH32H41*/build/*.o(.text .text.*)
		obj/main/CH32H41*/common/*.o(.text .text.*)
		obj/main/CH32H41*/common/stm32/*.o(.text .text.*)
		/* Core driver modules */
		obj/main/CH32H41*/drivers/*.o(.text .text.*)
		obj/main/CH32H41*/drivers/accgyro/*.o(.text .text.*)
		obj/main/CH32H41*/drivers/barometer/*.o(.text .text.*)
		obj/main/CH32H41*/drivers/compass/*.o(.text .text.*)
		obj/main/CH32H41*/drivers/flash/*.o(.text .text.*) 
		obj/main/CH32H41*/drivers/opticalflow/*.o(.text .text.*)
		obj/main/CH32H41*/drivers/rangefinder/*.o(.text .text.*)		
		obj/main/CH32H41*/drivers/rx/*.o(.text .text.*)
		/* Core infrastructure */
		obj/main/CH32H41*/scheduler/*.o(.text .text.*)
		obj/main/CH32H41*/sensors/*.o(.text .text.*)
		obj/main/CH32H41*/rx/*.o(.text .text.*)
		obj/main/CH32H41*/main.o(.text .text.*)
		/* Critical STDPERIPH peripherals (used in hot paths) */
		obj/main/CH32H41*/ch32h417_dma.o(.text .text.*)
		obj/main/CH32H41*/ch32h417_tim.o(.text .text.*)
		obj/main/CH32H41*/ch32h417_spi.o(.text .text.*)
		obj/main/CH32H41*/ch32h417_gpio.o(.text .text.*)
		/* Remaining modules (telemetry, cli, cms, blackbox, osd, msp,
		   board, other STDPERIPH libs, USB middleware, startup, target, lib)
		   will fall through to .ram_code in SRAM */
		. = ALIGN(4);
        tcm_code_end = .;
    } >ITCM AT >MOVABLE_FLASH

    ram_code = LOADADDR(.ram_code);
    .ram_code :
    {
        . = ALIGN(4);
        ram_code_start = .;
        *(.ram_code)
        *(.ram_code*)
        *(.text)
        *(.text.*)
		
		*(.rodata)
		*(.rodata*)
        
		. = ALIGN(4);
        ram_code_end = .;
    } >SRAM AT >MOVABLE_FLASH

    /* USB DMA buffers (CherryUSB USB_NOCACHE_RAM_SECTION) — must be in
       DMA-accessible SRAM, NOT in DTCM.  Without this section the buffers
       become orphan sections placed at an arbitrary address and the USBHS
       DMA engine cannot reach them, causing enumeration failure. */
    .noncacheable (NOLOAD) :
    {
        . = ALIGN(16);
        _snoncacheable = .;
        *(.noncacheable)
        *(.noncacheable.*)
        . = ALIGN(16);
        _enoncacheable = .;
    } >SRAM

    .text :
	{
		. = ALIGN(4);
/*		
		*(.text)
		*(.text.*)
*/
/*
		*(.rodata)
		*(.rodata*)
*/		
		*(.gnu.linkonce.t.*)
		. = ALIGN(4);
	} >FLASH1 AT>MOVABLE_FLASH 

	.fini :
	{
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >FLASH1 AT>MOVABLE_FLASH

	.preinit_array  :
	{
	  PROVIDE_HIDDEN (__preinit_array_start = .);
	  KEEP (*(.preinit_array))
	  PROVIDE_HIDDEN (__preinit_array_end = .);
	} >FLASH1 AT>MOVABLE_FLASH 
	
	.init_array     :
	{
	  PROVIDE_HIDDEN (__init_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
	  KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
	  PROVIDE_HIDDEN (__init_array_end = .);
	} >FLASH1 AT>MOVABLE_FLASH 
	
	.fini_array     :
	{
	  PROVIDE_HIDDEN (__fini_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
	  KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
	  PROVIDE_HIDDEN (__fini_array_end = .);
	} >FLASH1 AT>MOVABLE_FLASH 
	
  .pg_registry :
  {
	. = ALIGN(4);
    PROVIDE_HIDDEN (__pg_registry_start = .);
    KEEP (*(.pg_registry))
    KEEP (*(SORT(.pg_registry.*)))
	. = ALIGN(4);
    PROVIDE_HIDDEN (__pg_registry_end = .);
  } >MOVABLE_FLASH
  .pg_resetdata :
  {
	. = ALIGN(4);
    PROVIDE_HIDDEN (__pg_resetdata_start = .);
    KEEP (*(.pg_resetdata))
	. = ALIGN(4);
    PROVIDE_HIDDEN (__pg_resetdata_end = .);
  } >MOVABLE_FLASH

	.ctors          :
	{
	  /* gcc uses crtbegin.o to find the start of
	     the constructors, so we make sure it is
	     first.  Because this is a wildcard, it
	     doesn't matter if the user does not
	     actually link against crtbegin.o; the
	     linker won't look for a file to match a
	     wildcard.  The wildcard also means that it
	     doesn't matter which directory crtbegin.o
	     is in.  */
	  KEEP (*crtbegin.o(.ctors))
	  KEEP (*crtbegin?.o(.ctors))
	  /* We don't want to include the .ctor section from
	     the crtend.o file until after the sorted ctors.
	     The .ctor section from the crtend file contains the
	     end of ctors marker and it must be last */
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
	  KEEP (*(SORT(.ctors.*)))
	  KEEP (*(.ctors))
	} >FLASH1 AT>MOVABLE_FLASH 
	
	.dtors          :
	{
	  KEEP (*crtbegin.o(.dtors))
	  KEEP (*crtbegin?.o(.dtors))
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
	  KEEP (*(SORT(.dtors.*)))
	  KEEP (*(.dtors))
	} >FLASH1 AT>MOVABLE_FLASH 


   _sfastram_idata = LOADADDR(.fastram_data);   
    .fastram_data :
    {
        . = ALIGN(4);
        _sfastram_data = .;        /* create a global symbol at data start */
        *(.fastram_data)           /* .data sections */
        *(.fastram_data*)          /* .data* sections */
        . = ALIGN(4);
        _efastram_data = .;        /* define a global symbol at data end */
    } >FASTRAM AT >MOVABLE_FLASH

   . = ALIGN(4);
   .fastram_bss (NOLOAD) :
   {
        _sfastram_bss = .;
        __fastram_bss_start__ = _sfastram_bss;
        *(.fastram_bss)
        *(SORT_BY_ALIGNMENT(.fastram_bss*))
        . = ALIGN(4);
        _efastram_bss = .;
        __fastram_bss_end__ = _efastram_bss;
  } >FASTRAM


      _data_lma = LOADADDR(.data);
    .data :
	{
		. = ALIGN(4);
        PROVIDE(_data_vma = .);
		KEEP(*(.region_section))
    	*(.gnu.linkonce.r.*)
    	*(.data .data.*)
    	*(.gnu.linkonce.d.*)
		. = ALIGN(8);
    	PROVIDE( __global_pointer$ = . + 0x800 );
    	*(.sdata .sdata.*)
		*(.sdata2.*)
    	*(.gnu.linkonce.s.*)
    	. = ALIGN(8);
    	*(.srodata.cst16)
    	*(.srodata.cst8)
    	*(.srodata.cst4)
    	*(.srodata.cst2)
    	*(.srodata .srodata.*)
    	. = ALIGN(4);
		PROVIDE( _edata = .);
		PROVIDE( _dtcm_end = .);
	} >RAM AT>MOVABLE_FLASH

    .bss (NOLOAD) :
	{
		. = ALIGN(4);
		PROVIDE( _sbss = .);
        __bss_start__ = _sbss;
  	    *(.sbss*)
        *(.gnu.linkonce.sb.*)
		*(.bss*)
     	*(.gnu.linkonce.b.*)		
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE( _ebss = .);
        __bss_end__ = _ebss;
	} >RAM AT>MOVABLE_FLASH

    PROVIDE( _end = _ebss);
	PROVIDE( end = . );

    /* _eusrstack = ORIGIN(STACKRAM) + LENGTH(STACKRAM) - _Hot_Reboot_Flags_Size;
    _heap_stack_begin = _heap_stack_end - _Min_Stack_Size  - _Min_Heap_Size; */
 
    .stack ORIGIN(STACKRAM) + LENGTH(STACKRAM) - _Hot_Reboot_Flags_Size - _Min_Stack_Size:
    {
        PROVIDE( _heap_end = . );    
        . = ALIGN(4);
        PROVIDE(_susrstack = . );
        . = . + _Min_Stack_Size;
        PROVIDE( _eusrstack = .);
    } >STACKRAM 


}





