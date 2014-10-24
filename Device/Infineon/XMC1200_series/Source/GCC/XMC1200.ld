/* Generated Linker Script file */
/*
 * Template Version 1.4 dated 26 Aug 2013
 */

OUTPUT_FORMAT("elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(__Xmc1200_reset_cortex_m)

MEMORY
{
    FLASH(RX) : ORIGIN = 0x10001000, LENGTH = 0x32000
    SRAM(!RX) : ORIGIN = 0x20000000, LENGTH = 0x4000
}

stack_size = 2048;

SECTIONS
{
    /* TEXT section */

    .text : 
    {
        sText = .;
        KEEP(*(.Xmc1200.reset));
        KEEP(*(.Xmc1200.postreset));
        KEEP(*(.XmcStartup));
        *(.text .text.* .gnu.linkonce.t.*);

        /* ARM <->THUMB interworking */
        *(.glue*)
        *(.v4*)
        *(.vfp11_veneer)

        /* C++ Support */
        KEEP(*(.init))
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))
        KEEP(*(.fini))
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;

        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))

        /* Exception handling support */
        __extab_start = .;
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
        __extab_end = ABSOLUTE(.);
    } > FLASH

    /* Exception handling, exidx needs a dedicated section */
    .ARM.exidx : 
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        . = ALIGN(4);
        __exidx_end = ABSOLUTE(.);
    } > FLASH

    /* CONST data section */
    .rodata : 
    {
        *(.rodata .rodata.*)
        *(.gnu.linkonce.r*)
    } > FLASH

    . = ALIGN(16);

    /* End of RO-DATA and start of LOAD region for the veneers */
    eROData = . ;

    /* DSRAM layout (Lowest to highest)*/
    /* Veneer <-> Stack <-> BSS <-> DATA <-> HEAP */

    VeneerLoadAddr = ABSOLUTE(eROData);
    .VENEER_Code ABSOLUTE(0x2000000C): AT(ABSOLUTE(VeneerLoadAddr))
    {
        VeneerStart = .;
        KEEP(*(.XmcVeneerCode));
        . = ALIGN(4);
        VeneerEnd = .;
    } > SRAM

    VeneerSize = ABSOLUTE(VeneerEnd) - ABSOLUTE(VeneerStart);

    /* Dummy section for guard band */
    .GUARD_Band ABSOLUTE(VeneerEnd)(NOLOAD) : AT(0)
    {
        . = . + 3; /* Forces a guard band */
        . = ALIGN(16);
        Guard_Band_End = .;
    } > SRAM

    /* Dummy section for stack */
    Stack ABSOLUTE(Guard_Band_End)(NOLOAD) : AT(0)
    {
        . = . + stack_size;
        __Xmc1200_stack = .;
    } > SRAM

    /* BSS section */
    .bss : AT(ABSOLUTE(__Xmc1200_stack))
    {
        __Xmc1200_sBSS = .;
        * (.bss);
        * (.bss*);
        * (COMMON);
        *(.gnu.linkonce.b*)
        __Xmc1200_eBSS = ALIGN(4);
    } > SRAM
    /* Yes, the size must be kept outside */
    __Xmc1200_BSS_Size = __Xmc1200_eBSS - __Xmc1200_sBSS;

    /* Standard DATA and user defined DATA/BSS/CONST sections */
    DataLoadAddr = eROData + VeneerSize;
    .data ABSOLUTE(ALIGN(16)): AT(DataLoadAddr)
    {
        __Xmc1200_sData = .;
        * (.data);
        * (.data*);
        *(*.data);
        *(.gnu.linkonce.d*)
        __Xmc1200_eData = ALIGN(4);
    } > SRAM
    /* Yes, the size must be kept outside */
    __Xmc1200_Data_Size = __Xmc1200_eData - __Xmc1200_sData;

    /* Heap - Bank1*/
    __Xmc1200_heap_start = ALIGN(8);
    __Xmc1200_heap_end = ORIGIN(SRAM) + LENGTH (SRAM);
    Heap_Bank1_Start = __Xmc1200_heap_start;
    Heap_Bank1_Size  = __Xmc1200_heap_end - __Xmc1200_heap_start;

    /DISCARD/ :
    {
        *(.comment)
    }

    .stab               0 (NOLOAD) : { *(.stab) }
    .stabstr            0 (NOLOAD) : { *(.stabstr) }

    /* DWARF 1 */
    .debug              0 : { *(.debug) }
    .line               0 : { *(.line) }

    /* GNU DWARF 1 extensions */
    .debug_srcinfo      0 : { *(.debug_srcinfo) }
    .debug_sfnames      0 : { *(.debug_sfnames) }

    /* DWARF 1.1 and DWARF 2 */
    .debug_aranges      0 : { *(.debug_aranges) }
    .debug_pubnames     0 : { *(.debug_pubnames) }
    .debug_pubtypes     0 : { *(.debug_pubtypes) }

    /* DWARF 2 */
    .debug_info         0 : { *(.debug_info .gnu.linkonce.wi.*) }
    .debug_abbrev       0 : { *(.debug_abbrev) }
    .debug_line         0 : { *(.debug_line) }
    .debug_frame        0 : { *(.debug_frame) }
    .debug_str          0 : { *(.debug_str) }
    .debug_loc          0 : { *(.debug_loc) }
    .debug_macinfo      0 : { *(.debug_macinfo) }

    /* DWARF 2.1 */
    .debug_ranges       0 : { *(.debug_ranges) }

    /* SGI/MIPS DWARF 2 extensions */
    .debug_weaknames    0 : { *(.debug_weaknames) }
    .debug_funcnames    0 : { *(.debug_funcnames) }
    .debug_typenames    0 : { *(.debug_typenames) }
    .debug_varnames     0 : { *(.debug_varnames) }

    /* Build attributes */
    .build_attributes   0 : { *(.ARM.attributes) }
}
