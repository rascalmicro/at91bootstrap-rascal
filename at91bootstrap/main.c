/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

//------------------------------------------------------------------------------
/// \dir "Bootstrap application for AT91 Microcontrollers"
///
/// !!!Purpose
///
/// At91bootstrap project is part of the AT91 booting strategy.
///
/// AT91SAM9/AT91CAP9 devices can boot from NOR flash doing eXecute In Place (XIP) or it
/// can also boot from NAND flash, serial flash, SDCARD, ... Please refer to the
/// product reference manual to get details on booting capabilities for a particular
/// device.
///
/// AT91bootstrap is a 2nd level bootloader. It is executed in internal SRAM if the AT91
/// device boots from the ROM or it is executed in place in NOR flash.
///
/// The at91bootstrap project is used at startup to:
/// -# perform AT91 initialization: Oscillators, clocks, cache...
/// -# transfer one or several application binaries from any kind of NVM to main memory
/// -# transfer execution control to the next application
///
/// !!!Requirements
///
/// The at91bootstrap project includes settings for the different booting configurations
/// possible with all Atmel evaluation kits.
///
/// !!!Description
///
/// AT91 chips embed in ROM a program called "ROM code". It is started depending on BMS (Boot Mode Select)
/// pin state on reset. This ROM code scans the contents of different media like SPI DATAFLASH,
/// NAND FLASH or SDCARD to determine if a valid application is available then it downloads the
/// application into AT91 internal SRAM and run it. To determine if a valid application is
/// present, the ROM code checks the eight ARM exception vectors.
///
/// If no application is available then SAM-BA monitor is executed. It waits for transactions
/// either on the USB device, or on the DBGU serial port. Then the SAM-BA tool can be used to
/// program FLASH or EEPROM present on your board.
///
/// For more information on this topic, please check the corresponding AT91 product datasheet section
/// Boot Program .
///
/// \image 9263_rom_boot_sequence.png  "ROM code boot sequence example (AT91SAM9263)"
///
/// To set an exemple, the typical boot sequence of linux on AT91 devices is done in several steps :
///   -# Processor comes out of reset and branches to the ROM startup code.
///   -# The ROM startup code initializes the CPU and memory controller, performing
///      only minimal initialization of on-chip devices, such as the console serial port
///      to provide boot diagnostic messages. It also sets up the memory map for the
///      kernel to use in a format that is consistent across platforms, and then jumps
///      to the boot loader.
///   -# The boot loader decompresses the kernel into RAM, and jumps to it.
///   -# The kernel starts, mounts the root file sytem, execs the init and schedule the
///      first application.
///
/// \image linux_boot_sequence.png  "Linux startup sequence example"
///
/// !!!Customization
///
/// The most important data in this project is contained in the file #boot.h#
/// It is the array "tabDesc". It contains the list of the modules to copy,
/// the module size, the source memory address, the destination memory address
/// and an optional description of the module. By default, this array contains
/// only one line whose all data is C precompilator defines.
///
/// There are 2 cases :
///    - bootstrap has to copy only one module
///    - bootstrap has to copy several modules
///
/// -1- Bootstrap with one module
///
/// In this case, all the configuration can be given as parameters in 
/// the make command line. There is no need to modify the source files to build
/// a new bootstrap. The file #make_all# contains the command lines to build 
/// all bootstraps for the ATMEL boards. 
///
/// Below the description of the command line parameters :
///    - CHIP        Chip name (Ex: at91cap9)
///    - BOARD       board name (Ex: at91cap9-dk)
///    - ORIGIN      Source memory where the module to copy will read
///                  (dataflash|serialflash|sdcard|nandflash|norflash)
///    - DESTINATION Destination memory where the module will be copied
///                  (sdram|ddram)
///    - BIN_SIZE    Binary size to copy
///    - FROM_ADDR   Address in source memory where the module will be read
///                  (not used for SDCard boot)
///    - FILE_NAME   file name to copy (only for SDcard boot)
///    - DEST_ADDR   Address in destination memory where the module will be copied
///                  and where the bootstrap program will do a jump
///    - STR_DESCR   Description of the module to copy
///    - BOOTNAME    File name of the output binary. By default the output name is
///                  boot-$(BOARD)-$(ORIGIN)2$(DESTINATION).bin
///    - TRACE_LEVEL Determine the trace level. If not present, none trace.
///
/// \note Note 1: The use of FROM_ADDR and FILE_NAME is exclusive.
///
/// \note Note 2: the last 3 parameters are optional. All the other are mandatory.
///   
/// -2- Bootstrap with several modules
///    
/// In this case, as the number of binaries to copy is totally free, the 
/// configuration can not be given entirely in command lines parameters. 
/// CHIP, BOARD, ORIGIN, DESTINATION and BOOTNAME, TRACE_LEVEL are the 
/// only parameters in make command line. As there are several modules 
/// FROM_ADDR, FILE_NAME, DEST_ADDR and STR_DESCR are no more used.
/// The description of the different modules have to be given by modifying 
/// the array "tabDescr" in boot.h. The jump address at the end of the modules
/// transfert is the  destination address of the first line in the array.
///
/// It is possible to call another "boot.h" file that this of the at91bootstrap project.
/// Add "PATH_BOOT_H=your_path" in the make command line to use another "boot.h" file
/// This parameter allows user to have the same source file for several bootstrap project.
///
/// !!!Usage
///
/// -# Refer to make_all file to get building instructions matching your needs
/// -# Load at91bootstrap binary into board memory using SAM-BA or any other
///    flasher. The ROM Code looks for a valid application by analyzing
///    the first 28 bytes corresponding to the ARM exception vectors. at91bootstrap
///    implements ARM instructions foreither branch or load PC with PC relative
///    addressing. The ROM Code looks for the sixth vector, at offset 0x14.
///    It shall contains the size of the image to download.
///    The flash programmer shall replace this sixth vector with the size of the
///    image to download.
///    Programing with SAM-BA, please use "Send Boot File" script instead of the generic
///    "Send File" command.
/// -# Start a new power up sequence to reset the board.
///
/// !!! Note
/// -# For device at91sam9260, the bootstrap's binary size should be less than 4KB due 
///    to the limitation of the internal RAM size.
/// -# For devices at91sam9g20, the device CU-ES A, the ROM Code version is v1.4 and 
///    allows only to boot a file with size less than 8 KB, despite the fact that chip has 16KB
///    in first SRAM bank. on device CU-A, the ROM Code version is v1.5 and allows to boot 
///    a file with size max of  16 kbytes., what is normal behaviour
///
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \unit
///
/// !Purpose
///
/// This file contains the main at91bootstrap sequence. It includes a lot of
/// conditional instructions. At91bootstrap features are defined by default in
/// Makefile or project file. These default settings can be overloaded by
/// Makefile arguments.
///
/// !Contents
/// The at91bootstrap sequence can be roughly broken down as follows:
///    - Cache enable
///    - External RAM configuration
///    - Transfer from NVM to destination memory of one or several chunk of binary
///    - Jump to the next application
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <aic/aic.h>
#include <cp15/cp15.h>
#include <dbgu/dbgu.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include <utility/util.h>

#include "main.h"
#include <boot.h>

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

#if defined(at91cap9) && defined(FPGAINIT)
extern unsigned int BOARD_FPGA_InitMPBlock(void);
#endif

//#define DEBUG_DUMP_MEMORY 1

//------------------------------------------------------------------------------
/// Jump to the first address to execute application
//------------------------------------------------------------------------------
static void GoToJumpAddress(unsigned int jumpAddr, unsigned int matchType)
{
    typedef void(*fctType)(volatile unsigned int, volatile unsigned int);
    void(*pFct)(volatile unsigned int r0_val, volatile unsigned int r1_val);
    
    pFct = (fctType)jumpAddr;
    pFct(0/*dummy value in r0*/, matchType/*matchType in r1*/);

    while(1);//never reach
}

//------------------------------------------------------------------------------
/// Test if at91bootstrap shall be erased. Return 1 if this is the case.
/// It is not possible to physically disable soldered memories on some AT91SAM-EK
/// boards. In this case once a valid bootstrap has been programmed, there is no
/// to come back on SAM-BA boot.
/// ShallEraseBootstrap detects state of board buttons. If one button is pressed
/// by a user, the AT91bootstrap erase itself from NVM. After the next power-up
/// sequence, SAM-BA boot will be executed.
//------------------------------------------------------------------------------
static unsigned char ShallEraseBootstrap()
{
    const Pin pButtons[] = {PINS_PUSHBUTTONS};
    unsigned char buttonIdx;

    // Configure all PIO corresponding to buttons in input
    PIO_Configure(pButtons, PIO_LISTSIZE(pButtons));
    for (buttonIdx = 0; buttonIdx < PIO_LISTSIZE(pButtons); ++buttonIdx) {

        // Check if button is pressed
        if (!PIO_Get(&(pButtons[buttonIdx]))) {

            return 1;
        }
    }
    return 0;
}

//------------------------------------------------------------------------------
/// Bootstrap main application.
/// Transfer data from media to main memory and return the next application entry
/// point address.
//------------------------------------------------------------------------------
int main()
{
    #if defined(BOOT_RECOVERY)
    unsigned char shallEraseBootstrap = 0; // Flag used to activate the boot recovery procedure
    #endif

    #if defined(at91cap9) && defined (FPGAINIT)
    unsigned int ret = 0;
    #endif

    // Enable User Reset
    AT91C_BASE_RSTC->RSTC_RMR |= AT91C_RSTC_URSTEN | (0xA5<<24);

    //-------------------------------------------------------------------------
    // ROM Code Fixes
    //-------------------------------------------------------------------------
    // at91sam9261 with ROM code 1.4 fix : Disable USB pull-up
    #ifdef at91sam9261
    AT91C_BASE_MATRIX->MATRIX_USBPCR = ~AT91C_MATRIX_USBPCR_PUON;
    #endif


    //-------------------------------------------------------------------------
    // Test if at91bootstrap shall be erased (button pressed by user)
    //-------------------------------------------------------------------------
    #if defined(BOOT_RECOVERY)
    shallEraseBootstrap = ShallEraseBootstrap();
    #endif

    //-------------------------------------------------------------------------
    // Configure traces
    //-------------------------------------------------------------------------
    TRACE_CONFIGURE_ISP(DBGU_STANDARD, 115200, BOARD_MCK);

    TRACE_INFO_WP("\n\r");
    TRACE_INFO_WP("-- AT91bootstrap Project %s --\n\r", BOOTSTRAP_VERSION);
    TRACE_INFO_WP("-- %s\n\r", BOARD_NAME);
    TRACE_INFO_WP("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
        
    TRACE_INFO("Setting: MCK = %dMHz\n\r", (int)(BOARD_MCK/1000000));

    //-------------------------------------------------------------------------
    // Enable I-Cache
    //-------------------------------------------------------------------------
    CP15_Enable_I_Cache();

    //-------------------------------------------------------------------------
    // Configure MPBLOCK
    //-------------------------------------------------------------------------
    #if defined(at91cap9) && defined(FPGAINIT)
    ret = BOARD_FPGA_InitMPBlock();
    TRACE_INFO("return value BOARD_InitMPBlock() = 0x%x\n\r", ret);
    #endif

    //-------------------------------------------------------------------------
    // Configure External memories supply
    //-------------------------------------------------------------------------

    #if defined (at91cap9)

    // EBI 1V8
    //-------------------------------------------------------------------------
    #if defined(VDDMEMSEL_EBI1V8)
    BOARD_ConfigureVddMemSel(VDDMEMSEL_1V8);
    #endif
    // EBI 3V3
    //-------------------------------------------------------------------------
    #if defined(VDDMEMSEL_EBI3V3)
    BOARD_ConfigureVddMemSel(VDDMEMSEL_3V3);
    #endif
    
    #if !defined(VDDMEMSEL_EBI1V8) && !defined(VDDMEMSEL_EBI3V3)
    #error No external memories power supply selected
    #endif // VDDMEMSEL

    #endif // at91cap9

    //-------------------------------------------------------------------------
    // Configure external RAM where the application will be transfered
    //-------------------------------------------------------------------------

    // SDRAM
    //-------------------------------------------------------------------------
    #if defined(DESTINATION_sdram)
    TRACE_INFO("Init SDRAM\n\r");
    BOARD_ConfigureSdram(BOARD_SDRAM_BUSWIDTH);
    #endif

    // DDRAM
    //-------------------------------------------------------------------------
    #if defined(DESTINATION_ddram)
    TRACE_INFO("Init DDRAM\n\r");
    BOARD_ConfigureDdram(0, BOARD_DDRAM_BUSWIDTH);
    #endif

    // BCRAM
    //-------------------------------------------------------------------------
    #if defined(DESTINATION_bcram)
    TRACE_INFO("Init BCRAM\n\r");
    BOARD_ConfigureBcram(BOARD_BCRAM_BUSWIDTH);
    #endif

    // Check that a destination memory has be selected
    #if !defined(DESTINATION_sdram) && \
        !defined(DESTINATION_ddram) && \
        !defined(DESTINATION_bcram)        
    #error No destination memory selected
    #endif

    //-------------------------------------------------------------------------
    // Configure access to memory and transfer data from memory to external RAM
    //-------------------------------------------------------------------------

    // DataFlash
    //-------------------------------------------------------------------------
    #if defined(ORIGIN_dataflash)
    #if defined(BOOT_RECOVERY)
    if (shallEraseBootstrap)
        BOOT_AT45_EraseBoot();
    else
    #endif //BOOT_RECOVERY
        BOOT_AT45_CopyBin(tabDesc, TDESC_LISTSIZE(tabDesc));
    #endif

    // SerialFlash
    //-------------------------------------------------------------------------
    #if defined(ORIGIN_serialflash)
    BOOT_AT26_CopyBin(tabDesc, TDESC_LISTSIZE(tabDesc));
    #endif

    // NandFlash
    //-------------------------------------------------------------------------
    #if defined(ORIGIN_nandflash)
    #if defined(BOOT_RECOVERY)
    if (shallEraseBootstrap)
        BOOT_NAND_EraseBoot();
    else
    #endif //BOOT_RECOVERY
        BOOT_NAND_CopyBin(tabDesc, TDESC_LISTSIZE(tabDesc));
    #endif

    // SDcard
    //-------------------------------------------------------------------------
    #if defined(ORIGIN_sdcard)
    BOOT_SDcard_CopyFile(tabDesc, TDESC_LISTSIZE(tabDesc));
    #endif

    // NorFlash
    //-------------------------------------------------------------------------
    #if defined(ORIGIN_norflash)
    BOOT_NOR_CopyBin(tabDesc, TDESC_LISTSIZE(tabDesc));
    #endif

	// Eeprom
    //-------------------------------------------------------------------------
    #if defined(ORIGIN_eeprom)
    BOOT_EEPROM_CopyBin(tabDesc, TDESC_LISTSIZE(tabDesc));
    #endif

    // Check that an origin memory has be selected
    #if !defined(ORIGIN_dataflash) && \
        !defined(ORIGIN_serialflash) && \
        !defined(ORIGIN_nandflash) && \
        !defined(ORIGIN_sdcard) && \
        !defined(ORIGIN_norflash) && \
        !defined(ORIGIN_eeprom) 
    #error No origin memory selected
    #endif

    //-------------------------------------------------------------------------
    // Jump to external RAM
    //-------------------------------------------------------------------------
    #ifdef DEBUG_DUMP_MEMORY
    UTIL_DbguDumpMemory(tabDesc[0].dest, 0x200);
    #endif
 
    TRACE_INFO("Jump to 0x%08x\n\r", tabDesc[0].dest);
    GoToJumpAddress(tabDesc[0].dest, MACH_TYPE);

    return 0;//never reach
}

