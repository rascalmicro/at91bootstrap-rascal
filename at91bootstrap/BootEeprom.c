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

#ifdef ORIGIN_eeprom

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <aic/aic.h>
#include <dbgu/dbgu.h>
#include <twi/twi.h>
#include <utility/math.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include <drivers/twi/twid.h>
#include <drivers/async/async.h>

#include <stdio.h>
#include <string.h>
#include "main.h"

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

// Transfer return codes
#define BOOT_EEPROM_SUCCESS            0 /// All requested transfer are successfull
#define BOOT_EEPROM_ERROR_GP           1 /// 


//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

/// TWI clock frequency in Hz.
#define TWCK            400000

/// Slave address of AT24C chips.
#define AT24C_ADDRESS   0x50

/// Page size of an AT24C1024 chip (in bytes)
#define PAGE_SIZE       64

/// TWI driver instance.
static Twid twid;

/// Pio pins to configure.
static const Pin pins[] = {PINS_TWI};

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// TWI interrupt handler. Forwards the interrupt to the TWI driver handler.
//------------------------------------------------------------------------------
void ISR_Twi(void)
{
    TWID_Handler(&twid);
}

//------------------------------------------------------------------------------
/// Initialize EEPROM device. 
//------------------------------------------------------------------------------
static void EepromInit()
{
    //-------------------------------------------------------------------------
    TRACE_INFO("Init TWI EEPROM\n\r");        

    // Configure Pins
	PIO_Configure(pins, PIO_LISTSIZE(pins));

	// Configure TWI
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
    TWI_ConfigureMaster(AT91C_BASE_TWI, TWCK, BOARD_MCK);
    TWID_Initialize(&twid, AT91C_BASE_TWI);
    AIC_ConfigureIT(AT91C_ID_TWI, AT91C_AIC_PRIOR_LOWEST, ISR_Twi);
    AIC_EnableIT(AT91C_ID_TWI);

}

//------------------------------------------------------------------------------
/// Dummy callback, to test asynchronous transfer modes.
//------------------------------------------------------------------------------
void TestCallback()
{
    printf("-I- Callback fired !\n\r");
}

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Erase the first page of the EEPROM memory. This function allows to
/// auto erase the at91bootstrap in order to restart from SAM-BA boot atfter
/// the next power-up sequence.
//------------------------------------------------------------------------------
void BOOT_EEPROM_EraseBoot()
{
    // blank
}


//------------------------------------------------------------------------------
/// Initialize EEPROM devices and transfer one or sevral modules from EEPROM to the
/// target memory (SRAM/SDRAM).
/// \param pTd    Pointer to transfer descriptor array.
/// \param nbTd   Number of transfer descriptors.
//------------------------------------------------------------------------------
int BOOT_EEPROM_CopyBin(const Tdesc *pTd, unsigned char nbTd)
{ 
    unsigned char *pDest; // Dest pointer for copy
    unsigned int sizeToCopy; // remaining bytes number to copy
    unsigned int memoryOffset; // Dataflash read offset
    unsigned int packetSize; // Dataflash read size
    Async async;
    
    // Initialize EEPROM
    EepromInit();
	TRACE_INFO("Init Twi EEPROM \n\r");

	// Check word alignment
    if (pTd->offset % PAGE_SIZE) {
    
          TRACE_ERROR("Offset not word aligned\n\r");
          return BOOT_EEPROM_ERROR_GP;
    }
	
    // Transfert data from EEPROM to External RAM
    //-------------------------------------------------------------------------   
    memset(&async, 0, sizeof(async));
    async.callback = (void *) TestCallback;

    // Foreach module transfer data from EEPROM to memory
    while (nbTd--) {
       
        TRACE_INFO("Copy \"%s\" (%d bytes) from EEPROM 0x%08x to 0x%08x\n\r", 
                      pTd->strDescr, 
                      pTd->size, 
                      pTd->offset, 
                      pTd->dest
                      );    

        pDest = (unsigned char*)pTd->dest;
        sizeToCopy = pTd->size;
        memoryOffset = pTd->offset;

        while (sizeToCopy > 0) {

            // Write packet after packets
            packetSize = (sizeToCopy < PAGE_SIZE) ? sizeToCopy : PAGE_SIZE;

            // Read the page and copy
            TWID_Read(&twid, AT24C_ADDRESS, memoryOffset, 2, pDest, PAGE_SIZE, &async);
            while (!ASYNC_IsFinished(&async));

            // Update pointer
            memoryOffset += packetSize;
            pDest += packetSize;
            sizeToCopy -= packetSize;
        }

        ++pTd;
    }

    return BOOT_EEPROM_SUCCESS;
}

#endif //eeprom
