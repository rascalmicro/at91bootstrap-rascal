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

#ifdef ORIGIN_norflash

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <board_memories.h>
#include <pio/pio.h>
#include <utility/assert.h>
#include <utility/trace.h>

#include <string.h>
#include "main.h"

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

// Transfer return codes
#define BOOT_NOR_SUCCESS            0 /// All requested transfer are successfull
#define BOOT_NOR_ERROR_NO_DEVICE    1 /// No nor devices has been detected

#if defined(at91sam9263)
#define AT91_NORFLASH_BASE    AT91C_EBI0_CS0
#else
#define AT91_NORFLASH_BASE    AT91C_EBI_CS0
#endif


//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initialize NOR devices and transfer one or several modules from Nor to the
/// target memory (SRAM/SDRAM).
/// \param pTd    Pointer to transfer descriptor array.
/// \param nbTd  Number of transfer descriptors.
//------------------------------------------------------------------------------
int BOOT_NOR_CopyBin(const Tdesc *pTd, unsigned char nbTd)
{ 
    // Initialize Nor
    //-------------------------------------------------------------------------   
    // => the configuration was already done in LowLevelInit()
    
    // Transfert data from Nor to External RAM
    //-------------------------------------------------------------------------   
    
    while (nbTd--) {    
    
        TRACE_INFO("Copy \"%s\" (%d bytes) from NOR 0x%08x to 0x%08x\n\r", 
                      pTd->strDescr, 
                      pTd->size, 
                      pTd->offset, 
                      pTd->dest
                      );        
                       
        memcpy((char *)pTd->dest, (char *)(AT91_NORFLASH_BASE + pTd->offset), pTd->size);
        
        ++pTd;
    }

    return BOOT_NOR_SUCCESS;
}

#endif // ORIGIN_norflash
