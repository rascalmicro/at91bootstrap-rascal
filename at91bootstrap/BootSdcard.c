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
 
#ifdef ORIGIN_sdcard

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------
#include <stdio.h>
#include <string.h>
#include <board.h>
#include <utility/trace.h>
#include <utility/assert.h>
#include <utility/util.h>
#include <utility/math.h>
#include <memories/MEDSdcard.h>

#include "fatfs_config.h"
#if _FATFS_TINY != 1
#include <fatfs/src/ff.h>
#else
#include <fatfs/src/tff.h>
#endif
#include <fatfs/src/ff_util.h>

#include "main.h"

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

// Transfer return codes
#define BOOT_SDCARD_SUCCESS       0 /// All requested transfer are successfull
#define BOOT_SDCARD_ERROR         1 

//------------------------------------------------------------------------------
//         Local variables
//------------------------------------------------------------------------------
/// File system instance.
static FATFS fs; 
/// File object instance.
static FIL fileObject;

//------------------------------------------------------------------------------
//         Local constants
//------------------------------------------------------------------------------

/// Maximum number of LUNs which can be defined.
#define MAX_LUNS        1

/// Available medias.
Media medias[MAX_LUNS];

#define ID_DRV DRV_MMC

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initialize SDcard and transfer one or sevral modules from SDCard to the
/// target memory (SRAM/SDRAM).
/// \param pTd    Pointer to transfer descriptor array.
/// \param nbTd  Number of transfer descriptors.
/// \return 0 if successfull.
//------------------------------------------------------------------------------

FRESULT res;
    
int BOOT_SDcard_CopyFile(const Tdesc *pTd, unsigned char nbTd)
{
    unsigned int ByteToRead;
    unsigned int ByteRead;
  
    // Init Disk
    MEDSdcard_Initialize(&medias[ID_DRV],0);    
    
    memset(&fs, 0, sizeof(FATFS));  // Clear file system object    
    res = f_mount(0, &fs);
    if( res != FR_OK ) {
        printf("f_mount pb: 0x%X\n\r", res);
        return 0;
    }

    while (nbTd--) {

        TRACE_INFO("Copy \"%s\" from SdCard to 0x%08x\n\r", 
                      pTd->fileName,  
                      pTd->dest       
                      );
    
        res = f_open(&fileObject, pTd->fileName , FA_OPEN_EXISTING|FA_READ);
        if( res != FR_OK ) {
            TRACE_ERROR("f_open read pb: 0x%X\n\r", res);
            return 0;
        }
    
        ByteToRead = pTd->size;
        
        // check the file size
        if(ByteToRead < fileObject.fsize) {
            TRACE_ERROR("File size %d <-> Max allowed size %d\n\r", fileObject.fsize, ByteToRead); 
            return 0;           
        }
        
        res = f_read(&fileObject, (void*)(pTd->dest), ByteToRead, &ByteRead);
        if(res != FR_OK) {
            TRACE_ERROR("f_read pb: 0x%X\n\r", res);
            return 0;
        }    
    
        res = f_close(&fileObject);
        if( res != FR_OK ) {
            TRACE_ERROR("f_close pb: 0x%X\n\r", res);
            return 0;
        }
        
        ++pTd;
    }
    
    return 0;
}

#endif // ORIGIN_sdcard
