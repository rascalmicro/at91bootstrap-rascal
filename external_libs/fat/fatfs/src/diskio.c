/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include <board.h>
#include <board_memories.h>
#include <utility/trace.h>
#include <utility/assert.h>
#include <fatfs/src/diskio.h>
#include <memories/Media.h>

#include <string.h>
#include <stdio.h>
#include <fatfs_config.h>

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE drv                /* Physical drive number (0..) */
)
{
    DSTATUS stat = STA_NOINIT;

    switch (drv) {
        case DRV_SDRAM :
            stat = 0;
            break;

        case DRV_MMC :
            stat = 0;
            break;
    }

    return stat;
}

/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE drv        /* Physical drive number (0..) */
)
{
    DSTATUS stat=STA_NOINIT;

    switch (drv) {
        case DRV_SDRAM :
            stat = 0;  // ok
            break;

        case DRV_MMC :
            stat = 0;  // ok
            break;            
    }

    return stat;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE drv,        /* Physical drive number (0..) */
    BYTE *buff,        /* Data buffer to store read data */
    DWORD sector,    /* Sector number (LBA) */
    BYTE count        /* Sector count (1..255) */
)
{
    unsigned char result;
    DRESULT res = RES_ERROR;
    unsigned sectorSize = 0;
    
    if (drv == DRV_SDRAM) {
        sectorSize = SECTOR_SIZE_SDRAM;
    }
    else if(drv == DRV_MMC) {
        sectorSize = SECTOR_SIZE_SDCARD;
    }
    
    result = MED_Read(&medias[drv],
                      medias[drv].baseAddress + (sector * sectorSize), // address
                      (void*)buff,          // data                                            
                      count * sectorSize,  // data size
                      NULL,
                      NULL);

    if( result == MED_STATUS_SUCCESS ) {
        res = RES_OK;
    }
    else {
        TRACE_ERROR("MED_Read pb: 0x%X\n\r", result);
        res = RES_ERROR;
    }

    return res;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0

DRESULT disk_write (
    BYTE drv,            /* Physical drive number (0..) */
    const BYTE *buff,    /* Data to be written */
    DWORD sector,        /* Sector number (LBA) */
    BYTE count            /* Sector count (1..255) */
)
{
    DRESULT res=RES_PARERR;
    unsigned int result;
    unsigned sectorSize;
    
    if (drv == DRV_SDRAM) {
        sectorSize = SECTOR_SIZE_SDRAM;
    }
    else if(drv == DRV_MMC) {
        sectorSize = SECTOR_SIZE_SDCARD;
    }
      
    result = MED_Write(&medias[drv],
                       medias[drv].baseAddress + (sector * sectorSize), // address
                       (void*)buff,          // data
                       count * sectorSize,   // data size
                       NULL,
                       NULL);

    if( result == MED_STATUS_SUCCESS ) {
      
        res = RES_OK;
    }
    else {
      
        TRACE_ERROR("MED_Write pb: 0x%X\n\r", result);
        res = RES_ERROR;
    }

    return res;
}
#endif /* _READONLY */

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
// Command    Description 
//
//CTRL_SYNC    Make sure that the disk drive has finished pending write process.
// When the disk I/O module has a write back cache, flush the dirty sector immediately.
// In read-only configuration, this command is not needed.
//
//GET_SECTOR_COUNT    Returns total sectors on the drive into the DWORD variable pointed by Buffer.
// This command is used in only f_mkfs function.
//
//GET_BLOCK_SIZE    Returns erase block size of the memory array in unit
// of sector into the DWORD variable pointed by Buffer.
// When the erase block size is unknown or magnetic disk device, return 1.
// This command is used in only f_mkfs function.
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE drv,        /* Physical drive number (0..) */
    BYTE ctrl,        /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{ 
    DRESULT res=RES_PARERR;
   
    switch (drv) {
        case DRV_SDRAM :
        switch (ctrl) { 

            case GET_BLOCK_SIZE:
                *(WORD*)buff = 1; 
                res = RES_OK; 
                break; 
            
            case GET_SECTOR_COUNT :   /* Get number of sectors on the disk (DWORD) */ 
                *(DWORD*)buff = (DWORD)((medias[DRV_SDRAM].size)/SECTOR_SIZE_SDRAM); 
                res = RES_OK; 
                break; 

            case GET_SECTOR_SIZE :   /* Get sectors on the disk (WORD) */ 
                *(WORD*)buff = SECTOR_SIZE_SDRAM; 
                res = RES_OK; 
                break; 

            case CTRL_SYNC :   /* Make sure that data has been written */ 
                res = RES_OK; 
                break; 

            default: 
                res = RES_PARERR; 
        }
        break;

    case DRV_MMC :
        switch (ctrl) { 

            case GET_BLOCK_SIZE:
                *(WORD*)buff = 1; 
                res = RES_OK; 
                break; 
            
            case GET_SECTOR_COUNT :   /* Get number of sectors on the disk (DWORD) */ 
                *(DWORD*)buff = (DWORD)((medias[DRV_MMC].size)/SECTOR_SIZE_SDCARD);
                res = RES_OK; 
                break; 

            case GET_SECTOR_SIZE :   /* Get sectors on the disk (WORD) */ 
                *(WORD*)buff = SECTOR_SIZE_SDCARD; 
                res = RES_OK; 
                break; 

            case CTRL_SYNC :   /* Make sure that data has been written */ 
                res = RES_OK; 
                break; 

            default: 
                res = RES_PARERR; 
        }
        break;
    } 

   return res; 
}
