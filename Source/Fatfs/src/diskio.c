/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2016        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */

#include "SDMMC.h"

#define DEV_MMC		0	/* Example: Map MMC/SD card to physical drive 1 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
    BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
    switch (pdrv)
    {
    case DEV_MMC :
        return 0x00;
    }
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
    BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
    DSTATUS stat;

    switch (pdrv)
    {
    case DEV_MMC :
        if(BSP_SD_Init() == MSD_OK)
        {
            stat &= ~STA_NOINIT;
            return stat;
        }

        // translate the reslut code here

        return stat;
    }
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
    BYTE pdrv,		/* Physical drive nmuber to identify the drive */
    BYTE *buff,		/* Data buffer to store read data */
    DWORD sector,	/* Start sector in LBA */
    UINT count		/* Number of sectors to read */
)
{

    switch (pdrv)
    {
    case DEV_MMC :
        if(BSP_SD_ReadBlocks_DMA((uint32_t *)buff,
                                 (uint32_t) (sector),
                                 count) == MSD_OK)
        {
            while( xSemaphoreTake( SDCard.SDReadStatus, portMAX_DELAY ) != pdTRUE );
            while(BSP_SD_GetCardState() != SD_TRANSFER_OK)
            {
            }
            return RES_OK;
        }
    }

    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT disk_write (
    BYTE pdrv,			/* Physical drive nmuber to identify the drive */
    const BYTE *buff,	/* Data to be written */
    DWORD sector,		/* Start sector in LBA */
    UINT count			/* Number of sectors to write */
)
{

    switch (pdrv)
    {
    case DEV_MMC :
        if(BSP_SD_WriteBlocks_DMA((uint32_t *)buff,
                                  (uint32_t) (sector),
                                  count) == MSD_OK)
        {
            while( xSemaphoreTake( SDCard.SDWriteStatus, portMAX_DELAY ) != pdTRUE );
            while(BSP_SD_GetCardState() != SD_TRANSFER_OK)
            {
            }
            return RES_OK;
        }


    }

    return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
    BYTE pdrv,		/* Physical drive nmuber (0..) */
    BYTE cmd,		/* Control code */
    void *buff		/* Buffer to send/receive control data */
)
{

    switch (pdrv)
    {
    case DEV_MMC :

        switch(cmd)
        {
            /* Make sure that no pending write process */
        case CTRL_SYNC :
            return RES_OK;

            /* Get number of sectors on the disk (DWORD) */
        case GET_SECTOR_COUNT :
            *(DWORD *)buff = SDCard.LogBlockNbr;
            return RES_OK;

            /* Get R/W sector size (WORD) */
        case GET_SECTOR_SIZE :
            *(WORD *)buff = SDCard.LogBlockSize;
            return RES_OK;

            /* Get erase block size in unit of sector (DWORD) */
        case GET_BLOCK_SIZE :
            *(DWORD *)buff = SDCard.LogBlockSize;
            return RES_OK;

        default:
            return RES_PARERR;
        }
    }

    return RES_PARERR;
}

