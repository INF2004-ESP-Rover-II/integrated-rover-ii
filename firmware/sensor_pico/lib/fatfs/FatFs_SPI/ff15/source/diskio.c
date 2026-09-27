/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/
#include "ff.h"
#include "diskio.h"
#include "sd_card.h"

extern sd_card_t *sd_get_by_num(int num);
extern bool sd_init_driver(void);

//-----------------------------------------------------------------------
// Get Drive Status
//-----------------------------------------------------------------------
DSTATUS disk_status(BYTE pdrv) {
    if (pdrv != 0)
        return STA_NOINIT;

    sd_card_t *pSD = sd_get_by_num(0);
    if (!pSD)
        return STA_NOINIT;

    return pSD->m_Status;
}

//-----------------------------------------------------------------------
// Initialize a Drive
//-----------------------------------------------------------------------
DSTATUS disk_initialize(BYTE pdrv) {
    printf("disk_initialize called for drive %d\n", pdrv);

    if (!sd_init_driver()) {
        printf("sd_init_driver() failed\n");
        return STA_NOINIT;
    }

    sd_card_t *pSD = sd_get_by_num(0);
    if (!pSD) {
        printf("sd_get_by_num() returned NULL\n");
        return STA_NOINIT;
    }

    printf("Initializing SD card via pSD->init...\n");
    DSTATUS result = pSD->init(pSD);
    printf("Result from sd_init(): 0x%02X\n", result);
    return result;
}


//-----------------------------------------------------------------------
// Read Sector(s)
//-----------------------------------------------------------------------
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0)
        return RES_PARERR;

    sd_card_t *pSD = sd_get_by_num(0);
    if (!pSD)
        return RES_ERROR;

    int rc = pSD->read_blocks(pSD, buff, sector, count);
    return (rc == 0) ? RES_OK : RES_ERROR;
}

//-----------------------------------------------------------------------
// Write Sector(s)
//-----------------------------------------------------------------------
#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0)
        return RES_PARERR;

    sd_card_t *pSD = sd_get_by_num(0);
    if (!pSD)
        return RES_ERROR;

    int rc = pSD->write_blocks(pSD, buff, sector, count);
    return (rc == 0) ? RES_OK : RES_ERROR;
}
#endif

//-----------------------------------------------------------------------
// Miscellaneous Functions
//-----------------------------------------------------------------------
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != 0)
        return RES_PARERR;

    sd_card_t *pSD = sd_get_by_num(0);
    if (!pSD)
        return RES_ERROR;

    switch (cmd) {
        case CTRL_SYNC:
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512;
            return RES_OK;
        case GET_SECTOR_COUNT:
            *(LBA_t *)buff = pSD->sectors;
            return RES_OK;
        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}
