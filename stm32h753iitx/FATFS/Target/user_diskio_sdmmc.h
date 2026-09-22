#pragma once

#include "ff_gen_drv.h"
#include "user_diskio.h"

#ifdef SDMMC_INTERFACE

#define SD_DEFAULT_BLOCK_SIZE 512
#define SD_TIMEOUT 1000

DSTATUS USER_SDMMC_initialize (BYTE lun);
DSTATUS USER_SDMMC_status (BYTE lun);
DRESULT USER_SDMMC_read (BYTE lun, BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1
DRESULT USER_SDMMC_write (BYTE lun, const BYTE *buff, DWORD sector, UINT count);
#endif

#if _USE_IOCTL == 1
DRESULT USER_SDMMC_ioctl (BYTE lun, BYTE cmd, void *buff);
#endif

#endif
