/**
 ******************************************************************************
  * @file    user_diskio_spi.h
  * @brief   This file contains the common defines and functions prototypes for
  *          the user_diskio_spi driver implementation
  ******************************************************************************
  * Portions copyright (C) 2014, ChaN, all rights reserved.
  * Portions copyright (C) 2017, kiwih, all rights reserved.
  *
  * This software is a free software and there is NO WARRANTY.
  * No restriction on use. You can use, modify and redistribute it for
  * personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
  * Redistributions of source code must retain the above copyright notice.
  *
  ******************************************************************************
  */

#pragma once

#include "ff_gen_drv.h"
#include "user_diskio.h"

#ifdef SPI_INTERFACE

#define SD_TIMEOUT 1000

#ifdef __cplusplus
extern "C" {
#endif

void setSpiTxFlag(uint8_t val);

#ifdef __cplusplus
}
#endif

DSTATUS USER_SPI_initialize (BYTE drv);
DSTATUS USER_SPI_status (BYTE drv);
DRESULT USER_SPI_read (BYTE drv, BYTE *buff, DWORD sector, UINT count);

#if _USE_WRITE == 1
DRESULT USER_SPI_write (BYTE drv, BYTE *buff, DWORD sector, UINT count);
#endif

#if _USE_IOCTL == 1
DRESULT USER_SPI_ioctl (BYTE drv, BYTE cmd, void *buff);
#endif

#endif
