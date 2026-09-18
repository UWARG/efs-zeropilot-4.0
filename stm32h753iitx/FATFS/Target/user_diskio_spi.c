#include "stm32h7xx_hal.h"
#include "user_diskio_spi.h"
#include "utils.h"

#ifdef SPI_INTERFACE

extern SPI_HandleTypeDef hspi1;

#define FCLK_SLOW() { MODIFY_REG(hspi1.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_128); }
#define FCLK_FAST() { MODIFY_REG(hspi1.Instance->CR1, SPI_BAUDRATEPRESCALER_256, SPI_BAUDRATEPRESCALER_8); }

#define CS_HIGH() {HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);}
#define CS_LOW()  {HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);}

#define CMD0    (0)
#define CMD1    (1)
#define ACMD41  (0x80+41)
#define CMD8    (8)
#define CMD9    (9)
#define CMD10   (10)
#define CMD12   (12)
#define ACMD13  (0x80+13)
#define CMD16   (16)
#define CMD17   (17)
#define CMD18   (18)
#define CMD23   (23)
#define ACMD23  (0x80+23)
#define CMD24   (24)
#define CMD25   (25)
#define CMD32   (32)
#define CMD33   (33)
#define CMD38   (38)
#define CMD55   (55)
#define CMD58   (58)

#define CT_MMC      0x01
#define CT_SD1      0x02
#define CT_SD2      0x04
#define CT_SDC      (CT_SD1|CT_SD2)
#define CT_BLOCK    0x08

static volatile DSTATUS Stat = STA_NOINIT;
static volatile uint8_t spiTxFlag = 0;

void setSpiTxFlag(uint8_t val) {
  spiTxFlag = val;
}

static uint8_t blankTxBuf[512] = {0};
static BYTE CardType;

static BYTE xchg_spi (BYTE dat) {
    BYTE rxDat;
    if (osKernelGetState() == osKernelRunning) {
        HAL_SPI_TransmitReceive(&hspi1, &dat, &rxDat, 1, timeToTicks(SD_TIMEOUT));
    } else {
        HAL_SPI_TransmitReceive(&hspi1, &dat, &rxDat, 1, SD_TIMEOUT);
    }
    return rxDat;
}

static uint32_t spiTimerTickStart;
static uint32_t spiTimerTickDelay;

static void SPI_Timer_On(uint32_t waitTicks) {
    spiTimerTickStart = HAL_GetTick();
    spiTimerTickDelay = waitTicks;
}

static uint8_t SPI_Timer_Status() {
    return ((HAL_GetTick() - spiTimerTickStart) < spiTimerTickDelay);
}

static int wait_ready (UINT wt) {
    BYTE d;
    uint32_t waitSpiTimerTickStart = HAL_GetTick();
    uint32_t waitSpiTimerTickDelay = (uint32_t)wt;
    do {
        d = xchg_spi(0xFF);
    } while (d != 0xFF && ((HAL_GetTick() - waitSpiTimerTickStart) < waitSpiTimerTickDelay));
    return (d == 0xFF) ? 1 : 0;
}

static void rcvr_spi_multi (BYTE *buff, UINT btr) {
    if (osKernelGetState() == osKernelRunning) {
        HAL_SPI_TransmitReceive_DMA(&hspi1, blankTxBuf, buff, btr);
        uint32_t start = HAL_GetTick();
        while(!spiTxFlag && (HAL_GetTick() - start) < SD_TIMEOUT) {
            HAL_Delay(10);
        }
        spiTxFlag = 0;
    } else {
        HAL_SPI_TransmitReceive(&hspi1, blankTxBuf, buff, btr, HAL_MAX_DELAY);
    }
}

#if _USE_WRITE
static void xmit_spi_multi (BYTE *buff, UINT btx) {
    if (osKernelGetState() == osKernelRunning) {
        HAL_SPI_Transmit_DMA(&hspi1, buff, btx);
        uint32_t start = HAL_GetTick();
        while(!spiTxFlag && (HAL_GetTick() - start) < SD_TIMEOUT) {
            HAL_Delay(10);
        }
        spiTxFlag = 0;
    } else {
        HAL_SPI_Transmit(&hspi1, buff, btx, HAL_MAX_DELAY);
    }
}
#endif

static void despiselect (void) {
    CS_HIGH();
    xchg_spi(0xFF);
}

static int spiselect (void) {
    CS_LOW();
    xchg_spi(0xFF);
    if (wait_ready(500)) return 1;
    despiselect();
    return 0;
}

static int rcvr_datablock (BYTE *buff, UINT btr) {
    BYTE token;
    SPI_Timer_On(200);
    do {
        token = xchg_spi(0xFF);
    } while ((token == 0xFF) && SPI_Timer_Status());
    if(token != 0xFE) return 0;
    rcvr_spi_multi(buff, btr);
    xchg_spi(0xFF); xchg_spi(0xFF);
    return 1;
}

#if _USE_WRITE
static int xmit_datablock (BYTE *buff, BYTE token) {
    BYTE resp;
    if (!wait_ready(500)) return 0;
    xchg_spi(token);
    if (token != 0xFD) {
        xmit_spi_multi(buff, 512);
        xchg_spi(0xFF); xchg_spi(0xFF);
        resp = xchg_spi(0xFF);
        if ((resp & 0x1F) != 0x05) return 0;
    }
    return 1;
}
#endif

static BYTE send_cmd (BYTE cmd, DWORD arg) {
    BYTE n, res;
    if (cmd & 0x80) {
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }
    if (cmd != CMD12) {
        despiselect();
        if (!spiselect()) return 0xFF;
    }
    BYTE cmdPac[6] = {0};
    cmdPac[0] = (0x40 | cmd);
    cmdPac[1] = (arg >> 24);
    cmdPac[2] = (arg >> 16);
    cmdPac[3] = (arg >> 8);
    cmdPac[4] = arg;
    n = 0x01;
    if (cmd == CMD0) n = 0x95;
    if (cmd == CMD8) n = 0x87;
    cmdPac[5] = n;
    xmit_spi_multi(cmdPac, 6);
    if (cmd == CMD12) xchg_spi(0xFF);
    n = 10;
    do {
        res = xchg_spi(0xFF);
    } while ((res & 0x80) && --n);
    return res;
}

DSTATUS USER_SPI_initialize (BYTE drv) {
    BYTE n, cmd, ty, ocr[4];
    if (drv != 0) return STA_NOINIT;
    if (Stat & STA_NODISK) return Stat;
    FCLK_SLOW();
    for (n = 10; n; n--) xchg_spi(0xFF);
    ty = 0;
    if (send_cmd(CMD0, 0) == 1) {
        SPI_Timer_On(1000);
        if (send_cmd(CMD8, 0x1AA) == 1) {
            for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                while (SPI_Timer_Status() && send_cmd(ACMD41, 1UL << 30)) ;
                if (SPI_Timer_Status() && send_cmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = xchg_spi(0xFF);
                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        } else {
            if (send_cmd(ACMD41, 0) <= 1) {
                ty = CT_SD1; cmd = ACMD41;
            } else {
                ty = CT_MMC; cmd = CMD1;
            }
            while (SPI_Timer_Status() && send_cmd(cmd, 0)) ;
            if (!SPI_Timer_Status() || send_cmd(CMD16, 512) != 0)
                ty = 0;
        }
    }
    CardType = ty;
    despiselect();
    if (ty) {
        FCLK_FAST();
        Stat &= ~STA_NOINIT;
    } else {
        Stat = STA_NOINIT;
    }
    return Stat;
}

DSTATUS USER_SPI_status (BYTE drv) {
    if (drv) return STA_NOINIT;
    return Stat;
}

DRESULT USER_SPI_read (BYTE drv, BYTE *buff, DWORD sector, UINT count) {
    if (drv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (!(CardType & CT_BLOCK)) sector *= 512;
    if (count == 1) {
        if ((send_cmd(CMD17, sector) == 0) && rcvr_datablock(buff, 512)) {
            count = 0;
        }
    } else {
        if (send_cmd(CMD18, sector) == 0) {
            do {
                if (!rcvr_datablock(buff, 512)) break;
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0);
        }
    }
    despiselect();
    return count ? RES_ERROR : RES_OK;
}

#if _USE_WRITE
DRESULT USER_SPI_write (BYTE drv, BYTE *buff, DWORD sector, UINT count) {
    if (drv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;
    if (!(CardType & CT_BLOCK)) sector *= 512;
    if (count == 1) {
        if ((send_cmd(CMD24, sector) == 0) && xmit_datablock(buff, 0xFE)) {
            count = 0;
        }
    } else {
        if (CardType & CT_SDC) send_cmd(ACMD23, count);
        if (send_cmd(CMD25, sector) == 0) {
            do {
                if (!xmit_datablock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!xmit_datablock(0, 0xFD)) count = 1;
        }
    }
    despiselect();
    return count ? RES_ERROR : RES_OK;
}
#endif

#if _USE_IOCTL
DRESULT USER_SPI_ioctl (BYTE drv, BYTE cmd, void *buff) {
    DRESULT res;
    BYTE n, csd[16];
    DWORD *dp, st, ed, csize;
    if (drv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    res = RES_ERROR;
    switch (cmd) {
    case CTRL_SYNC :
        if (spiselect()) res = RES_OK;
        break;
    case GET_SECTOR_COUNT :
        if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
            if ((csd[0] >> 6) == 1) {
                csize = csd[9] + ((WORD)csd[8] << 8) + ((DWORD)(csd[7] & 63) << 16) + 1;
                *(DWORD*)buff = csize << 10;
            } else {
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((WORD)csd[7] << 2) + ((WORD)(csd[6] & 3) << 10) + 1;
                *(DWORD*)buff = csize << (n - 9);
            }
            res = RES_OK;
        }
        break;
    case GET_BLOCK_SIZE :
        if (CardType & CT_SD2) {
            if (send_cmd(ACMD13, 0) == 0) {
                xchg_spi(0xFF);
                if (rcvr_datablock(csd, 16)) {
                    for (n = 64 - 16; n; n--) xchg_spi(0xFF);
                    *(DWORD*)buff = 16UL << (csd[10] >> 4);
                    res = RES_OK;
                }
            }
        } else {
            if ((send_cmd(CMD9, 0) == 0) && rcvr_datablock(csd, 16)) {
                if (CardType & CT_SD1) {
                    *(DWORD*)buff = (((csd[10] & 63) << 1) + ((WORD)(csd[11] & 128) >> 7) + 1) << ((csd[13] >> 6) - 1);
                } else {
                    *(DWORD*)buff = ((WORD)((csd[10] & 124) >> 2) + 1) * (((csd[11] & 3) << 3) + ((csd[11] & 224) >> 5) + 1);
                }
                res = RES_OK;
            }
        }
        break;
    case CTRL_TRIM :
        if (!(CardType & CT_SDC)) break;
        if (USER_SPI_ioctl(drv, MMC_GET_CSD, csd)) break;
        if (!(csd[0] >> 6) && !(csd[10] & 0x40)) break;
        dp = buff; st = dp[0]; ed = dp[1];
        if (!(CardType & CT_BLOCK)) { st *= 512; ed *= 512; }
        if (send_cmd(CMD32, st) == 0 && send_cmd(CMD33, ed) == 0 && send_cmd(CMD38, 0) == 0 && wait_ready(30000)) {
            res = RES_OK;
        }
        break;
    default:
        res = RES_PARERR;
    }
    despiselect();
    return res;
}
#endif

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        spiTxFlag = 1;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        spiTxFlag = 1;
    }
}

#endif
