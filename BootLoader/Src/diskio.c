/*!
    \file    diskio.c
    \brief   FatFs底层磁盘I/O接口实现（最简化版本）
    
    \version 2025-01-28, V2.0.0, Simplified diskio for GD32F4xx SDIO
*/

#include "diskio.h"
#include "sdcard.h"
#include <string.h>

/* SD卡状态 */
static volatile DSTATUS Stat = STA_NOINIT;

/*-----------------------------------------------------------------------*/
/* 初始化磁盘驱动                                                         */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE pdrv)
{
    sd_error_enum status;
    sd_card_info_struct card_info;
 
    if (pdrv != 0) {
        return STA_NOINIT;
    }
 
    /* 物理无卡：标记为未初始化+无卡 */
    if (!sd_card_detect_pin_read()) {
        Stat = STA_NOINIT | STA_NODISK;
        return Stat;
    }
 
    status = sd_init();
    if (status != SD_OK) {
        Stat = STA_NOINIT;
        return Stat;
    }
 
    status = sd_card_information_get(&card_info);
    if (status != SD_OK) {
        Stat = STA_NOINIT;
        return Stat;
    }
 
    status = sd_card_select_deselect(card_info.card_rca);
    if (status != SD_OK) {
        Stat = STA_NOINIT;
        return Stat;
    }
 
    Stat &= ~(STA_NOINIT | STA_NODISK);
    return Stat;
}

/*-----------------------------------------------------------------------*/
/* 获取磁盘状态                                                           */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0) {
        return STA_NOINIT;
    }

    /* 动态根据检测脚更新状态位 */
    if (!sd_card_detect_pin_read()) {
        /* 物理上无卡 */
        Stat |= (STA_NODISK | STA_NOINIT);
    } else {
        /* 物理上有卡，清除 NODISK 标志
           NOINIT 由 disk_initialize 决定是否清掉 */
        Stat &= ~STA_NODISK;
    }

    return Stat;
}

/*-----------------------------------------------------------------------*/
/* 读取扇区                                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_read(
    BYTE pdrv,      /* 物理驱动器号 */
    BYTE *buff,     /* 数据缓冲区 */
    DWORD sector,   /* 起始扇区号 */
    UINT count      /* 扇区数 */
)
{
    sd_error_enum status;
    uint32_t addr;
    UINT i;
    uint32_t tmp[128];
    
    if (pdrv != 0 || !count) {
        return RES_PARERR;
    }
    
    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    /* 物理检测：卡已不存在，更新状态并立刻返回 */
    if (!sd_card_detect_pin_read()) {
        Stat |= (STA_NODISK | STA_NOINIT);
        return RES_NOTRDY;
    }

    /* 逐扇区读取 */
    for (i = 0; i < count; i++) {
        addr = (sector + i) * 512U;
        status = sd_block_read(tmp, addr, 512);
        if (status != SD_OK) {
            return RES_ERROR;
        }
        memcpy(buff + i * 512U, tmp, 512U);
    }
    
    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* 写入扇区                                                               */
/*-----------------------------------------------------------------------*/
#if FF_FS_READONLY == 0
DRESULT disk_write(
    BYTE pdrv,          /* 物理驱动器号 */
    const BYTE *buff,   /* 数据缓冲区 */
    DWORD sector,       /* 起始扇区号 */
    UINT count          /* 扇区数 */
)
{
    sd_error_enum status;
    uint32_t addr;
    UINT i;
    uint32_t tmp[128];
    
    if (pdrv != 0 || !count) {
        return RES_PARERR;
    }
    
    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    /* 物理检测：卡已不存在，更新状态并立刻返回 */
    if (!sd_card_detect_pin_read()) {
        Stat |= (STA_NODISK | STA_NOINIT);
        return RES_NOTRDY;
    }

    /* 逐扇区写入 */
    for (i = 0; i < count; i++) {
        addr = (sector + i) * 512U;
        memcpy(tmp, buff + i * 512U, 512U);
        status = sd_block_write(tmp, addr, 512);
        if (status != SD_OK) {
            return RES_ERROR;
        }
    }
    
    return RES_OK;
}
#endif

/*-----------------------------------------------------------------------*/
/* 其他控制功能                                                           */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl(
    BYTE pdrv,      /* 物理驱动器号 */
    BYTE cmd,       /* 控制命令 */
    void *buff      /* 缓冲区 */
)
{
    if (pdrv != 0) {
        return RES_PARERR;
    }
    
    if (Stat & STA_NOINIT) {
        return RES_NOTRDY;
    }

    /* 物理检测：卡已不存在，更新状态并立刻返回 */
    if (!sd_card_detect_pin_read()) {
        Stat |= (STA_NODISK | STA_NOINIT);
        return RES_NOTRDY;
    }
    
    switch (cmd) {
        case CTRL_SYNC:
            return RES_OK;
            
        case GET_SECTOR_COUNT:
            /* sd_card_capacity_get() returns KB */
            *(DWORD*)buff = (DWORD)(sd_card_capacity_get() * 2U);
            return RES_OK;
            
        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            return RES_OK;
            
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 1;
            return RES_OK;
            
        default:
            return RES_PARERR;
    }
}
