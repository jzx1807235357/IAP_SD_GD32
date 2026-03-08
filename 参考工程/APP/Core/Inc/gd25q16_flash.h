/*!
 * \file    gd25q16_flash.h
 * \brief   GD25Q16 SPI Flash driver interface (FreeRTOS optimized)
 */

#ifndef GD25Q16_FLASH_H
#define GD25Q16_FLASH_H

#include "gd32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === Hardware configuration ================================================= */
#define FLASH_SPI                   SPI0
#define FLASH_SPI_CLK               RCU_SPI0

#define FLASH_NSS_GPIO_PORT         GPIOA
#define FLASH_NSS_GPIO_CLK          RCU_GPIOA
#define FLASH_NSS_PIN               GPIO_PIN_15  /* PA15 - manual CS */

#define FLASH_SCK_GPIO_PORT         GPIOB
#define FLASH_SCK_GPIO_CLK          RCU_GPIOB
#define FLASH_SCK_PIN               GPIO_PIN_3  /* PB3 - SCK */

#define FLASH_MISO_GPIO_PORT        GPIOB
#define FLASH_MISO_GPIO_CLK         RCU_GPIOB
#define FLASH_MISO_PIN              GPIO_PIN_4  /* PB4 - MISO */

#define FLASH_MOSI_GPIO_PORT        GPIOB
#define FLASH_MOSI_GPIO_CLK         RCU_GPIOB
#define FLASH_MOSI_PIN              GPIO_PIN_5  /* PB5 - MOSI */

#define FLASH_SCK_GPIO_AF           GPIO_AF_5
#define FLASH_MISO_GPIO_AF          GPIO_AF_5
#define FLASH_MOSI_GPIO_AF          GPIO_AF_5

/* === Device parameters ====================================================== */
#define FLASH_SIZE                  (2U * 1024U * 1024U)  /* 2 MBytes */
#define FLASH_SECTOR_SIZE           4096U
#define FLASH_PAGE_SIZE             256U
#define FLASH_SECTOR_COUNT          (FLASH_SIZE / FLASH_SECTOR_SIZE)
#define FLASH_PAGE_COUNT            (FLASH_SIZE / FLASH_PAGE_SIZE)

/* === Command set ============================================================ */
#define FLASH_CMD_WRITE_ENABLE          0x06U
#define FLASH_CMD_WRITE_DISABLE         0x04U
#define FLASH_CMD_READ_STATUS           0x05U
#define FLASH_CMD_WRITE_STATUS          0x01U
#define FLASH_CMD_READ_DATA             0x03U
#define FLASH_CMD_FAST_READ             0x0BU
#define FLASH_CMD_PAGE_PROGRAM          0x02U
#define FLASH_CMD_SECTOR_ERASE          0x20U
#define FLASH_CMD_CHIP_ERASE            0xC7U
#define FLASH_CMD_POWER_DOWN            0xB9U
#define FLASH_CMD_RELEASE_POWER_DOWN    0xABU
#define FLASH_CMD_JEDEC_ID              0x9FU

/* === Status register bits =================================================== */
#define FLASH_STATUS_BUSY           0x01U
#define FLASH_STATUS_WEL            0x02U

/* === JEDEC identification =================================================== */
#define FLASH_MANUFACTURER_ID       0xC8U
#define FLASH_DEVICE_ID             0x15U
#define FLASH_JEDEC_ID              0xC84015UL

/* === Driver status codes ==================================================== */
typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR,
    FLASH_BUSY,
    FLASH_TIMEOUT,
    FLASH_WRITE_PROTECTED,
    FLASH_INVALID_PARAM
} flash_status_t;

/* === Public API ============================================================= */
flash_status_t gd25q16_init(void);
flash_status_t gd25q16_deinit(void);
bool          gd25q16_is_initialized(void);

flash_status_t gd25q16_wake_up(void);
flash_status_t gd25q16_power_down(void);

flash_status_t gd25q16_read_id(uint32_t* jedec_id);
flash_status_t gd25q16_read_status(uint8_t* status);
flash_status_t gd25q16_wait_busy(uint32_t timeout_ms);

flash_status_t gd25q16_read(uint32_t address, uint8_t* buffer, uint32_t length);
flash_status_t gd25q16_write_page(uint32_t address, const uint8_t* buffer, uint32_t length);
flash_status_t gd25q16_write(uint32_t address, const uint8_t* buffer, uint32_t length);
flash_status_t gd25q16_erase_sector(uint32_t sector_address);
flash_status_t gd25q16_erase_chip(void);

#ifdef __cplusplus
}
#endif

#endif /* GD25Q16_FLASH_H */

