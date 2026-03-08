/*!
 * \file    gd25q16_flash.c
 * \brief   GD25Q16 SPI Flash driver implementation
 * \note    Uses polling mode for reliable SPI communication
 */

#include "gd25q16_flash.h"
#include "systick.h"
#include "gd32f4xx_spi.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_gpio.h"
#include "gd32f4xx_misc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stddef.h>

/* === Driver tuning ========================================================= */
#define FLASH_SPI_DUMMY_BYTE            0xFFU
#define FLASH_SPI_XFER_TIMEOUT_MS       5U
#define FLASH_POLL_DELAY_US             2U

#define FLASH_LOCK_TIMEOUT_MS           100U
#define FLASH_RESET_DELAY_MS            1U
#define FLASH_WAKE_DELAY_MS             1U
#define FLASH_WRITE_TIMEOUT_MS          5000U
#define FLASH_ERASE_TIMEOUT_MS          10000U
#define FLASH_CHIP_ERASE_TIMEOUT_MS     30000U

/* === Driver state ========================================================== */
static bool g_flash_initialized = false;
static SemaphoreHandle_t g_flash_mutex = NULL;

/* === Forward declarations ================================================== */
static void flash_gpio_init(void);
static void flash_spi_init(void);
static inline void flash_cs_low(void);
static inline void flash_cs_high(void);

static bool flash_scheduler_running(void);
static uint32_t flash_get_tick_ms(void);
static void flash_delay_ms(uint32_t ms);
static void flash_delay_us(uint32_t us);

static flash_status_t flash_lock(uint32_t timeout_ms);
static void flash_unlock(void);

static flash_status_t flash_spi_wait_flag(uint32_t flag, FlagStatus state, uint32_t timeout_ms);
static flash_status_t flash_spi_exchange(uint8_t tx_byte, uint8_t* rx_byte);
static flash_status_t flash_transfer_bytes_locked(const uint8_t* tx, uint8_t* rx, size_t length);
static inline flash_status_t flash_write_bytes_locked(const uint8_t* data, size_t length);
static inline flash_status_t flash_read_bytes_locked(uint8_t* data, size_t length);

static flash_status_t flash_read_status_locked(uint8_t* status);
static flash_status_t flash_wait_ready_locked(uint32_t timeout_ms);
static flash_status_t flash_write_enable_locked(void);
static flash_status_t flash_program_page_locked(uint32_t address, const uint8_t* buffer, uint32_t length);
static flash_status_t flash_erase_sector_locked(uint32_t sector_address);
static flash_status_t flash_erase_chip_locked(void);

/* === OS helpers ============================================================ */
static bool flash_scheduler_running(void)
{
    return (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED);
}

static uint32_t flash_get_tick_ms(void)
{
    if (flash_scheduler_running()) {
        return (uint32_t)(xTaskGetTickCount() * (uint32_t)portTICK_PERIOD_MS);
    }
    return systick_get_tick();
}

static void flash_delay_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }

    if (flash_scheduler_running()) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        delay_1ms(ms);
    }
}

static void flash_delay_us(uint32_t us)
{
    if (us == 0U) {
        return;
    }

    if (flash_scheduler_running() && us >= 1000U) {
        TickType_t ticks = pdMS_TO_TICKS((us + 999U) / 1000U);
        if (ticks == 0U) {
            ticks = 1U;
        }
        vTaskDelay(ticks);
    } else {
        delay_us(us);
    }
}

static flash_status_t flash_lock(uint32_t timeout_ms)
{
    if (!flash_scheduler_running()) {
        return FLASH_OK;
    }

    if (g_flash_mutex == NULL) {
        g_flash_mutex = xSemaphoreCreateRecursiveMutex();
        if (g_flash_mutex == NULL) {
            return FLASH_ERROR;
        }
    }

    TickType_t wait_ticks = (timeout_ms == 0U) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (g_flash_mutex != NULL && xSemaphoreTakeRecursive(g_flash_mutex, wait_ticks) != pdTRUE) {
        return FLASH_TIMEOUT;
    }

    return FLASH_OK;
}

static void flash_unlock(void)
{
    if (flash_scheduler_running() && (g_flash_mutex != NULL)) {
        xSemaphoreGiveRecursive(g_flash_mutex);
    }
}

static void flash_gpio_init(void)
{
    rcu_periph_clock_enable(FLASH_NSS_GPIO_CLK);
    rcu_periph_clock_enable(FLASH_SCK_GPIO_CLK);
    rcu_periph_clock_enable(FLASH_MISO_GPIO_CLK);
    rcu_periph_clock_enable(FLASH_MOSI_GPIO_CLK);

    /* CS引脚: PA15 - 手动控制输出 */
    gpio_mode_set(FLASH_NSS_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, FLASH_NSS_PIN);
    gpio_output_options_set(FLASH_NSS_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, FLASH_NSS_PIN);
    gpio_bit_set(FLASH_NSS_GPIO_PORT, FLASH_NSS_PIN);  /* CS初始为高 */

    /* SPI引脚复用功能配置 - 匹配GD32官方Flash驱动 */
    gpio_af_set(FLASH_SCK_GPIO_PORT, FLASH_SCK_GPIO_AF, FLASH_SCK_PIN);
    gpio_af_set(FLASH_MISO_GPIO_PORT, FLASH_MISO_GPIO_AF, FLASH_MISO_PIN);
    gpio_af_set(FLASH_MOSI_GPIO_PORT, FLASH_MOSI_GPIO_AF, FLASH_MOSI_PIN);
    
    gpio_mode_set(FLASH_SCK_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, FLASH_SCK_PIN);
    gpio_mode_set(FLASH_MISO_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, FLASH_MISO_PIN);
    gpio_mode_set(FLASH_MOSI_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, FLASH_MOSI_PIN);
    
    /* 输出引脚特性配�? SCK和MOSI为推挽输�? 50MHz速度 */
    gpio_output_options_set(FLASH_SCK_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, FLASH_SCK_PIN);
    gpio_output_options_set(FLASH_MOSI_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, FLASH_MOSI_PIN);
    /* MISO是输入引脚，不需要配置输出特�?*/
}

static void flash_spi_init(void)
{
    rcu_periph_clock_enable(FLASH_SPI_CLK);

    spi_i2s_deinit(FLASH_SPI);
    spi_parameter_struct spi_init_struct;
    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;  /* Mode 0 */
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_32;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(FLASH_SPI, &spi_init_struct);

    /* 清空接收FIFO中的任何残留数据 */
    while (SET == spi_i2s_flag_get(FLASH_SPI, SPI_FLAG_RBNE)) {
        (void)spi_i2s_data_receive(FLASH_SPI);
    }

    spi_enable(FLASH_SPI);
}

static inline void flash_cs_low(void)
{
    gpio_bit_reset(FLASH_NSS_GPIO_PORT, FLASH_NSS_PIN);
    /* 添加少量延迟确保CS稳定 */
    for (volatile int i = 0; i < 20; i++);
}

static inline void flash_cs_high(void)
{
    uint32_t start = flash_get_tick_ms();

    /* 确保最后一个字节已经移�?*/
    while (SET == spi_i2s_flag_get(FLASH_SPI, SPI_FLAG_TRANS)) {
        if ((flash_get_tick_ms() - start) >= FLASH_SPI_XFER_TIMEOUT_MS) {
            break;
        }
    }
    for (volatile int i = 0; i < 20; i++);
    gpio_bit_set(FLASH_NSS_GPIO_PORT, FLASH_NSS_PIN);
    for (volatile int i = 0; i < 20; i++);
}

flash_status_t gd25q16_init(void)
{
    if (g_flash_initialized) {
        return FLASH_OK;
    }

    flash_gpio_init();
    flash_spi_init();
    
    /* 添加延迟让硬件稳�?*/
    flash_delay_ms(10);

    flash_status_t status = gd25q16_wake_up();
    if (status != FLASH_OK) {
        return status;
    }
    
    /* 添加延迟让Flash从深度睡眠中唤醒 */
    flash_delay_ms(10);

    uint32_t jedec_id = 0U;
    status = gd25q16_read_id(&jedec_id);
    if (status != FLASH_OK) {
        return status;
    }

    if (jedec_id != FLASH_JEDEC_ID) {
        return FLASH_ERROR;
    }

    /* 清除所有写保护�?(确保Flash可写) */
    status = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (status == FLASH_OK) {
        /* 发送写使能 */
        flash_cs_low();
        flash_spi_exchange(0x06, NULL);  /* WREN - 使用exchange确保清空RX缓冲 */
        flash_cs_high();
        
        flash_delay_us(100);  /* 等待tSHWL (100ns典型) */
        
        /* 写状态寄存器 - 根据手册必须�?字节(SR1+SR2) */
        flash_cs_low();
        flash_spi_exchange(0x01, NULL);  /* WRSR命令 */
        flash_spi_exchange(0x00, NULL);  /* SR1=0x00: BP4-BP0=0, SRP0=0 */
        flash_spi_exchange(0x00, NULL);  /* SR2=0x00: CMP=0, QE=0, SRP1=0 */
        flash_cs_high();
        
        flash_delay_us(100);
        flash_unlock();
    }

    g_flash_initialized = true;
    
    return FLASH_OK;
}

flash_status_t gd25q16_deinit(void)
{
    g_flash_initialized = false;
    if (flash_scheduler_running() && g_flash_mutex != NULL) {
    #if (configSUPPORT_DYNAMIC_ALLOCATION == 1)
        vSemaphoreDelete(g_flash_mutex);
    #endif
        g_flash_mutex = NULL;
    }
    return FLASH_OK;
}



bool gd25q16_is_initialized(void)
{
    return g_flash_initialized;
}

flash_status_t gd25q16_wake_up(void)
{
    flash_status_t status = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (status != FLASH_OK) {
        return status;
    }

    flash_cs_low();
    status = flash_spi_exchange(FLASH_CMD_RELEASE_POWER_DOWN, NULL);
    flash_cs_high();

    if (status != FLASH_OK) {
        flash_unlock();
        return status;
    }

    flash_delay_ms(FLASH_WAKE_DELAY_MS);

    flash_unlock();
    return FLASH_OK;
}

flash_status_t gd25q16_power_down(void)
{
    flash_status_t status = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (status != FLASH_OK) {
        return status;
    }

    flash_cs_low();
    status = flash_spi_exchange(FLASH_CMD_POWER_DOWN, NULL);
    flash_cs_high();

    flash_unlock();
    return status;
}

flash_status_t gd25q16_read_id(uint32_t* jedec_id)
{
    if (jedec_id == NULL) {
        return FLASH_INVALID_PARAM;
    }

    flash_status_t status = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (status != FLASH_OK) {
        return status;
    }

    uint8_t id_bytes[3] = {0};
    
    flash_cs_low();
    
    flash_status_t ret = flash_spi_exchange(FLASH_CMD_JEDEC_ID, NULL);
    if (ret != FLASH_OK) {
        flash_cs_high();
        flash_unlock();
        return ret;
    }

    /* 直接读取3字节ID(Manufacturer, Device[15:8], Device[7:0]) */
    for (size_t i = 0; i < 3; ++i) {
        ret = flash_spi_exchange(FLASH_SPI_DUMMY_BYTE, &id_bytes[i]);
        if (ret != FLASH_OK) {
            flash_cs_high();
            flash_unlock();
            return ret;
        }
    }
    
    flash_cs_high();

    *jedec_id = ((uint32_t)id_bytes[0] << 16) |
                ((uint32_t)id_bytes[1] << 8)  |
                (uint32_t)id_bytes[2];

    flash_unlock();
    return FLASH_OK;
}

flash_status_t gd25q16_read_status(uint8_t* status)
{
    if (status == NULL) {
        return FLASH_INVALID_PARAM;
    }

    flash_status_t ret = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (ret != FLASH_OK) {
        return ret;
    }

    ret = flash_read_status_locked(status);
    flash_unlock();
    return ret;
}

flash_status_t gd25q16_wait_busy(uint32_t timeout_ms)
{
    flash_status_t ret = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (ret != FLASH_OK) {
        return ret;
    }

    ret = flash_wait_ready_locked(timeout_ms);
    flash_unlock();
    return ret;
}

flash_status_t gd25q16_read(uint32_t address, uint8_t* buffer, uint32_t length)
{
    if ((buffer == NULL) || (length == 0U) || (address >= FLASH_SIZE) ||
        (length > (FLASH_SIZE - address))) {
        return FLASH_INVALID_PARAM;
    }

    flash_status_t ret = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (ret != FLASH_OK) {
        return ret;
    }

    if (!g_flash_initialized) {
        flash_unlock();
        return FLASH_ERROR;
    }

    ret = flash_wait_ready_locked(FLASH_WRITE_TIMEOUT_MS);
    if (ret != FLASH_OK) {
        flash_unlock();
        return ret;
    }

    flash_cs_low();
    uint8_t header[4] = {
        FLASH_CMD_READ_DATA,
        (uint8_t)((address >> 16) & 0xFFU),
        (uint8_t)((address >> 8) & 0xFFU),
        (uint8_t)(address & 0xFFU)
    };
    ret = flash_write_bytes_locked(header, sizeof(header));
    if (ret == FLASH_OK) {
        ret = flash_read_bytes_locked(buffer, length);
    }
    flash_cs_high();

    flash_unlock();
    return ret;
}

flash_status_t gd25q16_write_page(uint32_t address, const uint8_t* buffer, uint32_t length)
{
    if ((buffer == NULL) || (length == 0U) || (length > FLASH_PAGE_SIZE) ||
        (address >= FLASH_SIZE) || (length > (FLASH_SIZE - address))) {
        return FLASH_INVALID_PARAM;
    }

    if ((address % FLASH_PAGE_SIZE) + length > FLASH_PAGE_SIZE) {
        return FLASH_INVALID_PARAM;
    }

    flash_status_t ret = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (ret != FLASH_OK) {
        return ret;
    }

    if (!g_flash_initialized) {
        flash_unlock();
        return FLASH_ERROR;
    }

    ret = flash_program_page_locked(address, buffer, length);
    flash_unlock();
    return ret;
}

flash_status_t gd25q16_write(uint32_t address, const uint8_t* buffer, uint32_t length)
{
    if ((buffer == NULL) || (length == 0U) || (address >= FLASH_SIZE) ||
        (length > (FLASH_SIZE - address))) {
        return FLASH_INVALID_PARAM;
    }

    flash_status_t ret = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (ret != FLASH_OK) {
        return ret;
    }

    if (!g_flash_initialized) {
        flash_unlock();
        return FLASH_ERROR;
    }

    uint32_t offset = 0U;
    while (offset < length) {
        uint32_t current_address = address + offset;
        uint32_t page_offset = current_address % FLASH_PAGE_SIZE;
        uint32_t chunk = FLASH_PAGE_SIZE - page_offset;
        if (chunk > (length - offset)) {
            chunk = length - offset;
        }

        ret = flash_program_page_locked(current_address, buffer + offset, chunk);
        if (ret != FLASH_OK) {
            break;
        }
        offset += chunk;
    }

    flash_unlock();
    return ret;
}

flash_status_t gd25q16_erase_sector(uint32_t sector_address)
{
    if ((sector_address >= FLASH_SIZE) || ((sector_address % FLASH_SECTOR_SIZE) != 0U)) {
        return FLASH_INVALID_PARAM;
    }

    flash_status_t ret = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (ret != FLASH_OK) {
        return ret;
    }

    if (!g_flash_initialized) {
        flash_unlock();
        return FLASH_ERROR;
    }

    ret = flash_erase_sector_locked(sector_address);
    flash_unlock();
    return ret;
}

flash_status_t gd25q16_erase_chip(void)
{
    flash_status_t ret = flash_lock(FLASH_LOCK_TIMEOUT_MS);
    if (ret != FLASH_OK) {
        return ret;
    }

    if (!g_flash_initialized) {
        flash_unlock();
        return FLASH_ERROR;
    }

    ret = flash_erase_chip_locked();
    flash_unlock();
    return ret;
}



/* === Private helpers ======================================================= */

static flash_status_t flash_spi_wait_flag(uint32_t flag, FlagStatus state, uint32_t timeout_ms)
{
    uint32_t start = flash_get_tick_ms();
    while (spi_i2s_flag_get(FLASH_SPI, flag) != state) {
        if ((flash_get_tick_ms() - start) >= timeout_ms) {
            return FLASH_TIMEOUT;
        }
    }
    return FLASH_OK;
}

static flash_status_t flash_spi_exchange(uint8_t tx_byte, uint8_t* rx_byte)
{
    /* 等待发送缓冲区�?*/
    flash_status_t status = flash_spi_wait_flag(SPI_FLAG_TBE, SET, FLASH_SPI_XFER_TIMEOUT_MS);
    if (status != FLASH_OK) {
        return status;
    }

    /* 发送数�?*/
    spi_i2s_data_transmit(FLASH_SPI, tx_byte);

    /* 等待接收缓冲区非�?*/
    status = flash_spi_wait_flag(SPI_FLAG_RBNE, SET, FLASH_SPI_XFER_TIMEOUT_MS);
    if (status != FLASH_OK) {
        return status;
    }
    
    uint8_t rx_val = (uint8_t)spi_i2s_data_receive(FLASH_SPI);

    if (rx_byte != NULL) {
        *rx_byte = rx_val;
    }
    return FLASH_OK;
}

static flash_status_t flash_transfer_bytes_locked(const uint8_t* tx, uint8_t* rx, size_t length)
{
    for (size_t i = 0; i < length; ++i) {
        uint8_t out = (tx != NULL) ? tx[i] : FLASH_SPI_DUMMY_BYTE;
        flash_status_t status = flash_spi_exchange(out, (rx != NULL) ? &rx[i] : NULL);
        if (status != FLASH_OK) {
            return status;
        }
    }
    return FLASH_OK;
}

static inline flash_status_t flash_write_bytes_locked(const uint8_t* data, size_t length)
{
    return flash_transfer_bytes_locked(data, NULL, length);
}

static inline flash_status_t flash_read_bytes_locked(uint8_t* data, size_t length)
{
    return flash_transfer_bytes_locked(NULL, data, length);
}

static flash_status_t flash_read_status_locked(uint8_t* status)
{
    flash_cs_low();
    flash_status_t ret = flash_spi_exchange(FLASH_CMD_READ_STATUS, NULL);
    if (ret == FLASH_OK) {
        ret = flash_spi_exchange(FLASH_SPI_DUMMY_BYTE, status);
    }
    flash_cs_high();
    return ret;
}

static flash_status_t flash_wait_ready_locked(uint32_t timeout_ms)
{
    uint32_t start = flash_get_tick_ms();
    flash_status_t status;
    uint8_t sr = 0U;

    do {
        status = flash_read_status_locked(&sr);
        if (status != FLASH_OK) {
            return status;
        }
        if ((sr & FLASH_STATUS_BUSY) == 0U) {
            return FLASH_OK;
        }
        flash_delay_us(FLASH_POLL_DELAY_US);
    } while ((flash_get_tick_ms() - start) < timeout_ms);

    return FLASH_TIMEOUT;
}

static flash_status_t flash_write_enable_locked(void)
{
    uint8_t sr = 0U;
    flash_status_t status = flash_wait_ready_locked(FLASH_WRITE_TIMEOUT_MS);
    if (status != FLASH_OK) {
        return status;
    }

    /* 尝试3次设置WEL�?*/
    for (uint8_t retry = 0; retry < 3U; ++retry) {
        flash_cs_low();
        status = flash_spi_exchange(FLASH_CMD_WRITE_ENABLE, NULL);
        flash_cs_high();
        if (status != FLASH_OK) {
            return status;
        }

        flash_delay_us(1);  /* tSHWL 最�?00ns */

        status = flash_read_status_locked(&sr);
        if (status != FLASH_OK) {
            return status;
        }

        if ((sr & FLASH_STATUS_WEL) != 0U) {
            return FLASH_OK;
        }

        flash_delay_us(50);
    }

    return FLASH_ERROR;
}

static flash_status_t flash_program_page_locked(uint32_t address, const uint8_t* buffer, uint32_t length)
{
    flash_status_t status = flash_write_enable_locked();
    if (status != FLASH_OK) {
        return status;
    }

    flash_cs_low();
    uint8_t header[4] = {
        FLASH_CMD_PAGE_PROGRAM,
        (uint8_t)((address >> 16) & 0xFFU),
        (uint8_t)((address >> 8) & 0xFFU),
        (uint8_t)(address & 0xFFU)
    };
    status = flash_write_bytes_locked(header, sizeof(header));
    if (status == FLASH_OK) {
        status = flash_write_bytes_locked(buffer, length);
    }
    flash_cs_high();

    if (status == FLASH_OK) {
        status = flash_wait_ready_locked(FLASH_WRITE_TIMEOUT_MS);
        /* 额外验证WIP�?防止超时误判 */
        if (status == FLASH_OK) {
            uint8_t sr_verify;
            flash_read_status_locked(&sr_verify);
            if (sr_verify & FLASH_STATUS_BUSY) {
                return FLASH_BUSY;  /* 芯片仍忙,写入可能失败 */
            }
        }
    }
    return status;
}

static flash_status_t flash_erase_sector_locked(uint32_t sector_address)
{
    flash_status_t status = flash_write_enable_locked();
    if (status != FLASH_OK) {
        return status;
    }

    flash_cs_low();
    uint8_t header[4] = {
        FLASH_CMD_SECTOR_ERASE,
        (uint8_t)((sector_address >> 16) & 0xFFU),
        (uint8_t)((sector_address >> 8) & 0xFFU),
        (uint8_t)(sector_address & 0xFFU)
    };
    status = flash_write_bytes_locked(header, sizeof(header));
    flash_cs_high();

    if (status == FLASH_OK) {
        status = flash_wait_ready_locked(FLASH_ERASE_TIMEOUT_MS);
        /* 额外验证WIP�?防止超时误判 */
        if (status == FLASH_OK) {
            uint8_t sr_verify;
            flash_read_status_locked(&sr_verify);
            if (sr_verify & FLASH_STATUS_BUSY) {
                return FLASH_BUSY;  /* 芯片仍忙,擦除可能失败 */
            }
        }
    }
    return status;
}

static flash_status_t flash_erase_chip_locked(void)
{
    flash_status_t status = flash_write_enable_locked();
    if (status != FLASH_OK) {
        return status;
    }

    flash_cs_low();
    flash_spi_exchange(FLASH_CMD_CHIP_ERASE, NULL);
    flash_cs_high();

    status = flash_wait_ready_locked(FLASH_CHIP_ERASE_TIMEOUT_MS);
    return status;
}

