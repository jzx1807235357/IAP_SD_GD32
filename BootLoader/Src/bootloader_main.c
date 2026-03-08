#define BOOTLOADER_MODE  /* 标识当前为 BootLoader 模式 */

#include "gd32f4xx.h"
#include "gd32f4xx_misc.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_pmu.h"
#include "gd32f4xx_fmc.h"
#include "gd32f4xx_gpio.h"

#include "iap_shared.h"
#include "systick.h"
#include "flash_driver.h"
#include "sd_iap.h"
#include "sdcard.h"
#include "ff.h"
#include "delay.h"
#include "debug_uart.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ========== 测试模式配置 ========== */
#define BOOTLOADER_TEST_MODE        0   /* 0: 正常IAP模式, 1: 分步测试模式 */
#define TEST_SD_CARD                1   /* 测试SD卡基本功能 */
#define TEST_FLASH_ERASE_WRITE      1   /* 测试Flash擦写 */
#define TEST_JUMP_TO_APP            1   /* 测试跳转到APP */

/* ========== 函数声明 ========== */
static void bootloader_system_clock_config(void);
static void bootloader_jump_to_application(void);
static void bootloader_enter_sd_iap_mode(void);

/* 测试函数 */
static void test_sd_card_basic(void);
static void test_flash_erase_write(void);
static void test_jump_to_app(void);

/* ========== 全局变量 ========== */
static sd_iap_handle_t g_sd_iap_handle;
static sd_iap_config_t g_sd_iap_config = {
    .firmware_path = "0:firmware.bin",  /* SD卡根目录下的firmware.bin */
    .expected_size = 0,                   /* 不检查大小 */
    .expected_crc = 0                     /* 不检查CRC */
};

/* ========== 主函数 ========== */
int main(void)
{
    SystemInit();
    bootloader_system_clock_config();
    SystemCoreClockUpdate();
    systick_config();
    
    /* Initialize debug UART for printf */
    debug_uart_init();
    printf("\r\n========================================\r\n");
    printf("BootLoader Start (USART0 Debug)\r\n");
    printf("========================================\r\n");
    
#if BOOTLOADER_TEST_MODE
    /* ========== 测试模式 ========== */
    
    /* 测试1: SD卡基本功能 */
#if TEST_SD_CARD
    test_sd_card_basic();
#endif
    
    /* 测试2: Flash擦写 */
#if TEST_FLASH_ERASE_WRITE
    test_flash_erase_write();
#endif
    
    /* 测试3: 跳转到APP */
#if TEST_JUMP_TO_APP
    test_jump_to_app();
#endif
    
    /* 所有测试完成 */
    while (1)
    {
        __NOP();
    }
#else
    /* ========== 正常IAP模式 ========== */
    
    /* 检查是否需要进入IAP模式 */
    if (!iap_flag_is_set())
    {
        bootloader_jump_to_application();
    }
    
    /* 启动即清除IAP标志，避免因异常而反复进入BootLoader */
    iap_flag_clear();
    
    /* 进入SD卡IAP升级模式 */
    bootloader_enter_sd_iap_mode();

    /* 永不返回 */
    for (;;)
    {
        
    }
#endif
}

/* ========== 系统时钟配置 ========== */
static void bootloader_system_clock_config(void)
{
    /* 使能PMU外设以配置高驱动模式 */
    rcu_periph_clock_enable(RCU_PMU);
    pmu_highdriver_mode_enable();
    while (pmu_flag_get(PMU_FLAG_HDRF) == RESET)
    {
    }
    pmu_highdriver_switch_select(PMU_HIGHDR_SWITCH_EN);
    while (pmu_flag_get(PMU_FLAG_HDSRF) == RESET)
    {
    }

    rcu_deinit();
    rcu_osci_on(RCU_HXTAL);
    (void)rcu_osci_stab_wait(RCU_HXTAL);

    if (rcu_pll_config(RCU_PLLSRC_HXTAL, 13U, 208U, 2U, 8U) != SUCCESS)
    {
        while (1)
        {
        }
    }

    rcu_osci_on(RCU_PLL_CK);
    (void)rcu_osci_stab_wait(RCU_PLL_CK);

    rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
    rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV4);
    rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV2);

    /* 配置Flash等待状态（200MHz） */
    fmc_wscnt_set(WS_WSCNT_5);

    /* 切换系统时钟到PLL */
    rcu_system_clock_source_config(RCU_CKSYSSRC_PLLP);
    while (rcu_system_clock_source_get() != RCU_SCSS_PLLP)
    {
    }
}

/* ========== 跳转到应用程序 ========== */
static void bootloader_jump_to_application(void)
{
    typedef void (*entry_fn_t)(void);
    uint32_t app_stack = REG32(APPLICATION_START_ADDRESS);
    uint32_t app_reset = REG32(APPLICATION_START_ADDRESS + 4U);
    entry_fn_t app_entry = (entry_fn_t)app_reset;

    if ((app_stack == 0xFFFFFFFFU) || (app_reset == 0xFFFFFFFFU))
    {
        printf("APP invalid: stack=0x%08lX, reset=0x%08lX\r\n", 
               (unsigned long)app_stack, (unsigned long)app_reset);
        while (1)
        {
            __NOP();
        }
    }

    printf("\r\n========================================\r\n");
    printf("Jump to APP @ 0x%08lX\r\n", (unsigned long)APPLICATION_START_ADDRESS);
    printf("Stack: 0x%08lX, Reset: 0x%08lX\r\n", 
           (unsigned long)app_stack, (unsigned long)app_reset);
    printf("========================================\r\n\r\n");
    
    /* 关闭所有中断 */
    __disable_irq();
    
    /* 复位所有外设到默认状态（防止残留配置影响APP） */
    RCU_APB1RST = 0xFFFFFFFFU;
    RCU_APB1RST = 0x00000000U;
    RCU_APB2RST = 0xFFFFFFFFU;
    RCU_APB2RST = 0x00000000U;
    RCU_AHB1RST = 0xFFFFFFFFU;
    RCU_AHB1RST = 0x00000000U;
    
    /* 关闭SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    
    /* 设置中断向量表 */
    SCB->VTOR = APPLICATION_START_ADDRESS;
    
    /* 设置主栈指针并跳转 */
    __set_MSP(app_stack);
    app_entry();
}

/* ========== 进入SD卡IAP升级模式 ========== */
static void bootloader_enter_sd_iap_mode(void)
{
    int result;
    
    /* 检查SD卡是否存在 */
    if (!sd_iap_card_detected())
    {
        /* 无SD卡，死循环 */
        while (1)
        {
        }
    }
    
    /* 初始化SD卡 */
    if (!sd_iap_init())
    {
        /* SD卡初始化失败，死循环 */
        while (1)
        {
        }
    }
    
    /* 挂载文件系统 */
    result = sd_iap_mount();
    if (result != 0)
    {
        /* 挂载失败，死循环 */
        while (1)
        {
        }
    }
    
    /* 检查固件文件是否存在 */
    if (!sd_iap_file_exists(g_sd_iap_config.firmware_path))
    {
        /* 固件文件不存在，死循环 */
        while (1)
        {
        }
    }
    
    /* 执行SD-IAP升级 */
    result = sd_iap_upgrade(&g_sd_iap_handle, &g_sd_iap_config);
    
    if (result == 0 && g_sd_iap_handle.status == SD_IAP_SUCCESS)
    {
        /* 升级成功，跳转到应用程序 */
        bootloader_jump_to_application();
    }
    else
    {
        /* 升级失败，死循环 */
        while (1)
        {
        }
    }
}

/* ========== 测试函数实现 ========== */

/* 测试SD卡基本功能 - 完整版：挂载+打开+读写+验证+关闭 */
static void test_sd_card_basic(void)
{
    sd_card_info_struct card_info;
    FATFS fatfs;
    FIL fil;
    FRESULT fres;
    uint32_t card_capacity;
    sd_error_enum sd_status;
    UINT bytes_written, bytes_read;
    uint32_t test_write_data[16];
    uint32_t test_read_data[16];
    int i;
    
    /* 步骤1: 初始化SD卡硬件（GPIO、SDIO、上电） */
    sd_status = sd_init();
    if (sd_status != SD_OK)
    {
        printf("SD init failed: %d\r\n", (int)sd_status);
        while (1);
    }
    
    /* 步骤2: 检测SD卡是否存在（GPIO已初始化） */
    if (!sd_card_detect_pin_read())
    {
        printf("SD card not detected\r\n");
        while (1);
    }
    
    /* 步骤3: 获取SD卡信息并选择卡片 */
    sd_status = sd_card_information_get(&card_info);
    if (sd_status != SD_OK)
    {
        printf("SD card_information_get failed: %d\r\n", (int)sd_status);
        while (1);
    }
    
    /* 选择SD卡（CMD7）- 参考工程必需步骤 */
    sd_status = sd_card_select_deselect(card_info.card_rca);
    if (sd_status != SD_OK)
    {
        printf("SD card_select_deselect failed: %d\r\n", (int)sd_status);
        while (1);
    }
    
    /* 步骤4: 挂载文件系统 */
    fres = f_mount(&fatfs, "0:", 1);
    if (fres != FR_OK)
    {
        printf("f_mount failed: %d\r\n", (int)fres);
        while (1);
    }
    
    /* 步骤5: 获取SD卡容量 */
    card_capacity = sd_card_capacity_get();
    printf("SD capacity: %lu KB\r\n", (unsigned long)card_capacity);
    
    /* ========== 新增：文件读写验证 ========== */
    
    /* 准备测试数据 (64 bytes) */
    for (i = 0; i < 16; i++)
    {
        test_write_data[i] = 0x12345678U + (uint32_t)i;
    }
    
    /* 步骤6: 创建并写入测试文件 */
    fres = f_open(&fil, "0:test_rw.bin", FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK)
    {
        printf("f_open(write) failed: %d\r\n", (int)fres);
        while (1);
    }
    
    fres = f_write(&fil, test_write_data, sizeof(test_write_data), &bytes_written);
    if (fres != FR_OK || bytes_written != sizeof(test_write_data))
    {
        printf("f_write failed: fres=%d bw=%u/%u\r\n", (int)fres, (unsigned)bytes_written, (unsigned)sizeof(test_write_data));
        (void)f_close(&fil);
        while (1);
    }
    
    fres = f_sync(&fil);
    if (fres != FR_OK)
    {
        printf("f_sync failed: %d\r\n", (int)fres);
        (void)f_close(&fil);
        while (1);
    }
    
    fres = f_close(&fil);
    if (fres != FR_OK)
    {
        printf("f_close(after write) failed: %d\r\n", (int)fres);
        while (1);
    }
    
    /* 步骤7: 读取测试文件并验证 */
    fres = f_open(&fil, "0:test_rw.bin", FA_READ);
    if (fres != FR_OK)
    {
        printf("f_open(read) failed: %d\r\n", (int)fres);
        while (1);
    }
    
    fres = f_read(&fil, test_read_data, sizeof(test_read_data), &bytes_read);
    if (fres != FR_OK || bytes_read != sizeof(test_read_data))
    {
        printf("f_read failed: fres=%d br=%u/%u\r\n", (int)fres, (unsigned)bytes_read, (unsigned)sizeof(test_read_data));
        (void)f_close(&fil);
        while (1);
    }
    
    fres = f_close(&fil);
    if (fres != FR_OK)
    {
        printf("f_close(after read) failed: %d\r\n", (int)fres);
        while (1);
    }
    
    /* 步骤8: 验证数据一致性 */
    if (memcmp(test_write_data, test_read_data, sizeof(test_write_data)) != 0)
    {
        printf("memcmp verify failed\r\n");
        while (1);
    }
    
    /* 步骤10: 尝试打开firmware.bin（生产环境实际使用） */
    fres = f_open(&fil, "0:firmware.bin", FA_READ);
    if (fres == FR_OK)
    {
        printf("firmware.bin exists\r\n");
        (void)f_close(&fil);
    }
    else
    {
        printf("firmware.bin not found (f_open=%d)\r\n", (int)fres);
    }
    
    /* SD卡完整测试成功 */
    printf("SD R/W self-test OK\r\n");
}

/* 测试Flash擦写功能 */
static void test_flash_erase_write(void)
{
    flash_status_t flash_status;
    uint32_t test_address;
    uint32_t test_data[4];
    uint32_t read_data[4];
    
    /* 使用APP区域起始地址进行测试（扇区4起始地址） */
    test_address = APPLICATION_START_ADDRESS;
    
    /* 准备测试数据 */
    test_data[0] = 0x12345678;
    test_data[1] = 0xABCDEF00;
    test_data[2] = 0x55AA55AA;
    test_data[3] = 0xDEADBEEF;
    
    /* 步骤1: 擦除扇区4（APP起始扇区） */
    flash_status = flash_sector_erase(test_address);
    if (flash_status != FLASH_OP_SUCCESS)
    {
        printf("flash_sector_erase failed: %d\r\n", (int)flash_status);
        while (1);
    }
    
    /* 步骤2: 验证擦除结果（应为0xFFFFFFFF） */
    flash_status = flash_verify_blank(test_address, 16);
    if (flash_status != FLASH_OP_SUCCESS)
    {
        printf("flash_verify_blank failed: %d\r\n", (int)flash_status);
        while (1);
    }
    
    /* 步骤3: 写入测试数据 */
    flash_status = flash_program_words(test_address, (uint8_t *)test_data, sizeof(test_data));
    if (flash_status != FLASH_OP_SUCCESS)
    {
        printf("flash_program_words failed: %d\r\n", (int)flash_status);
        while (1);
    }
    
    /* 步骤4: 读回并验证数据 */
    memcpy(read_data, (void *)test_address, sizeof(read_data));
    if (memcmp(test_data, read_data, sizeof(test_data)) != 0)
    {
        printf("flash verify mismatch\r\n");
        while (1);
    }
    
    /* 步骤5: 再次擦除，恢复空白状态 */
    flash_status = flash_sector_erase(test_address);
    if (flash_status != FLASH_OP_SUCCESS)
    {
        printf("flash_sector_erase(restore) failed: %d\r\n", (int)flash_status);
        while (1);
    }
    
    printf("Flash R/W self-test OK\r\n");
}

/* 测试跳转到APP */
static void test_jump_to_app(void)
{
    uint32_t app_stack;
    uint32_t app_reset;
    
    /* 读取APP起始地址的栈指针和复位向量 */
    app_stack = REG32(APPLICATION_START_ADDRESS);
    app_reset = REG32(APPLICATION_START_ADDRESS + 4U);
    
    /* 检查APP是否有效 */
    if (app_stack == 0xFFFFFFFFU || app_reset == 0xFFFFFFFFU)
    {
        /* APP区域为空 */
        return;  /* 不跳转，继续后续测试 */
    }
    
    /* 检查栈指针是否在有效RAM范围 */
    if (app_stack < 0x20000000U || app_stack > 0x20030000U)
    {
        /* 栈指针无效 */
        return;
    }
    
    /* 检查复位向量是否在APP区域 */
    if (app_reset < APPLICATION_START_ADDRESS || 
        app_reset > (APPLICATION_START_ADDRESS + APPLICATION_SIZE))
    {
        /* 复位向量无效 */
        return;
    }
    
    /* APP有效 */
    
    /* 注意：此处不实际跳转，仅验证APP有效性 */
    /* 实际跳转在正常IAP模式中执行 */
}
