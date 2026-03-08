#define BOOTLOADER_MODE  /* 标识当前为 BootLoader 模式 */

#include "gd32f4xx.h"
#include "gd32f4xx_misc.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx_pmu.h"
#include "gd32f4xx_fmc.h"
#include "gd32f4xx_gpio.h"

#include "rs485_host.h"
#include "rs485_host_cfg.h"
#include "iap_shared.h"
#include "systick.h"
#include "flash_driver.h"
#include "modbus_rtu_iap.h"
#include "modbus_iap.h"
#include "led_control.h"
#include <stdbool.h>

/* ========== 函数声明 ========== */
static void bootloader_system_clock_config(void);
static void bootloader_jump_to_application(void);
static void bootloader_enter_modbus_mode(void);
static uint8_t bootloader_wait_modbus_ready(uint32_t timeout_ms);

/* ========== 全局变量 ========== */
static modbus_rtu_handle_t g_modbus_handle;
static modbus_iap_handle_t g_iap_handle;  /* IAP句柄 */

/* BootLoader超时配置 */
#define BOOTLOADER_TIMEOUT_MS    10000U  /* 10秒超时：如果10秒内没有收到有效Modbus命令，自动跳转到APP */

/* ========== 主函数 ========== */
int main(void)
{
    SystemInit();
    bootloader_system_clock_config();
    SystemCoreClockUpdate();
    systick_config();
    rs485_host_init();

    /* 检查是否需要进入IAP模式 */
    if (!iap_flag_is_set())
    {
        bootloader_jump_to_application();
    }
    else
    {
        /* 启动即清除IAP标志，避免因异常而反复进入BootLoader */
        iap_flag_clear();
        
        /* 初始化LED并点亮，指示进入BootLoader模式 */
        led_init();
        led_on();  /* LED常亮表示进入IAP模式 */
    }

    /* 初始化 Modbus RTU */
    modbus_rtu_init(&g_modbus_handle, MODBUS_RTU_SLAVE_ADDR_DEFAULT);
    
    /* 初始化 IAP句柄 */
    modbus_iap_init(&g_iap_handle);

    /*在响应Modbus之前先擦除Flash*/
    /* LED熄灭，表示正在擦除Flash */
    led_off();
    /* 擦除APP Flash区域（这个过程会阻塞，无法响应Modbus请求） */
    if (flash_app_erase(FLASH_APP_MAX_SIZE_BYTES) == FLASH_OP_SUCCESS)
    {
        /* 擦除成功，LED点亮 */
        led_on();
        /* 设置状态为准备接收 */
        g_iap_handle.status = IAP_STATUS_RECEIVING;
        g_iap_handle.received_size = 0;
        g_iap_handle.flash_address = APPLICATION_START_ADDRESS;
    }
    else
    {
        /* 擦除失败，LED保持熄灭 */
        led_off();
        /* 设置错误状态 */
        g_iap_handle.status = IAP_STATUS_FLASH_ERROR;
        g_iap_handle.error_code = 0x0001;
    }

    /* 进入Modbus IAP模式 */
    bootloader_enter_modbus_mode();

    /* 永不返回 */
    for (;;)
    {
        
    }
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
        while (1)
        {
            __NOP();
        }
    }

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

/* ========== 等待Modbus命令（超时返回0） ========== */
static uint8_t bootloader_wait_modbus_ready(uint32_t timeout_ms)
{
    uint32_t elapsed = 0U;
    const uint32_t check_interval_ms = 100U;  /* 每100ms检查一次 */
    
    while (elapsed < timeout_ms)
    {
        /* 尝试接收Modbus帧，使用较短的超时避免阻塞太久 */
        modbus_status_t rx_status = modbus_rtu_receive_frame(&g_modbus_handle, 100U);
        
        if (rx_status == MODBUS_OK)
        {
            /* 收到有效帧，处理并返回成功 */
            modbus_rtu_process_frame(&g_modbus_handle);
            return 1U;  /* 有数据，继续IAP模式 */
        }
        else if (rx_status == MODBUS_CRC_ERROR)
        {
            /* CRC错误也算作有通信尝试，重置并继续等待 */
            return 1U;
        }
        
        elapsed += check_interval_ms;
    }
    
    return 0U;  /* 超时，无有效通信 */
}

/* ========== 进入Modbus IAP模式 ========== */
static void bootloader_enter_modbus_mode(void)
{
    /* 等待第一个有效的Modbus命令，30秒超时 */
    if (!bootloader_wait_modbus_ready(30000U))
    {
        /* 30秒内没有收到有效Modbus命令，清除IAP标志并跳转到APP */
        iap_flag_clear();
        delay_ms(10U);  /* 短暂延时确保标志清除完成 */
        bootloader_jump_to_application();
    }
    
    /* Modbus IAP主循环 */
    for (;;)
    {
        /* 读取当前IAP状态 */
        uint16_t iap_status = modbus_iap_get_register(&g_iap_handle, MODBUS_IAP_REG_STATUS);
        
        /* 如果正在校验,执行分段CRC计算(不阻塞Modbus) */
        if (iap_status == IAP_STATUS_VERIFYING)
        {
            /* 分段计算CRC,保持Modbus响应能力 */
            modbus_iap_verify_step(&g_iap_handle);
        }
        
        /* 接收Modbus RTU帧 */
        modbus_status_t rx_status = modbus_rtu_receive_frame(&g_modbus_handle, 100U);  /* 缩短超时到100ms */
        
        if (rx_status == MODBUS_OK)
        {
            /* 处理Modbus IAP请求 */
            modbus_iap_process(&g_modbus_handle, &g_iap_handle);
        }
        else if (rx_status == MODBUS_CRC_ERROR)
        {
            /* CRC错误，忽略该帧 */
            continue;
        }
        
        /* 处理完本轮所有事情后再读一次状态，用最新状态做跳转决策 */
        iap_status = modbus_iap_get_register(&g_iap_handle, MODBUS_IAP_REG_STATUS);
        
        /* 检查是否需要跳转到应用程序(IAP升级完成后) */
        if (iap_status == IAP_STATUS_READY_JUMP)
        {
            /* 校验完成且收到跳转命令,延时后跳转到APP */
            led_off();  /* 跳转前熄灭LED */
            delay_ms(100U);  /* 留时间让Modbus响应发送完成 */
            bootloader_jump_to_application();
        }
        else if (iap_status == IAP_STATUS_VERIFY_OK)
        {
            /* 状态3: 校验成功 - 继续等待上位机发送JUMP命令 */
            /* 这个状态允许上位机确认校验结果后再决定是否跳转 */
        }
        else if (iap_status == IAP_STATUS_FLASH_ERROR || iap_status == IAP_STATUS_CRC_ERROR)
        {
            /* Flash错误或CRC错误：LED闪烁指示错误状态 */
            /* 上位机可以通过读取状态和错误码了解具体原因 */
            /* 上位机可以发送CANCEL命令清除错误状态，或者等待设备复位 */
            static uint32_t error_blink_tick = 0;
            uint32_t now = systick_get_tick();
            if ((now - error_blink_tick) > 500U)  /* 每500ms切换一次LED状态 */
            {
                error_blink_tick = now;
                static uint8_t led_state = 0;
                led_state = !led_state;
                if (led_state)
                {
                    led_on();
                }
                else
                {
                    led_off();
                }
            }
        }
    }
}
