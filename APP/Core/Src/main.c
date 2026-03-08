/*!
    \file    main.c
    \brief   IAP APP - SD卡固件升级程序

    功能：
    1. 从SD卡读取固件文件(firmware.bin)
    2. 擦除APP区域FLASH
    3. 写入新固件到FLASH
    4. 校验固件完整性
    5. 设置升级标志并重启进入BootLoader

    \version 2024-12-21, V3.0.0, firmware for GD32F4xx
*/

#include "gd32f4xx.h"
#include "gd32f4xx_misc.h"
#include "systick.h"
#include <stdio.h>
#include <stdbool.h>
#include "iap_update.h"
#include "iap_shared.h"

/* FatFs time function (required by ff.c) */
unsigned long get_fattime(void)
{
    /* Return fixed time: 2024-01-01 00:00:00 */
    return ((unsigned long)(2024 - 1980) << 25)  /* Year 2024 */
         | ((unsigned long)1 << 21)              /* Month 1 */
         | ((unsigned long)1 << 16)              /* Day 1 */
         | ((unsigned long)0 << 11)              /* Hour 0 */
         | ((unsigned long)0 << 5)               /* Minute 0 */
         | ((unsigned long)0 >> 1);              /* Second 0 */
}

int main(void)
{
    iap_status_t result;

    /* Configure NVIC */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* Initialize systick */
    systick_config();

    /* 初始化IAP模块 */
    iap_update_init();

    /* 等待系统稳定 */
    delay_1ms(1000);

    /* 检查是否存在固件文件 */
    if (iap_check_firmware_file(IAP_FIRMWARE_PATH)) {
        /* 执行升级 */
        result = iap_perform_upgrade(IAP_FIRMWARE_PATH);

        if (result == IAP_RESULT_OK) {
            /* 升级成功，设置标志并重启 */
            iap_trigger_bootloader();
        }
    }

    /* 主循环 */
    while (1) {
        /* 正常应用运行 */
    }
}
