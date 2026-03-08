/*
 * FreeRTOS Kernel V11.1.0
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * FreeRTOS配置文件 - 针对GD32F425 (ARM Cortex-M4F) 配置
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * 内核配置
 *----------------------------------------------------------*/

/* 配置为1使用抢占式调度器，配置为0使用协程 */
#define configUSE_PREEMPTION                    1

/* 配置为1使用空闲任务钩子函数，配置为0则不使用 */
#define configUSE_IDLE_HOOK                     0

/* 配置为1使用时间片钩子函数，配置为0则不使用 */
#define configUSE_TICK_HOOK                     1

/* 系统时钟频率，GD32F425最高200MHz，根据实际配置修改 */
#define configCPU_CLOCK_HZ                      ((unsigned long)200000000)

/* SysTick节拍频率，1000表示1ms一个节拍 */
#define configTICK_RATE_HZ                      ((TickType_t)1000)

/* 可使用的最大优先级，最大不能超过32 */
#define configMAX_PRIORITIES                    (5)

/* 空闲任务使用的堆栈大小，单位为字（4字节） */
#define configMINIMAL_STACK_SIZE                ((unsigned short)128)

/* 系统所有任务可使用的堆大小，单位为字节 */
#define configTOTAL_HEAP_SIZE                   ((size_t)(64 * 1024))

/* 任务名字字符串长度 */
#define configMAX_TASK_NAME_LEN                 (16)

/* FreeRTOS V11.x: 使用新的Tick类型宽度定义（替代旧的configUSE_16_BIT_TICKS） */
#define TICK_TYPE_WIDTH_16_BITS                 0
#define TICK_TYPE_WIDTH_32_BITS                 1
#define TICK_TYPE_WIDTH_64_BITS                 2
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS

/* 空闲任务放弃CPU使用权给其他同优先级的用户任务 */
#define configIDLE_SHOULD_YIELD                 1

/* 启用队列 */
#define configUSE_QUEUE_SETS                    0

/* 使能任务通知功能，默认开启 */
#define configUSE_TASK_NOTIFICATIONS            1

/* 使用互斥信号量 */
#define configUSE_MUTEXES                       1

/* 使用递归互斥信号量 */
#define configUSE_RECURSIVE_MUTEXES             1

/* 为1时使用计数信号量 */
#define configUSE_COUNTING_SEMAPHORES           1

/* 使能任务的FPU支持（GD32F425带FPU） */
#define configUSE_TASK_FPU_SUPPORT              1

/*-----------------------------------------------------------
 * 内存申请相关定义
 *----------------------------------------------------------*/
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0

/*-----------------------------------------------------------
 * 钩子函数相关定义
 *----------------------------------------------------------*/
#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2

/*-----------------------------------------------------------
 * 运行时间和任务状态收集相关定义
 *----------------------------------------------------------*/
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/*-----------------------------------------------------------
 * 协程相关定义
 *----------------------------------------------------------*/
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         (2)

/*-----------------------------------------------------------
 * 软件定时器相关定义
 *----------------------------------------------------------*/
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (2)
#define configTIMER_QUEUE_LENGTH                5
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE * 2)

/*-----------------------------------------------------------
 * 可选函数定义
 *----------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xQueueGetMutexHolder            1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_eTaskGetState                   1

/*-----------------------------------------------------------
 * 中断优先级配置
 * Cortex-M4使用8位优先级，但GD32F425只使用了高4位
 * 因此实际可用的优先级为0-15（数值越小，优先级越高）
 *----------------------------------------------------------*/

/* Cortex-M具体型号的中断优先级配置宏 */
#ifdef __NVIC_PRIO_BITS
    #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
    #define configPRIO_BITS         4        /* GD32F425使用4位优先级 */
#endif

/* 可以在中断服务函数中安全调用FreeRTOS API的最低中断优先级 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15

/* 系统可管理的最高中断优先级 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

/* 
 * 设置为configLIBRARY_LOWEST_INTERRUPT_PRIORITY左移configPRIO_BITS位
 * 因为Cortex-M4的优先级寄存器是8位，但只使用了高4位
 */
#define configKERNEL_INTERRUPT_PRIORITY         (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* 
 * 设置为configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY左移configPRIO_BITS位
 * 优先级数值在0-5之间的中断可以安全调用FreeRTOS API
 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*-----------------------------------------------------------
 * 断言配置
 *----------------------------------------------------------*/
#define configASSERT(x) if((x) == 0) {taskDISABLE_INTERRUPTS(); for(;;);}

/*-----------------------------------------------------------
 * FreeRTOS与Cortex-M4中断服务函数映射
 *----------------------------------------------------------*/
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */

