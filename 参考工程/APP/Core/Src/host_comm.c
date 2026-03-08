/*!
    \file    host_comm.c

    \version 2024-12-21, V1.0.0, Host Communication for GD32F4xx
*/

#include "host_comm.h"
#include "rs485_hal.h"
#include "systick.h"
#include "gimbal_control.h"
#include "flash_config.h"
#include "sun_position.h"
#include "sd_logger.h"
#include "iap_shared.h"
#include "gd32f4xx_pmu.h"  /* 支持PMU备份寄存器写保护 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <math.h>

/* ==================== USART5 RX DMA 配置 ==================== */

/* USART5 RX -> DMA1 CH2 */
#define HOST_COMM_DMA_RCU           RCU_DMA1
#define HOST_COMM_DMA_PERIPH        DMA1
#define HOST_COMM_DMA_RX_CHANNEL    DMA_CH2

/* USART5_RX 对应的 SUBPERI 编号 */
#define HOST_COMM_DMA_RX_SUBPERI    DMA_SUBPERI5

/* RX 环形缓冲区大小 */
#define HOST_COMM_RX_DMA_BUF_SIZE   256

static uint8_t s_host_rx_dma_buf[HOST_COMM_RX_DMA_BUF_SIZE] __attribute__((aligned(4)));
static volatile uint16_t s_host_rx_dma_pos = 0;

extern QueueHandle_t xHostCmdQueue;

volatile host_comm_state_t g_host_comm_state = HOST_COMM_IDLE;
volatile bool g_host_data_updated = false;

/* 函数声明 */
static void host_comm_usart5_dma_rx_init(void);
static void host_comm_on_rx_byte_from_isr(uint8_t byte, BaseType_t* pxHigherPriorityTaskWoken);
static void host_comm_update_gimbal_gain(void);
static void host_comm_save_params_to_flash(void);
host_data_t g_host_data = {0};

static uint32_t g_last_update_time = 0;

static TaskHandle_t s_host_task_handle = NULL;

static volatile bool s_calib_save_pending = false;
static float s_pending_az_offset = 0.0f;
static float s_pending_el_offset = 0.0f;

static void host_comm_update_data_from_registers(void);
static void host_comm_update_gimbal_gain(void);
static bool host_comm_is_offset_flash_window(void);

/*!
    \brief    初始化上位机通信模块
    \detail   硬件初始化由rs485_hal模块统一管理，本函数只初始化协议栈和逻辑
              必须先调用rs485_init(RS485_BUS_HOST, ...)初始化RS485总线
*/
bool host_comm_init(void)
{
    /* 验证RS485总线已初始化 */
    if (!rs485_is_initialized(RS485_BUS_HOST)) {
        return false;
    }

    if (!modbus_rtu_slave_init(MODBUS_SLAVE_ADDR_HOST)) {
        return false;
    }

    memset(&g_host_data, 0, sizeof(host_data_t));
    g_host_data.valid = true;
    
    /* 初始化经纬度寄存器默认值（0，0）- GPS定位后会自动更新 */
    uint16_t reg_h, reg_l;
    modbus_rtu_float_to_registers(0.0f, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_LATITUDE_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_LATITUDE_L, reg_l);
    modbus_rtu_slave_set_register(REG_HOST_LONGITUDE_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_LONGITUDE_L, reg_l);
    
    /* 初始化时区寄存器默认值（UTC+8） */
    modbus_rtu_slave_set_register(REG_HOST_TIMEZONE, 8);
    
    /* 初始化增益系数寄存器 */
    extern gimbal_gain_config_t g_gimbal_gain;
    modbus_rtu_float_to_registers(g_gimbal_gain.azimuth_gain, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_GAIN_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_GAIN_L, reg_l);
    modbus_rtu_float_to_registers(g_gimbal_gain.elevation_gain, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_ELEVATION_GAIN_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_ELEVATION_GAIN_L, reg_l);
    
    /* 初始化纠偏值寄存器为0（稍后会从Flash加载实际值） */
    modbus_rtu_float_to_registers(0.0f, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_OFFSET_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_OFFSET_L, reg_l);
    modbus_rtu_slave_set_register(REG_HOST_ELEVATION_OFFSET_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_ELEVATION_OFFSET_L, reg_l);
    
    /* 初始化夜间归位寄存器默认值（方位角90°，仰角90°）- 稍后会从Flash加载实际值 */
    modbus_rtu_float_to_registers(90.0f, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_NIGHT_AZIMUTH_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_NIGHT_AZIMUTH_L, reg_l);
    modbus_rtu_slave_set_register(REG_HOST_NIGHT_ELEVATION_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_NIGHT_ELEVATION_L, reg_l);

    /* 初始化太阳高度角工作范围寄存器默认值（-5°到90°）- 稍后会从Flash加载实际值 */
    modbus_rtu_float_to_registers(-5.0f, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MIN_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MIN_L, reg_l);
    modbus_rtu_float_to_registers(90.0f, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MAX_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MAX_L, reg_l);

    /* 初始化云台仰角工作范围寄存器默认值（5°到90°）- 稍后会从Flash加载实际值 */
    modbus_rtu_float_to_registers(5.0f, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MIN_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MIN_L, reg_l);
    modbus_rtu_float_to_registers(90.0f, &reg_h, &reg_l);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MAX_H, reg_h);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MAX_L, reg_l);

    /* ★启用USART5的DMA+IDLE接收 */
    host_comm_usart5_dma_rx_init();

    g_host_comm_state = HOST_COMM_IDLE;
    g_host_data_updated = false;
    g_last_update_time = 0;

    return true;
}

/*!
*/
bool host_comm_get_data(host_data_t* data)
{
    if (data == NULL) {
        return false;
    }
    
    *data = g_host_data;
    return g_host_data.valid;
}

/*!
*/
bool host_comm_data_updated(void)
{
    return g_host_data_updated;
}

/*!
*/
void host_comm_clear_update_flag(void)
{
    g_host_data_updated = false;
}

/*!
*/
void host_comm_set_light_data(float up, float left, float down, float right)
{
    modbus_rtu_slave_set_light_data(up, left, down, right);
    
    g_host_data.valid = true;
    g_host_data_updated = true;
}

/*!
*/
void host_comm_set_gimbal_data(float azimuth, float elevation)
{
    uint16_t az_h, az_l, el_h, el_l;
    
    modbus_rtu_float_to_registers(azimuth, &az_h, &az_l);
    modbus_rtu_float_to_registers(elevation, &el_h, &el_l);
    
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_AZIMUTH_H, az_h);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_AZIMUTH_L, az_l);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEVATION_H, el_h);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEVATION_L, el_l);
    
    g_host_data.gimbal_azimuth = azimuth;
    g_host_data.gimbal_elevation = elevation;
    g_host_data.valid = true;
    g_host_data_updated = true;
}

/*!
*/
void host_comm_set_device_status(uint16_t status)
{
    g_host_data.device_status = status;
    modbus_rtu_slave_set_register(REG_HOST_DEVICE_STATUS, status);
}

/*!
*/
void host_comm_set_error_code(uint16_t error_code)
{
    g_host_data.error_code = error_code;
    modbus_rtu_slave_set_register(REG_HOST_ERROR_CODE, error_code);
}

/*!
*/
void host_comm_process(void)
{
    modbus_rtu_slave_process_request();
    
    host_comm_update_data_from_registers();
    
    host_comm_update_gimbal_gain();
    
    /* Check for IAP start request */
    if (modbus_rtu_check_and_clear_iap_request()) {
        /* Update IAP status register */
        modbus_rtu_slave_set_register(REG_HOST_IAP_STATUS, IAP_STATUS_RESETTING);
        
        /* Set IAP flag in RTC backup register */
        iap_flag_set();
        
        /* Short delay to ensure response is sent */
        vTaskDelay(pdMS_TO_TICKS(100));
        
        /* System reset to enter bootloader */
        NVIC_SystemReset();
    }
}

/*!
*/
void host_comm_set_register(uint16_t addr, uint16_t value)
{
    modbus_rtu_slave_set_register(addr, value);
}

/*!
*/
uint16_t host_comm_get_register(uint16_t addr)
{
    return modbus_rtu_slave_get_register(addr);
}

/*!
*/
host_comm_state_t host_comm_get_state(void)
{
    return g_host_comm_state;
}

/*!
    \brief    初始化USART5 RX DMA功能
    \note     使用 DMA1 CH2 + IDLE中断
*/
static void host_comm_usart5_dma_rx_init(void)
{
    dma_single_data_parameter_struct dma_init;

    /* 1. 使能DMA1时钟 */
    rcu_periph_clock_enable(HOST_COMM_DMA_RCU);

    /* 2. 配置DMA1 CH2为USART5 RX -> 内存 */
    dma_single_data_para_struct_init(&dma_init);
    dma_deinit(HOST_COMM_DMA_PERIPH, HOST_COMM_DMA_RX_CHANNEL);

    dma_init.direction           = DMA_PERIPH_TO_MEMORY;
    dma_init.memory0_addr        = (uint32_t)s_host_rx_dma_buf;
    dma_init.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init.number              = HOST_COMM_RX_DMA_BUF_SIZE;
    dma_init.periph_addr         = (uint32_t)&USART_DATA(HOST_COMM_USART);
    dma_init.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init.priority            = DMA_PRIORITY_HIGH;

    dma_single_data_mode_init(HOST_COMM_DMA_PERIPH,
                              HOST_COMM_DMA_RX_CHANNEL,
                              &dma_init);

    /* 环形模式，持续接收 */
    dma_circulation_enable(HOST_COMM_DMA_PERIPH,
                           HOST_COMM_DMA_RX_CHANNEL);

    dma_channel_subperipheral_select(HOST_COMM_DMA_PERIPH,
                                     HOST_COMM_DMA_RX_CHANNEL,
                                     HOST_COMM_DMA_RX_SUBPERI);

    dma_flag_clear(HOST_COMM_DMA_PERIPH,
                   HOST_COMM_DMA_RX_CHANNEL,
                   DMA_FLAG_FTF | DMA_FLAG_HTF | DMA_FLAG_TAE | DMA_FLAG_FEE);

    dma_channel_enable(HOST_COMM_DMA_PERIPH,
                       HOST_COMM_DMA_RX_CHANNEL);

    /* 3. 使能USART5的Rx DMA请求 */
    usart_dma_receive_config(HOST_COMM_USART, USART_RECEIVE_DMA_ENABLE);

    /* 4. 打开IDLE中断，关闭RBNE中断 */
    usart_interrupt_disable(HOST_COMM_USART, USART_INT_RBNE);
    usart_interrupt_enable(HOST_COMM_USART, USART_INT_IDLE);
}

/*!
    \brief    USART5 IDLE中断下处理DMA新数据
    \param[in]  new_pos: DMA当前写指针位置
    \param[in/out]  pxHigherPriorityTaskWoken: FreeRTOS任务唤醒标志
*/
void host_comm_uart_idle_dma_rx_handler(uint16_t new_pos,
                                        BaseType_t* pxHigherPriorityTaskWoken)
{
    uint16_t old_pos = s_host_rx_dma_pos;

    if (new_pos == old_pos) {
        return;   /* 没有新数据 */
    }

    if (new_pos > old_pos) {
        /* 正常未回绕 */
        for (uint16_t i = old_pos; i < new_pos; i++) {
            host_comm_on_rx_byte_from_isr(s_host_rx_dma_buf[i],
                                          pxHigherPriorityTaskWoken);
        }
    } else {
        /* 环形缓冲区发生回绕 */
        for (uint16_t i = old_pos; i < HOST_COMM_RX_DMA_BUF_SIZE; i++) {
            host_comm_on_rx_byte_from_isr(s_host_rx_dma_buf[i],
                                          pxHigherPriorityTaskWoken);
        }
        for (uint16_t i = 0; i < new_pos; i++) {
            host_comm_on_rx_byte_from_isr(s_host_rx_dma_buf[i],
                                          pxHigherPriorityTaskWoken);
        }
    }

    s_host_rx_dma_pos = new_pos;
}

/*!
    \brief    处理单个接收字节（仅在中断上下文调用）
    \param[in]  byte: 接收到的字节
    \param[in/out]  pxHigherPriorityTaskWoken: FreeRTOS任务唤醒标志
*/
static void host_comm_on_rx_byte_from_isr(uint8_t byte,
                                          BaseType_t* pxHigherPriorityTaskWoken)
{
    modbus_rtu_process_received_byte(byte);

    if (g_modbus_frame_received &&
        s_host_task_handle != NULL &&
        pxHigherPriorityTaskWoken != NULL &&
        xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskNotifyGiveFromISR(s_host_task_handle, pxHigherPriorityTaskWoken);
    }
}

/* host_comm_uart_irq_handler已删除 - 现在只使用DMA+IDLE中断接收 */

/*!
*/
static void host_comm_update_data_from_registers(void)
{
    uint32_t current_time = systick_get_tick();
    
    if ((current_time - g_last_update_time) < 50) {
        return;
    }
    
    g_last_update_time = current_time;
    
    g_host_data.device_status = modbus_rtu_slave_get_register(REG_HOST_DEVICE_STATUS);
    g_host_data.run_mode = modbus_rtu_slave_get_register(REG_HOST_RUN_MODE);
    g_host_data.gps_status = modbus_rtu_slave_get_register(REG_HOST_GPS_STATUS);
    g_host_data.comm_status = modbus_rtu_slave_get_register(REG_HOST_COMM_STATUS);
    g_host_data.error_code = modbus_rtu_slave_get_register(REG_HOST_ERROR_CODE);
    
    uint16_t sun_az_h = modbus_rtu_slave_get_register(REG_HOST_SUN_AZIMUTH_H);
    uint16_t sun_az_l = modbus_rtu_slave_get_register(REG_HOST_SUN_AZIMUTH_L);
    g_host_data.sun_azimuth = modbus_rtu_registers_to_float(sun_az_h, sun_az_l);
    
    uint16_t sun_el_h = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEVATION_H);
    uint16_t sun_el_l = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEVATION_L);
    g_host_data.sun_elevation = modbus_rtu_registers_to_float(sun_el_h, sun_el_l);
    
    uint16_t gimbal_az_h = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_AZIMUTH_H);
    uint16_t gimbal_az_l = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_AZIMUTH_L);
    g_host_data.gimbal_azimuth = modbus_rtu_registers_to_float(gimbal_az_h, gimbal_az_l);
    
    uint16_t gimbal_el_h = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEVATION_H);
    uint16_t gimbal_el_l = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEVATION_L);
    g_host_data.gimbal_elevation = modbus_rtu_registers_to_float(gimbal_el_h, gimbal_el_l);
    
    /* 读取纠偏值寄存器（光学追踪校准后的偏差补偿） */
    /* 上位机需要显示这些值以监控系统校准状态 */
    uint16_t az_offset_h = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_OFFSET_H);
    uint16_t az_offset_l = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_OFFSET_L);
    uint16_t el_offset_h = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_OFFSET_H);
    uint16_t el_offset_l = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_OFFSET_L);
    
    /* 纠偏值无需存储到host_data结构体，上位机直接读取寄存器即可 */
    /* 这里只是确保寄存器数据定期被访问，保持数据新鲜度 */
    (void)az_offset_h;
    (void)az_offset_l;
    (void)el_offset_h;
    (void)el_offset_l;
    
    g_host_data.valid = true;
    g_host_data_updated = true;
}

/*!
*/
void host_comm_execute_command(uint16_t cmd_code)
{
    host_cmd_msg_t cmd_msg;
    BaseType_t result = pdFAIL;

    cmd_msg.cmd_code = cmd_code;
    cmd_msg.param1 = 0.0f;
    cmd_msg.param2 = 0.0f;
    
    /* 处理Flash参数保存命令（不需要发送到队列） */
    if (cmd_code == CMD_SAVE_PARAMS) {
        host_comm_save_params_to_flash();
        modbus_rtu_slave_set_register(REG_HOST_ERROR_CODE, 0);
        return;
    }

    if (cmd_code == CMD_SET_POSITION) {
        uint16_t az_h = modbus_rtu_slave_get_register(REG_HOST_TARGET_AZIMUTH_H);
        uint16_t az_l = modbus_rtu_slave_get_register(REG_HOST_TARGET_AZIMUTH_L);
        uint16_t el_h = modbus_rtu_slave_get_register(REG_HOST_TARGET_ELEVATION_H);
        uint16_t el_l = modbus_rtu_slave_get_register(REG_HOST_TARGET_ELEVATION_L);

        cmd_msg.param1 = modbus_rtu_registers_to_float(az_h, az_l);
        cmd_msg.param2 = modbus_rtu_registers_to_float(el_h, el_l);
    }

    if (xHostCmdQueue != NULL) {
        /* 使用50ms超时，避免命令丢失 */
        result = xQueueSend(xHostCmdQueue, &cmd_msg, pdMS_TO_TICKS(50));
        
        /* 如果仍然失败，再重试一次 */
        if (result != pdPASS) {
            vTaskDelay(pdMS_TO_TICKS(10));
            result = xQueueSend(xHostCmdQueue, &cmd_msg, pdMS_TO_TICKS(50));
        }
    }

    if (result != pdPASS) {
        /* 命令队列满，设置错误码 */
        modbus_rtu_slave_set_register(REG_HOST_ERROR_CODE, (uint16_t)GIMBAL_BUSY);
    } else {
        /* 命令成功加入队列 */
        modbus_rtu_slave_set_register(REG_HOST_ERROR_CODE, 0);
    }
}

/*!
*/
static void host_comm_update_gimbal_gain(void)
{
    static uint32_t last_update = 0;
    uint32_t current_time = systick_get_tick();
    
    if ((current_time - last_update) < 1000) {
        return;
    }
    last_update = current_time;
    
    uint16_t az_gain_h = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_GAIN_H);
    uint16_t az_gain_l = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_GAIN_L);
    uint16_t el_gain_h = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_GAIN_H);
    uint16_t el_gain_l = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_GAIN_L);
    
    float reg_azimuth_gain = modbus_rtu_registers_to_float(az_gain_h, az_gain_l);
    float reg_elevation_gain = modbus_rtu_registers_to_float(el_gain_h, el_gain_l);
    
    extern gimbal_gain_config_t g_gimbal_gain;
    
    if ((az_gain_h != 0 || az_gain_l != 0) && 
        (reg_azimuth_gain >= 0.5f && reg_azimuth_gain <= 2.0f) &&
        (reg_elevation_gain >= 0.5f && reg_elevation_gain <= 2.0f)) {
        
        if (reg_azimuth_gain != g_gimbal_gain.azimuth_gain || 
            reg_elevation_gain != g_gimbal_gain.elevation_gain) {
            gimbal_set_gain(reg_azimuth_gain, reg_elevation_gain);
        }
    } else {
        modbus_rtu_float_to_registers(g_gimbal_gain.azimuth_gain, &az_gain_h, &az_gain_l);
        modbus_rtu_float_to_registers(g_gimbal_gain.elevation_gain, &el_gain_h, &el_gain_l);
        modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_GAIN_H, az_gain_h);
        modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_GAIN_L, az_gain_l);
        modbus_rtu_slave_set_register(REG_HOST_ELEVATION_GAIN_H, el_gain_h);
        modbus_rtu_slave_set_register(REG_HOST_ELEVATION_GAIN_L, el_gain_l);
    }
}

void host_comm_register_task_handle(TaskHandle_t task_handle)
{
    s_host_task_handle = task_handle;
}

/*!
    \brief    保存Modbus寄存器参数到Flash
    \detail   将光控阈值、时区、校准数据等参数保存到Flash
*/
static void host_comm_save_params_to_flash(void)
{
    /* 1. 读取光控阈值（单位：0.01V） */
    uint16_t enter_threshold_reg = modbus_rtu_slave_get_register(REG_HOST_OPTICAL_ENTER_THRESHOLD);
    uint16_t exit_threshold_reg = modbus_rtu_slave_get_register(REG_HOST_OPTICAL_EXIT_THRESHOLD);
    float enter_threshold = (float)enter_threshold_reg / 100.0f;
    float exit_threshold = (float)exit_threshold_reg / 100.0f;

    /* 1.1 读取光学追踪死区阈值（单位：0.001） */
    uint16_t err_enter_reg = modbus_rtu_slave_get_register(REG_HOST_OPTICAL_ERR_ENTER);
    uint16_t err_exit_reg = modbus_rtu_slave_get_register(REG_HOST_OPTICAL_ERR_EXIT);
    float err_enter = (float)err_enter_reg / 1000.0f;
    float err_exit = (float)err_exit_reg / 1000.0f;
    
    /* 2. 读取时区 */
    int16_t timezone = (int16_t)modbus_rtu_slave_get_register(REG_HOST_TIMEZONE);
    
    /* 3. 读取校准偏差（float格式） */
    uint16_t az_offset_h = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_OFFSET_H);
    uint16_t az_offset_l = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_OFFSET_L);
    uint16_t el_offset_h = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_OFFSET_H);
    uint16_t el_offset_l = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_OFFSET_L);
    
    float az_offset = modbus_rtu_registers_to_float(az_offset_h, az_offset_l);
    float el_offset = modbus_rtu_registers_to_float(el_offset_h, el_offset_l);
    
    /* 4. 读取夜间归位位置（float格式） */
    uint16_t night_az_h = modbus_rtu_slave_get_register(REG_HOST_NIGHT_AZIMUTH_H);
    uint16_t night_az_l = modbus_rtu_slave_get_register(REG_HOST_NIGHT_AZIMUTH_L);
    uint16_t night_el_h = modbus_rtu_slave_get_register(REG_HOST_NIGHT_ELEVATION_H);
    uint16_t night_el_l = modbus_rtu_slave_get_register(REG_HOST_NIGHT_ELEVATION_L);
    
    float night_az = modbus_rtu_registers_to_float(night_az_h, night_az_l);
    float night_el = modbus_rtu_registers_to_float(night_el_h, night_el_l);
    
    /* 5. 读取GPS校时间隔 */
    uint16_t gps_interval = modbus_rtu_slave_get_register(REG_HOST_GPS_CALIB_INTERVAL);
    
    /* 6. 读取经纬度（float格式） */
    uint16_t lat_h = modbus_rtu_slave_get_register(REG_HOST_LATITUDE_H);
    uint16_t lat_l = modbus_rtu_slave_get_register(REG_HOST_LATITUDE_L);
    uint16_t lon_h = modbus_rtu_slave_get_register(REG_HOST_LONGITUDE_H);
    uint16_t lon_l = modbus_rtu_slave_get_register(REG_HOST_LONGITUDE_L);
    
    float latitude = modbus_rtu_registers_to_float(lat_h, lat_l);
    float longitude = modbus_rtu_registers_to_float(lon_h, lon_l);
    
    /* 7. 读取云台增益（float格式） */
    uint16_t az_gain_h = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_GAIN_H);
    uint16_t az_gain_l = modbus_rtu_slave_get_register(REG_HOST_AZIMUTH_GAIN_L);
    uint16_t el_gain_h = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_GAIN_H);
    uint16_t el_gain_l = modbus_rtu_slave_get_register(REG_HOST_ELEVATION_GAIN_L);
    
    float az_gain = modbus_rtu_registers_to_float(az_gain_h, az_gain_l);
    float el_gain = modbus_rtu_registers_to_float(el_gain_h, el_gain_l);

    /* 7.1 读取太阳高度角工作范围 */
    uint16_t sun_elev_min_h = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEV_MIN_H);
    uint16_t sun_elev_min_l = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEV_MIN_L);
    uint16_t sun_elev_max_h = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEV_MAX_H);
    uint16_t sun_elev_max_l = modbus_rtu_slave_get_register(REG_HOST_SUN_ELEV_MAX_L);

    float sun_elev_min = modbus_rtu_registers_to_float(sun_elev_min_h, sun_elev_min_l);
    float sun_elev_max = modbus_rtu_registers_to_float(sun_elev_max_h, sun_elev_max_l);

    /* 7.2 读取云台仰角工作范围 */
    uint16_t gimbal_elev_min_h = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEV_MIN_H);
    uint16_t gimbal_elev_min_l = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEV_MIN_L);
    uint16_t gimbal_elev_max_h = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEV_MAX_H);
    uint16_t gimbal_elev_max_l = modbus_rtu_slave_get_register(REG_HOST_GIMBAL_ELEV_MAX_L);

    float gimbal_elev_min = modbus_rtu_registers_to_float(gimbal_elev_min_h, gimbal_elev_min_l);
    float gimbal_elev_max = modbus_rtu_registers_to_float(gimbal_elev_max_h, gimbal_elev_max_l);

    /* 8. 保存到Flash */
    bool success = true;
    
    if (!flash_config_set_optical_thresholds(enter_threshold, exit_threshold)) {
        success = false;
    }

    if (!flash_config_set_optical_error_deadband(err_enter, err_exit)) {
        success = false;
    }
    
    if (!flash_config_set_timezone(timezone)) {
        success = false;
    }
    
    if (!flash_config_save_calibration(az_offset, el_offset)) {
        success = false;
    }
    
    if (!flash_config_set_night_position(night_az, night_el)) {
        success = false;
    }
    
    if (!flash_config_set_gps_interval(gps_interval)) {
        success = false;
    }
    
    if (!flash_config_set_location(latitude, longitude)) {
        success = false;
    }
    
    if (!flash_config_set_gimbal_gain(az_gain, el_gain)) {
        success = false;
    }

    if (!flash_config_set_sun_elevation_work_range(sun_elev_min, sun_elev_max)) {
        success = false;
    }

    if (!flash_config_set_gimbal_elevation_range(gimbal_elev_min, gimbal_elev_max)) {
        success = false;
    }

    /* 10. 记录到SD卡日志 */
    if (sd_logger_is_mounted()) {
        if (success) {
            sd_logger_printf("Parameters saved: Tz=%d, OpEnter=%.2fV, OpExit=%.2fV, AzOff=%.3f, ElOff=%.3f, Night=(%.1f,%.1f), GpsInt=%dh, Loc=(%.4f,%.4f), Gain=(%.3f,%.3f)",
                           timezone, enter_threshold, exit_threshold, az_offset, el_offset, night_az, night_el, gps_interval, latitude, longitude, az_gain, el_gain);
        } else {
            sd_logger_error(0x2001, "Failed to save parameters to Flash");
        }
    }
}

/*!
    \brief    从Flash加载参数到Modbus寄存器
    \detail   系统启动时调用，恢复上次保存的参数
*/
void host_comm_load_params_from_flash(void)
{
    const config_runtime_t* config = flash_config_get_runtime();
    if (config == NULL) {
        return;
    }
    
    /* 1. 加载光控阈值 */
    modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ENTER_THRESHOLD, 
                                  (uint16_t)(config->optical_enter_threshold * 100.0f + 0.5f));
    modbus_rtu_slave_set_register(REG_HOST_OPTICAL_EXIT_THRESHOLD, 
                                  (uint16_t)(config->optical_exit_threshold * 100.0f + 0.5f));

    /* 1.1 加载光学追踪死区阈值 (Version 8新增) */
    modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ERR_ENTER, 
                                  (uint16_t)(config->optical_error_enter * 1000.0f + 0.5f));
    modbus_rtu_slave_set_register(REG_HOST_OPTICAL_ERR_EXIT, 
                                  (uint16_t)(config->optical_error_exit * 1000.0f + 0.5f));
    
    /* 2. 加载时区 */
    modbus_rtu_slave_set_register(REG_HOST_TIMEZONE, (uint16_t)(int16_t)config->timezone_offset_hours);
    
    /* 3. 加载校准偏差（无论calibration_valid状态如何都加载，确保寄存器值与Flash一致） */
    {
        uint16_t az_h, az_l, el_h, el_l;
        modbus_rtu_float_to_registers(config->azimuth_offset, &az_h, &az_l);
        modbus_rtu_float_to_registers(config->elevation_offset, &el_h, &el_l);
        
        modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_OFFSET_H, az_h);
        modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_OFFSET_L, az_l);
        modbus_rtu_slave_set_register(REG_HOST_ELEVATION_OFFSET_H, el_h);
        modbus_rtu_slave_set_register(REG_HOST_ELEVATION_OFFSET_L, el_l);
    }
    
    /* 4. 加载夜间归位位置 */
    uint16_t night_az_h, night_az_l, night_el_h, night_el_l;
    modbus_rtu_float_to_registers(config->night_azimuth, &night_az_h, &night_az_l);
    modbus_rtu_float_to_registers(config->night_elevation, &night_el_h, &night_el_l);
    
    modbus_rtu_slave_set_register(REG_HOST_NIGHT_AZIMUTH_H, night_az_h);
    modbus_rtu_slave_set_register(REG_HOST_NIGHT_AZIMUTH_L, night_az_l);
    modbus_rtu_slave_set_register(REG_HOST_NIGHT_ELEVATION_H, night_el_h);
    modbus_rtu_slave_set_register(REG_HOST_NIGHT_ELEVATION_L, night_el_l);
    
    /* 5. 加载GPS校时间隔 */
    modbus_rtu_slave_set_register(REG_HOST_GPS_CALIB_INTERVAL, config->gps_calib_interval_hours);
    
    /* 6. 加载经纬度 */
    uint16_t lat_h, lat_l, lon_h, lon_l;
    modbus_rtu_float_to_registers(config->latitude, &lat_h, &lat_l);
    modbus_rtu_float_to_registers(config->longitude, &lon_h, &lon_l);
    
    modbus_rtu_slave_set_register(REG_HOST_LATITUDE_H, lat_h);
    modbus_rtu_slave_set_register(REG_HOST_LATITUDE_L, lat_l);
    modbus_rtu_slave_set_register(REG_HOST_LONGITUDE_H, lon_h);
    modbus_rtu_slave_set_register(REG_HOST_LONGITUDE_L, lon_l);
    
    /* 7. 加载云台增益（写到寄存器并立即同步到全局变量） */
    uint16_t gain_az_h, gain_az_l, gain_el_h, gain_el_l;
    modbus_rtu_float_to_registers(config->azimuth_gain, &gain_az_h, &gain_az_l);
    modbus_rtu_float_to_registers(config->elevation_gain, &gain_el_h, &gain_el_l);
    
    modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_GAIN_H, gain_az_h);
    modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_GAIN_L, gain_az_l);
    modbus_rtu_slave_set_register(REG_HOST_ELEVATION_GAIN_H, gain_el_h);
    modbus_rtu_slave_set_register(REG_HOST_ELEVATION_GAIN_L, gain_el_l);
    
    /* 立即同步增益到全局变量g_gimbal_gain（避免等待host_comm_update_gimbal_gain周期性更新） */
    if (config->azimuth_gain >= 0.5f && config->azimuth_gain <= 2.0f &&
        config->elevation_gain >= 0.5f && config->elevation_gain <= 2.0f) {
        gimbal_set_gain(config->azimuth_gain, config->elevation_gain);
    }

    /* 8. 加载太阳高度角工作范围 */
    uint16_t sun_elev_min_h, sun_elev_min_l, sun_elev_max_h, sun_elev_max_l;
    modbus_rtu_float_to_registers(config->sun_elevation_work_min, &sun_elev_min_h, &sun_elev_min_l);
    modbus_rtu_float_to_registers(config->sun_elevation_work_max, &sun_elev_max_h, &sun_elev_max_l);

    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MIN_H, sun_elev_min_h);
    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MIN_L, sun_elev_min_l);
    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MAX_H, sun_elev_max_h);
    modbus_rtu_slave_set_register(REG_HOST_SUN_ELEV_MAX_L, sun_elev_max_l);

    /* TODO: [TEST] 加载云台仰角工作范围到寄存器 - 测试后需移除此代码块 (约8行) */
    /* 9. 加载云台仰角工作范围 */
    uint16_t gimbal_elev_min_h, gimbal_elev_min_l, gimbal_elev_max_h, gimbal_elev_max_l;
    modbus_rtu_float_to_registers(config->gimbal_elevation_min, &gimbal_elev_min_h, &gimbal_elev_min_l);
    modbus_rtu_float_to_registers(config->gimbal_elevation_max, &gimbal_elev_max_h, &gimbal_elev_max_l);

    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MIN_H, gimbal_elev_min_h);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MIN_L, gimbal_elev_min_l);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MAX_H, gimbal_elev_max_h);
    modbus_rtu_slave_set_register(REG_HOST_GIMBAL_ELEV_MAX_L, gimbal_elev_max_l);

    /* 记录加载成功 */
    if (sd_logger_is_mounted()) {
        sd_logger_printf("Parameters loaded from Flash: AzGain=%.3f, ElGain=%.3f, AzOff=%.3f, ElOff=%.3f",
                        config->azimuth_gain, config->elevation_gain,
                        config->azimuth_offset, config->elevation_offset);
    }
}

/*===========================================================================*/
/* GPS经纬度自动更新（防抖动写入Flash）                                        */
/*===========================================================================*/
#define LOCATION_CHANGE_THRESHOLD_DEG   0.001f             /* ~100m级别变化才写 */
#define LOCATION_FLASH_MIN_INTERVAL_MS  (3600U * 1000U)    /* 最小写入间隔1小时 */

static uint32_t s_last_location_flash_update_tick = 0;

void host_comm_update_location_from_gps(float new_lat, float new_lon)
{
    const config_runtime_t* cfg = flash_config_get_runtime();
    if (cfg == NULL) {
        return;
    }
    
    /* 1. 经纬度变化太小，不写 */
    if (fabsf(new_lat - cfg->latitude) < LOCATION_CHANGE_THRESHOLD_DEG &&
        fabsf(new_lon - cfg->longitude) < LOCATION_CHANGE_THRESHOLD_DEG) {
        return;
    }
    
    /* 2. 距离上次写入时间太近，不写 */
    uint32_t now = xTaskGetTickCount();
    if ((now - s_last_location_flash_update_tick) < pdMS_TO_TICKS(LOCATION_FLASH_MIN_INTERVAL_MS)) {
        return;
    }
    
    /* 3. 更新Modbus寄存器（供上位机读取） */
    uint16_t lat_h, lat_l, lon_h, lon_l;
    modbus_rtu_float_to_registers(new_lat, &lat_h, &lat_l);
    modbus_rtu_float_to_registers(new_lon, &lon_h, &lon_l);
    
    modbus_rtu_slave_set_register(REG_HOST_LATITUDE_H, lat_h);
    modbus_rtu_slave_set_register(REG_HOST_LATITUDE_L, lat_l);
    modbus_rtu_slave_set_register(REG_HOST_LONGITUDE_H, lon_h);
    modbus_rtu_slave_set_register(REG_HOST_LONGITUDE_L, lon_l);
    
    /* 4. 更新运行时配置 + 刷新Flash（内部有wear-leveling） */
    if (flash_config_set_location(new_lat, new_lon)) {
        s_last_location_flash_update_tick = now;
        
        if (sd_logger_is_mounted()) {
            sd_logger_printf("GPS location auto-updated to Flash: (%.6f, %.6f)", new_lat, new_lon);
        }
    }
}

/*===========================================================================*/
/* 二次校准offset自动更新（8点窗口内允许连续写入，其余时段节流）               */
/*===========================================================================*/
#define OFFSET_CHANGE_THRESHOLD_DEG     0.05f              /* 0.05度以上变化才写 */
#define OFFSET_FLASH_MIN_INTERVAL_MS    (6U * 3600U * 1000U)  /* 最小写入间隔6小时 */

/* 使用标志位标记首次写入，避免时间计算的溢出问题 */
static uint32_t s_last_offset_flash_update_tick = 0;
static bool s_offset_first_write_done = false;  /* 首次写入标志，上电后第一次允许立即写入 */

static bool host_comm_is_offset_flash_window(void)
{
    rtc_datetime_t local_time;

    if (!time_get_local_datetime(&local_time)) {
        return false;
    }

    return (local_time.hour == 8U);
}

bool host_comm_request_calibration_save(float az_offset, float el_offset)
{
    taskENTER_CRITICAL();
    s_pending_az_offset = az_offset;
    s_pending_el_offset = el_offset;
    s_calib_save_pending = true;
    taskEXIT_CRITICAL();

    return true;
}

void host_comm_process_pending_calibration_save(void)
{
    bool pending = false;
    float az = 0.0f;
    float el = 0.0f;

    taskENTER_CRITICAL();
    if (s_calib_save_pending) {
        az = s_pending_az_offset;
        el = s_pending_el_offset;
        s_calib_save_pending = false;
        pending = true;
    }
    taskEXIT_CRITICAL();

    if (pending) {
        host_comm_update_calibration_offset(az, el);
    }
}

void host_comm_update_calibration_offset(float new_az_offset, float new_el_offset)
{
    const config_runtime_t* cfg = flash_config_get_runtime();
    if (cfg == NULL) {
        return;
    }
    
    /* 1. offset变化太小，不写 */
    if (fabsf(new_az_offset - cfg->azimuth_offset) < OFFSET_CHANGE_THRESHOLD_DEG &&
        fabsf(new_el_offset - cfg->elevation_offset) < OFFSET_CHANGE_THRESHOLD_DEG) {
        return;
    }
    
    /* 2. 判断是否允许写Flash：8点窗口内允许连续写入，其余时段保持原有节流 */
    uint32_t now = xTaskGetTickCount();
    bool write_flash = false;
    
    if (host_comm_is_offset_flash_window()) {
        write_flash = true;
    } else if (!s_offset_first_write_done) {
        /* 首次写入，允许立即写Flash */
        write_flash = true;
    } else {
        /* 非首次写入，检查时间间隔 */
        write_flash = ((now - s_last_offset_flash_update_tick) >= pdMS_TO_TICKS(OFFSET_FLASH_MIN_INTERVAL_MS));
    }
    
    /* 3. 更新Modbus寄存器（供上位机读取） */
    uint16_t az_h, az_l, el_h, el_l;
    modbus_rtu_float_to_registers(new_az_offset, &az_h, &az_l);
    modbus_rtu_float_to_registers(new_el_offset, &el_h, &el_l);
    
    modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_OFFSET_H, az_h);
    modbus_rtu_slave_set_register(REG_HOST_AZIMUTH_OFFSET_L, az_l);
    modbus_rtu_slave_set_register(REG_HOST_ELEVATION_OFFSET_H, el_h);
    modbus_rtu_slave_set_register(REG_HOST_ELEVATION_OFFSET_L, el_l);
    
    /* 4. 如果满足写Flash条件，则保存 */
    if (write_flash) {
        if (flash_config_save_calibration(new_az_offset, new_el_offset)) {
            s_last_offset_flash_update_tick = now;
            s_offset_first_write_done = true;  /* 标记首次写入已完成 */
            
            if (sd_logger_is_mounted()) {
                sd_logger_printf("Calibration offset auto-updated to Flash: Az=%.3f, El=%.3f", 
                               new_az_offset, new_el_offset);
            }
        }
    }
}

