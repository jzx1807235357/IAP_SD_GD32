/*!
    \file    modbus_rtu.c

    \version 2024-12-21, V1.0.0, Modbus RTU for GD32F4xx
*/

#include "modbus_rtu.h"
#include "rs485_hal.h"
#include "rs485_config.h"
#include "systick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "main.h"
#include "app_rtc.h"
#include "iap_shared.h"
#include "sd_logger.h"
#include <string.h>

volatile modbus_rtu_state_t g_modbus_state = MODBUS_RTU_IDLE;
volatile bool g_modbus_frame_received = false;
volatile bool g_modbus_tx_complete = true;

static uint8_t rx_buffer[MODBUS_RTU_MAX_FRAME_SIZE];
static volatile uint16_t rx_index = 0;
static volatile uint16_t s_rx_frame_len = 0;
static volatile uint32_t last_rx_time = 0;
static uint8_t g_slave_address = 0x01;

/* 扩大寄存器数组以容纳IAP寄存器 (REG_HOST_IAP_ERROR=0x0102 -> 需要259个元素) */
#define HOLDING_REGISTER_COUNT  512
static uint16_t g_holding_registers[HOLDING_REGISTER_COUNT];
static uint16_t g_input_registers[256];

/* SD日志读取缓冲区 */
static uint8_t s_sdlog_buf[240];

/* Modbus TX 缓冲区（由 rs485_send_dma_aware 处理） */
static uint8_t modbus_tx_buffer[MODBUS_RTU_MAX_FRAME_SIZE];
static uint16_t modbus_tx_length = 0;

/* IAP request flag */
static volatile bool g_iap_start_request = false;

/* 上位机写经纬度的事件序号（用于区分手动写 vs GPS更新） */
static volatile uint32_t s_host_location_write_seq = 0;

uint32_t modbus_rtu_slave_get_location_write_seq(void)
{
    return s_host_location_write_seq;
}

static void modbus_rtu_send_response(const modbus_rtu_frame_t* frame);
static void modbus_rtu_send_exception_response(uint8_t slave_addr, uint8_t function_code, uint8_t exception_code);
static bool modbus_rtu_parse_frame(const uint8_t* data, uint16_t length, modbus_rtu_frame_t* frame);
static void modbus_rtu_handle_read_holding_registers(const modbus_rtu_frame_t* request);
static void modbus_rtu_handle_read_input_registers(const modbus_rtu_frame_t* request);
static void modbus_rtu_handle_write_single_register(const modbus_rtu_frame_t* request);
static void modbus_rtu_handle_write_multiple_registers(const modbus_rtu_frame_t* request);

static int modbus_is_leap_year(int year);
static int modbus_days_in_month(int year, int month);
static bool modbus_local_to_utc(int16_t tz_hours, const rtc_datetime_t* local_dt, rtc_datetime_t* utc_dt);

/*!
*/
bool modbus_rtu_master_init(void)
{
    return true;
}

static int modbus_is_leap_year(int year)
{
    if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
        return 1;
    }
    return 0;
}

static int modbus_days_in_month(int year, int month)
{
    switch (month) {
    case 1: case 3: case 5: case 7: case 8: case 10: case 12:
        return 31;
    case 4: case 6: case 9: case 11:
        return 30;
    case 2:
        return modbus_is_leap_year(year) ? 29 : 28;
    default:
        return 30;
    }
}

static bool modbus_local_to_utc(int16_t tz_hours, const rtc_datetime_t* local_dt, rtc_datetime_t* utc_dt)
{
    if (!local_dt || !utc_dt) {
        return false;
    }

    int tz_minutes = (int)tz_hours * 60;
    int total_minutes = ((int)local_dt->hour * 60 + (int)local_dt->minute) - tz_minutes;
    int carry_day = 0;

    while (total_minutes >= 1440) {
        total_minutes -= 1440;
        carry_day++;
    }
    while (total_minutes < 0) {
        total_minutes += 1440;
        carry_day--;
    }

    int hour = total_minutes / 60;
    int minute = total_minutes % 60;

    int year = (int)local_dt->year;
    int month = (int)local_dt->month;
    int day = (int)local_dt->day + carry_day;

    while (day > modbus_days_in_month(year, month)) {
        day -= modbus_days_in_month(year, month);
        month++;
        if (month > 12) {
            month = 1;
            year++;
        }
    }

    while (day < 1) {
        month--;
        if (month < 1) {
            month = 12;
            year--;
        }
        day += modbus_days_in_month(year, month);
    }

    utc_dt->year = (uint16_t)year;
    utc_dt->month = (uint8_t)month;
    utc_dt->day = (uint8_t)day;
    utc_dt->hour = (uint8_t)hour;
    utc_dt->minute = (uint8_t)minute;
    utc_dt->second = local_dt->second;

    return true;
}

/*!
*/
bool modbus_rtu_slave_init(uint8_t slave_addr)
{
    g_slave_address = slave_addr;
    
    memset(g_holding_registers, 0, sizeof(g_holding_registers));
    memset(g_input_registers, 0, sizeof(g_input_registers));
    
    
    return true;
}

/*!
*/
bool modbus_rtu_master_read_holding_registers(uint8_t slave_addr, uint16_t start_addr, uint16_t count, uint16_t* data)
{
    (void)slave_addr;
    (void)start_addr;
    (void)count;
    (void)data;
    return false;
}

/*!
*/
bool modbus_rtu_master_read_input_registers(uint8_t slave_addr, uint16_t start_addr, uint16_t count, uint16_t* data)
{
    (void)slave_addr;
    (void)start_addr;
    (void)count;
    (void)data;
    return false;
}

/*!
*/
bool modbus_rtu_master_write_single_register(uint8_t slave_addr, uint16_t addr, uint16_t value)
{
    (void)slave_addr;
    (void)addr;
    (void)value;
    return false;
}

/*!
*/
bool modbus_rtu_master_write_multiple_registers(uint8_t slave_addr, uint16_t start_addr, uint16_t count, const uint16_t* data)
{
    (void)slave_addr;
    (void)start_addr;
    (void)count;
    (void)data;
    return false;
}

/*!
*/
void modbus_rtu_slave_process_request(void)
{
    if (!g_modbus_frame_received) {
        return;
    }

    g_modbus_frame_received = false;

    uint16_t frame_len = s_rx_frame_len;
    s_rx_frame_len = 0;
    if (frame_len == 0U) {
        return;
    }
    
    modbus_rtu_frame_t request;
    if (!modbus_rtu_parse_frame(rx_buffer, frame_len, &request)) {
        return;
    }

    if (request.slave_addr != g_slave_address && request.slave_addr != 0) {
        return;
    }

    switch (request.function_code) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
            modbus_rtu_handle_read_holding_registers(&request);
            break;
            
        case MODBUS_FC_READ_INPUT_REGISTERS:
            modbus_rtu_handle_read_input_registers(&request);
            break;
            
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            modbus_rtu_handle_write_single_register(&request);
            break;
            
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            modbus_rtu_handle_write_multiple_registers(&request);
            break;
            
        default:
            modbus_rtu_send_exception_response(g_slave_address, request.function_code, MODBUS_EX_ILLEGAL_FUNCTION);
            break;
    }
}

/*!
*/
void modbus_rtu_slave_set_register(uint16_t addr, uint16_t value)
{
    if (addr < sizeof(g_holding_registers) / sizeof(g_holding_registers[0])) {
        g_holding_registers[addr] = value;
    }
}

/*!
*/
uint16_t modbus_rtu_slave_get_register(uint16_t addr)
{
    if (addr < sizeof(g_holding_registers) / sizeof(g_holding_registers[0])) {
        return g_holding_registers[addr];
    }
    return 0;
}

/*!
*/
void modbus_rtu_slave_set_light_data(float up, float left, float down, float right)
{
    uint16_t up_h, up_l, left_h, left_l, down_h, down_l, right_h, right_l;
    
    modbus_rtu_float_to_registers(up, &up_h, &up_l);
    modbus_rtu_float_to_registers(left, &left_h, &left_l);
    modbus_rtu_float_to_registers(down, &down_h, &down_l);
    modbus_rtu_float_to_registers(right, &right_h, &right_l);
    
    g_holding_registers[REG_LIGHT_UP_H] = up_h;
    g_holding_registers[REG_LIGHT_UP_L] = up_l;
    g_holding_registers[REG_LIGHT_LEFT_H] = left_h;
    g_holding_registers[REG_LIGHT_LEFT_L] = left_l;
    g_holding_registers[REG_LIGHT_DOWN_H] = down_h;
    g_holding_registers[REG_LIGHT_DOWN_L] = down_l;
    g_holding_registers[REG_LIGHT_RIGHT_H] = right_h;
    g_holding_registers[REG_LIGHT_RIGHT_L] = right_l;
}

/*!
*/
uint16_t modbus_rtu_calculate_crc16(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

/*!
*/
bool modbus_rtu_verify_crc16(const uint8_t* data, uint16_t length)
{
    if (length < 2) {
        return false;
    }
    
    uint16_t received_crc = (data[length - 1] << 8) | data[length - 2];
    uint16_t calculated_crc = modbus_rtu_calculate_crc16(data, length - 2);
    
    return received_crc == calculated_crc;
}

/*!
*/
void modbus_rtu_send_frame(const modbus_rtu_frame_t* frame)
{
    if (frame == NULL) {
        return;
    }
    
    uint8_t send_buf[MODBUS_RTU_MAX_FRAME_SIZE];
    uint16_t send_len = 0;
    
    send_buf[send_len++] = frame->slave_addr;
    send_buf[send_len++] = frame->function_code;
    
    for (uint16_t i = 0; i < frame->data_length; i++) {
        send_buf[send_len++] = frame->data[i];
    }
    
    uint16_t crc = modbus_rtu_calculate_crc16(send_buf, send_len);
    send_buf[send_len++] = (uint8_t)(crc & 0xFF);
    send_buf[send_len++] = (uint8_t)(crc >> 8);
    
    modbus_rtu_send_bytes(send_buf, send_len);
}

/*!
*/
void modbus_rtu_process_received_byte(uint8_t byte)
{
    uint32_t current_time = systick_get_tick();
    
    if ((current_time - last_rx_time) > MODBUS_RTU_BYTE_TIMEOUT_MS && g_modbus_state != MODBUS_RTU_IDLE) {
        g_modbus_state = MODBUS_RTU_IDLE;
        rx_index = 0;
    }
    
    last_rx_time = current_time;
    
    if (g_modbus_state == MODBUS_RTU_IDLE) {
        g_modbus_state = MODBUS_RTU_RECEIVING;
    }
    
    if (rx_index < MODBUS_RTU_MAX_FRAME_SIZE) {
        rx_buffer[rx_index++] = byte;
        if (rx_index <= 16) {
        }
        
        if (rx_index >= 4) {
            uint16_t expected_length = 0;
            
            switch (rx_buffer[1]) {
                case MODBUS_FC_READ_HOLDING_REGISTERS:
                case MODBUS_FC_READ_INPUT_REGISTERS:
                    /* 读寄存器命令固定8字节: 地址+功能码+起始地址(2)+数量(2)+CRC(2) */
                    expected_length = 8;
                    break;
                    
                case MODBUS_FC_WRITE_SINGLE_REGISTER:
                case MODBUS_FC_WRITE_SINGLE_COIL:
                    expected_length = 8;
                    break;
                    
                case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
                    /* 写多个寄存器: 地址+功能码+起始地址(2)+数量(2)+字节数(1)+数据(N)+CRC(2) */
                    if (rx_index >= 7) {
                        uint8_t byte_count = rx_buffer[6];
                        expected_length = 7 + byte_count + 2;  /* 前7字节 + 数据 + CRC */
                    }
                    break;
                    
                default:
                    if (rx_index >= 5) {
                    }
                    break;
            }
            
            if (expected_length > 0 && rx_index >= expected_length) {
                uint16_t frame_len = rx_index;
                bool crc_ok = modbus_rtu_verify_crc16(rx_buffer, frame_len);
                if (crc_ok) {
                    s_rx_frame_len = frame_len;
                    g_modbus_frame_received = true;
                }
                g_modbus_state = MODBUS_RTU_IDLE;
                rx_index = 0;
            }
        }
    }
}

/*!
    \brief    UART中断处理函数（已废弃，保留以兼容旧代码）
    \note     USART5 发送已改用 DMA，此函数不再被调用
*/
void modbus_rtu_uart_irq_handler(void)
{
    /* 空实现，保留以保持 API 兼容性 */
}

/*!
*/
void modbus_rtu_float_to_registers(float value, uint16_t* high, uint16_t* low)
{
    union {
        float f;
        uint32_t i;
    } converter;
    
    converter.f = value;
    
    *high = (uint16_t)(converter.i >> 16);
    *low = (uint16_t)(converter.i & 0xFFFF);
}

/*!
*/
float modbus_rtu_registers_to_float(uint16_t high, uint16_t low)
{
    union {
        float f;
        uint32_t i;
    } converter;
    
    converter.i = ((uint32_t)high << 16) | low;
    
    return converter.f;
}


/*!
*/
static void modbus_rtu_send_response(const modbus_rtu_frame_t* frame)
{
    modbus_rtu_send_frame(frame);
}

/*!
*/
static void modbus_rtu_send_exception_response(uint8_t slave_addr, uint8_t function_code, uint8_t exception_code)
{
    modbus_rtu_frame_t response;
    response.slave_addr = slave_addr;
    response.function_code = function_code | 0x80;
    response.data[0] = exception_code;
    response.data_length = 1;
    
    modbus_rtu_send_response(&response);
}

/*!
*/
static bool modbus_rtu_parse_frame(const uint8_t* data, uint16_t length, modbus_rtu_frame_t* frame)
{
    if (data == NULL || length < 4 || frame == NULL) {
        return false;
    }
    
    frame->slave_addr = data[0];
    frame->function_code = data[1];
    frame->data_length = length - 4;
    
    for (uint16_t i = 0; i < frame->data_length; i++) {
        frame->data[i] = data[2 + i];
    }
    
    frame->crc = (data[length - 1] << 8) | data[length - 2];
    
    return true;
}

/*!
*/
static void modbus_rtu_handle_read_holding_registers(const modbus_rtu_frame_t* request)
{
    if (request->data_length != 4) {
        modbus_rtu_send_exception_response(g_slave_address, request->function_code, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t start_addr = (request->data[0] << 8) | request->data[1];
    uint16_t count = (request->data[2] << 8) | request->data[3];
    
    if (count == 0 || count > 125 || start_addr + count > sizeof(g_holding_registers) / sizeof(g_holding_registers[0])) {
        modbus_rtu_send_exception_response(g_slave_address, request->function_code, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    modbus_rtu_frame_t response;
    response.slave_addr = g_slave_address;
    response.function_code = MODBUS_FC_READ_HOLDING_REGISTERS;
    
    /* 第一个字节是字节计数 */
    response.data[0] = (uint8_t)(count * 2);
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t value = g_holding_registers[start_addr + i];
        response.data[1 + i * 2] = (uint8_t)(value >> 8);
        response.data[2 + i * 2] = (uint8_t)(value & 0xFF);
    }
    
    response.data_length = 1 + count * 2;
    modbus_rtu_send_response(&response);
}

/*!
*/
static void modbus_rtu_handle_read_input_registers(const modbus_rtu_frame_t* request)
{
    if (request->data_length != 4) {
        modbus_rtu_send_exception_response(g_slave_address, request->function_code, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t start_addr = (request->data[0] << 8) | request->data[1];
    uint16_t count = (request->data[2] << 8) | request->data[3];
    
    if (count == 0 || count > 125 || start_addr + count > sizeof(g_input_registers) / sizeof(g_input_registers[0])) {
        modbus_rtu_send_exception_response(g_slave_address, request->function_code, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    modbus_rtu_frame_t response;
    response.slave_addr = g_slave_address;
    response.function_code = MODBUS_FC_READ_INPUT_REGISTERS;
    
    /* 第一个字节是字节计数 */
    response.data[0] = (uint8_t)(count * 2);
    
    for (uint16_t i = 0; i < count; i++) {
        uint16_t value = g_input_registers[start_addr + i];
        response.data[1 + i * 2] = (uint8_t)(value >> 8);
        response.data[2 + i * 2] = (uint8_t)(value & 0xFF);
    }
    
    response.data_length = 1 + count * 2;
    modbus_rtu_send_response(&response);
}

/*!
*/
static void modbus_rtu_handle_write_single_register(const modbus_rtu_frame_t* request)
{
    if (request->data_length != 4) {
        modbus_rtu_send_exception_response(g_slave_address, request->function_code, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t addr = (request->data[0] << 8) | request->data[1];
    uint16_t value = (request->data[2] << 8) | request->data[3];
    
    if (addr >= sizeof(g_holding_registers) / sizeof(g_holding_registers[0])) {
        modbus_rtu_send_exception_response(g_slave_address, request->function_code, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    /* Check for IAP start command: APP IAP control register */
    if (addr == REG_HOST_IAP_COMMAND && value == IAP_CMD_ENTER_BOOTLOADER) {
        g_iap_start_request = true;
        g_holding_registers[REG_HOST_IAP_STATUS] = IAP_STATUS_REQUESTED;
    }
    
    /* 处理SD日志读取命令 */
    if (addr == REG_SDLOG_CMD) {
        uint16_t year, month, day;
        int bytes_read;
        uint16_t reg_count;
        uint16_t i;
        
        switch (value) {
            case SDLOG_CMD_OPEN:
                /* 从寄存器读取日期 */
                year  = g_holding_registers[REG_SDLOG_YEAR];
                month = (uint8_t)g_holding_registers[REG_SDLOG_MONTH];
                day   = (uint8_t)g_holding_registers[REG_SDLOG_DAY];
                
                /* 尝试打开文件 */
                if (year == 0 && month == 0 && day == 0) {
                    if (sd_logger_open_tracking_index()) {
                        g_holding_registers[REG_SDLOG_STATUS] = SDLOG_STATUS_IDLE;
                    } else {
                        g_holding_registers[REG_SDLOG_STATUS] = SDLOG_STATUS_ERROR;
                    }
                } else {
                    if (sd_logger_open_tracking_file_by_date(year, month, day)) {
                        g_holding_registers[REG_SDLOG_STATUS] = SDLOG_STATUS_IDLE;
                    } else {
                        g_holding_registers[REG_SDLOG_STATUS] = SDLOG_STATUS_ERROR;
                    }
                }
                break;
                
            case SDLOG_CMD_READ_NEXT:
                /* 读取下一块数据 */
                bytes_read = sd_logger_read_tracking_chunk(s_sdlog_buf, sizeof(s_sdlog_buf));
                
                if (bytes_read > 0) {
                    /* 有数据 */
                    g_holding_registers[REG_SDLOG_DATA_LEN] = (uint16_t)bytes_read;
                    
                    /* 将字节数据打包到寄存器（每个寄存器2字节） */
                    reg_count = (bytes_read + 1) / 2;
                    for (i = 0; i < reg_count; i++) {
                        uint16_t reg_value;
                        uint8_t b0 = s_sdlog_buf[i * 2];
                        uint8_t b1 = (i * 2 + 1 < bytes_read) ? s_sdlog_buf[i * 2 + 1] : 0;
                        reg_value = ((uint16_t)b0 << 8) | b1;
                        g_holding_registers[REG_SDLOG_DATA_BASE + i] = reg_value;
                    }
                    
                    g_holding_registers[REG_SDLOG_STATUS] = SDLOG_STATUS_DATA;
                } else {
                    /* EOF */
                    g_holding_registers[REG_SDLOG_DATA_LEN] = 0;
                    g_holding_registers[REG_SDLOG_STATUS] = SDLOG_STATUS_EOF;
                }
                break;
                
            case SDLOG_CMD_CLOSE:
                /* 关闭文件 */
                sd_logger_close_tracking_file();
                g_holding_registers[REG_SDLOG_STATUS] = SDLOG_STATUS_IDLE;
                break;

            case SDLOG_CMD_REBUILD_INDEX:
                /* 触发异步索引重建：快速返回，由后台任务执行实际重建 */
                sd_logger_request_rebuild_index_async();
                break;
                
            default:
                break;
        }
        
        /* 命令执行完成后清除命令寄存器，不再执行后面的通用赋值 */
        g_holding_registers[REG_SDLOG_CMD] = SDLOG_CMD_NONE;
    } else {
        /* 非SDLOG_CMD寄存器，执行通用赋值 */
        g_holding_registers[addr] = value;
        
        /* 上位机写经纬度时递增事件序号 */
        if (addr >= REG_HOST_LATITUDE_H && addr <= REG_HOST_LONGITUDE_L) {
            s_host_location_write_seq++;
        }
    }
    
    extern void host_comm_execute_command(uint16_t cmd_code);
    if (addr == REG_HOST_CONTROL_COMMAND && value != CMD_NONE) {
        host_comm_execute_command(value);
    }
    
    modbus_rtu_frame_t response;
    response.slave_addr = g_slave_address;
    response.function_code = MODBUS_FC_WRITE_SINGLE_REGISTER;
    response.data[0] = (uint8_t)(addr >> 8);
    response.data[1] = (uint8_t)(addr & 0xFF);
    response.data[2] = (uint8_t)(value >> 8);
    response.data[3] = (uint8_t)(value & 0xFF);
    response.data_length = 4;
    
    modbus_rtu_send_response(&response);
}

/*!
*/
static void modbus_rtu_handle_write_multiple_registers(const modbus_rtu_frame_t* request)
{
    if (request->data_length < 5) {
        modbus_rtu_send_exception_response(g_slave_address, request->function_code, MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    
    uint16_t start_addr = (request->data[0] << 8) | request->data[1];
    uint16_t count = (request->data[2] << 8) | request->data[3];
    uint8_t byte_count = request->data[4];
    
    if (count == 0 || count > 123 || byte_count != count * 2 || 
        start_addr + count > sizeof(g_holding_registers) / sizeof(g_holding_registers[0])) {
        modbus_rtu_send_exception_response(g_slave_address, request->function_code, MODBUS_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
    
    if (start_addr == REG_HOST_RTC_YEAR && count == 6) {
        rtc_datetime_t local_dt;
        local_dt.year = (uint16_t)((request->data[5] << 8) | request->data[6]);
        local_dt.month = (uint8_t)((request->data[7] << 8) | request->data[8]);
        local_dt.day = (uint8_t)((request->data[9] << 8) | request->data[10]);
        local_dt.hour = (uint8_t)((request->data[11] << 8) | request->data[12]);
        local_dt.minute = (uint8_t)((request->data[13] << 8) | request->data[14]);
        local_dt.second = (uint8_t)((request->data[15] << 8) | request->data[16]);

        bool valid = true;
        if (local_dt.year < 2000 || local_dt.year > 2100 ||
            local_dt.month < 1 || local_dt.month > 12 ||
            local_dt.day < 1 || local_dt.day > 31 ||
            local_dt.hour > 23 || local_dt.minute > 59 || local_dt.second > 59) {
            valid = false;
        } else {
            int dim = modbus_days_in_month(local_dt.year, local_dt.month);
            if (local_dt.day > dim) {
                valid = false;
            }
        }

        if (valid) {
            int16_t tz_hours = (int16_t)g_holding_registers[REG_HOST_TIMEZONE];
            rtc_datetime_t utc_dt;
            if (modbus_local_to_utc(tz_hours, &local_dt, &utc_dt) && app_rtc_set_datetime(&utc_dt)) {
                g_holding_registers[REG_HOST_RTC_YEAR] = local_dt.year;
                g_holding_registers[REG_HOST_RTC_MONTH] = local_dt.month;
                g_holding_registers[REG_HOST_RTC_DAY] = local_dt.day;
                g_holding_registers[REG_HOST_RTC_HOUR] = local_dt.hour;
                g_holding_registers[REG_HOST_RTC_MINUTE] = local_dt.minute;
                g_holding_registers[REG_HOST_RTC_SECOND] = local_dt.second;
                g_holding_registers[REG_HOST_ERROR_CODE] = 0;
            } else {
                g_holding_registers[REG_HOST_ERROR_CODE] = 0x3001;
            }
        } else {
            g_holding_registers[REG_HOST_ERROR_CODE] = 0x3002;
        }
    } else {
        bool touched_location = false;
        for (uint16_t i = 0; i < count; i++) {
            uint16_t reg_addr = start_addr + i;
            uint16_t value = (request->data[5 + i * 2] << 8) | request->data[6 + i * 2];
            g_holding_registers[reg_addr] = value;
            if (reg_addr >= REG_HOST_LATITUDE_H && reg_addr <= REG_HOST_LONGITUDE_L) {
                touched_location = true;
            }
        }
        /* 上位机写经纬度时递增事件序号（一次写多个寄存器只递增一次） */
        if (touched_location) {
            s_host_location_write_seq++;
        }
    }
    
    /* RTC时间寄存器支持上位机写入：
     * - 上位机写入的时间被视为"设备本地时间"
     * - 固件根据REG_HOST_TIMEZONE做 local→UTC 转换后存入RTC
     * - GPS定位成功时仍可自动校时（根据配置的间隔，默认24小时） */
    
    modbus_rtu_frame_t response;
    response.slave_addr = g_slave_address;
    response.function_code = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    response.data[0] = (uint8_t)(start_addr >> 8);
    response.data[1] = (uint8_t)(start_addr & 0xFF);
    response.data[2] = (uint8_t)(count >> 8);
    response.data[3] = (uint8_t)(count & 0xFF);
    response.data_length = 4;
    
    modbus_rtu_send_response(&response);
}

/*!
*/
void modbus_rtu_send_bytes(const uint8_t* data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return;
    }

    /* 移除旧的xUART0_TxSemaphore检查，rs485_hal已处理互斥 */

    taskENTER_CRITICAL();
    memcpy(modbus_tx_buffer, data, length);
    modbus_tx_length = length;
    g_modbus_tx_complete = false;
    taskEXIT_CRITICAL();

    /* 使用DMA优化的RS485 HAL发送（上位机总线会优先使用DMA）*/
    rs485_result_t result = rs485_send_dma_aware(RS485_BUS_HOST, modbus_tx_buffer, modbus_tx_length, MODBUS_RTU_TIMEOUT_MS);
    
    if (result == RS485_OK) {
        g_modbus_tx_complete = true;
    } else {
        g_modbus_tx_complete = false;
    }

    /* 发送完成，无需释放信号量 */
}

/*!
    \brief      Check and clear IAP start request
    \param[in]  none
    \param[out] none
    \retval     true if IAP start was requested, false otherwise
*/
bool modbus_rtu_check_and_clear_iap_request(void)
{
    bool requested = g_iap_start_request;
    if (requested) {
        g_iap_start_request = false;
    }
    return requested;
}

