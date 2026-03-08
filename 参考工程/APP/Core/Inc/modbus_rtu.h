/*!
    \file    modbus_rtu.h
    \brief   Modbus RTU协议实现 - 支持主站和从站功能

    \version 2024-12-21, V1.0.0, Modbus RTU for GD32F4xx
*/

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gd32f4xx.h"
#include "FreeRTOS.h"
#include <stdbool.h>
#include <stdint.h>

/* Modbus RTU配置参数 */
#define MODBUS_RTU_MAX_FRAME_SIZE     256
#define MODBUS_RTU_TIMEOUT_MS         500
#define MODBUS_RTU_FRAME_DELAY_MS     20
#define MODBUS_RTU_BYTE_TIMEOUT_MS    10

/* 设备地址定义 */
#define MODBUS_SLAVE_ADDR_SENSOR       0x01    /* 传感器设备地址（自定义协议，非Modbus） */
#define MODBUS_SLAVE_ADDR_HOST         0x01    /* 下位机Modbus从站地址（对接上位机） */

/* Modbus功能码定义 */
typedef enum {
    MODBUS_FC_READ_COILS = 0x01,
    MODBUS_FC_READ_DISCRETE_INPUTS = 0x02,
    MODBUS_FC_READ_HOLDING_REGISTERS = 0x03,
    MODBUS_FC_READ_INPUT_REGISTERS = 0x04,
    MODBUS_FC_WRITE_SINGLE_COIL = 0x05,
    MODBUS_FC_WRITE_SINGLE_REGISTER = 0x06,
    MODBUS_FC_WRITE_MULTIPLE_COILS = 0x0F,
    MODBUS_FC_WRITE_MULTIPLE_REGISTERS = 0x10
} modbus_function_code_t;

/* Modbus异常码定义 */
typedef enum {
    MODBUS_EX_ILLEGAL_FUNCTION = 0x01,
    MODBUS_EX_ILLEGAL_DATA_ADDRESS = 0x02,
    MODBUS_EX_ILLEGAL_DATA_VALUE = 0x03,
    MODBUS_EX_SLAVE_DEVICE_FAILURE = 0x04,
    MODBUS_EX_ACKNOWLEDGE = 0x05,
    MODBUS_EX_SLAVE_DEVICE_BUSY = 0x06,
    MODBUS_EX_MEMORY_PARITY_ERROR = 0x08
} modbus_exception_code_t;

/* Modbus RTU状态 */
typedef enum {
    MODBUS_RTU_IDLE = 0,
    MODBUS_RTU_RECEIVING,
    MODBUS_RTU_TRANSMITTING,
    MODBUS_RTU_WAITING_RESPONSE
} modbus_rtu_state_t;

/* Modbus RTU帧结构 */
typedef struct {
    uint8_t slave_addr;
    uint8_t function_code;
    uint8_t data[MODBUS_RTU_MAX_FRAME_SIZE - 4];
    uint16_t data_length;
    uint16_t crc;
} modbus_rtu_frame_t;

/* 光传感器数据寄存器映射（对应上位机40041-40048） */
#define REG_LIGHT_UP_H         0x0028      /* 上传感器电压高位（40041） */
#define REG_LIGHT_UP_L         0x0029      /* 上传感器电压低位（40042） */
#define REG_LIGHT_LEFT_H       0x002A      /* 左传感器电压高位（40043） */
#define REG_LIGHT_LEFT_L       0x002B      /* 左传感器电压低位（40044） */
#define REG_LIGHT_DOWN_H       0x002C      /* 下传感器电压高位（40045） */
#define REG_LIGHT_DOWN_L       0x002D      /* 下传感器电压低位（40046） */
#define REG_LIGHT_RIGHT_H      0x002E      /* 右传感器电压高位（40047） */
#define REG_LIGHT_RIGHT_L      0x002F      /* 右传感器电压低位（40048） */

/* 传感器电压寄存器（单寄存器存储，单位：毫伏） */
#define REG_HOST_SENSOR_V0     0x0005      /* 上传感器电压mV（40006） */
#define REG_HOST_SENSOR_V1     0x0006      /* 左传感器电压mV（40007） */
#define REG_HOST_SENSOR_V2     0x0007      /* 下传感器电压mV（40008） */
#define REG_HOST_SENSOR_V3     0x0008      /* 右传感器电压mV（40009） */

/* 系统状态寄存器（对应上位机40001-40010） */
#define REG_HOST_DEVICE_STATUS     0x0000  /* 设备状态（40001） */
#define REG_HOST_RUN_MODE          0x0001  /* 运行模式（40002） */
#define REG_HOST_GPS_STATUS        0x0002  /* GPS状态（40003） */
#define REG_HOST_COMM_STATUS       0x0003  /* 通信状态（40004） */
#define REG_HOST_ERROR_CODE        0x0004  /* 错误代码（40005） */

/* 固件版本寄存器（对应上位机40010） */
#define REG_HOST_FIRMWARE_VERSION  0x0009  /* 固件版本号（40010，格式：0xMMNN = MM.NN） */

/* 太阳和云台位置寄存器（对应上位机40021-40034） */
#define REG_HOST_SUN_AZIMUTH_H     0x0014  /* 太阳方位角高位（40021） */
#define REG_HOST_SUN_AZIMUTH_L     0x0015  /* 太阳方位角低位（40022） */
#define REG_HOST_SUN_ELEVATION_H   0x0016  /* 太阳仰角高位（40023） */
#define REG_HOST_SUN_ELEVATION_L   0x0017  /* 太阳仰角低位（40024） */
#define REG_HOST_GIMBAL_AZIMUTH_H  0x001E  /* 云台方位角高位（40031） */
#define REG_HOST_GIMBAL_AZIMUTH_L  0x001F  /* 云台方位角低位（40032） */
#define REG_HOST_GIMBAL_ELEVATION_H 0x0020 /* 云台仰角高位（40033） */
#define REG_HOST_GIMBAL_ELEVATION_L 0x0021 /* 云台仰角低位（40034） */

/* 控制命令寄存器 */
#define REG_HOST_CONTROL_COMMAND   0x0032  /* 控制命令（对应40051） */
#define REG_HOST_TARGET_AZIMUTH_H  0x0033  /* 目标方位角高位（对应40052） */
#define REG_HOST_TARGET_AZIMUTH_L  0x0034  /* 目标方位角低位（对应40053） */
#define REG_HOST_TARGET_ELEVATION_H 0x0035 /* 目标仰角高位（对应40054） */
#define REG_HOST_TARGET_ELEVATION_L 0x0036 /* 目标仰角低位（对应40055） */
#define REG_HOST_MANUAL_DIRECTION  0x0037  /* 手动方向（对应40056） */

/* 位置和时区寄存器 */
#define REG_HOST_LATITUDE_H        0x0046  /* 纬度高位（对应40071） */
#define REG_HOST_LATITUDE_L        0x0047  /* 纬度低位（对应40072） */
#define REG_HOST_LONGITUDE_H       0x0048  /* 经度高位（对应40073） */
#define REG_HOST_LONGITUDE_L       0x0049  /* 经度低位（对应40074） */
#define REG_HOST_TIMEZONE          0x004A  /* 时区偏移（对应40075，有符号整数，单位：小时） */

/* 增益和偏移寄存器 */
#define REG_HOST_AZIMUTH_GAIN_H    0x004B  /* 方位角增益高位（对应40076） */
#define REG_HOST_AZIMUTH_GAIN_L    0x004C  /* 方位角增益低位（对应40077） */
#define REG_HOST_AZIMUTH_OFFSET_H  0x004D  /* 方位角偏移高位（对应40078） */
#define REG_HOST_AZIMUTH_OFFSET_L  0x004E  /* 方位角偏移低位（对应40079） */
#define REG_HOST_ELEVATION_GAIN_H  0x004F  /* 仰角增益高位（对应40080） */
#define REG_HOST_ELEVATION_GAIN_L  0x0050  /* 仰角增益低位（对应40081） */
#define REG_HOST_ELEVATION_OFFSET_H 0x0051 /* 仰角偏移高位（对应40082） */
#define REG_HOST_ELEVATION_OFFSET_L 0x0052 /* 仰角偏移低位（对应40083） */

/* 光控阈值寄存器 (40091-40092) */
#define REG_HOST_OPTICAL_ENTER_THRESHOLD  0x005A  /* 光控进入阈值（对应40091，单位：0.01V，范围0-400） */
#define REG_HOST_OPTICAL_EXIT_THRESHOLD   0x005B  /* 光控退出阈值（对应40092，单位：0.01V，范围0-400） */

/* 夜间归位位置寄存器 (40093-40096) */
#define REG_HOST_NIGHT_AZIMUTH_H          0x005C  /* 夜间归位方位角高位（对应40093） */
#define REG_HOST_NIGHT_AZIMUTH_L          0x005D  /* 夜间归位方位角低位（对应40094） */
#define REG_HOST_NIGHT_ELEVATION_H        0x005E  /* 夜间归位仰角高位（对应40095） */
#define REG_HOST_NIGHT_ELEVATION_L        0x005F  /* 夜间归位仰角低位（对应40096） */

/* GPS校时间隔寄存器 (40097) */
#define REG_HOST_GPS_CALIB_INTERVAL       0x0060  /* GPS校时间隔（对应40097，单位：小时） */

/* 光学追踪死区阈值寄存器 (40098-40099) */
#define REG_HOST_OPTICAL_ERR_ENTER        0x0061  /* 进入死区阈值（对应40098，单位：0.001，范围1-1000，对应0.001-1.0） */
#define REG_HOST_OPTICAL_ERR_EXIT         0x0062  /* 退出死区阈值（对应40099，单位：0.001，范围1-1000，对应0.001-1.0） */

/* RTC时间寄存器 (40101-40106) */
#define REG_HOST_RTC_YEAR          0x0064  /* RTC年份（对应40101） */
#define REG_HOST_RTC_MONTH         0x0065  /* RTC月份（对应40102） */
#define REG_HOST_RTC_DAY           0x0066  /* RTC日期（对应40103） */
#define REG_HOST_RTC_HOUR          0x0067  /* RTC小时（对应40104） */
#define REG_HOST_RTC_MINUTE        0x0068  /* RTC分钟（对应40105） */
#define REG_HOST_RTC_SECOND        0x0069  /* RTC秒（对应40106） */

/* APP侧IAP控制寄存器（在合法范围内，不超出g_holding_registers[256]数组） */
#define REG_HOST_IAP_COMMAND       0x006A  /* IAP控制命令（对应40107） */
#define REG_HOST_IAP_STATUS        0x006B  /* IAP状态（对应40108） */
#define REG_HOST_IAP_ERROR         0x006C  /* IAP错误码（对应40109） */

/* 太阳高度角工作范围寄存器 (40110-40113) */
#define REG_HOST_SUN_ELEV_MIN_H    0x006D  /* 太阳高度角工作范围下限高位（对应40110） */
#define REG_HOST_SUN_ELEV_MIN_L    0x006E  /* 太阳高度角工作范围下限低位（对应40111） */
#define REG_HOST_SUN_ELEV_MAX_H    0x006F  /* 太阳高度角工作范围上限高位（对应40112） */
#define REG_HOST_SUN_ELEV_MAX_L    0x0070  /* 太阳高度角工作范围上限低位（对应40113） */

/* TODO: [TEST] 云台仰角工作范围寄存器 - 测试后需移除此4个寄存器定义 (40114-40117) */
#define REG_HOST_GIMBAL_ELEV_MIN_H 0x0071  /* 云台仰角工作范围下限高位（对应40114） */
#define REG_HOST_GIMBAL_ELEV_MIN_L 0x0072  /* 云台仰角工作范围下限低位（对应40115） */
#define REG_HOST_GIMBAL_ELEV_MAX_H 0x0073  /* 云台仰角工作范围上限高位（对应40116） */
#define REG_HOST_GIMBAL_ELEV_MAX_L 0x0074  /* 云台仰角工作范围上限低位（对应40117） */

/* IAP命令码定义 */
#define IAP_CMD_NONE               0       /* 无IAP命令 */
#define IAP_CMD_ENTER_BOOTLOADER   1       /* 请求进入IAP升级模式 */

/* IAP状态码定义 */
#define IAP_STATUS_IDLE            0       /* 空闲 */
#define IAP_STATUS_REQUESTED       1       /* 已接收到进入IAP请求 */
#define IAP_STATUS_RESETTING       2       /* 即将复位进入BootLoader */

/* SD日志读取寄存器（对应40257-40272 + 数据区） */
#define REG_SDLOG_CMD              0x0100  /* SD日志命令（对应40257） */
#define REG_SDLOG_STATUS           0x0101  /* SD日志状态（对应40258） */
#define REG_SDLOG_YEAR             0x0102  /* 日期-年份（对应40259） */
#define REG_SDLOG_MONTH            0x0103  /* 日期-月份（对应40260） */
#define REG_SDLOG_DAY              0x0104  /* 日期-日（对应40261） */
#define REG_SDLOG_DATA_LEN         0x0105  /* 当前块数据长度（对应40262） */
#define REG_SDLOG_DATA_BASE        0x0110  /* 数据区起始地址（对应40273） */

/* SD日志命令码定义 */
#define SDLOG_CMD_NONE             0       /* 无命令 */
#define SDLOG_CMD_OPEN             1       /* 打开指定日期的日志文件 */
#define SDLOG_CMD_READ_NEXT        2       /* 读取下一块数据 */
#define SDLOG_CMD_CLOSE            3       /* 关闭当前日志文件 */
#define SDLOG_CMD_REBUILD_INDEX    4       /* 重建index.csv索引（扫描现有文件） */
#define SDLOG_CMD_REBUILD_INDEX    4       /* 重建 index.csv 索引（扫描现有文件） */

/* SD日志状态码定义 */
#define SDLOG_STATUS_IDLE          0       /* 空闲 */
#define SDLOG_STATUS_BUSY          1       /* 忙碌 */
#define SDLOG_STATUS_DATA          2       /* 有数据可读 */
#define SDLOG_STATUS_EOF           3       /* 文件读取完成 */
#define SDLOG_STATUS_ERROR         4       /* 错误（文件不存在或SD卡错误） */

/* 控制命令码定义 */
#define CMD_NONE                   0       /* 无命令 */
#define CMD_ENTER_GPS_MODE         1       /* 进入GPS跟踪模式 */
#define CMD_ENTER_OPTICAL_MODE     2       /* 进入光学跟踪模式 */
#define CMD_ENTER_IDLE_MODE        3       /* 进入空闲模式 */
#define CMD_SET_POSITION           4       /* 设置绝对位置 */
#define CMD_MOVE_UP                5       /* 向上移动 */
#define CMD_MOVE_DOWN              6       /* 向下移动 */
#define CMD_MOVE_LEFT              7       /* 向左移动 */
#define CMD_MOVE_RIGHT             8       /* 向右移动 */
#define CMD_STOP_MOVE              9       /* 停止移动 */
#define CMD_SAVE_PARAMS            10      /* 保存参数 */
#define CMD_GIMBAL_SELFTEST        20      /* 云台恢复默认参数+自检（Pelco-D: FF 01 00 07 00 7D 85） */

/* 全局变量声明 */
extern volatile modbus_rtu_state_t g_modbus_state;
extern volatile bool g_modbus_frame_received;
extern volatile bool g_modbus_tx_complete;

/* 主站功能函数 */
bool modbus_rtu_master_init(void);
bool modbus_rtu_master_read_holding_registers(uint8_t slave_addr, uint16_t start_addr, uint16_t count, uint16_t* data);
bool modbus_rtu_master_read_input_registers(uint8_t slave_addr, uint16_t start_addr, uint16_t count, uint16_t* data);
bool modbus_rtu_master_write_single_register(uint8_t slave_addr, uint16_t addr, uint16_t value);
bool modbus_rtu_master_write_multiple_registers(uint8_t slave_addr, uint16_t start_addr, uint16_t count, const uint16_t* data);

/* 从站功能函数 */
bool modbus_rtu_slave_init(uint8_t slave_addr);
void modbus_rtu_slave_process_request(void);
void modbus_rtu_slave_set_register(uint16_t addr, uint16_t value);
uint16_t modbus_rtu_slave_get_register(uint16_t addr);
void modbus_rtu_slave_set_light_data(float up, float left, float down, float right);
bool modbus_rtu_check_and_clear_iap_request(void);
uint32_t modbus_rtu_slave_get_location_write_seq(void);

/* 通用功能函数 */
uint16_t modbus_rtu_calculate_crc16(const uint8_t* data, uint16_t length);
bool modbus_rtu_verify_crc16(const uint8_t* data, uint16_t length);
void modbus_rtu_send_frame(const modbus_rtu_frame_t* frame);
void modbus_rtu_send_bytes(const uint8_t* data, uint16_t length);
void modbus_rtu_process_received_byte(uint8_t byte);
void modbus_rtu_uart_irq_handler(void);  /* 已废弃,保留以兼容旧代码 */

/* 辅助函数 */
void modbus_rtu_float_to_registers(float value, uint16_t* high, uint16_t* low);
float modbus_rtu_registers_to_float(uint16_t high, uint16_t low);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
