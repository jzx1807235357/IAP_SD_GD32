/*
 * pcf8563.c - Minimal PCF8563 RTC driver (software I2C on PB8/PB9)
 */
#include "pcf8563.h"
#include "app_rtc.h"
#include "gd32f4xx.h"

/* Register map */
#define REG_CTRL1      0x00
#define REG_CTRL2      0x01
#define REG_SECONDS    0x02
#define REG_MINUTES    0x03
#define REG_HOURS      0x04
#define REG_DAYS       0x05
#define REG_WEEKDAYS   0x06
#define REG_MONTHS     0x07
#define REG_YEARS      0x08
#define REG_CLKOUT     0x0D
#define REG_TIMER_CTRL 0x0E
#define REG_TIMER_VAL  0x0F

#define CTRL1_STOP     (1u << 5)
#define CTRL2_AF       (1u << 3)
#define CTRL2_TF       (1u << 2)
#define CTRL2_AIE      (1u << 1)
#define CTRL2_TIE      (1u << 0)

/* Helpers */
static inline uint8_t bcd_from_bin(uint8_t value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static inline uint8_t bin_from_bcd(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) * 10) + (bcd & 0x0F));
}

static inline uint8_t clamp_u8(uint8_t v, uint8_t min, uint8_t max)
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

static uint8_t calc_weekday(uint16_t year, uint8_t month, uint8_t day)
{
    /* Zeller congruence, result: 0 = Sunday ... 6 = Saturday */
    if (month < 3) {
        month = (uint8_t)(month + 12);
        year--;
    }
    uint32_t k = year % 100u;
    uint32_t j = year / 100u;
    uint32_t h = (day + 13u * (month + 1u) / 5u + k + k / 4u + j / 4u + 5u * j) % 7u;
    return (uint8_t)((h + 6u) % 7u);
}

/* Software I2C pins - PB8 (SCL) / PB9 (SDA) */
#define PCF_SCL_PORT   GPIOB
#define PCF_SDA_PORT   GPIOB
#define PCF_SCL_PIN    GPIO_PIN_8
#define PCF_SDA_PIN    GPIO_PIN_9

static void i2c_delay_inline(void)
{
    for (volatile int i = 0; i < 80; ++i) { __NOP(); }
}

static inline void scl_high(void) { gpio_bit_set(PCF_SCL_PORT, PCF_SCL_PIN); i2c_delay_inline(); }
static inline void scl_low(void)  { gpio_bit_reset(PCF_SCL_PORT, PCF_SCL_PIN); i2c_delay_inline(); }
static inline void sda_high(void) { gpio_bit_set(PCF_SDA_PORT, PCF_SDA_PIN); i2c_delay_inline(); }
static inline void sda_low(void)  { gpio_bit_reset(PCF_SDA_PORT, PCF_SDA_PIN); i2c_delay_inline(); }
static inline int  sda_read(void) { return gpio_input_bit_get(PCF_SDA_PORT, PCF_SDA_PIN) != RESET; }

static void i2c_gpio_init_sw(void)
{
    static bool initialized = false;
    if (initialized) {
        return;
    }
    rcu_periph_clock_enable(RCU_GPIOB);

    gpio_mode_set(PCF_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PCF_SCL_PIN);
    gpio_output_options_set(PCF_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, PCF_SCL_PIN);
    gpio_mode_set(PCF_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PCF_SDA_PIN);
    gpio_output_options_set(PCF_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, PCF_SDA_PIN);

    scl_high();
    sda_high();
    initialized = true;
}

static void i2c_start_sw(void)
{
    sda_high(); scl_high();
    sda_low();
    scl_low();
}

static void i2c_stop_sw(void)
{
    sda_low(); scl_high();
    sda_high();
}

static bool i2c_write_byte_sw(uint8_t byte)
{
    for (int i = 0; i < 8; ++i) {
        if (byte & 0x80) sda_high(); else sda_low();
        scl_high();
        scl_low();
        byte <<= 1;
    }

    sda_high(); /* release for ACK */
    scl_high();
    bool ack = (sda_read() == 0);
    scl_low();
    return ack;
}

static uint8_t i2c_read_byte_sw(bool ack)
{
    uint8_t val = 0;
    sda_high();
    for (int i = 0; i < 8; ++i) {
        val <<= 1;
        scl_high();
        if (sda_read()) val |= 1u;
        scl_low();
    }
    if (ack) sda_low(); else sda_high();
    scl_high();
    scl_low();
    sda_high();
    return val;
}

static bool pcf8563_i2c_write(uint8_t reg, const uint8_t *data, uint8_t len)
{
    if (!data && len) return false;

    i2c_start_sw();
    if (!i2c_write_byte_sw((PCF8563_I2C_ADDR_7BIT << 1) | 0x00)) { i2c_stop_sw(); return false; }
    if (!i2c_write_byte_sw(reg)) { i2c_stop_sw(); return false; }

    for (uint8_t i = 0; i < len; ++i) {
        if (!i2c_write_byte_sw(data[i])) { i2c_stop_sw(); return false; }
    }

    i2c_stop_sw();
    return true;
}

static bool pcf8563_i2c_read(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (!data || !len) return false;

    i2c_start_sw();
    if (!i2c_write_byte_sw((PCF8563_I2C_ADDR_7BIT << 1) | 0x00)) { i2c_stop_sw(); return false; }
    if (!i2c_write_byte_sw(reg)) { i2c_stop_sw(); return false; }

    i2c_start_sw();
    if (!i2c_write_byte_sw((PCF8563_I2C_ADDR_7BIT << 1) | 0x01)) { i2c_stop_sw(); return false; }

    for (uint8_t i = 0; i < len; ++i) {
        data[i] = i2c_read_byte_sw(i < (len - 1));
    }

    i2c_stop_sw();
    return true;
}

static void pcf8563_clear_flags(void)
{
    uint8_t ctrl2;
    if (pcf8563_i2c_read(REG_CTRL2, &ctrl2, 1)) {
        ctrl2 &= (uint8_t)~(CTRL2_AF | CTRL2_TF | CTRL2_AIE | CTRL2_TIE);
        (void)pcf8563_i2c_write(REG_CTRL2, &ctrl2, 1);
    }
}

bool pcf8563_init(void)
{
    i2c_gpio_init_sw();

    uint8_t ctrl1 = 0x00;
    if (!pcf8563_i2c_write(REG_CTRL1, &ctrl1, 1)) {
        return false;
    }

    pcf8563_clear_flags();

    /* Disable CLKOUT and countdown timer outputs (not used) */
    const uint8_t clkout_disable = 0x00;
    (void)pcf8563_i2c_write(REG_CLKOUT, &clkout_disable, 1);
    const uint8_t timer_stop = 0x00;
    (void)pcf8563_i2c_write(REG_TIMER_CTRL, &timer_stop, 1);
    (void)pcf8563_i2c_write(REG_TIMER_VAL, &timer_stop, 1);

    return true;
}

bool pcf8563_get_datetime(pcf8563_datetime_t *dt)
{
    if (!dt) return false;

    uint8_t raw[7];
    if (!pcf8563_i2c_read(REG_SECONDS, raw, sizeof(raw))) {
        return false;
    }

    bool vl = (raw[0] & 0x80u) != 0;
    bool valid = !vl;

    uint8_t seconds = bin_from_bcd(raw[0] & 0x7Fu);
    if (seconds > 59u) { valid = false; seconds = clamp_u8(seconds, 0, 59); }

    uint8_t minutes = bin_from_bcd(raw[1] & 0x7Fu);
    if (minutes > 59u) { valid = false; minutes = clamp_u8(minutes, 0, 59); }

    uint8_t hours = bin_from_bcd(raw[2] & 0x3Fu);
    if (hours > 23u) { valid = false; hours = clamp_u8(hours, 0, 23); }

    uint8_t day = bin_from_bcd(raw[3] & 0x3Fu);
    if (day < 1u || day > 31u) { valid = false; day = clamp_u8(day, 1, 31); }

    uint8_t weekday = (uint8_t)(raw[4] & 0x07u);
    if (weekday > 6u) { valid = false; weekday %= 7u; }

    uint8_t month = bin_from_bcd(raw[5] & 0x1Fu);
    if (month < 1u || month > 12u) { valid = false; month = clamp_u8(month, 1, 12); }

    uint16_t base_year = (raw[5] & 0x80u) ? 1900u : 2000u;
    uint16_t year = (uint16_t)(base_year + bin_from_bcd(raw[6]));
    if (year < 2000u || year > 2099u) {
        valid = false;
        year = (uint16_t)clamp_u8((uint8_t)(year >= 2000u ? (year - 2000u) : 0u), 0u, 99u);
        year = (uint16_t)(2000u + year);
    }

    dt->seconds = seconds;
    dt->minutes = minutes;
    dt->hours   = hours;
    dt->day     = day;
    dt->weekday = weekday;
    dt->month   = month;
    dt->year    = year;
    dt->valid   = valid;

    return true;
}

bool pcf8563_set_datetime(const pcf8563_datetime_t *dt)
{
    if (!dt) return false;

    uint16_t year = dt->year;
    if (year < 2000u) year = 2000u;
    if (year > 2099u) year = 2099u;

    uint8_t month = clamp_u8(dt->month, 1, 12);
    uint8_t day   = clamp_u8(dt->day, 1, 31);
    uint8_t weekday = calc_weekday(year, month, day);

    uint8_t raw[7];
    raw[0] = (uint8_t)(bcd_from_bin(clamp_u8(dt->seconds, 0, 59)) & 0x7Fu);
    raw[1] = (uint8_t)(bcd_from_bin(clamp_u8(dt->minutes, 0, 59)) & 0x7Fu);
    raw[2] = (uint8_t)(bcd_from_bin(clamp_u8(dt->hours,   0, 23)) & 0x3Fu);
    raw[3] = (uint8_t)(bcd_from_bin(day) & 0x3Fu);
    raw[4] = (uint8_t)(weekday & 0x07u);
    raw[5] = (uint8_t)(bcd_from_bin(month) & 0x1Fu); /* century bit cleared for 2000-2099 */
    raw[6] = bcd_from_bin((uint8_t)(year - 2000u));

    uint8_t ctrl1;
    if (!pcf8563_i2c_read(REG_CTRL1, &ctrl1, 1)) {
        return false;
    }

    uint8_t stop = (uint8_t)(ctrl1 | CTRL1_STOP);
    if (!pcf8563_i2c_write(REG_CTRL1, &stop, 1)) return false;

    if (!pcf8563_i2c_write(REG_SECONDS, raw, sizeof(raw))) return false;

    ctrl1 &= (uint8_t)~CTRL1_STOP;
    if (!pcf8563_i2c_write(REG_CTRL1, &ctrl1, 1)) return false;

    return true;
}

/* -------------------------------------------------------------------------- */
/* Unified RTC interface                                                      */
/* -------------------------------------------------------------------------- */

static bool s_app_rtc_initialized = false;
static bool s_app_rtc_last_valid = false;

static void convert_to_rtc(const pcf8563_datetime_t *src, rtc_datetime_t *dest)
{
    dest->year   = src->year;
    dest->month  = src->month;
    dest->day    = src->day;
    dest->hour   = src->hours;
    dest->minute = src->minutes;
    dest->second = src->seconds;
}

static void convert_from_rtc(const rtc_datetime_t *src, pcf8563_datetime_t *dest)
{
    dest->year    = src->year;
    dest->month   = src->month;
    dest->day     = src->day;
    dest->weekday = calc_weekday(src->year, src->month, src->day);
    dest->hours   = src->hour;
    dest->minutes = src->minute;
    dest->seconds = src->second;
    dest->valid   = true;
}

bool app_rtc_init(void)
{
    if (!pcf8563_init()) {
        s_app_rtc_initialized = false;
        s_app_rtc_last_valid = false;
        return false;
    }

    pcf8563_datetime_t dt;
    s_app_rtc_last_valid = pcf8563_get_datetime(&dt) && dt.valid;
    s_app_rtc_initialized = true;
    return s_app_rtc_last_valid;
}

bool app_rtc_has_valid_time(void)
{
    return s_app_rtc_initialized && s_app_rtc_last_valid;
}

bool app_rtc_get_datetime(rtc_datetime_t *dt)
{
    if (!dt || !s_app_rtc_initialized) {
        return false;
    }

    pcf8563_datetime_t raw;
    if (!pcf8563_get_datetime(&raw)) {
        return false;
    }

    s_app_rtc_last_valid = raw.valid;
    
    /* 只有当时间有效时才转换数据，避免返回2000年等无效时间 */
    if (raw.valid) {
        convert_to_rtc(&raw, dt);
        return true;
    }
    
    return false;
}

bool app_rtc_set_datetime(const rtc_datetime_t *dt)
{
    if (!dt || !s_app_rtc_initialized) {
        return false;
    }

    pcf8563_datetime_t raw;
    convert_from_rtc(dt, &raw);

    if (!pcf8563_set_datetime(&raw)) {
        return false;
    }

    s_app_rtc_last_valid = true;
    return true;
}
