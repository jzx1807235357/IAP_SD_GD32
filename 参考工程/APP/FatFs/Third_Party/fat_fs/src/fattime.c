

#include "integer.h"
#include "fattime.h"
#include "app_rtc.h"  /* 使用RTC获取真实时间 */

DWORD get_fattime (void)
{
    rtc_datetime_t rtc_time;
    
    /* 尝试从RTC获取时间 */
    if (app_rtc_has_valid_time() && app_rtc_get_datetime(&rtc_time)) {
        return    ((DWORD)(rtc_time.year - 1980) << 25)
                | ((DWORD)rtc_time.month << 21)
                | ((DWORD)rtc_time.day << 16)
                | ((DWORD)rtc_time.hour << 11)
                | ((DWORD)rtc_time.minute << 5)
                | ((DWORD)rtc_time.second >> 1);
    }
    
    /* RTC无效时返回默认时间：2025-01-01 00:00:00 */
    return    ((DWORD)(2025 - 1980) << 25)
            | ((DWORD)1 << 21)
            | ((DWORD)1 << 16)
            | ((DWORD)0 << 11)
            | ((DWORD)0 << 5)
            | ((DWORD)0 >> 1);
}

