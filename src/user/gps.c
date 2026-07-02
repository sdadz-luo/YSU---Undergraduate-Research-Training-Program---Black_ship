#include "gps.h"
#include <string.h>
#include <stdlib.h>

double gps_lat = 0.0;
double gps_lon = 0.0;
bool   gps_valid = false;

/* NMEA 格式 (DDMM.MMMM) 转十进制 (DD.DDDD) */
static double nmea_to_decimal(const char *str)
{
    double raw = atof(str);
    int deg = (int)(raw / 100);
    double min = raw - deg * 100;
    return deg + min / 60.0;
}

/* 解析 $GPRMC / $GNRMC 语句 */
void GPS_Parse(char *sentence)
{
    /* 只处理 RMC 语句 */
    if (sentence[0] != '$') return;
    if (!(sentence[3] == 'R' && sentence[4] == 'M' && sentence[5] == 'C'))
        return;

    char *p = sentence;
    int field = 0;
    char *lat_str = NULL, *lon_str = NULL;
    char ns = 0, ew = 0;

    while (*p && field <= 6)
    {
        if (*p == ',')
        {
            field++;
            p++;
            switch (field)
            {
                case 2: /* 定位状态 A=有效 V=无效 */
                    if (*p == 'A') gps_valid = true;
                    else           gps_valid = false;
                    break;
                case 3: /* 纬度 */
                    lat_str = p;
                    break;
                case 4: /* N/S */
                    ns = *p;
                    break;
                case 5: /* 经度 */
                    lon_str = p;
                    break;
                case 6: /* E/W */
                    ew = *p;
                    break;
            }
            /* 跳过当前字段内容到下一个逗号 */
            while (*p && *p != ',') p++;
        }
        else p++;
    }

    if (gps_valid && lat_str && lon_str)
    {
        gps_lat = nmea_to_decimal(lat_str);
        gps_lon = nmea_to_decimal(lon_str);
        if (ns == 'S') gps_lat = -gps_lat;
        if (ew == 'W') gps_lon = -gps_lon;
    }
}
