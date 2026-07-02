#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>
#include <stdbool.h>

/* NMEA 缓冲区大小 */
#define GPS_BUF_LEN     128

/* 解析后的经纬度（十进制） */
extern double gps_lat;
extern double gps_lon;
extern bool   gps_valid;

void GPS_Parse(char *sentence);

#endif
