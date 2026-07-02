#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>

/* ===================== 缓冲区大小 ===================== */
#define IMU_RX_BUF_SIZE     22      /* JY901B 两帧 11x2 */
#define LORA_RX_BUF_SIZE    256     /* DMAC 单次传输长度 */
#define GPS_RX_BUF_SIZE     128     /* NMEA 语句最大长度 */

/* ===================== 方向定义 ===================== */
#define DIR_STOP        0
#define DIR_FORWARD     1
#define DIR_BACKWARD    2
#define DIR_LEFT        3
#define DIR_RIGHT       4

/* ===================== LoRa 组件编号 ===================== */
#define CMD_LIGHT       0x01
#define CMD_PUMP        0x02
#define CMD_GIMBAL_UD   0x03
#define CMD_GIMBAL_LR   0x04
#define CMD_ARM_SERVO   0x05
#define CMD_ARM_DUTY    0x06
#define CMD_SPEED       0x07
#define CMD_SWITCH      0x08

/* ===================== 接收缓冲区 ===================== */
extern volatile uint8_t  imu_rx_buf[IMU_RX_BUF_SIZE];
extern volatile bool     imu_rx_complete;

extern volatile uint8_t  lora_rx_buf[LORA_RX_BUF_SIZE];
extern volatile bool     lora_rx_complete;

extern volatile uint8_t  gps_rx_buf[GPS_RX_BUF_SIZE];
extern volatile bool     gps_rx_complete;

/* ===================== 函数声明 ===================== */
void UART5_IMU_Init(void);
void UART2_LoRa_Init(void);
void UART9_GPS_Init(void);
void DMAC_Init(void);
void IMU_DMAC_Reset(void);
void GPS_DMAC_Reset(void);

#endif