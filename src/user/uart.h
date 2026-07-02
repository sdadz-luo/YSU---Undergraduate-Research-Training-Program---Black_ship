#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>

/* ===================== 缓冲区大小 ===================== */
#define IMU_RX_BUF_SIZE     22      /* JY901B 两帧 11x2 */
#define LORA_RX_BUF_SIZE    256     /* 中断接收缓冲 */
#define N10_RX_BUF_SIZE     58      /* N10 雷达一帧 */
#define N10_DATA_NUM        18      /* N10 雷达 18 个点 */
#define UART8_TX_BUF_SIZE   256     /* 4G 发送缓冲 */

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

/* ===================== N10 雷达 (SCI3 + DMAC4) ===================== */
extern volatile uint8_t  n10_rx_buf[N10_RX_BUF_SIZE]; /* DMAC 接收原始 58 字节 */
extern volatile bool     n10_rx_complete;              /* 新数据就绪标志 */
extern volatile int      n10_data[N10_DATA_NUM];       /* 解析后的 18 个距离值 */

/* ===================== 4G 发送 (SCI8，中断模式) ===================== */
extern volatile bool     uart8_tx_complete;            /* TX_COMPLETE 标志 */
void UART8_4G_Send(const char *str);                   /* 阻塞发送字符串 */

/* ===================== 函数声明 ===================== */
void UART5_IMU_Init(void);      /* IMU:    SCI5 + DMAC0 */
void UART2_LoRa_Init(void);     /* LoRa:   SCI2 中断接收 */
void UART9_GPS_Init(void);      /* GPS:    SCI9 中断接收 */
void UART3_N10_Init(void);      /* 雷达:   SCI3 + DMAC4 */
void UART8_4G_Init(void);       /* 4G 发:  SCI8 中断发送 */
void DMAC_Init(void);           /* DMAC0 (IMU) */
void DMAC4_N10_Init(void);      /* DMAC4 (雷达) */
void DMAC_Init(void);
void DMAC4_N10_Init(void);
void DMAC2_4G_Init(void);
void IMU_DMAC_Reset(void);

#endif