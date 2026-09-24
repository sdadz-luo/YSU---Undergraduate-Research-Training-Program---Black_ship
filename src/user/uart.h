#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>

/* ===================== 缓冲区大小 ===================== */
#define IMU_RX_BUF_SIZE     22      /* JY901B 一帧 11x2 */
#define LORA_RX_BUF_SIZE    256     /* 中断接收缓冲区 */
#define N10_RX_BUF_SIZE     58      /* N10 雷达一帧 */
#define N10_DATA_NUM        18      /* N10 雷达 18 个数据 */
#define UART8_TX_BUF_SIZE   256     /* 4G 发送缓冲区 */

/* ===================== 方向定义 ===================== */
#define DIR_STOP        0
#define DIR_FORWARD     1
#define DIR_BACKWARD    2
#define DIR_LEFT        3
#define DIR_RIGHT       4

/* ===================== LoRa 命令定义 ===================== */
#define CMD_LIGHT       0x01
#define CMD_PUMP        0x02
#define CMD_GIMBAL_UD   0x03
#define CMD_GIMBAL_LR   0x04
#define CMD_ARM_SERVO   0x05
#define CMD_ARM_DUTY    0x06
#define CMD_SPEED       0x07
#define CMD_SWITCH      0x08
#define CMD_P500        0x09    /* P500 保持 1s */

/* ===================== P500 1s 保持计数器 (GPT0 5ms * 200=1000ms) ===================== */
extern volatile uint16_t    p500_hold_count;

/* ===================== 接收缓冲区 ===================== */
extern volatile uint8_t  imu_rx_buf[IMU_RX_BUF_SIZE];
extern volatile bool     imu_rx_complete;

extern volatile uint8_t  lora_rx_buf[LORA_RX_BUF_SIZE];
extern volatile bool     lora_rx_complete;

/* ===================== N10 雷达 (SCI3 + DMAC4) ===================== */
extern volatile uint8_t  n10_rx_buf[N10_RX_BUF_SIZE]; /* DMAC 接收原始 58 字节 */
extern volatile bool     n10_rx_complete;              /* 接收数据完成标志 */
extern volatile int      n10_data[N10_DATA_NUM];       /* 解析后的 18 个数据值 */

/* ===================== 4G 模块 (SCI8中断发送模式) ===================== */
extern volatile bool     uart8_tx_complete;            /* TX_COMPLETE 标志 */
void UART8_4G_Send(const char *str);                   /* 发送一个字符串 */

/* ===================== 初始化函数 ===================== */
void UART5_IMU_Init(void);      /* IMU:    SCI5 + DMAC0 */
void UART2_LoRa_Init(void);     /* LoRa:   SCI2 中断接收 */
void UART9_GPS_Init(void);      /* GPS:    SCI9 中断接收 */
void UART3_N10_Init(void);      /* 雷达:   SCI3 + DMAC4 */
void UART8_4G_Init(void);       /* 4G 模块:  SCI8 中断发送 */
void DMAC_Init(void);           /* DMAC0 (IMU) */
void DMAC4_N10_Init(void);      /* DMAC4 (雷达) */
void DMAC2_4G_Init(void);
void IMU_DMAC_Reset(void);

#endif
