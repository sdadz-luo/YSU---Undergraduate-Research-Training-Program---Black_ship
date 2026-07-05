#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>

/* ===================== ��������С ===================== */
#define IMU_RX_BUF_SIZE     22      /* JY901B ��֡ 11x2 */
#define LORA_RX_BUF_SIZE    256     /* �жϽ��ջ��� */
#define N10_RX_BUF_SIZE     58      /* N10 �״�һ֡ */
#define N10_DATA_NUM        18      /* N10 �״� 18 ���� */
#define UART8_TX_BUF_SIZE   256     /* 4G ���ͻ��� */

/* ===================== ������ ===================== */
#define DIR_STOP        0
#define DIR_FORWARD     1
#define DIR_BACKWARD    2
#define DIR_LEFT        3
#define DIR_RIGHT       4

/* ===================== LoRa ������ ===================== */
#define CMD_LIGHT       0x01
#define CMD_PUMP        0x02
#define CMD_GIMBAL_UD   0x03
#define CMD_GIMBAL_LR   0x04
#define CMD_ARM_SERVO   0x05
#define CMD_ARM_DUTY    0x06
#define CMD_SPEED       0x07
#define CMD_SWITCH      0x08
#define CMD_P500        0x09    /* P500 ��� 1s */

/* ===================== P500 1s ���ּ��� (GPT0 5ms * 200=1000ms) ===================== */
extern volatile uint16_t    p500_hold_count;

/* ===================== ���ջ����� ===================== */
extern volatile uint8_t  imu_rx_buf[IMU_RX_BUF_SIZE];
extern volatile bool     imu_rx_complete;

extern volatile uint8_t  lora_rx_buf[LORA_RX_BUF_SIZE];
extern volatile bool     lora_rx_complete;

/* ===================== N10 �״� (SCI3 + DMAC4) ===================== */
extern volatile uint8_t  n10_rx_buf[N10_RX_BUF_SIZE]; /* DMAC ����ԭʼ 58 �ֽ� */
extern volatile bool     n10_rx_complete;              /* �����ݾ�����־ */
extern volatile int      n10_data[N10_DATA_NUM];       /* ������� 18 ������ֵ */

/* ===================== 4G ���� (SCI8���ж�ģʽ) ===================== */
extern volatile bool     uart8_tx_complete;            /* TX_COMPLETE ��־ */
void UART8_4G_Send(const char *str);                   /* ���������ַ��� */

/* ===================== �������� ===================== */
void UART5_IMU_Init(void);      /* IMU:    SCI5 + DMAC0 */
void UART2_LoRa_Init(void);     /* LoRa:   SCI2 �жϽ��� */
void UART9_GPS_Init(void);      /* GPS:    SCI9 �жϽ��� */
void UART3_N10_Init(void);      /* �״�:   SCI3 + DMAC4 */
void UART8_4G_Init(void);       /* 4G ��:  SCI8 �жϷ��� */
void DMAC_Init(void);           /* DMAC0 (IMU) */
void DMAC4_N10_Init(void);      /* DMAC4 (�״�) */
void DMAC_Init(void);
void DMAC4_N10_Init(void);
void DMAC2_4G_Init(void);
void IMU_DMAC_Reset(void);

#endif