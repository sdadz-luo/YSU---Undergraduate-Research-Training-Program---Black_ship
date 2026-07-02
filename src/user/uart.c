#include <stdio.h>
#include <string.h>
#include "headfile.h"
#include "uart.h"

/* 引用于 hal_entry.c 的控制变量 */
extern uint8_t v, move, move_flag;

/* printf 发送完成标志 */
static volatile bool uart_send_complete_flag = false;

/* =========================================================================
 *  1. IMU (JY901B) - SCI5/UART5 + DMAC0
 *     DMAC 自动搬运数据，回调中复位 DMAC
 * ========================================================================= */
volatile uint8_t imu_rx_buf[IMU_RX_BUF_SIZE];
volatile bool imu_rx_complete = false;

void imu_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_TX_COMPLETE:
            uart_send_complete_flag = true;
            break;
        default:
            break;
    }
}

void UART5_IMU_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart5_ctrl, &g_uart5_cfg);
    assert(FSP_SUCCESS == err);
}

/* =========================================================================
 *  1.5 N10 雷达 - SCI3/UART3 + DMAC4
 *     58 字节 DMAC 接收，回调中解析为 18 个距离值
 * ========================================================================= */

/* DMAC 辅助函数前向声明 */
void set_transfer_length(transfer_cfg_t const * const p_config, volatile uint16_t _length);
void set_transfer_dst_src_address(transfer_cfg_t const * const p_config,
                                   const volatile uint8_t * _p_src,
                                   const volatile uint8_t * _p_dest);

volatile uint8_t  n10_rx_buf[N10_RX_BUF_SIZE];
volatile bool     n10_rx_complete = false;
volatile int      n10_data[N10_DATA_NUM];

/** UART3 回调 - 仅 TX_COMPLETE */
void N10_callback(uart_callback_args_t *p_args)
{
    (void)p_args;
}

/** DMAC4 回调 - DMAC 传输完成，解析 N10 数据 */
void transfer_N10_rx_callback(transfer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

    /* 解析 N10 雷达 58 字节 → 18 个距离值 */
    {
        uint16_t tmp;
        tmp = (uint16_t)(((uint16_t)n10_rx_buf[5] << 8) | n10_rx_buf[6]);
        n10_data[0] = (int)((float)tmp / 100.0f + 0.5f);
        tmp = (uint16_t)(((uint16_t)n10_rx_buf[55] << 8) | n10_rx_buf[56]);
        n10_data[17] = (int)((float)tmp / 100.0f + 0.5f);
    }
    for (int i = 0; i <= 15; i++)
    {
        n10_data[i + 1] = (int)(((uint16_t)n10_rx_buf[3 * i + 7] << 8) + n10_rx_buf[3 * i + 8]);
    }

    n10_rx_complete = true;

    /* 复位 DMAC */
    (void)g_transfer_on_dmac.open(&g_transfer4_ctrl, &g_transfer4_cfg);
    (void)g_transfer_on_dmac.enable(&g_transfer4_ctrl);
}

void UART3_N10_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart3_ctrl, &g_uart3_cfg);
    assert(FSP_SUCCESS == err);

    /* 清除 IELSR（仅 RXI） */
    R_ICU->IELSR[SCI3_RXI_IRQn] = 0U;
}

void DMAC4_N10_Init(void)
{
    fsp_err_t err;

    set_transfer_length(&g_transfer4_cfg, N10_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer4_cfg,
            (const volatile uint8_t *)&R_SCI3->RDR,
            (const volatile uint8_t *)n10_rx_buf);
    err = g_transfer_on_dmac.open(&g_transfer4_ctrl, &g_transfer4_cfg);
    assert(FSP_SUCCESS == err);
    err = g_transfer_on_dmac.enable(&g_transfer4_ctrl);
    assert(FSP_SUCCESS == err);
}

/* =========================================================================
 *  2. LoRa 无线 - SCI2/UART2（中断接收）
 *     状态机解析协议：EE=屏包(速度/开关), CC=摇杆包(方向)
 * ========================================================================= */
volatile uint8_t lora_rx_buf[LORA_RX_BUF_SIZE];
volatile bool lora_rx_complete = false;

/* CRC8 校验 */
#define CRC8_POLY  0x31
static uint8_t calc_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ CRC8_POLY);
            else            crc <<= 1;
        }
    }
    return crc;
}

/* 包解析状态机 */
#define LORA_PKT_MAX    8
static uint8_t  lora_pkt[LORA_PKT_MAX];
static uint8_t  lora_pkt_idx = 0;
static bool     lora_pkt_start = false;

void lora_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            uint8_t ch = (uint8_t)p_args->data;

            /* 检测帧头启动新包 */
            if (!lora_pkt_start && (ch == 0xEE || ch == 0xCC))
            {
                lora_pkt_start = true;
                lora_pkt_idx = 0;
            }

            if (!lora_pkt_start) break;

            lora_pkt[lora_pkt_idx++] = ch;

            /* --- 屏包 (6B): EE 02 07 速度 CRC8 FF --- */
            if (lora_pkt_idx == 6 && lora_pkt[0] == 0xEE
                && lora_pkt[1] == 0x02 && lora_pkt[5] == 0xFF)
            {
                if (calc_crc8(lora_pkt, 4) == lora_pkt[4])
                {
                    if (lora_pkt[2] == CMD_SPEED)   v = lora_pkt[3];
                    if (lora_pkt[2] == CMD_SWITCH)  move_flag = lora_pkt[3];
                }
                lora_pkt_start = false;
            }
            /* --- 摇杆包 (7B): CC 01 白方向 02 黑方向 CRC8 33 --- */
            else if (lora_pkt_idx == 7 && lora_pkt[0] == 0xCC
                     && lora_pkt[3] == 0x02 && lora_pkt[6] == 0x33)
            {
                if (calc_crc8(lora_pkt, 5) == lora_pkt[5])
                    move = lora_pkt[4];
                lora_pkt_start = false;
            }
            /* 超长保护 */
            else if (lora_pkt_idx >= LORA_PKT_MAX)
                lora_pkt_start = false;

            break;
        }
        case UART_EVENT_TX_COMPLETE:
            uart_send_complete_flag = true;
            break;
        default:
            break;
    }
}

void UART2_LoRa_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart2_ctrl, &g_uart2_cfg);
    assert(FSP_SUCCESS == err);
}

/* =========================================================================
 *  3. GPS (SCI9/UART9) — 中断接收
 * ========================================================================= */
char buf[GPS_BUF_LEN];
/** GPS 回调 - 积累 NMEA 语句，遇 \n 解析 */
void gps_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            static uint16_t idx = 0;
         
            char ch = (char)p_args->data;
            if (idx < GPS_BUF_LEN - 1) buf[idx++] = ch;
            if (ch == '\n')
            {
                buf[idx] = '\0';
                GPS_Parse(buf);
                idx = 0;
            }
            break;
        }
        case UART_EVENT_TX_COMPLETE:
            uart_send_complete_flag = true;
            break;
        default:
            break;
    }
}

void UART9_GPS_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart9_ctrl, &g_uart9_cfg);
    assert(FSP_SUCCESS == err);
}

/* =========================================================================
 *  4. 4G 发送 - SCI8/UART8（中断发送）
 *     4 秒定时发送 GPS + N10 JSON 数据
 * ========================================================================= */
volatile bool uart8_tx_complete = false;

void G_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_TX_COMPLETE)
        uart8_tx_complete = true;
}

/** DMAC2 TX 回调 - SCI8 发送完成 */
void transfer_4G_tx_callback(transfer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    /* DMAC 传输完成，实际数据已被 UART 发送 */
}

void UART8_4G_Init(void)
{
    fsp_err_t err = R_SCI_UART_Open(&g_uart8_ctrl, &g_uart8_cfg);
    assert(FSP_SUCCESS == err);

    /* 清除 IELSR（仅 RXI，参考 FSP 示例） */
    R_ICU->IELSR[SCI8_RXI_IRQn] = 0U;
}

/** 发送 JSON 字符串到 4G */
void UART8_4G_Send(const char *str)
{
    uart8_tx_complete = false;
    fsp_err_t err = R_SCI_UART_Write(&g_uart8_ctrl, (const uint8_t *)str, strlen(str));
    if (FSP_SUCCESS != err) __BKPT();
    while (!uart8_tx_complete) {}
}

/* =========================================================================
 *  5. DMAC 辅助函数
 * ========================================================================= */

/* DMAC2 初始化（4G TX，预留，当前未启用） */
void DMAC2_4G_Init(void)
{
    fsp_err_t err;
    err = g_transfer_on_dmac.open(&g_transfer2_ctrl, &g_transfer2_cfg);
    assert(FSP_SUCCESS == err);
    err = g_transfer_on_dmac.enable(&g_transfer2_ctrl);
    assert(FSP_SUCCESS == err);
}

void set_transfer_length(transfer_cfg_t const * const p_config, volatile uint16_t _length)
{
    p_config->p_info->length = _length;
}

void set_transfer_dst_src_address(transfer_cfg_t const * const p_config,
                                   const volatile uint8_t * _p_src,
                                   const volatile uint8_t * _p_dest)
{
    p_config->p_info->p_src  = (void const * volatile)_p_src;
    p_config->p_info->p_dest = (void * volatile)_p_dest;
}

/* =========================================================================
 *  5. DMAC 初始化
 *     DMAC0=IMU(SCI5), DMAC2=LoRa(SCI2), DMAC4=GPS(SCI9)
 * ========================================================================= */
void DMAC_Init(void)
{
    fsp_err_t err;

    /* --- IMU: DMAC0, SCI5 RXI --- */
    set_transfer_length(&g_transfer0_cfg, IMU_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer0_cfg,
            (const volatile uint8_t *)&R_SCI5->RDR, (const volatile uint8_t *)imu_rx_buf);
    err = g_transfer_on_dmac.open(&g_transfer0_ctrl, &g_transfer0_cfg);
    assert(FSP_SUCCESS == err);
    err = g_transfer_on_dmac.enable(&g_transfer0_ctrl);
    assert(FSP_SUCCESS == err);
}

/** IMU: 传满 22 字节后自动复位 DMAC */
void transfer_imu_rx_callback(transfer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    imu_rx_complete = true;
    set_transfer_length(&g_transfer0_cfg, IMU_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer0_cfg,
            (const volatile uint8_t *)&R_SCI5->RDR, (const volatile uint8_t *)imu_rx_buf);
    (void)g_transfer_on_dmac.reconfigure(&g_transfer0_ctrl, g_transfer0_cfg.p_info);
}

/* =========================================================================
 *  DMAC 重置
 * ========================================================================= */
void IMU_DMAC_Reset(void)
{
    fsp_err_t err;
    imu_rx_complete = false;
    set_transfer_length(&g_transfer0_cfg, IMU_RX_BUF_SIZE);
    set_transfer_dst_src_address(&g_transfer0_cfg,
                                  (const volatile uint8_t *)&R_SCI5->RDR,
                                  (const volatile uint8_t *)imu_rx_buf);
    err = g_transfer_on_dmac.reconfigure(&g_transfer0_ctrl, g_transfer0_cfg.p_info);
    assert(FSP_SUCCESS == err);
}


