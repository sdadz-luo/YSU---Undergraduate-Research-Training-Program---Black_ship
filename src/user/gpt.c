#include "gpt.h"
#include "headfile.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 *  全局控制变量
 * ========================================================================= */
float gyro, yaw;                /* IMU 解析值 */
float pwm_l, pwm_r, pwm_turn;   /* PWM 计算结果 */
extern pid pid_gyro, pid_yaw;
extern uint8_t v, move, move_flag;

/* GPS 数据 */
extern double gps_lat, gps_lon;
extern bool   gps_valid;

/* N10 雷达数据 */
extern volatile int n10_data[N10_DATA_NUM];

/* =========================================================================
 *  4G 发送
 * ========================================================================= */
volatile uint8_t gpt1_send = 0;  /* 0=空闲, 1=发雷达, 2=发GPS */
static char json_buf[512];       /* JSON 格式化缓冲 */

#define JSON_HEAD  "{\"id\":\"123\",\"version\":\"1.0\",\"params\":{"
#define JSON_TAIL  "}}"

/* =========================================================================
 *  GPT0 — 5ms PID 控制
 * ========================================================================= */

void gpt0_init(void)
{
    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    R_GPT_Start(&g_timer0_ctrl);
}

void gpt0_callback(timer_callback_args_t *p_args)
{
    (void)p_args;

    /* ---- 读取最新 IMU 数据 ---- */
    if (imu_rx_complete)
    {
        int16_t gyro_raw = (int16_t)((imu_rx_buf[7] << 8) | imu_rx_buf[6]);
        int16_t yaw_raw  = (int16_t)((imu_rx_buf[18] << 8) | imu_rx_buf[17]);
        gyro = (float)gyro_raw / 32768.0f * 2000.0f;
        yaw  = (float)yaw_raw  / 32768.0f * 180.0f;
        imu_rx_complete = false;
    }

    /* ---- PID 自稳 ---- */
    pwm_turn = pid_location(&pid_gyro, gyro);

    if (move_flag)
    {
        /* 摇杆模式：速度 v + PID 转向 */
        pwm_l = 1000 * v;
        pwm_r = 1000 * v;
        switch (move)
        {
            case DIR_STOP:     pwm_l = 0; pwm_r = 0; break;
            case DIR_LEFT:     pwm_l = -pwm_l; pwm_turn = 0; break;
            case DIR_RIGHT:    pwm_r = -pwm_r; pwm_turn = 0; break;
            case DIR_BACKWARD: pwm_l = -pwm_l; pwm_r = -pwm_r; break;
            default: break;
        }
        pwm_setduty(pwm_l - pwm_turn, pwm_r + pwm_turn);
    }
    else
    {
        /* 纯自稳模式：仅 PID 修正，无前进速度 */
        pwm_setduty(-pwm_turn, pwm_turn);
    }
}

/* =========================================================================
 *  GPT1 — 1 秒定时上报 4G（交替发雷达和 GPS）
 * ========================================================================= */

void gpt1_init(void)
{
    R_GPT_Open(&g_timer1_ctrl, &g_timer1_cfg);
    R_GPT_Start(&g_timer1_ctrl);
}

/** 发送 N10 雷达数据 */
void send_n10(void)
{
    char *p = json_buf;
    p += sprintf(p, JSON_HEAD);
    p += sprintf(p, "\"N10\":{\"value\":[");
    for (int i = 0; i < 18; i++)
    {
        p += sprintf(p, "%d", n10_data[i]);
        if (i < 17) p += sprintf(p, ",");
    }
    sprintf(p, "]}" JSON_TAIL);
    UART8_4G_Send(json_buf);
}

/** 发送 GPS 经纬度 */
void send_gps(void)
{
    sprintf(json_buf, JSON_HEAD
            "\"black_lat\":{\"value\":%.6f},"
            "\"black_lon\":{\"value\":%.6f}"
            JSON_TAIL,
            gps_lat, gps_lon);
    UART8_4G_Send(json_buf);
}

/** GPT1 溢出中断回调 — 前 28s 注网，之后雷达/GPS 按序发送 */
void gpt1_callback(timer_callback_args_t *p_args)
{
    (void)p_args;
    static uint8_t startup = 28;  /* 28 × 1s = 28s 启动延迟 */
    static uint8_t tick = 0;      /* 0=发雷达, 1=发GPS, 2=等待 */

    if (startup)
    {
        startup--;
        return;
    }

    if (tick == 0)
        gpt1_send = 2;   /* 发 GPS */
    else if (tick == 1)
        gpt1_send = 1;   /* 发雷达（距 GPS 1s） */
    /* tick == 2: 跳过，额外等 1s */

    tick++;
    if (tick > 2) tick = 0;
}