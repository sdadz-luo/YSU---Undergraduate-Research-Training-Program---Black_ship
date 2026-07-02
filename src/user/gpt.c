#include "gpt.h"
#include "headfile.h"
#include <stdio.h>
#include <string.h>

/* 控制变量 */
float gyro, yaw;                /* IMU 解析值 */
float pwm_l, pwm_r, pwm_turn;   /* PWM 计算结果 */
extern pid pid_gyro, pid_yaw;
extern uint8_t v, move, move_flag;

/* GPS 数据 */
extern double gps_lat, gps_lon;
extern bool   gps_valid;

/* N10 雷达数据 */
extern volatile int n10_data[N10_DATA_NUM];

/* 4G 发送相关 */
volatile uint8_t gpt1_flag = 0;  /* GPT1 回调置 1，主循环发 */
static char json_buf[300];       /* JSON 格式化缓冲 */

/* 5ms 定时器初始化 */
void gpt0_init(void)
{
    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    R_GPT_Start(&g_timer0_ctrl);
}

/* 5ms 定时回调：读取 IMU → PID 计算 → 输出 PWM */
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
 *  GPT1 — 4 秒定时：发送 GPS + N10 到 4G
 * ========================================================================= */

void gpt1_init(void)
{
    R_GPT_Open(&g_timer1_ctrl, &g_timer1_cfg);
    R_GPT_Start(&g_timer1_ctrl);
}

void gpt1_send_4g(void)
{
    /* 1. N10 雷达数据 */
    sprintf(json_buf, "{\"id\":\"123\",\"version\":\"1.0\",\"params\":{\"N10\":{\"value\":[");
    UART8_4G_Send(json_buf);
    for (int i = 0; i < 18; i++)
    {
        sprintf(json_buf, "%d", n10_data[i]);
        UART8_4G_Send(json_buf);
        if (i < 17) UART8_4G_Send(",");
    }
    UART8_4G_Send("]}}");

    /* 2. GPS 纬度 */
    sprintf(json_buf, "{\"id\":\"123\",\"version\":\"1.0\",\"params\":{\"black_lat\":{\"value\":%.6f}}}",
            gps_lat);
    UART8_4G_Send(json_buf);

    /* 3. GPS 经度 */
    sprintf(json_buf, "{\"id\":\"123\",\"version\":\"1.0\",\"params\":{\"black_lon\":{\"value\":%.6f}}}",
            gps_lon);
    UART8_4G_Send(json_buf);
}

void gpt1_callback(timer_callback_args_t *p_args)
{
    (void)p_args;
    gpt1_flag = 1;  /* 通知主循环发送 */
}