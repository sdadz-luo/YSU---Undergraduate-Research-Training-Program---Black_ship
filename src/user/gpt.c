#include "gpt.h"
#include "headfile.h"
#include <stdio.h>
#include <string.h>

/* =========================================================================
 *  全局变量
 * ========================================================================= */
float gyro, yaw;                /* IMU 原始数据 */
float pwm_l, pwm_r, pwm_turn;   /* PWM 中间变量 */
extern pid pid_gyro, pid_yaw;
extern uint8_t v, move, move_flag;

/* GPS 数据 */
extern double gps_lat, gps_lon;
extern bool   gps_valid;

/* N10 雷达数据 */
extern volatile int n10_data[N10_DATA_NUM];

/* P500 1s 保持计数器 (在 uart.c lora_callback 中置 200) */
extern volatile uint16_t p500_hold_count;

/* =========================================================================
 *  4G 上传
 * ========================================================================= */
volatile uint8_t gpt1_send = 0;  /* 0=空闲, 1=发雷达, 2=发GPS */
static char json_buf[512];       /* JSON 格式化缓冲区 */

#define JSON_HEAD  "{\"id\":\"123\",\"version\":\"1.0\",\"params\":{"
#define JSON_TAIL  "}}"

/* =========================================================================
 *  GPT0 - 5ms PID 控制
 * ========================================================================= */

void gpt0_init(void)
{
    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    R_GPT_Start(&g_timer0_ctrl);
}

void gpt0_callback(timer_callback_args_t *p_args)
{
    (void)p_args;

    /* ---- IO 输出控制 ---- */
    /* P015: move_flag==1(摇杆模式) 输出高电平, move_flag==0(PID循线) 输出低电平 */
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_00_PIN_15,
                      (bsp_io_level_t)(move_flag ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW));

    /* P500: LoRa CMD_P500 触发后保持 1s (200 x 5ms) 高电平 */
    if (p500_hold_count > 0)
    {
        R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_05_PIN_00, BSP_IO_LEVEL_HIGH);
        p500_hold_count--;
        if (p500_hold_count == 0)
            R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_05_PIN_00, BSP_IO_LEVEL_LOW);
    }

    if (imu_rx_complete)
    {
        int16_t gyro_raw = (int16_t)((imu_rx_buf[7] << 8) | imu_rx_buf[6]);
        int16_t yaw_raw  = (int16_t)((imu_rx_buf[18] << 8) | imu_rx_buf[17]);
        gyro = (float)gyro_raw / 32768.0f * 2000.0f;
        yaw  = (float)yaw_raw  / 32768.0f * 180.0f;
        imu_rx_complete = false;
    }

    /* ---- PID 计算 ---- */
    pwm_turn = pid_location(&pid_gyro, gyro);

    if (move_flag)
    {
        /* 摇杆模式: 基础速度 v + PID 转向 */
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
        /* PID 循线模式: 陀螺仪 PID 差速转向前进 */
        pwm_setduty(-pwm_turn, pwm_turn);
    }
}

/* =========================================================================
 *  GPT1 - 1 秒定时器, 4G 上传调度 (GPS/N10)
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
		p += sprintf(p, "]}" JSON_TAIL);
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

/** GPT1 回调函数: 前 10s 注册等待, 之后雷达/GPS 轮流发送 */
void gpt1_callback(timer_callback_args_t *p_args)
{
    (void)p_args;
    static uint8_t startup = 10;  /* 10 个 1s = 10s 启动等待 */
    static uint8_t tick = 0;      /* 0=发雷达, 1=发GPS, 2=空闲 */

    if (startup)
    {
        startup--;
        return;
    }

    if (tick == 0)
        gpt1_send = 2;   /* 发 GPS */
    else if (tick == 2)
        gpt1_send = 1;   /* 发雷达（距 GPS 1s 间隔） */
    /* tick == 3: 空闲等待 2s */

    tick++;
    if (tick > 4) tick = 0;
}
