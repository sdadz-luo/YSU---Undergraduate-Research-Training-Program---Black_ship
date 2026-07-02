#include "hal_data.h"
#include "headfile.h"

/* LoRa 控制变量：v=速度(0-10), move=方向(0-3) */
uint8_t move_flag = 0, v = 0, move = 0;

#if (1 == BSP_MULTICORE_PROJECT) && BSP_TZ_SECURE_BUILD
bsp_ipc_semaphore_handle_t g_core_start_semaphore = { .semaphore_num = 0 };
#endif

void hal_entry(void)
{
    /* ========== 外设初始化 ========== */
    UART5_IMU_Init();
    UART2_LoRa_Init();
    UART9_GPS_Init();
    DMAC_Init();
    pwm_init();
    pid_init();
    gpt0_init();

    /* ========== 主循环 ========== */
    while (1)
    {
        /* GPS 数据解析（NMEA 协议） */
        if (gps_rx_complete)
        {
            // TODO: 解析 gps_rx_buf[] 的 NMEA 字符串
        }
    }

    /* 多核 / TrustZone 启动 */
#if (0 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && !BSP_TZ_NONSECURE_BUILD
    #if BSP_TZ_SECURE_BUILD
    R_BSP_IpcSemaphoreTake(&g_core_start_semaphore);
    #endif
    R_BSP_SecondaryCoreStart();
    #if BSP_TZ_SECURE_BUILD
    while (FSP_ERR_IN_USE == R_BSP_IpcSemaphoreTake(&g_core_start_semaphore));
    #endif
#endif
#if (1 == _RA_CORE) && (1 == BSP_MULTICORE_PROJECT) && BSP_TZ_SECURE_BUILD
    R_BSP_IpcSemaphoreGive(&g_core_start_semaphore);
#endif
#if BSP_TZ_SECURE_BUILD
    R_BSP_NonSecureEnter();
#endif
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif
