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
    UART3_N10_Init();
    UART8_4G_Init();
    DMAC_Init();
    DMAC4_N10_Init();
    pwm_init();
    pid_init();
    gpt0_init();
    gpt1_init();

    /* ========== 主循环 ========== */
    while (1)
    {
        if (gpt1_flag)
        {
            gpt1_flag = 0;
            gpt1_send_4g();
        }
    }

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
