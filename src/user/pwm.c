#include "headfile.h"
#include "pwm.h"

#define PWM_MAX     8000

void pwm_init(void)
{
    R_GPT_Open(&g_timer6_ctrl, &g_timer6_cfg);
    R_GPT_Start(&g_timer6_ctrl);
    R_GPT_Open(&g_timer7_ctrl, &g_timer7_cfg);
    R_GPT_Start(&g_timer7_ctrl);
}

/**
 * ?? PWM ????????
 * @param left   ?? (?=??, ?=??)
 * @param right  ?? (?=??, ?=??)
 *
 * GPT6 GTIOCA=????  GTIOCB=????
 * GPT7 GTIOCA=????  GTIOCB=????
 */
void pwm_setduty(float left, float right)
{
    float fwd, rev;

    /* ======= ?? ======= */
    if (left >= 0) { fwd =  left; rev = 0; }
    else           { fwd = 0;     rev = -left; }
    if (fwd > PWM_MAX) fwd = PWM_MAX;
    if (rev > PWM_MAX) rev = PWM_MAX;
    R_GPT_DutyCycleSet(&g_timer6_ctrl, (uint32_t)fwd, GPT_IO_PIN_GTIOCA);
    R_GPT_DutyCycleSet(&g_timer6_ctrl, (uint32_t)rev, GPT_IO_PIN_GTIOCB);

    /* ======= ?? ======= */
    if (right >= 0) { fwd =  right; rev = 0; }
    else            { fwd = 0;      rev = -right; }
    if (fwd > PWM_MAX) fwd = PWM_MAX;
    if (rev > PWM_MAX) rev = PWM_MAX;
    R_GPT_DutyCycleSet(&g_timer7_ctrl, (uint32_t)fwd, GPT_IO_PIN_GTIOCA);
    R_GPT_DutyCycleSet(&g_timer7_ctrl, (uint32_t)rev, GPT_IO_PIN_GTIOCB);
}
