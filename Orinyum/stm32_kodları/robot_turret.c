/* ===========================================================================
 * robot_turret.c  –  Taret ve Nişan Sistemi
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#include "robot_turret.h"
#include <math.h>

/* ---------------------------------------------------------------------------
 * Global değişken tanımı
 * -------------------------------------------------------------------------*/
TurretState_t turret_state = TURRET_SEARCHING;

/* ---------------------------------------------------------------------------
 * Modül-içi statik değişkenler
 * -------------------------------------------------------------------------*/
/* Kp, Ki, Kd, error_sum, last_error */
static Turret_PID_t pid_pitch = {2.0f, 0.05f, 0.10f, 0.0f, 0.0f};
static Turret_PID_t pid_yaw   = {2.0f, 0.05f, 0.10f, 0.0f, 0.0f};

static uint32_t lock_start_ms = 0U;

/* ---------------------------------------------------------------------------
 * turret_pid_step  (yerel yardımcı)
 *   Anti-windup kırpmalı tek eksen PID adımı.
 * -------------------------------------------------------------------------*/
static float turret_pid_step(Turret_PID_t *pid, float error)
{
    pid->error_sum += error * TURRET_PID_DT;

    /* İntegral doyma koruması */
    if (pid->error_sum >  300.0f) { pid->error_sum =  300.0f; }
    if (pid->error_sum < -300.0f) { pid->error_sum = -300.0f; }

    float deriv  = (error - pid->last_error) / TURRET_PID_DT;
    float output = (pid->Kp * error)
                 + (pid->Ki * pid->error_sum)
                 + (pid->Kd * deriv);

    pid->last_error = error;

    /* PWM sınırlama */
    if (output >  (float)TURRET_PWM_LIMIT) { output =  (float)TURRET_PWM_LIMIT; }
    if (output < -(float)TURRET_PWM_LIMIT) { output = -(float)TURRET_PWM_LIMIT; }

    return output;
}

/* ---------------------------------------------------------------------------
 * Turret_Init
 * -------------------------------------------------------------------------*/
void Turret_Init(void)
{
    turret_state         = TURRET_SEARCHING;
    pid_pitch.error_sum  = 0.0f;
    pid_pitch.last_error = 0.0f;
    pid_yaw.error_sum    = 0.0f;
    pid_yaw.last_error   = 0.0f;
    lock_start_ms        = 0U;
}

/* ---------------------------------------------------------------------------
 * Turret_Update
 *   FSM:
 *   SEARCHING ──(hedef görüldü)──→ TRACKING
 *   TRACKING  ──(1 s kilit)──────→ LOCKED → FIRING
 *   FIRING    ──(hedef kayboldu)──→ SEARCHING
 * -------------------------------------------------------------------------*/
void Turret_Update(float pixel_err_pitch, float pixel_err_yaw,
                   TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                   TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                   TIM_HandleTypeDef *htim_laser, uint32_t ch_laser)
{
    bool on_target = (fabsf(pixel_err_pitch) < TURRET_LOCK_THRESH) &&
                     (fabsf(pixel_err_yaw)   < TURRET_LOCK_THRESH);

    switch (turret_state)
    {
    /* ------------------------------------------------------------------
     * SEARCHING / TRACKING: servo hizala, lazer kapalı
     * ----------------------------------------------------------------*/
    case TURRET_SEARCHING:
    case TURRET_TRACKING:
    {
        float out_pitch = turret_pid_step(&pid_pitch, pixel_err_pitch);
        float out_yaw   = turret_pid_step(&pid_yaw,   pixel_err_yaw);

        __HAL_TIM_SET_COMPARE(htim_pitch, ch_pitch,
                              (uint32_t)fabsf(out_pitch));
        __HAL_TIM_SET_COMPARE(htim_yaw,   ch_yaw,
                              (uint32_t)fabsf(out_yaw));
        __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0U);

        if (on_target)
        {
            if (turret_state == TURRET_SEARCHING)
            {
                /* Hedefi ilk gördük: zamanlayıcıyı başlat */
                lock_start_ms = HAL_GetTick();
                turret_state  = TURRET_TRACKING;
            }
            else
            {
                /* TRACKING: kilit süresi doldu mu? */
                if ((HAL_GetTick() - lock_start_ms) >= TURRET_LOCK_TIME_MS)
                {
                    turret_state = TURRET_LOCKED;
                }
            }
        }
        else
        {
            /* Hedef kayboldu: başa dön */
            lock_start_ms = 0U;
            turret_state  = TURRET_SEARCHING;
        }
        break;
    }

    /* ------------------------------------------------------------------
     * LOCKED: tek adım geçiş → FIRING
     * ----------------------------------------------------------------*/
    case TURRET_LOCKED:
        turret_state = TURRET_FIRING;
        /* fall-through: aynı döngüde FIRING işlemini hemen uygula */
        __attribute__((fallthrough));

    /* ------------------------------------------------------------------
     * FIRING: lazer aktif, servo ince ayar devam eder
     * ----------------------------------------------------------------*/
    case TURRET_FIRING:
    {
        if (on_target)
        {
            float out_pitch = turret_pid_step(&pid_pitch, pixel_err_pitch);
            float out_yaw   = turret_pid_step(&pid_yaw,   pixel_err_yaw);

            __HAL_TIM_SET_COMPARE(htim_pitch, ch_pitch,
                                  (uint32_t)fabsf(out_pitch));
            __HAL_TIM_SET_COMPARE(htim_yaw,   ch_yaw,
                                  (uint32_t)fabsf(out_yaw));
            __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, TURRET_PWM_LIMIT);
        }
        else
        {
            /* Hedef kaçtı: lazeri söndür, sıfırla */
            __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0U);
            Turret_Init();   /* state = SEARCHING, PID sıfır */
        }
        break;
    }

    default:
        turret_state = TURRET_SEARCHING;
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Turret_Emergency_Stop
 * -------------------------------------------------------------------------*/
void Turret_Emergency_Stop(TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                            TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                            TIM_HandleTypeDef *htim_laser, uint32_t ch_laser)
{
    __HAL_TIM_SET_COMPARE(htim_pitch, ch_pitch, 0U);
    __HAL_TIM_SET_COMPARE(htim_yaw,   ch_yaw,   0U);
    __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0U);
    turret_state = TURRET_SEARCHING;
}
