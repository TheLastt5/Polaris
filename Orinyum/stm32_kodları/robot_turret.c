/* ===========================================================================
 * robot_turret.c  –  Taret ve Nişan Sistemi
 * ===========================================================================*/
#include "robot_turret.h"
#include <math.h>

TurretState_t turret_state = TURRET_SEARCHING;

static Turret_PID_t pid_pitch = {2.0f, 0.05f, 0.10f, 0.0f, 0.0f};
static Turret_PID_t pid_yaw   = {2.0f, 0.05f, 0.10f, 0.0f, 0.0f};

static uint32_t lock_start_ms = 0U;

/* ---------------------------------------------------------------------------
 * turret_set_axis (Dahili Yardımcı Fonksiyon)
 * Yön pinlerini ayarlar ve mutlak PWM çıkışını uygular.
 * -------------------------------------------------------------------------*/
static void turret_set_axis(TIM_HandleTypeDef *htim, uint32_t channel, float output, 
                            GPIO_TypeDef *port, uint16_t pin_in1, uint16_t pin_in2)
{
    if (output >= 0.0f)
    {
        HAL_GPIO_WritePin(port, pin_in1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(port, pin_in2, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(port, pin_in1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(port, pin_in2, GPIO_PIN_SET);
    }
    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)fabsf(output));
}

static float turret_pid_step(Turret_PID_t *pid, float error)
{
    pid->error_sum += error * TURRET_PID_DT;

    if (pid->error_sum >  300.0f) { pid->error_sum =  300.0f; }
    if (pid->error_sum < -300.0f) { pid->error_sum = -300.0f; }

    float deriv  = (error - pid->last_error) / TURRET_PID_DT;
    float output = (pid->Kp * error)
                 + (pid->Ki * pid->error_sum)
                 + (pid->Kd * deriv);

    pid->last_error = error;

    if (output >  (float)TURRET_PWM_LIMIT) { output =  (float)TURRET_PWM_LIMIT; }
    if (output < -(float)TURRET_PWM_LIMIT) { output = -(float)TURRET_PWM_LIMIT; }

    return output;
}

void Turret_Init(void)
{
    turret_state         = TURRET_SEARCHING;
    pid_pitch.error_sum  = 0.0f;
    pid_pitch.last_error = 0.0f;
    pid_yaw.error_sum    = 0.0f;
    pid_yaw.last_error   = 0.0f;
    lock_start_ms        = 0U;
}

void Turret_Update(float pixel_err_pitch, float pixel_err_yaw,
                   TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                   TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                   TIM_HandleTypeDef *htim_laser, uint32_t ch_laser)
{
    bool on_target = (fabsf(pixel_err_pitch) < TURRET_LOCK_THRESH) &&
                     (fabsf(pixel_err_yaw)   < TURRET_LOCK_THRESH);

    switch (turret_state)
    {
    case TURRET_SEARCHING:
    case TURRET_TRACKING:
    {
        float out_pitch = turret_pid_step(&pid_pitch, pixel_err_pitch);
        float out_yaw   = turret_pid_step(&pid_yaw,   pixel_err_yaw);

        /* Yön denetimli eksen sürme */
        turret_set_axis(htim_pitch, ch_pitch, out_pitch, TURRET_GPIO_PORT, TURRET_PITCH_IN1_PIN, TURRET_PITCH_IN2_PIN);
        turret_set_axis(htim_yaw, ch_yaw, out_yaw, TURRET_GPIO_PORT, TURRET_YAW_IN1_PIN, TURRET_YAW_IN2_PIN);
        
        __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0U);

        if (on_target)
        {
            if (turret_state == TURRET_SEARCHING)
            {
                lock_start_ms = HAL_GetTick();
                turret_state  = TURRET_TRACKING;
            }
            else
            {
                if ((HAL_GetTick() - lock_start_ms) >= TURRET_LOCK_TIME_MS)
                {
                    turret_state = TURRET_LOCKED;
                }
            }
        }
        else
        {
            lock_start_ms = 0U;
            turret_state  = TURRET_SEARCHING;
        }
        break;
    }

    case TURRET_LOCKED:
        turret_state = TURRET_FIRING;
        __attribute__((fallthrough));

    case TURRET_FIRING:
    {
        if (on_target)
        {
            float out_pitch = turret_pid_step(&pid_pitch, pixel_err_pitch);
            float out_yaw   = turret_pid_step(&pid_yaw,   pixel_err_yaw);

            turret_set_axis(htim_pitch, ch_pitch, out_pitch, TURRET_GPIO_PORT, TURRET_PITCH_IN1_PIN, TURRET_PITCH_IN2_PIN);
            turret_set_axis(htim_yaw, ch_yaw, out_yaw, TURRET_GPIO_PORT, TURRET_YAW_IN1_PIN, TURRET_YAW_IN2_PIN);
            
            __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, TURRET_PWM_LIMIT);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0U);
            Turret_Init();
        }
        break;
    }

    default:
        turret_state = TURRET_SEARCHING;
        break;
    }
}

void Turret_Emergency_Stop(TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                            TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                            TIM_HandleTypeDef *htim_laser, uint32_t ch_laser)
{
    __HAL_TIM_SET_COMPARE(htim_pitch, ch_pitch, 0U);
    __HAL_TIM_SET_COMPARE(htim_yaw,   ch_yaw,   0U);
    __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0U);
    turret_state = TURRET_SEARCHING;
}
