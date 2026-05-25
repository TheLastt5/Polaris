#include "robot_turret.h"
#include <math.h>

TurretState_t turret_state = TURRET_SEARCHING;

static Turret_PID_t pid_pitch = {2.0f, 0.05f, 0.10f, 0.0f, 0.0f};
static Turret_PID_t pid_yaw   = {2.0f, 0.05f, 0.10f, 0.0f, 0.0f};

static uint32_t lock_start_ms  = 0;

void Turret_Init(void)
{
    turret_state        = TURRET_SEARCHING;
    pid_pitch.error_sum = 0.0f;
    pid_pitch.last_error= 0.0f;
    pid_yaw.error_sum   = 0.0f;
    pid_yaw.last_error  = 0.0f;
    lock_start_ms       = 0;
}

static float turret_pid_step(Turret_PID_t *pid, float error)
{
    pid->error_sum += error * TURRET_PID_DT;
    if (pid->error_sum >  300.0f) pid->error_sum =  300.0f;
    if (pid->error_sum < -300.0f) pid->error_sum = -300.0f;

    float deriv  = (error - pid->last_error) / TURRET_PID_DT;
    float output = (pid->Kp * error) + (pid->Ki * pid->error_sum) + (pid->Kd * deriv);

    pid->last_error = error;

    if (output >  (float)TURRET_PWM_LIMIT) output =  (float)TURRET_PWM_LIMIT;
    if (output < -(float)TURRET_PWM_LIMIT) output = -(float)TURRET_PWM_LIMIT;

    return output;
}

void Turret_Update(float pixel_err_pitch, float pixel_err_yaw,
                   TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                   TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                   TIM_HandleTypeDef *htim_laser, uint32_t ch_laser)
{
    float abs_ep = fabsf(pixel_err_pitch);
    float abs_ey = fabsf(pixel_err_yaw);
    bool  on_target = (abs_ep < TURRET_LOCK_THRESH) && (abs_ey < TURRET_LOCK_THRESH);

    switch (turret_state)
    {
    case TURRET_SEARCHING:
    case TURRET_TRACKING:
    {
        float out_pitch = turret_pid_step(&pid_pitch, pixel_err_pitch);
        float out_yaw   = turret_pid_step(&pid_yaw,   pixel_err_yaw);

        __HAL_TIM_SET_COMPARE(htim_pitch, ch_pitch, (uint32_t)fabsf(out_pitch));
        __HAL_TIM_SET_COMPARE(htim_yaw,   ch_yaw,   (uint32_t)fabsf(out_yaw));
        __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0); 

        if (on_target)
        {
            if (turret_state == TURRET_SEARCHING) {
                lock_start_ms = HAL_GetTick();
                turret_state  = TURRET_TRACKING;
            } else {
                if ((HAL_GetTick() - lock_start_ms) >= TURRET_LOCK_TIME_MS)
                    turret_state = TURRET_LOCKED;
            }
        }
        else
        {
            lock_start_ms = 0;
            turret_state  = TURRET_SEARCHING;
        }
        break;
    }
    case TURRET_LOCKED:
        turret_state = TURRET_FIRING;
    case TURRET_FIRING:
    {
        if (on_target)
        {
            float out_pitch = turret_pid_step(&pid_pitch, pixel_err_pitch);
            float out_yaw   = turret_pid_step(&pid_yaw,   pixel_err_yaw);
            __HAL_TIM_SET_COMPARE(htim_pitch, ch_pitch, (uint32_t)fabsf(out_pitch));
            __HAL_TIM_SET_COMPARE(htim_yaw,   ch_yaw,   (uint32_t)fabsf(out_yaw));
            __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, TURRET_PWM_LIMIT);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0);
            turret_state = TURRET_SEARCHING;
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
    __HAL_TIM_SET_COMPARE(htim_pitch, ch_pitch, 0);
    __HAL_TIM_SET_COMPARE(htim_yaw,   ch_yaw,   0);
    __HAL_TIM_SET_COMPARE(htim_laser, ch_laser, 0);
    turret_state = TURRET_SEARCHING;
}
