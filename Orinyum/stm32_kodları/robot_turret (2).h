/* ===========================================================================
 * robot_turret.h  –  Taret ve Nişan Sistemi
 * ===========================================================================*/
#ifndef ROBOT_TURRET_H
#define ROBOT_TURRET_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define TURRET_PID_DT       0.020f
#define TURRET_LOCK_THRESH  3.0f
#define TURRET_LOCK_TIME_MS 1000U
#define TURRET_PWM_LIMIT    800U

/* Taret Donanım Yön Pinleri (CubeMX konfigürasyonunuza göre düzenleyebilirsiniz) */
#define TURRET_GPIO_PORT      GPIOB
#define TURRET_PITCH_IN1_PIN  GPIO_PIN_0
#define TURRET_PITCH_IN2_PIN  GPIO_PIN_1
#define TURRET_YAW_IN1_PIN    GPIO_PIN_2
#define TURRET_YAW_IN2_PIN    GPIO_PIN_3

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error_sum;
    float last_error;
} Turret_PID_t;

typedef enum {
    TURRET_SEARCHING = 0,
    TURRET_TRACKING,
    TURRET_LOCKED,
    TURRET_FIRING
} TurretState_t;

extern TurretState_t turret_state;

void Turret_Init(void);
void Turret_Update(float pixel_err_pitch, float pixel_err_yaw,
                   TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                   TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                   TIM_HandleTypeDef *htim_laser, uint32_t ch_laser);
void Turret_Emergency_Stop(TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                            TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                            TIM_HandleTypeDef *htim_laser, uint32_t ch_laser);

#endif /* ROBOT_TURRET_H */
