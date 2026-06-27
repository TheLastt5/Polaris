/* ===========================================================================
 * robot_turret.h  –  Taret ve Nişan Sistemi
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#ifndef ROBOT_TURRET_H
#define ROBOT_TURRET_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * Taret kontrol sabitleri
 * -------------------------------------------------------------------------*/
#define TURRET_PID_DT        0.020f   /* s   – döngü periyodu              */
#define TURRET_LOCK_THRESH   3.0f     /* px  – kilitlenme eşiği            */
#define TURRET_LOCK_TIME_MS  1000U    /* ms  – kilitli kalma süresi        */
#define TURRET_PWM_LIMIT     800U     /* ARR birimine göre maks. PWM       */

/* ---------------------------------------------------------------------------
 * Servo yön GPIO pin tanımları
 *   CubeMX pinout'unuza göre güncelleyin.
 *   Varsayım: GPIOB üzerinde dört pin.
 *
 *   PB0 / PB1 → Pitch servo IN1 / IN2
 *   PB2 / PB3 → Yaw   servo IN1 / IN2
 * -------------------------------------------------------------------------*/
#define TURRET_GPIO_PORT       GPIOB
#define TURRET_PITCH_IN1_PIN   GPIO_PIN_0
#define TURRET_PITCH_IN2_PIN   GPIO_PIN_1
#define TURRET_YAW_IN1_PIN     GPIO_PIN_2
#define TURRET_YAW_IN2_PIN     GPIO_PIN_3

/* ---------------------------------------------------------------------------
 * Veri tipleri
 * -------------------------------------------------------------------------*/
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error_sum;
    float last_error;
} Turret_PID_t;

typedef enum {
    TURRET_SEARCHING = 0,   /* Hedef aranıyor                      */
    TURRET_TRACKING,        /* Hedef görüldü, kilit bekleniyor     */
    TURRET_LOCKED,          /* Kilit süresi doldu → FIRING geçişi  */
    TURRET_FIRING           /* Lazer aktif, ince ayar devam        */
} TurretState_t;

/* ---------------------------------------------------------------------------
 * Global değişken
 * -------------------------------------------------------------------------*/
extern TurretState_t turret_state;

/* ---------------------------------------------------------------------------
 * Fonksiyon prototipleri
 * -------------------------------------------------------------------------*/
void Turret_Init(void);

/**
 * @brief Ana taret döngüsü – her 20 ms çağrılmalı.
 * @param pixel_err_pitch  Kamera merkezi – hedef Y farkı (piksel)
 * @param pixel_err_yaw    Kamera merkezi – hedef X farkı (piksel)
 * @param htim_pitch       Pitch servo timer  (örn. &htim4)
 * @param ch_pitch         Pitch PWM kanalı   (örn. TIM_CHANNEL_1)
 * @param htim_yaw         Yaw servo timer    (örn. &htim4)
 * @param ch_yaw           Yaw PWM kanalı     (örn. TIM_CHANNEL_2)
 * @param htim_laser       Lazer timer        (örn. &htim4)
 * @param ch_laser         Lazer PWM kanalı   (örn. TIM_CHANNEL_3)
 */
void Turret_Update(float pixel_err_pitch, float pixel_err_yaw,
                   TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                   TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                   TIM_HandleTypeDef *htim_laser, uint32_t ch_laser);

/** Acil durdurma: tüm servo ve lazer PWM'lerini sıfırlar. */
void Turret_Emergency_Stop(TIM_HandleTypeDef *htim_pitch, uint32_t ch_pitch,
                            TIM_HandleTypeDef *htim_yaw,   uint32_t ch_yaw,
                            TIM_HandleTypeDef *htim_laser, uint32_t ch_laser);

#endif /* ROBOT_TURRET_H */
