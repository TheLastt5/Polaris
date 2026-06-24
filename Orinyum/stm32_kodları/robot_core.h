/* ===========================================================================
 * robot_core.h  –  Kontrol ve Güvenlik Çekirdeği
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * Sabitler
 * -------------------------------------------------------------------------*/
#define TILT_LIMIT        25.0f   /* derece  – devrilme sınırı             */
#define TILT_HYSTERESIS    2.0f   /* derece  – histerezis bandı            */
#define PID_DT             0.020f /* saniye  – döngü periyodu (20 ms)      */

/* Motor yön pinleri: L298N / benzeri sürücü varsayımı
 * Sol:  IN1 = PA1, IN2 = PA2
 * Sağ:  IN3 = PA3, IN4 = PA4                                               */
#define MOTOR_GPIO_PORT    GPIOA
#define MOTOR_L_IN1_PIN    GPIO_PIN_1
#define MOTOR_L_IN2_PIN    GPIO_PIN_2
#define MOTOR_R_IN3_PIN    GPIO_PIN_3
#define MOTOR_R_IN4_PIN    GPIO_PIN_4

/* BNO055 I2C adresi (ADDR pini GND → 0x28)                                 */
#define BNO055_I2C_ADDR   (0x28U << 1)
#define BNO055_REG_EULER   0x1AU  /* EUL_Heading_LSB – 6 byte              */

/* ---------------------------------------------------------------------------
 * Veri tipleri
 * -------------------------------------------------------------------------*/
typedef struct {
    float roll;         /* derece */
    float pitch;        /* derece */
    float yaw;          /* derece */
    float heading_rad;  /* radyan */
    bool  is_safe;
} IMU_Data_t;

typedef struct {
    float    Kp;
    float    Ki;
    float    Kd;
    float    target_speed;   /* m/s  */
    float    error_sum;      /* integral biriktirici */
    float    last_error;
    float    output_limit;   /* PWM karşılığı maks. çıkış */
} PID_Controller_t;

/* ---------------------------------------------------------------------------
 * Global değişkenler (diğer modüller tarafından da okunur)
 * -------------------------------------------------------------------------*/
extern volatile IMU_Data_t robot_imu;
extern PID_Controller_t    motor_pid_left;
extern PID_Controller_t    motor_pid_right;

/* ---------------------------------------------------------------------------
 * Fonksiyon prototipleri
 * -------------------------------------------------------------------------*/
void  Robot_Core_Init   (void);

/**
 * @brief BNO055'ten Euler açılarını okur, robot_imu'yu günceller.
 * @param hi2c  CubeMX'te yapılandırılmış I2C tanıtıcısı (örn. &hi2c1)
 */
void  Robot_Update_IMU  (I2C_HandleTypeDef *hi2c);

/**
 * @brief Eğim kontrolü yapar; güvensizse motorları durdurur.
 * @param htim     Motor PWM timer'ı (örn. &htim1)
 * @param ch_left  Sol kanal (örn. TIM_CHANNEL_1)
 * @param ch_right Sağ kanal (örn. TIM_CHANNEL_2)
 */
void  Robot_Check_Safety(TIM_HandleTypeDef *htim,
                          uint32_t ch_left, uint32_t ch_right);

/**
 * @brief Tek eksen PID adımı.
 * @param pid           PID yapısı
 * @param current_speed Mevcut encoder hızı (m/s)
 * @return  PWM çıkış değeri (sınırlandırılmış)
 */
float Robot_Compute_PID (PID_Controller_t *pid, float current_speed);

/**
 * @brief PID çıkışlarını motor sürücüsüne yazar.
 *        Negatif değerlerde yön pinlerini çevirir.
 */
void  Robot_Drive_Motors(TIM_HandleTypeDef *htim,
                          uint32_t ch_left,  float pid_out_left,
                          uint32_t ch_right, float pid_out_right);

#endif /* ROBOT_CORE_H */
