/* ===========================================================================
 * robot_dead_reckoning.h  –  Ölü Hesaplama ile Konum Kestirimi
 * Hedef: STM32F4xx  (HAL bağımlılığı yok – saf C/math)
 * ===========================================================================*/
#ifndef ROBOT_DEAD_RECKONING_H
#define ROBOT_DEAD_RECKONING_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Robot geometrisi
 *   DR_TRACK_WIDTH_M: iki teker merkezi arasındaki mesafe (metre)
 *   Skid-steer araçlar için gerçek mekanik ölçüyle güncelleyin.
 * -------------------------------------------------------------------------*/
#define DR_TRACK_WIDTH_M   0.50f   /* metre */

/* ---------------------------------------------------------------------------
 * Veri tipi
 * -------------------------------------------------------------------------*/
typedef struct {
    float x;        /* metre, başlangıç noktasına göre doğu (+) */
    float y;        /* metre, başlangıç noktasına göre kuzey (+) */
    float heading;  /* radyan, CCW pozitif                       */
    float speed;    /* m/s, merkez hızı                          */
} Pose_t;

/* ---------------------------------------------------------------------------
 * Global değişken
 * -------------------------------------------------------------------------*/
extern Pose_t robot_pose;

/* ---------------------------------------------------------------------------
 * Fonksiyon prototipleri
 * -------------------------------------------------------------------------*/

/**
 * @brief Başlangıç konumu sıfırlar, IMU'dan ilk yön bilgisini alır.
 * @param initial_heading_rad  DR_Init çağrısı anındaki IMU yönü (radyan)
 */
void DR_Init(float initial_heading_rad);

/**
 * @brief Her döngüde çağrılır; konumu ve yönü günceller.
 *
 * Adaptif complementary filter:
 *   Düz gidiş  (|omega| < 0.1 rad/s) → encoder %98, IMU %02
 *   Dönüş      (|omega| >= 0.1 rad/s)→ encoder %70, IMU %30
 *   Dönüş ağırlığı skid-steer kaymasını kısmen telafi eder.
 *
 * @param left_vel         Sol tekerlek hızı (m/s)
 * @param right_vel        Sağ tekerlek hızı (m/s)
 * @param imu_heading_rad  Güncel IMU yönü (radyan)
 * @param dt               Geçen süre (saniye), tipik 0.020
 */
void DR_Update(float left_vel, float right_vel,
               float imu_heading_rad, float dt);

/**
 * @brief GPS fix geldiğinde XY konumunu GPS'e hizalar.
 *        Yön (heading) değiştirilmez.
 * @param gps_lat     Güncel GPS enlemi (ondalık derece)
 * @param gps_lon     Güncel GPS boylamı (ondalık derece)
 * @param origin_lat  Harita başlangıç enlemi (ondalık derece)
 * @param origin_lon  Harita başlangıç boylamı (ondalık derece)
 */
void DR_Reset_To_GPS(float gps_lat,    float gps_lon,
                     float origin_lat, float origin_lon);

#endif /* ROBOT_DEAD_RECKONING_H */
