/* ===========================================================================
 * robot_dead_reckoning.c  –  Ölü Hesaplama ile Konum Kestirimi
 * Hedef: STM32F4xx  (HAL bağımlılığı yok – saf C/math)
 * ===========================================================================*/
#include "robot_dead_reckoning.h"
#include <math.h>

/* ---------------------------------------------------------------------------
 * Global değişken tanımı
 * -------------------------------------------------------------------------*/
Pose_t robot_pose = {0.0f, 0.0f, 0.0f, 0.0f};

/* ---------------------------------------------------------------------------
 * DR_Init
 * -------------------------------------------------------------------------*/
void DR_Init(float initial_heading_rad)
{
    robot_pose.x       = 0.0f;
    robot_pose.y       = 0.0f;
    robot_pose.heading = initial_heading_rad;
    robot_pose.speed   = 0.0f;
}

/* ---------------------------------------------------------------------------
 * DR_Update
 *   Diferansiyel sürüş kinematiği + adaptif complementary filter
 * -------------------------------------------------------------------------*/
void DR_Update(float left_vel, float right_vel,
               float imu_heading_rad, float dt)
{
    float v_center = (left_vel + right_vel) * 0.5f;
    float omega    = (right_vel - left_vel) / DR_TRACK_WIDTH_M;  /* rad/s */

    /* Encoder tahminli yön (ham integral) */
    float heading_enc = robot_pose.heading + omega * dt;

    /* Adaptif ağırlık seçimi */
    float enc_weight, imu_weight;
    if (fabsf(omega) > 0.1f)
    {
        /* Dönüş: IMU ağırlığı artırılır – skid kayması telafisi */
        enc_weight = 0.70f;
        imu_weight = 0.30f;
    }
    else
    {
        /* Düz gidiş: encoder güvenilirdir, IMU drift etkisi azaltılır */
        enc_weight = 0.98f;
        imu_weight = 0.02f;
    }

    robot_pose.heading = enc_weight * heading_enc
                       + imu_weight * imu_heading_rad;

    /* Konum entegrasyonu */
    robot_pose.x    += v_center * cosf(robot_pose.heading) * dt;
    robot_pose.y    += v_center * sinf(robot_pose.heading) * dt;
    robot_pose.speed = v_center;
}

/* ---------------------------------------------------------------------------
 * DR_Reset_To_GPS
 *   GPS koordinatlarını düz-yüzey (haversine yerine ekvatoryal yaklaşım)
 *   yerel XY metrik koordinatlara dönüştürür.
 *   Hata: ~1 m / 10 km → Teknofest yarışma alanı için yeterli.
 * -------------------------------------------------------------------------*/
void DR_Reset_To_GPS(float gps_lat,    float gps_lon,
                     float origin_lat, float origin_lon)
{
    const float R   = 6371000.0f;                  /* Dünya yarıçapı (m)    */
    const float DEG = 3.14159265f / 180.0f;        /* derece → radyan çarpanı */

    float dlat = (gps_lat  - origin_lat) * DEG;
    float dlon = (gps_lon  - origin_lon) * DEG;
    float mlat = (gps_lat  + origin_lat) * 0.5f * DEG;  /* orta enlem        */

    /* Yalnızca X-Y güncellenir; heading değiştirilmez */
    robot_pose.x = R * dlon * cosf(mlat);
    robot_pose.y = R * dlat;
}
