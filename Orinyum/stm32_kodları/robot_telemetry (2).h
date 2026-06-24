/* ===========================================================================
 * robot_telemetry.h  –  Veri Toplama ve Paketleme
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Encoder sabitleri
 *   DISTANCE_PER_PULSE: tekerlek çevresi / encoder çözünürlüğü  (metre/darbe)
 *   LOOP_PERIOD_S     : ana döngü periyodu = 20 ms
 * -------------------------------------------------------------------------*/
#define DISTANCE_PER_PULSE  0.005f   /* m/darbe  */
#define LOOP_PERIOD_S       0.020f   /* s         */

/* ---------------------------------------------------------------------------
 * ROS seri çerçeve başlığı (magic word)
 * Python tarafı bu değere göre frame'i hizalar.
 * -------------------------------------------------------------------------*/
#define ROS_FRAME_HEADER  0xAA55AA55UL

/* ---------------------------------------------------------------------------
 * Veri tipleri
 * -------------------------------------------------------------------------*/
typedef struct {
    float   latitude;
    float   longitude;
    uint8_t fix_quality;   /* 0 = fix yok, 1 = GPS, 2 = DGPS, ... */
} GPS_Data_t;

typedef struct {
    int32_t total_pulses;      /* birikimli darbe sayısı    */
    float   current_velocity;  /* anlık hız (m/s)           */
} Encoder_Data_t;

/**
 * @brief ROS'a gönderilen seri veri çerçevesi.
 *
 * Alan sırası Python struct formatı '<IfffffffBH' ile bire bir eşleşmeli:
 *   I  → header          (uint32)
 *   f  → imu_roll        (float)
 *   f  → imu_pitch       (float)
 *   f  → imu_yaw         (float)
 *   f  → encoder_vel_left  (float)
 *   f  → encoder_vel_right (float)
 *   f  → gps_lat         (float)
 *   f  → gps_lon         (float)
 *   B  → gps_fix         (uint8)
 *   H  → checksum        (uint16)  ← XOR, imu_roll'dan gps_fix'e kadar
 *
 * packed: padding yok, sizeof = 4+4*7+1+2 = 39 byte
 */
typedef struct __attribute__((packed)) {
    uint32_t header;
    float    imu_roll;
    float    imu_pitch;
    float    imu_yaw;
    float    encoder_vel_left;
    float    encoder_vel_right;
    float    gps_lat;
    float    gps_lon;
    uint8_t  gps_fix;
    uint16_t checksum;
} ROS_Frame_t;

/* ---------------------------------------------------------------------------
 * Global değişkenler
 * -------------------------------------------------------------------------*/
extern GPS_Data_t     robot_gps;
extern Encoder_Data_t robot_encoder_left;
extern Encoder_Data_t robot_encoder_right;

/* ---------------------------------------------------------------------------
 * Fonksiyon prototipleri
 * -------------------------------------------------------------------------*/

/**
 * @brief Encoder timer'larını başlatır (HAL_TIM_Encoder_Start).
 * @param htim_left   Sol encoder timer'ı  (örn. &htim2)
 * @param htim_right  Sağ encoder timer'ı  (örn. &htim3)
 */
void Telemetry_Init(TIM_HandleTypeDef *htim_left,
                    TIM_HandleTypeDef *htim_right);

/**
 * @brief Her döngüde encoder sayacını okur, sıfırlar, hız hesaplar.
 */
void Telemetry_Update_Encoders(TIM_HandleTypeDef *htim_left,
                                TIM_HandleTypeDef *htim_right);

/**
 * @brief $GPGGA NMEA cümlesini ayrıştırır, robot_gps'i günceller.
 * @param nmea_buffer  UART alım tamponu
 * @param len          Geçerli byte sayısı
 */
void Telemetry_Parse_GPS(uint8_t *nmea_buffer, uint16_t len);

/**
 * @brief Tüm sensör verilerini paketleyip ROS çerçevesi döndürür.
 */
ROS_Frame_t Telemetry_Build_ROS_Frame(float roll, float pitch, float yaw);

#endif /* ROBOT_TELEMETRY_H */
