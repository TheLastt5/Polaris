/* ===========================================================================
 * robot_telemetry.h  –  Veri Toplama ve Paketleme
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Encoder / döngü sabitleri
 * -------------------------------------------------------------------------*/
#define DISTANCE_PER_PULSE  0.005f   /* m / darbe  */
#define LOOP_PERIOD_S       0.020f   /* s  (50 Hz) */

/* ---------------------------------------------------------------------------
 * ROS seri çerçeve başlığı
 * -------------------------------------------------------------------------*/
#define ROS_FRAME_HEADER  0xAA55AA55UL

/* ---------------------------------------------------------------------------
 * Veri tipleri
 * -------------------------------------------------------------------------*/
typedef struct {
    float   latitude;
    float   longitude;
    uint8_t fix_quality;
} GPS_Data_t;

typedef struct {
    int32_t total_pulses;
    float   current_velocity;   /* m/s */
} Encoder_Data_t;

/**
 * ROS seri çerçevesi – little-endian, packed (35 byte)
 * Python format: '<I f f f f f f f B H'
 */
typedef struct __attribute__((packed)) {
    uint32_t header;             /* 0xAA55AA55 */
    float    imu_roll;
    float    imu_pitch;
    float    imu_yaw;
    float    encoder_vel_left;
    float    encoder_vel_right;
    float    gps_lat;
    float    gps_lon;
    uint8_t  gps_fix;
    uint16_t checksum;           /* XOR: imu_roll'dan gps_fix'e */
} ROS_Frame_t;

/* ---------------------------------------------------------------------------
 * ISR ↔ main-loop GPS bayrak mekanizması
 *   ISR  → Telemetry_GPS_Stage1_ISR() çağırır (strtok yok)
 *   main → gps_data_ready != 0 ise Telemetry_Parse_GPS() çağırır
 *
 *   gps_data_ready burada tanımlanır (robot_telemetry.c'de),
 *   main_user_code.c'de extern olarak kullanılır.
 * -------------------------------------------------------------------------*/
extern volatile uint8_t gps_data_ready;

/* ---------------------------------------------------------------------------
 * Global sensör değişkenleri
 * -------------------------------------------------------------------------*/
extern GPS_Data_t     robot_gps;
extern Encoder_Data_t robot_encoder_left;
extern Encoder_Data_t robot_encoder_right;

/* ---------------------------------------------------------------------------
 * Fonksiyon prototipleri
 * -------------------------------------------------------------------------*/

/** Encoder timer'larını başlatır (TIM_CHANNEL_ALL, sayaç sıfır). */
void Telemetry_Init(TIM_HandleTypeDef *htim_left,
                    TIM_HandleTypeDef *htim_right);

/** Her 20 ms'de encoder delta okur, hız hesaplar. */
void Telemetry_Update_Encoders(TIM_HandleTypeDef *htim_left,
                                TIM_HandleTypeDef *htim_right);

/**
 * [ISR-GÜVENLİ] HAL_UART_RxCpltCallback'ten çağrılır.
 * strtok() ÇAĞIRMAZ – yalnızca veriyi iç tampona kopyalar,
 * gps_data_ready bayrağını 1 yapar.
 */
void Telemetry_GPS_Stage1_ISR(uint8_t *nmea_buffer, uint16_t len);

/**
 * [MAIN LOOP] gps_data_ready != 0 olduğunda NMEA'yı ayrıştırır.
 * strtok() burada güvenle kullanılır.
 * Her döngüde çağrılmalı; veri yoksa hızla döner.
 */
void Telemetry_Parse_GPS(void);

/** Tüm sensör verilerini paketleyip ROS çerçevesi döndürür. */
ROS_Frame_t Telemetry_Build_ROS_Frame(float roll, float pitch, float yaw);

#endif /* ROBOT_TELEMETRY_H */
