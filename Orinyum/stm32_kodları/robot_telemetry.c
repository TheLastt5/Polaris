/* ===========================================================================
 * robot_telemetry.c  –  Veri Toplama ve Paketleme
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#include "robot_telemetry.h"
#include <string.h>
#include <stdlib.h>

/* ---------------------------------------------------------------------------
 * Global değişken tanımları
 * -------------------------------------------------------------------------*/
GPS_Data_t     robot_gps           = {0.0f, 0.0f, 0U};
Encoder_Data_t robot_encoder_left  = {0, 0.0f};
Encoder_Data_t robot_encoder_right = {0, 0.0f};

/* ---------------------------------------------------------------------------
 * Telemetry_Init
 * -------------------------------------------------------------------------*/
void Telemetry_Init(TIM_HandleTypeDef *htim_left,
                    TIM_HandleTypeDef *htim_right)
{
    HAL_TIM_Encoder_Start(htim_left,  TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(htim_right, TIM_CHANNEL_ALL);

    /* Sayaçları sıfırla */
    __HAL_TIM_SET_COUNTER(htim_left,  0U);
    __HAL_TIM_SET_COUNTER(htim_right, 0U);

    robot_encoder_left.total_pulses      = 0;
    robot_encoder_left.current_velocity  = 0.0f;
    robot_encoder_right.total_pulses     = 0;
    robot_encoder_right.current_velocity = 0.0f;
}

/* ---------------------------------------------------------------------------
 * Telemetry_Update_Encoders
 *   Her ana döngü adımında çağrılmalı (her 20 ms).
 *   Quadrature encoder: TIM_EncoderMode_TI12 → negatif yönde negatif sayar.
 * -------------------------------------------------------------------------*/
void Telemetry_Update_Encoders(TIM_HandleTypeDef *htim_left,
                                TIM_HandleTypeDef *htim_right)
{
    /* int16_t döküm: 16-bit timer için taşma yönetimi sağlar */
    int16_t delta_left  = (int16_t)__HAL_TIM_GET_COUNTER(htim_left);
    int16_t delta_right = (int16_t)__HAL_TIM_GET_COUNTER(htim_right);

    __HAL_TIM_SET_COUNTER(htim_left,  0U);
    __HAL_TIM_SET_COUNTER(htim_right, 0U);

    robot_encoder_left.current_velocity  =
        ((float)delta_left  * DISTANCE_PER_PULSE) / LOOP_PERIOD_S;
    robot_encoder_right.current_velocity =
        ((float)delta_right * DISTANCE_PER_PULSE) / LOOP_PERIOD_S;

    robot_encoder_left.total_pulses  += (int32_t)delta_left;
    robot_encoder_right.total_pulses += (int32_t)delta_right;
}

/* ---------------------------------------------------------------------------
 * nmea_to_decimal  (yerel yardımcı)
 *   NMEA formatı: DDDMM.MMMMM → ondalık derece
 * -------------------------------------------------------------------------*/
static float nmea_to_decimal(const char *field)
{
    if (field == NULL || field[0] == '\0') { return 0.0f; }

    float raw     = strtof(field, NULL);
    int   degrees = (int)(raw / 100.0f);
    float minutes = raw - (float)(degrees * 100);
    return (float)degrees + minutes / 60.0f;
}

/* ---------------------------------------------------------------------------
 * Telemetry_Parse_GPS
 *   Yalnızca $GPGGA cümlesini işler.
 *   Beklenen alan sırası:
 *   $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,...*hh
 *   [0]    [1]       [2]     [3][4]     [5][6]
 * -------------------------------------------------------------------------*/
void Telemetry_Parse_GPS(uint8_t *nmea_buffer, uint16_t len)
{
    char buf[128];

    if (len == 0U || len >= (uint16_t)sizeof(buf)) { return; }

    memcpy(buf, nmea_buffer, len);
    buf[len] = '\0';

    /* Yalnızca $GPGGA işle */
    if (strstr(buf, "$GPGGA") == NULL) { return; }

    char   *fields[15];
    uint8_t field_count = 0U;
    char   *tok = strtok(buf, ",");

    while (tok != NULL && field_count < 15U)
    {
        fields[field_count++] = tok;
        tok = strtok(NULL, ",");
    }

    if (field_count < 7U) { return; }

    /* Enlem */
    float lat = nmea_to_decimal(fields[2]);
    if (fields[3][0] == 'S') { lat = -lat; }
    robot_gps.latitude = lat;

    /* Boylam */
    float lon = nmea_to_decimal(fields[4]);
    if (fields[5][0] == 'W') { lon = -lon; }
    robot_gps.longitude = lon;

    /* Fix kalitesi (0 = fix yok) */
    robot_gps.fix_quality = (uint8_t)atoi(fields[6]);
}

/* ---------------------------------------------------------------------------
 * calc_checksum  (yerel yardımcı)
 *   XOR toplamı: imu_roll alanından checksum alanının hemen öncesine kadar.
 * -------------------------------------------------------------------------*/
static uint16_t calc_checksum(const ROS_Frame_t *f)
{
    const uint8_t *p   = (const uint8_t *)&f->imu_roll;
    const uint8_t *end = (const uint8_t *)&f->checksum;
    uint16_t       sum = 0U;

    while (p < end) { sum ^= (uint16_t)*p++; }
    return sum;
}

/* ---------------------------------------------------------------------------
 * Telemetry_Build_ROS_Frame
 * -------------------------------------------------------------------------*/
ROS_Frame_t Telemetry_Build_ROS_Frame(float roll, float pitch, float yaw)
{
    ROS_Frame_t frame;

    frame.header            = ROS_FRAME_HEADER;
    frame.imu_roll          = roll;
    frame.imu_pitch         = pitch;
    frame.imu_yaw           = yaw;
    frame.encoder_vel_left  = robot_encoder_left.current_velocity;
    frame.encoder_vel_right = robot_encoder_right.current_velocity;
    frame.gps_lat           = robot_gps.latitude;
    frame.gps_lon           = robot_gps.longitude;
    frame.gps_fix           = robot_gps.fix_quality;
    frame.checksum          = calc_checksum(&frame);

    return frame;
}
