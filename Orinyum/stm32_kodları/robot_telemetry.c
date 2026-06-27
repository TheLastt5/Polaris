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
 * GPS ISR ↔ main-loop arası iç tampon ve bayrak
 *   gps_data_ready burada TANIMLANIR (header'da extern bildirimi var).
 * -------------------------------------------------------------------------*/
volatile uint8_t gps_data_ready  = 0U;
static uint8_t   gps_pending_buf[128];
static uint16_t  gps_pending_len = 0U;

/* ---------------------------------------------------------------------------
 * Telemetry_Init
 * -------------------------------------------------------------------------*/
void Telemetry_Init(TIM_HandleTypeDef *htim_left,
                    TIM_HandleTypeDef *htim_right)
{
    HAL_TIM_Encoder_Start(htim_left,  TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(htim_right, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(htim_left,  0U);
    __HAL_TIM_SET_COUNTER(htim_right, 0U);

    robot_encoder_left.total_pulses      = 0;
    robot_encoder_left.current_velocity  = 0.0f;
    robot_encoder_right.total_pulses     = 0;
    robot_encoder_right.current_velocity = 0.0f;
}

/* ---------------------------------------------------------------------------
 * Telemetry_Update_Encoders
 *   TIM2/TIM5 → 32-bit (int32_t),  TIM3/TIM4 → 16-bit (int16_t)
 * -------------------------------------------------------------------------*/
void Telemetry_Update_Encoders(TIM_HandleTypeDef *htim_left,
                                TIM_HandleTypeDef *htim_right)
{
    int32_t delta_left, delta_right;

    if (htim_left->Instance == TIM2 || htim_left->Instance == TIM5)
        delta_left = (int32_t)__HAL_TIM_GET_COUNTER(htim_left);
    else
        delta_left = (int16_t)__HAL_TIM_GET_COUNTER(htim_left);

    if (htim_right->Instance == TIM2 || htim_right->Instance == TIM5)
        delta_right = (int32_t)__HAL_TIM_GET_COUNTER(htim_right);
    else
        delta_right = (int16_t)__HAL_TIM_GET_COUNTER(htim_right);

    __HAL_TIM_SET_COUNTER(htim_left,  0U);
    __HAL_TIM_SET_COUNTER(htim_right, 0U);

    robot_encoder_left.current_velocity  =
        ((float)delta_left  * DISTANCE_PER_PULSE) / LOOP_PERIOD_S;
    robot_encoder_right.current_velocity =
        ((float)delta_right * DISTANCE_PER_PULSE) / LOOP_PERIOD_S;

    robot_encoder_left.total_pulses  += delta_left;
    robot_encoder_right.total_pulses += delta_right;
}

/* ---------------------------------------------------------------------------
 * Telemetry_GPS_Stage1_ISR  — ISR BAĞLAMI, strtok YOK
 * -------------------------------------------------------------------------*/
void Telemetry_GPS_Stage1_ISR(uint8_t *nmea_buffer, uint16_t len)
{
    if (len == 0U || len >= (uint16_t)sizeof(gps_pending_buf)) { return; }
    if (gps_data_ready != 0U) { return; }   /* önceki veri henüz işlenmedi */

    memcpy(gps_pending_buf, nmea_buffer, len);
    gps_pending_len = len;
    gps_data_ready  = 1U;
}

/* ---------------------------------------------------------------------------
 * nmea_to_decimal  (yerel yardımcı)
 *   NMEA DDDMM.MMMMM → ondalık derece
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
 * Telemetry_Parse_GPS  — MAIN LOOP BAĞLAMI, strtok güvenli
 * -------------------------------------------------------------------------*/
void Telemetry_Parse_GPS(void)
{
    if (gps_data_ready == 0U) { return; }

    char buf[128];
    memcpy(buf, gps_pending_buf, gps_pending_len);
    buf[gps_pending_len] = '\0';
    gps_data_ready = 0U;   /* bayrağı kopyalama sonrası temizle */

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

    float lat = nmea_to_decimal(fields[2]);
    if (fields[3][0] == 'S') { lat = -lat; }
    robot_gps.latitude = lat;

    float lon = nmea_to_decimal(fields[4]);
    if (fields[5][0] == 'W') { lon = -lon; }
    robot_gps.longitude = lon;

    robot_gps.fix_quality = (uint8_t)atoi(fields[6]);
}

/* ---------------------------------------------------------------------------
 * calc_checksum  (yerel yardımcı)
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
