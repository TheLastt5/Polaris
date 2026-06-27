#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <stdint.h>
#include "driver/uart.h"
#include "driver/pcnt.h"

#define DISTANCE_PER_PULSE  0.005f
#define LOOP_PERIOD_S       0.020f
#define ROS_FRAME_HEADER    0xAA55AA55UL

/* UART Pinleri */
#define GPS_TX_PIN 17
#define GPS_RX_PIN 16
#define OP_TX_PIN  19
#define OP_RX_PIN  18

/* Encoder Pinleri */
#define ENC_L_A 34
#define ENC_L_B 35
#define ENC_R_A 32
#define ENC_R_B 33

typedef struct {
    float latitude, longitude;
    uint8_t fix_quality;
} GPS_Data_t;

typedef struct {
    int32_t total_pulses;
    float   current_velocity;
} Encoder_Data_t;

typedef struct __attribute__((packed)) {
    uint32_t header;
    float imu_roll, imu_pitch, imu_yaw;
    float encoder_vel_left, encoder_vel_right;
    float gps_lat, gps_lon;
    uint8_t  gps_fix;
    uint16_t checksum;
} ROS_Frame_t;

extern GPS_Data_t robot_gps;
extern Encoder_Data_t robot_encoder_left;
extern Encoder_Data_t robot_encoder_right;

void Telemetry_Init(void);
void Telemetry_Update_Encoders(void);
void Telemetry_Parse_GPS_Buffer(uint8_t *nmea_buffer);
ROS_Frame_t Telemetry_Build_ROS_Frame(float roll, float pitch, float yaw);

#endif
