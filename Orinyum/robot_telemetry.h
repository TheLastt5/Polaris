#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define DISTANCE_PER_PULSE  0.005f
#define LOOP_PERIOD_S       0.020f

typedef struct {
    float   latitude;
    float   longitude;
    uint8_t fix_quality;
} GPS_Data_t;

typedef struct {
    int32_t total_pulses;
    float   current_velocity;
} Encoder_Data_t;

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

#define ROS_FRAME_HEADER  0xAA55AA55U

extern GPS_Data_t     robot_gps;
extern Encoder_Data_t robot_encoder_left;
extern Encoder_Data_t robot_encoder_right;

void        Telemetry_Init            (TIM_HandleTypeDef *htim_left,
                                       TIM_HandleTypeDef *htim_right);
void        Telemetry_Update_Encoders (TIM_HandleTypeDef *htim_left,
                                       TIM_HandleTypeDef *htim_right);
void        Telemetry_Parse_GPS       (uint8_t *nmea_buffer, uint16_t len);
ROS_Frame_t Telemetry_Build_ROS_Frame (float roll, float pitch, float yaw);

#endif /* ROBOT_TELEMETRY_H */
