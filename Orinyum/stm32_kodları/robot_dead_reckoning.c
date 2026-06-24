/* ===========================================================================
 * robot_dead_reckoning.c  –  Ölü Hesaplama ile Konum Kestirimi
 * ===========================================================================*/
#include "robot_dead_reckoning.h"
#include <math.h>

Pose_t robot_pose = {0.0f, 0.0f, 0.0f, 0.0f};

void DR_Init(float initial_heading_rad)
{
    robot_pose.x       = 0.0f;
    robot_pose.y       = 0.0f;
    robot_pose.heading = initial_heading_rad;
    robot_pose.speed   = 0.0f;
}

void DR_Update(float left_vel, float right_vel,
               float imu_heading_rad, float dt)
{
    float v_center = (left_vel + right_vel) * 0.5f;
    float omega    = (right_vel - left_vel) / DR_TRACK_WIDTH_M;

    float heading_enc = robot_pose.heading + omega * dt;

    float enc_weight, imu_weight;
    if (fabsf(omega) > 0.1f)
    {
        enc_weight = 0.70f;
        imu_weight = 0.30f;
    }
    else
    {
        enc_weight = 0.98f;
        imu_weight = 0.02f;
    }

    robot_pose.heading = enc_weight * heading_enc + imu_weight * imu_heading_rad;

    /* Açısal Hassasiyet Koruması: Heading Açısını [-PI, PI] Arasında Normalize Et */
    robot_pose.heading = fmodf(robot_pose.heading + 3.14159265f, 2.0f * 3.14159265f);
    if (robot_pose.heading < 0.0f)
    {
        robot_pose.heading += 2.0f * 3.14159265f;
    }
    robot_pose.heading -= 3.14159265f;

    robot_pose.x    += v_center * cosf(robot_pose.heading) * dt;
    robot_pose.y    += v_center * sinf(robot_pose.heading) * dt;
    robot_pose.speed = v_center;
}

void DR_Reset_To_GPS(float gps_lat,    float gps_lon,
                     float origin_lat, float origin_lon)
{
    const float R   = 6371000.0f;
    const float DEG = 3.14159265f / 180.0f;

    float dlat = (gps_lat  - origin_lat) * DEG;
    float dlon = (gps_lon  - origin_lon) * DEG;
    float mlat = (gps_lat  + origin_lat) * 0.5f * DEG;

    robot_pose.x = R * dlon * cosf(mlat);
    robot_pose.y = R * dlat;
}
