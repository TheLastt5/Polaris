#ifndef ROBOT_DEAD_RECKONING_H
#define ROBOT_DEAD_RECKONING_H

#include <stdint.h>

#define DR_TRACK_WIDTH_M   0.50f   /* metre */

typedef struct {
    float x;        
    float y;        
    float heading;  
    float speed;    
} Pose_t;

extern Pose_t robot_pose;

void DR_Init(float initial_heading_rad);
void DR_Update(float left_vel, float right_vel, float imu_heading_rad, float dt);
void DR_Reset_To_GPS(float gps_lat, float gps_lon, float origin_lat, float origin_lon);

#endif /* ROBOT_DEAD_RECKONING_H */
