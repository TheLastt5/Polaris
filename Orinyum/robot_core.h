#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define TILT_LIMIT       25.0f
#define TILT_HYSTERESIS  2.0f
#define PID_DT           0.020f

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float heading_rad;
    bool  is_safe;
} IMU_Data_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target_speed;
    float error_sum;
    float last_error;
    float output_limit;
} PID_Controller_t;

extern volatile IMU_Data_t robot_imu;
extern PID_Controller_t    motor_pid_left;
extern PID_Controller_t    motor_pid_right;

void  Robot_Core_Init   (void);
void  Robot_Update_IMU  (I2C_HandleTypeDef *hi2c);
void  Robot_Check_Safety(TIM_HandleTypeDef *htim,
                          uint32_t ch_left, uint32_t ch_right);
float Robot_Compute_PID (PID_Controller_t *pid, float current_speed);
void  Robot_Drive_Motors(TIM_HandleTypeDef *htim,
                          uint32_t ch_left,  float pid_output_left,
                          uint32_t ch_right, float pid_output_right);

#endif /* ROBOT_CORE_H */
