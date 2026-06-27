#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

/* Motor Pinleri */
#define MOTOR_L_PWM_PIN  13
#define MOTOR_L_IN1_PIN  12
#define MOTOR_L_IN2_PIN  14
#define MOTOR_R_PWM_PIN  27
#define MOTOR_R_IN3_PIN  26
#define MOTOR_R_IN4_PIN  25

/* I2C BNO055 */
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM    I2C_NUM_0
#define BNO055_I2C_ADDR   0x28
#define BNO055_REG_EULER  0x1A

#define TILT_LIMIT        25.0f
#define TILT_HYSTERESIS    2.0f
#define PID_DT             0.020f

typedef struct {
    float roll, pitch, yaw, heading_rad;
    bool  is_safe;
} IMU_Data_t;

typedef struct {
    float Kp, Ki, Kd, target_speed, error_sum, last_error, output_limit;
} PID_Controller_t;

extern volatile IMU_Data_t robot_imu;
extern PID_Controller_t    motor_pid_left;
extern PID_Controller_t    motor_pid_right;

void  Robot_Core_Init(void);
void  Robot_Update_IMU(void);
void  Robot_Check_Safety(void);
float Robot_Compute_PID(PID_Controller_t *pid, float current_speed);
void  Robot_Drive_Motors(float pid_out_left, float pid_out_right);

#endif
