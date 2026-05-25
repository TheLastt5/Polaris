#include "robot_core.h"
#include <math.h>

volatile IMU_Data_t robot_imu = {0.0f, 0.0f, 0.0f, 0.0f, true};

PID_Controller_t motor_pid_left  = {1.5f, 0.1f, 0.05f, 0.0f, 0.0f, 0.0f, 1000.0f};
PID_Controller_t motor_pid_right = {1.5f, 0.1f, 0.05f, 0.0f, 0.0f, 0.0f, 1000.0f};

void Robot_Core_Init(void)
{
    robot_imu.is_safe = true;
    motor_pid_left.error_sum   = 0.0f;
    motor_pid_left.last_error  = 0.0f;
    motor_pid_right.error_sum  = 0.0f;
    motor_pid_right.last_error = 0.0f;
}

void Robot_Update_IMU(I2C_HandleTypeDef *hi2c)
{
    uint8_t buffer[6];
    if (HAL_I2C_Mem_Read(hi2c, (0x28 << 1), 0x1A, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100) == HAL_OK)
    {
        robot_imu.yaw   = (float)((int16_t)((buffer[1] << 8) | buffer[0])) / 16.0f;
        robot_imu.roll  = (float)((int16_t)((buffer[3] << 8) | buffer[2])) / 16.0f;
        robot_imu.pitch = (float)((int16_t)((buffer[5] << 8) | buffer[4])) / 16.0f;
        robot_imu.heading_rad = robot_imu.yaw * (3.14159265f / 180.0f);
    }
}

void Robot_Check_Safety(TIM_HandleTypeDef *htim, uint32_t ch_left, uint32_t ch_right)
{
    float abs_roll  = fabsf(robot_imu.roll);
    float abs_pitch = fabsf(robot_imu.pitch);

    if (robot_imu.is_safe)
    {
        if (abs_roll > TILT_LIMIT || abs_pitch > TILT_LIMIT)
            robot_imu.is_safe = false;
    }
    else
    {
        if (abs_roll  < (TILT_LIMIT - TILT_HYSTERESIS) &&
            abs_pitch < (TILT_LIMIT - TILT_HYSTERESIS))
            robot_imu.is_safe = true;
    }

    if (!robot_imu.is_safe)
    {
        __HAL_TIM_SET_COMPARE(htim, ch_left,  0);
        __HAL_TIM_SET_COMPARE(htim, ch_right, 0);
    }
}

float Robot_Compute_PID(PID_Controller_t *pid, float current_speed)
{
    float error = pid->target_speed - current_speed;
    pid->error_sum += error * PID_DT;

    if (pid->error_sum >  500.0f) pid->error_sum =  500.0f;
    if (pid->error_sum < -500.0f) pid->error_sum = -500.0f;

    float derivative = (error - pid->last_error) / PID_DT;
    float output     = (pid->Kp * error) + (pid->Ki * pid->error_sum) + (pid->Kd * derivative);

    pid->last_error = error;

    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

void Robot_Drive_Motors(TIM_HandleTypeDef *htim,
                         uint32_t ch_left,  float pid_output_left,
                         uint32_t ch_right, float pid_output_right)
{
    if (!robot_imu.is_safe) return;

    if (pid_output_left >= 0.0f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   
    }

    if (pid_output_right >= 0.0f) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   
    }

    __HAL_TIM_SET_COMPARE(htim, ch_left,  (uint32_t)fabsf(pid_output_left));
    __HAL_TIM_SET_COMPARE(htim, ch_right, (uint32_t)fabsf(pid_output_right));
}
