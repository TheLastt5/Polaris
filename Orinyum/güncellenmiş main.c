/* USER CODE BEGIN 0 */
#include "robot_core.h"
#include "robot_telemetry.h"
#include "robot_modes.h"
#include "robot_dead_reckoning.h"
#include "robot_turret.h"

#define ORIGIN_LAT 41.000000f
#define ORIGIN_LON 29.000000f

float   autonomous_target_left  = 0.0f;
float   autonomous_target_right = 0.0f;
float   pixel_err_pitch         = 0.0f;
float   pixel_err_yaw           = 0.0f;
uint8_t gps_rx_buffer[128];
uint8_t operator_rx_buffer[sizeof(Operator_Cmd_t) + 4];
/* USER CODE END 0 */

/* USER CODE BEGIN 2 */
Robot_Core_Init();
Telemetry_Init(&htim2, &htim3);
Robot_Update_IMU(&hi2c1);
DR_Init(robot_imu.heading_rad);
Modes_Init();
Turret_Init();

HAL_UART_Receive_IT(&huart3, gps_rx_buffer,      sizeof(gps_rx_buffer));
HAL_UART_Receive_IT(&huart4, operator_rx_buffer, sizeof(operator_rx_buffer));
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
while (1)
{
    Telemetry_Update_Encoders(&htim2, &htim3);
    Robot_Update_IMU(&hi2c1);
    Robot_Check_Safety(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);

    DR_Update(robot_encoder_left.current_velocity,
              robot_encoder_right.current_velocity,
              robot_imu.heading_rad,
              LOOP_PERIOD_S);

    if (robot_gps.fix_quality > 0)
        DR_Reset_To_GPS(robot_gps.latitude, robot_gps.longitude,
                        ORIGIN_LAT, ORIGIN_LON);

    if (Modes_Is_Safe())
    {
        if (Modes_Is_Autonomous())
        {
            motor_pid_left.target_speed  = autonomous_target_left;
            motor_pid_right.target_speed = autonomous_target_right;
        }
        else
        {
            motor_pid_left.target_speed  = Modes_Get_Left_Speed();
            motor_pid_right.target_speed = Modes_Get_Right_Speed();
        }

        float pid_left  = Robot_Compute_PID(&motor_pid_left,
                              robot_encoder_left.current_velocity);
        float pid_right = Robot_Compute_PID(&motor_pid_right,
                              robot_encoder_right.current_velocity);

        Robot_Drive_Motors(&htim1,
                           TIM_CHANNEL_1, pid_left,
                           TIM_CHANNEL_2, pid_right);

        /* Taret — htim4 (htim3 encoder için ayrıldı) */
        Turret_Update(pixel_err_pitch, pixel_err_yaw,
                      &htim4, TIM_CHANNEL_1,
                      &htim4, TIM_CHANNEL_2,
                      &htim4, TIM_CHANNEL_3);
    }
    else
    {
        Turret_Emergency_Stop(&htim4, TIM_CHANNEL_1,
                               &htim4, TIM_CHANNEL_2,
                               &htim4, TIM_CHANNEL_3);
    }

    ROS_Frame_t frame = Telemetry_Build_ROS_Frame(
                            robot_imu.roll,
                            robot_imu.pitch,
                            robot_imu.yaw);
    HAL_UART_Transmit(&huart2, (uint8_t *)&frame, sizeof(frame), 10);

    HAL_Delay(20);
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        Telemetry_Parse_GPS(gps_rx_buffer, sizeof(gps_rx_buffer));
        HAL_UART_Receive_IT(&huart3, gps_rx_buffer, sizeof(gps_rx_buffer));
    }
    if (huart->Instance == USART4)
    {
        Modes_Parse_Command(operator_rx_buffer, sizeof(operator_rx_buffer));
        HAL_UART_Receive_IT(&huart4, operator_rx_buffer,
                            sizeof(operator_rx_buffer));
    }
}
/* USER CODE END 4 */
