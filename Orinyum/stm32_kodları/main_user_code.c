/* ===========================================================================
 * main.c  –  USER CODE blokları
 * ===========================================================================*/

/* USER CODE BEGIN Includes */
#include "robot_core.h"
#include "robot_telemetry.h"
#include "robot_modes.h"
#include "robot_dead_reckoning.h"
#include "robot_turret.h"
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN 0 */
#define ORIGIN_LAT  41.000000f   /* harita başlangıç enlemi  */
#define ORIGIN_LON  29.000000f   /* harita başlangıç boylamı */

/* Otonom mod hız hedefleri (Dışarıdan erişim için static kaldırıldı, volatile yapıldı) */
volatile float autonomous_target_left  = 0.0f;
volatile float autonomous_target_right = 0.0f;

/* Kamera piksel hata değerleri */
static volatile float pixel_err_pitch = 0.0f;
static volatile float pixel_err_yaw   = 0.0f;

/* UART alım tamponları ve ISR Emniyet Katmanı Değişkenleri */
static uint8_t gps_rx_buffer[128];
static uint8_t operator_rx_buffer[sizeof(Operator_Cmd_t) + 4U];

volatile bool gps_data_ready = false;
uint8_t gps_process_buffer[128];
/* USER CODE END 0 */


/* USER CODE BEGIN 2 */
Robot_Core_Init();
Telemetry_Init(&htim2, &htim3);
Robot_Update_IMU(&hi2c1);
DR_Init(robot_imu.heading_rad);
Modes_Init();
Turret_Init();

/* UART interrupt alımlarını başlat */
HAL_UART_Receive_IT(&huart3, gps_rx_buffer,      sizeof(gps_rx_buffer));
HAL_UART_Receive_IT(&huart4, operator_rx_buffer, sizeof(operator_rx_buffer));
/* USER CODE END 2 */


/* USER CODE BEGIN 3 */
while (1)
{
    /* 0. İki Aşamalı Seri Veri İşleme (strtok'ı ISR dışına çıkarma) */
    if (gps_data_ready)
    {
        Telemetry_Parse_GPS(gps_process_buffer, sizeof(gps_process_buffer));
        gps_data_ready = false;
    }

    /* 1. Sensör güncelleme */
    Telemetry_Update_Encoders(&htim2, &htim3);
    Robot_Update_IMU(&hi2c1);

    /* 2. Güvenlik kontrolü (eğim → otomatik motor durdurma) */
    Robot_Check_Safety(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2);

    /* 3. Konum kestirimi */
    DR_Update(robot_encoder_left.current_velocity,
              robot_encoder_right.current_velocity,
              robot_imu.heading_rad,
              LOOP_PERIOD_S);

    /* 4. GPS fix varsa konumu düzelt */
    if (robot_gps.fix_quality > 0U)
    {
        DR_Reset_To_GPS(robot_gps.latitude, robot_gps.longitude,
                        ORIGIN_LAT, ORIGIN_LON);
    }

    /* 5. Motor kontrolü (güvenli modda) */
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

        /* 6. Taret güncelleme */
        Turret_Update((float)pixel_err_pitch, (float)pixel_err_yaw,
                      &htim4, TIM_CHANNEL_1,
                      &htim4, TIM_CHANNEL_2,
                      &htim4, TIM_CHANNEL_3);
    }
    else
    {
        /* Güvensiz durum: taret acil durdurma */
        Turret_Emergency_Stop(&htim4, TIM_CHANNEL_1,
                               &htim4, TIM_CHANNEL_2,
                               &htim4, TIM_CHANNEL_3);
    }

    /* 7. Telemetri çerçevesini ROS'a gönder */
    ROS_Frame_t frame = Telemetry_Build_ROS_Frame(
                            robot_imu.roll,
                            robot_imu.pitch,
                            robot_imu.yaw);

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)&frame,
                      (uint16_t)sizeof(frame),
                      10U);

    HAL_Delay(20U);  /* 50 Hz döngü */
}
/* USER CODE END 3 */


/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        /* Stage 1: Sadece veriyi emniyetli tampona kopyala ve çık */
        memcpy(gps_process_buffer, gps_rx_buffer, sizeof(gps_rx_buffer));
        gps_data_ready = true;
        
        HAL_UART_Receive_IT(&huart3,
                            gps_rx_buffer,
                            sizeof(gps_rx_buffer));
    }

    if (huart->Instance == UART4)
    {
        Modes_Parse_Command(operator_rx_buffer, sizeof(operator_rx_buffer));
        HAL_UART_Receive_IT(&huart4,
                            operator_rx_buffer,
                            sizeof(operator_rx_buffer));
    }
}
/* USER CODE END 4 */
