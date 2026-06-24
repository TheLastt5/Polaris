/* ===========================================================================
 * main.c  –  USER CODE blokları
 * STM32CubeMX projesindeki main.c'ye yapıştırın.
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 *
 * Pin / peripheral özeti (CubeMX'te eşleşmeli):
 *   htim1  – Motor PWM  (CH1: sol, CH2: sağ)
 *   htim2  – Encoder sol (TIM_ENCODERMODE_TI12)
 *   htim3  – Encoder sağ (TIM_ENCODERMODE_TI12)
 *   htim4  – Taret PWM  (CH1: pitch, CH2: yaw, CH3: lazer)
 *   hi2c1  – BNO055 IMU (0x28)
 *   huart2 – ROS'a telemetri gönderme (115200)
 *   huart3 – GPS NMEA alma   (9600 / 115200)
 *   huart4 – Operatör komutu alma (115200)
 *   GPIOA  – PA1/PA2 sol motor IN, PA3/PA4 sağ motor IN
 * ===========================================================================*/

/* USER CODE BEGIN Includes */
#include "robot_core.h"
#include "robot_telemetry.h"
#include "robot_modes.h"
#include "robot_dead_reckoning.h"
#include "robot_turret.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 0 */
#define ORIGIN_LAT  41.000000f   /* harita başlangıç enlemi  */
#define ORIGIN_LON  29.000000f   /* harita başlangıç boylamı */

/* Otonom mod hız hedefleri (ROS/görev planı tarafından doldurulur) */
static float autonomous_target_left  = 0.0f;
static float autonomous_target_right = 0.0f;

/* Kamera piksel hata değerleri (görüntü işleme ISR / ROS callback'ten gelir) */
static volatile float pixel_err_pitch = 0.0f;
static volatile float pixel_err_yaw   = 0.0f;

/* UART alım tamponları */
static uint8_t gps_rx_buffer[128];
static uint8_t operator_rx_buffer[sizeof(Operator_Cmd_t) + 4U];
/* USER CODE END 0 */


/* USER CODE BEGIN 2  (MX_xxx_Init() çağrılarının ardından) */
Robot_Core_Init();
Telemetry_Init(&htim2, &htim3);
Robot_Update_IMU(&hi2c1);                    /* IMU'yu ilk kez oku          */
DR_Init(robot_imu.heading_rad);              /* Dead reckoning'i IMU yönüyle başlat */
Modes_Init();
Turret_Init();

/* UART interrupt alımlarını başlat */
HAL_UART_Receive_IT(&huart3, gps_rx_buffer,      sizeof(gps_rx_buffer));
HAL_UART_Receive_IT(&huart4, operator_rx_buffer, sizeof(operator_rx_buffer));
/* USER CODE END 2 */


/* USER CODE BEGIN 3 */
while (1)
{
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

        /* 6. Taret – htim4 (htim3 encoder için ayrıldı) */
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

    /* 7. Telemetri çerçevesi ROS'a gönder */
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
/**
 * @brief UART alım tamamlama ISR
 *        GPS ve operatör komutlarını non-blocking olarak işler.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        Telemetry_Parse_GPS(gps_rx_buffer, sizeof(gps_rx_buffer));
        HAL_UART_Receive_IT(&huart3,
                            gps_rx_buffer,
                            sizeof(gps_rx_buffer));
    }

    if (huart->Instance == UART4)   /* USART4 → UART4 olarak tanımlı */
    {
        Modes_Parse_Command(operator_rx_buffer, sizeof(operator_rx_buffer));
        HAL_UART_Receive_IT(&huart4,
                            operator_rx_buffer,
                            sizeof(operator_rx_buffer));
    }
}
/* USER CODE END 4 */
