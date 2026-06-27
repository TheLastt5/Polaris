#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "robot_core.h"
#include "robot_telemetry.h"
#include "robot_modes.h"
#include "robot_dead_reckoning.h"
#include "robot_turret.h"

#define ORIGIN_LAT  41.000000f
#define ORIGIN_LON  29.000000f

volatile float autonomous_target_left  = 0.0f;
volatile float autonomous_target_right = 0.0f;
volatile float pixel_err_pitch = 0.0f;
volatile float pixel_err_yaw   = 0.0f;

/* --- FreeRTOS Görevleri --- */

// 50Hz (20ms) Ana Kontrol Döngüsü
void task_control_loop(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    while (1) {
        Telemetry_Update_Encoders();
        Robot_Update_IMU();
        Robot_Check_Safety();

        DR_Update(robot_encoder_left.current_velocity,
                  robot_encoder_right.current_velocity,
                  robot_imu.heading_rad, LOOP_PERIOD_S);

        if (robot_gps.fix_quality > 0U) {
            DR_Reset_To_GPS(robot_gps.latitude, robot_gps.longitude, ORIGIN_LAT, ORIGIN_LON);
        }

        if (Modes_Is_Safe()) {
            if (Modes_Is_Autonomous()) {
                motor_pid_left.target_speed  = autonomous_target_left;
                motor_pid_right.target_speed = autonomous_target_right;
            } else {
                motor_pid_left.target_speed  = Modes_Get_Left_Speed();
                motor_pid_right.target_speed = Modes_Get_Right_Speed();
            }

            float pid_left  = Robot_Compute_PID(&motor_pid_left, robot_encoder_left.current_velocity);
            float pid_right = Robot_Compute_PID(&motor_pid_right, robot_encoder_right.current_velocity);

            Robot_Drive_Motors(pid_left, pid_right);
            Turret_Update(pixel_err_pitch, pixel_err_yaw);
        } else {
            Turret_Emergency_Stop();
        }

        /* ROS Çerçevesi Gönderimi (UART0) */
        ROS_Frame_t frame = Telemetry_Build_ROS_Frame(robot_imu.roll, robot_imu.pitch, robot_imu.yaw);
        uart_write_bytes(UART_NUM_0, (const char*)&frame, sizeof(frame));

        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Tam 50Hz senkronizasyon
    }
}

// GPS Dinleme Görevi (Bloklayıcı, CPU yormaz)
void task_gps_rx(void *pvParameters) {
    uint8_t rx_buf[128];
    while (1) {
        int rx_bytes = uart_read_bytes(UART_NUM_1, rx_buf, sizeof(rx_buf) - 1, pdMS_TO_TICKS(100));
        if (rx_bytes > 0) {
            rx_buf[rx_bytes] = '\0';
            Telemetry_Parse_GPS_Buffer(rx_buf);
        }
    }
}

// Operatör Komut Dinleme Görevi
void task_op_rx(void *pvParameters) {
    uint8_t rx_buf[64];
    while (1) {
        int rx_bytes = uart_read_bytes(UART_NUM_2, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(100));
        if (rx_bytes > 0) {
            Modes_Parse_Command(rx_buf, rx_bytes);
        }
    }
}

void app_main(void) {
    // Donanım Başlatma
    Robot_Core_Init();
    Telemetry_Init();
    Turret_Init();
    DR_Init(0.0f);
    Modes_Init();

    // Görevleri Oluştur (Core 1'e sabitleyebilir veya serbest bırakabilirsin)
    xTaskCreate(task_control_loop, "CtrlLoop", 4096, NULL, 5, NULL);
    xTaskCreate(task_gps_rx,       "GPS_Rx",   4096, NULL, 3, NULL);
    xTaskCreate(task_op_rx,        "Op_Rx",    4096, NULL, 4, NULL);
}
