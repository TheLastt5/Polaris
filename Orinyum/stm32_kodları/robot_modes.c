/* ===========================================================================
 * robot_modes.h  –  Durum Makinesi ve Mod Yönetimi
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#ifndef ROBOT_MODES_H
#define ROBOT_MODES_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------------
 * Robot çalışma modları
 * -------------------------------------------------------------------------*/
typedef enum {
    MODE_MANUAL     = 0,
    MODE_AUTONOMOUS = 1
} RobotMode_t;

/* ---------------------------------------------------------------------------
 * Operatör komut çerçevesi  (PC/ROS → STM32, UART4)
 *
 * Packed yapı (15 byte), little-endian:
 *   uint32  header       = 0xBB66BB66
 *   uint8   mode_request (0=Manuel, 1=Otonom)
 *   float   left_speed   (m/s, negatif=geri)
 *   float   right_speed  (m/s, negatif=geri)
 *   uint8   e_stop       (0=normal, 1=acil dur)
 *   uint16  checksum     (XOR: mode_request → e_stop)
 * -------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    uint32_t header;
    uint8_t  mode_request;
    float    left_speed;
    float    right_speed;
    uint8_t  e_stop;
    uint16_t checksum;
} Operator_Cmd_t;

#define OP_CMD_HEADER  0xBB66BB66UL
#define OP_CMD_SIZE    ((uint16_t)sizeof(Operator_Cmd_t))

/* ---------------------------------------------------------------------------
 * Global değişkenler
 * -------------------------------------------------------------------------*/
extern volatile RobotMode_t current_mode;
extern volatile bool        e_stop_active;

/* ---------------------------------------------------------------------------
 * Fonksiyon prototipleri
 * -------------------------------------------------------------------------*/
void  Modes_Init          (void);

/**
 * [ISR BAĞLAMI] UART alım tamponunu tarar, checksum doğrular,
 * mod ve e-stop'u günceller.
 * İçerik kısa (memcpy + küçük döngü) — düşük baud rate'de kabul edilebilir.
 */
void  Modes_Parse_Command (uint8_t *buf, uint16_t len);

/** @return true → otonom mod aktif VE e-stop yok */
bool  Modes_Is_Autonomous (void);

/** @return true → e-stop yok VE IMU eğim güvenli */
bool  Modes_Is_Safe       (void);

float Modes_Get_Left_Speed (void);
float Modes_Get_Right_Speed(void);

#endif /* ROBOT_MODES_H */
