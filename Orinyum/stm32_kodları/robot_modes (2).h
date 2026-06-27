/* ===========================================================================
 * robot_modes.c  –  Durum Makinesi ve Mod Yönetimi
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#include "robot_modes.h"
#include "robot_core.h"   /* robot_imu.is_safe için */
#include <string.h>

/* ---------------------------------------------------------------------------
 * Global değişken tanımları
 * -------------------------------------------------------------------------*/
volatile RobotMode_t current_mode  = MODE_MANUAL;
volatile bool        e_stop_active = false;

/* Geçerli son komut (hız bilgisini saklar) */
static Operator_Cmd_t last_cmd;

/* ---------------------------------------------------------------------------
 * cmd_checksum  (yerel yardımcı)
 *   XOR toplamı: mode_request'ten checksum alanının öncesine kadar.
 * -------------------------------------------------------------------------*/
static uint16_t cmd_checksum(const Operator_Cmd_t *c)
{
    const uint8_t *p   = (const uint8_t *)&c->mode_request;
    const uint8_t *end = (const uint8_t *)&c->checksum;
    uint16_t       sum = 0U;

    while (p < end) { sum ^= (uint16_t)*p++; }
    return sum;
}

/* ---------------------------------------------------------------------------
 * Modes_Init
 * -------------------------------------------------------------------------*/
void Modes_Init(void)
{
    current_mode  = MODE_MANUAL;
    e_stop_active = false;
    memset(&last_cmd, 0, sizeof(last_cmd));
}

/* ---------------------------------------------------------------------------
 * Modes_Parse_Command
 *   Tamponu tarar, magic word'ü bulur, checksum doğrular.
 *   Geçerli frame bulunursa mod ve e-stop güncellenir.
 * -------------------------------------------------------------------------*/
void Modes_Parse_Command(uint8_t *buf, uint16_t len)
{
    if (len < OP_CMD_SIZE) { return; }

    /* Magic word baytlarını yerel değişkene al (endianness taşınabilirliği) */
    uint8_t  magic[4];
    uint32_t h = OP_CMD_HEADER;
    memcpy(magic, &h, 4U);

    for (uint16_t i = 0U; i <= (len - OP_CMD_SIZE); i++)
    {
        /* Header kontrolü */
        if (memcmp(&buf[i], magic, 4U) != 0) { continue; }

        /* Çerçeveyi yapıya kopyala */
        Operator_Cmd_t cmd;
        memcpy(&cmd, &buf[i], OP_CMD_SIZE);

        /* Checksum doğrulama – bozuk frame'leri reddet */
        if (cmd_checksum(&cmd) != cmd.checksum) { continue; }

        /* Acil durdurma önceliklidir */
        if (cmd.e_stop != 0U)
        {
            e_stop_active = true;
            current_mode  = MODE_MANUAL;
            return;
        }

        e_stop_active = false;
        current_mode  = (cmd.mode_request == 1U) ? MODE_AUTONOMOUS : MODE_MANUAL;
        last_cmd      = cmd;
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Durum sorgu fonksiyonları
 * -------------------------------------------------------------------------*/
bool Modes_Is_Autonomous(void)
{
    return (!e_stop_active) && (current_mode == MODE_AUTONOMOUS);
}

bool Modes_Is_Safe(void)
{
    return (!e_stop_active) && (robot_imu.is_safe);
}

float Modes_Get_Left_Speed(void)  { return last_cmd.left_speed;  }
float Modes_Get_Right_Speed(void) { return last_cmd.right_speed; }
