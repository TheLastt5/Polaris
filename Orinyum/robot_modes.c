#include "robot_modes.h"
#include "robot_core.h"
#include <string.h>

volatile RobotMode_t current_mode  = MODE_MANUAL;
volatile bool        e_stop_active = false;

static Operator_Cmd_t last_cmd = {0};

static uint16_t cmd_checksum(const Operator_Cmd_t *c)
{
    const uint8_t *p   = (const uint8_t *)&c->mode_request;
    const uint8_t *end = (const uint8_t *)&c->checksum;
    uint16_t       sum = 0;
    while (p < end) sum ^= *p++;
    return sum;
}

void Modes_Init(void)
{
    current_mode  = MODE_MANUAL;
    e_stop_active = false;
}

void Modes_Parse_Command(uint8_t *buf, uint16_t len)
{
    if (len < (uint16_t)OP_CMD_SIZE) return;

    uint8_t  hdr[4];
    uint32_t h = OP_CMD_HEADER;
    memcpy(hdr, &h, 4);

    for (uint16_t i = 0; i <= len - (uint16_t)OP_CMD_SIZE; i++)
    {
        if (memcmp(&buf[i], hdr, 4) != 0) continue;

        Operator_Cmd_t cmd;
        memcpy(&cmd, &buf[i], OP_CMD_SIZE);

        if (cmd_checksum(&cmd) != cmd.checksum) continue; 

        if (cmd.e_stop)
        {
            e_stop_active = true;
            current_mode  = MODE_MANUAL;
            return;
        }
        e_stop_active = false;

        if (cmd.mode_request == 1)
            current_mode = MODE_AUTONOMOUS;
        else
            current_mode = MODE_MANUAL;

        last_cmd = cmd;
        break;
    }
}

bool Modes_Is_Autonomous(void)
{
    return (!e_stop_active) && (current_mode == MODE_AUTONOMOUS);
}

bool Modes_Is_Safe(void)
{
    return (!e_stop_active) && (robot_imu.is_safe);
}

float Modes_Get_Left_Speed (void) { return last_cmd.left_speed;  }
float Modes_Get_Right_Speed(void) { return last_cmd.right_speed; }
