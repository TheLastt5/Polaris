#ifndef ROBOT_MODES_H
#define ROBOT_MODES_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MODE_MANUAL     = 0,
    MODE_AUTONOMOUS = 1
} RobotMode_t;

typedef struct __attribute__((packed)) {
    uint32_t header;          
    uint8_t  mode_request;    
    float    left_speed;      
    float    right_speed;     
    uint8_t  e_stop;          
    uint16_t checksum;
} Operator_Cmd_t;

#define OP_CMD_HEADER  0xBB66BB66U
#define OP_CMD_SIZE    sizeof(Operator_Cmd_t)

extern volatile RobotMode_t current_mode;
extern volatile bool        e_stop_active;

void  Modes_Init          (void);
void  Modes_Parse_Command (uint8_t *buf, uint16_t len);
bool  Modes_Is_Autonomous (void);
bool  Modes_Is_Safe       (void);  
float Modes_Get_Left_Speed(void);
float Modes_Get_Right_Speed(void);

#endif /* ROBOT_MODES_H */
