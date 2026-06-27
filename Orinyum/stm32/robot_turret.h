#ifndef ROBOT_TURRET_H
#define ROBOT_TURRET_H

#include <stdint.h>
#include <stdbool.h>

#define TURRET_PID_DT        0.020f   /* s   – döngü periyodu       */
#define TURRET_LOCK_THRESH   3.0f     /* px  – kilitlenme eşiği     */
#define TURRET_LOCK_TIME_MS  1000U    /* ms  – kilitli kalma süresi */
#define TURRET_PWM_LIMIT     8191U    /* ESP32 13-bit LEDC max PWM  */

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error_sum;
    float last_error;
} Turret_PID_t;

typedef enum {
    TURRET_SEARCHING = 0,
    TURRET_TRACKING,
    TURRET_LOCKED,
    TURRET_FIRING
} TurretState_t;

extern TurretState_t turret_state;

void Turret_Init(void);
void Turret_Update(float pixel_err_pitch, float pixel_err_yaw);
void Turret_Emergency_Stop(void);

#endif /* ROBOT_TURRET_H */
