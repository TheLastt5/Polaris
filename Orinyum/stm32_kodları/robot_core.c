/* ===========================================================================
 * robot_core.c  –  Kontrol ve Güvenlik Çekirdeği
 * Hedef: STM32F4xx  |  HAL sürücü kütüphanesi
 * ===========================================================================*/
#include "robot_core.h"
#include <math.h>

/* ---------------------------------------------------------------------------
 * Global değişken tanımları
 * -------------------------------------------------------------------------*/
volatile IMU_Data_t robot_imu = {
    .roll        = 0.0f,
    .pitch       = 0.0f,
    .yaw         = 0.0f,
    .heading_rad = 0.0f,
    .is_safe     = true
};

/* Kp, Ki, Kd, target, error_sum, last_error, output_limit */
PID_Controller_t motor_pid_left  = {1.5f, 0.10f, 0.05f, 0.0f, 0.0f, 0.0f, 1000.0f};
PID_Controller_t motor_pid_right = {1.5f, 0.10f, 0.05f, 0.0f, 0.0f, 0.0f, 1000.0f};

/* ---------------------------------------------------------------------------
 * Robot_Core_Init
 * -------------------------------------------------------------------------*/
void Robot_Core_Init(void)
{
    robot_imu.is_safe = true;

    motor_pid_left.error_sum  = 0.0f;
    motor_pid_left.last_error = 0.0f;

    motor_pid_right.error_sum  = 0.0f;
    motor_pid_right.last_error = 0.0f;
}

/* ---------------------------------------------------------------------------
 * Robot_Update_IMU
 *   BNO055, EUL_Heading_LSB (0x1A) adresinden 6 byte okur:
 *   [0-1] Yaw  (1/16 derece)
 *   [2-3] Roll (1/16 derece)
 *   [4-5] Pitch(1/16 derece)
 * -------------------------------------------------------------------------*/
void Robot_Update_IMU(I2C_HandleTypeDef *hi2c)
{
    uint8_t buf[6];

    if (HAL_I2C_Mem_Read(hi2c,
                          BNO055_I2C_ADDR,
                          BNO055_REG_EULER,
                          I2C_MEMADD_SIZE_8BIT,
                          buf, 6,
                          100) == HAL_OK)
    {
        robot_imu.yaw   = (float)((int16_t)((uint16_t)buf[1] << 8 | buf[0])) / 16.0f;
        robot_imu.roll  = (float)((int16_t)((uint16_t)buf[3] << 8 | buf[2])) / 16.0f;
        robot_imu.pitch = (float)((int16_t)((uint16_t)buf[5] << 8 | buf[4])) / 16.0f;
        robot_imu.heading_rad = robot_imu.yaw * (3.14159265f / 180.0f);
    }
    /* HAL_ERROR durumunda önceki değerler korunur; is_safe güvenlik
     * kontrolünde değiştirilir, burada dokunulmaz.                         */
}

/* ---------------------------------------------------------------------------
 * Robot_Check_Safety
 *   Histerezisli eğim denetimi. Güvensizse motor PWM sıfırlanır.
 * -------------------------------------------------------------------------*/
void Robot_Check_Safety(TIM_HandleTypeDef *htim,
                         uint32_t ch_left, uint32_t ch_right)
{
    float abs_roll  = fabsf(robot_imu.roll);
    float abs_pitch = fabsf(robot_imu.pitch);

    if (robot_imu.is_safe)
    {
        if (abs_roll > TILT_LIMIT || abs_pitch > TILT_LIMIT)
        {
            robot_imu.is_safe = false;
        }
    }
    else
    {
        /* Her iki açı da (limit - histerezis) altına inerse tekrar güvenli */
        if (abs_roll  < (TILT_LIMIT - TILT_HYSTERESIS) &&
            abs_pitch < (TILT_LIMIT - TILT_HYSTERESIS))
        {
            robot_imu.is_safe = true;
        }
    }

    if (!robot_imu.is_safe)
    {
        __HAL_TIM_SET_COMPARE(htim, ch_left,  0U);
        __HAL_TIM_SET_COMPARE(htim, ch_right, 0U);
    }
}

/* ---------------------------------------------------------------------------
 * Robot_Compute_PID
 *   Anti-windup kırpmalı standart PID hesabı.
 * -------------------------------------------------------------------------*/
float Robot_Compute_PID(PID_Controller_t *pid, float current_speed)
{
    float error      = pid->target_speed - current_speed;

    pid->error_sum  += error * PID_DT;

    /* İntegral doyma koruması */
    if (pid->error_sum >  500.0f) { pid->error_sum =  500.0f; }
    if (pid->error_sum < -500.0f) { pid->error_sum = -500.0f; }

    float derivative = (error - pid->last_error) / PID_DT;
    float output     = (pid->Kp * error)
                     + (pid->Ki * pid->error_sum)
                     + (pid->Kd * derivative);

    pid->last_error = error;

    /* Çıkış sınırlama */
    if (output >  pid->output_limit) { output =  pid->output_limit; }
    if (output < -pid->output_limit) { output = -pid->output_limit; }

    return output;
}

/* ---------------------------------------------------------------------------
 * Robot_Drive_Motors
 *   Pozitif çıkış → ileri, negatif → geri.
 *   Yön pinleri: IN1/IN2 (sol) ve IN3/IN4 (sağ).
 * -------------------------------------------------------------------------*/
void Robot_Drive_Motors(TIM_HandleTypeDef *htim,
                         uint32_t ch_left,  float pid_out_left,
                         uint32_t ch_right, float pid_out_right)
{
    if (!robot_imu.is_safe) { return; }

    /* --- Sol motor yön kontrolü --- */
    if (pid_out_left >= 0.0f)
    {
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_L_IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_L_IN2_PIN, GPIO_PIN_SET);
    }

    /* --- Sağ motor yön kontrolü --- */
    if (pid_out_right >= 0.0f)
    {
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_R_IN3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_R_IN4_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_R_IN3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_GPIO_PORT, MOTOR_R_IN4_PIN, GPIO_PIN_SET);
    }

    /* --- PWM görev döngüsü yaz (mutlak değer) --- */
    __HAL_TIM_SET_COMPARE(htim, ch_left,  (uint32_t)fabsf(pid_out_left));
    __HAL_TIM_SET_COMPARE(htim, ch_right, (uint32_t)fabsf(pid_out_right));
}
