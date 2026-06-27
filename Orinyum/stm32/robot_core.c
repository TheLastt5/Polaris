#include "robot_core.h"
#include <math.h>

volatile IMU_Data_t robot_imu = {0.0f, 0.0f, 0.0f, 0.0f, true};
PID_Controller_t motor_pid_left  = {1.5f, 0.10f, 0.05f, 0.0f, 0.0f, 0.0f, 8191.0f}; // 13-bit PWM max
PID_Controller_t motor_pid_right = {1.5f, 0.10f, 0.05f, 0.0f, 0.0f, 0.0f, 8191.0f};

void Robot_Core_Init(void) {
    // 1. I2C Başlatma
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    // 2. Motor Yön GPIO'ları
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<MOTOR_L_IN1_PIN) | (1ULL<<MOTOR_L_IN2_PIN) | 
                        (1ULL<<MOTOR_R_IN3_PIN) | (1ULL<<MOTOR_R_IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // 3. LEDC PWM Başlatma (13-bit, 5kHz)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num  = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_ch_l = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_L_PWM_PIN, .duty = 0, .hpoint = 0
    };
    ledc_channel_config(&ledc_ch_l);

    ledc_channel_config_t ledc_ch_r = ledc_ch_l;
    ledc_ch_r.channel = LEDC_CHANNEL_1;
    ledc_ch_r.gpio_num = MOTOR_R_PWM_PIN;
    ledc_channel_config(&ledc_ch_r);
}

void Robot_Update_IMU(void) {
    uint8_t buf[6];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BNO055_REG_EULER, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BNO055_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + 5, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    if (i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10)) == ESP_OK) {
        robot_imu.yaw   = (float)((int16_t)((uint16_t)buf[1] << 8 | buf[0])) / 16.0f;
        robot_imu.roll  = (float)((int16_t)((uint16_t)buf[3] << 8 | buf[2])) / 16.0f;
        robot_imu.pitch = (float)((int16_t)((uint16_t)buf[5] << 8 | buf[4])) / 16.0f;
        robot_imu.heading_rad = robot_imu.yaw * (M_PI / 180.0f);
    }
    i2c_cmd_link_delete(cmd);
}

void Robot_Check_Safety(void) {
    float abs_roll  = fabsf(robot_imu.roll);
    float abs_pitch = fabsf(robot_imu.pitch);
    if (robot_imu.is_safe) {
        if (abs_roll > TILT_LIMIT || abs_pitch > TILT_LIMIT) robot_imu.is_safe = false;
    } else {
        if (abs_roll < (TILT_LIMIT - TILT_HYSTERESIS) && abs_pitch < (TILT_LIMIT - TILT_HYSTERESIS))
            robot_imu.is_safe = true;
    }
    if (!robot_imu.is_safe) Robot_Drive_Motors(0.0f, 0.0f);
}

float Robot_Compute_PID(PID_Controller_t *pid, float current_speed) {
    // Aynı matematik, değişiklik yok. (STM32 koduyla birebir aynı)
    float error = pid->target_speed - current_speed;
    pid->error_sum += error * PID_DT;
    if (pid->error_sum >  500.0f) pid->error_sum =  500.0f;
    if (pid->error_sum < -500.0f) pid->error_sum = -500.0f;
    float derivative = (error - pid->last_error) / PID_DT;
    float output = (pid->Kp * error) + (pid->Ki * pid->error_sum) + (pid->Kd * derivative);
    pid->last_error = error;
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    return output;
}

void Robot_Drive_Motors(float pid_out_left, float pid_out_right) {
    if (!robot_imu.is_safe) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        return;
    }

    gpio_set_level(MOTOR_L_IN1_PIN, pid_out_left >= 0 ? 1 : 0);
    gpio_set_level(MOTOR_L_IN2_PIN, pid_out_left >= 0 ? 0 : 1);
    
    gpio_set_level(MOTOR_R_IN3_PIN, pid_out_right >= 0 ? 1 : 0);
    gpio_set_level(MOTOR_R_IN4_PIN, pid_out_right >= 0 ? 0 : 1);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)fabsf(pid_out_left));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)fabsf(pid_out_right));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}
