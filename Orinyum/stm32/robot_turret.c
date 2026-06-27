#include "robot_turret.h"
#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#define TURRET_PITCH_PWM_PIN 4
#define TURRET_YAW_PWM_PIN   5
#define TURRET_LASER_PIN     23

#define TURRET_PITCH_IN1 2
#define TURRET_PITCH_IN2 15
#define TURRET_YAW_IN1   18
#define TURRET_YAW_IN2   19

TurretState_t turret_state = TURRET_SEARCHING;
static Turret_PID_t pid_pitch = {2.0f, 0.05f, 0.10f, 0.0f, 0.0f};
static Turret_PID_t pid_yaw   = {2.0f, 0.05f, 0.10f, 0.0f, 0.0f};
static uint32_t lock_start_ms = 0U;

static void turret_set_axis(ledc_channel_t channel, float output, int pin_in1, int pin_in2) {
    gpio_set_level(pin_in1, output >= 0 ? 1 : 0);
    gpio_set_level(pin_in2, output >= 0 ? 0 : 1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, (uint32_t)fabsf(output));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void Turret_Init(void) {
    // Yön pinleri ayarı
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<TURRET_PITCH_IN1)|(1ULL<<TURRET_PITCH_IN2)|
                        (1ULL<<TURRET_YAW_IN1)|(1ULL<<TURRET_YAW_IN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    // Taret PWM (Timer 1 kullanalım)
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_13_BIT, .freq_hz = 5000, .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ch_pitch = { .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_1, .gpio_num = TURRET_PITCH_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE };
    ledc_channel_config_t ch_yaw   = { .channel = LEDC_CHANNEL_3, .timer_sel = LEDC_TIMER_1, .gpio_num = TURRET_YAW_PWM_PIN, .speed_mode = LEDC_LOW_SPEED_MODE };
    ledc_channel_config_t ch_laser = { .channel = LEDC_CHANNEL_4, .timer_sel = LEDC_TIMER_1, .gpio_num = TURRET_LASER_PIN, .speed_mode = LEDC_LOW_SPEED_MODE };
    
    ledc_channel_config(&ch_pitch);
    ledc_channel_config(&ch_yaw);
    ledc_channel_config(&ch_laser);
}

// ... turret_pid_step() AYNI KALACAK ...

void Turret_Update(float pixel_err_pitch, float pixel_err_yaw) {
    // STM32'deki ile aynı State Machine FSM çalışacak.
    // Tek fark `__HAL_TIM_SET_COMPARE` yerine:
    // turret_set_axis(LEDC_CHANNEL_2, out_pitch, TURRET_PITCH_IN1, TURRET_PITCH_IN2);
    // turret_set_axis(LEDC_CHANNEL_3, out_yaw, TURRET_YAW_IN1, TURRET_YAW_IN2);
    // ledc_set_duty(..., LEDC_CHANNEL_4, Lazer_Degeri); ledc_update_duty(...);
}

void Turret_Emergency_Stop(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    turret_state = TURRET_SEARCHING;
}
