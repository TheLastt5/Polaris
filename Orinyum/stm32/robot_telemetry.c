#include "robot_telemetry.h"
#include <string.h>
#include <stdlib.h>

GPS_Data_t     robot_gps           = {0.0f, 0.0f, 0U};
Encoder_Data_t robot_encoder_left  = {0, 0.0f};
Encoder_Data_t robot_encoder_right = {0, 0.0f};

static void pcnt_init_channel(pcnt_unit_t unit, int pinA, int pinB) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pinA, .ctrl_gpio_num = pinB,
        .channel = PCNT_CHANNEL_0, .unit = unit,
        .pos_mode = PCNT_COUNT_INC, .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_KEEP, .hctrl_mode = PCNT_MODE_REVERSE,
        .counter_h_lim = 30000, .counter_l_lim = -30000,
    };
    pcnt_unit_config(&pcnt_config);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void Telemetry_Init(void) {
    // 1. Enkoder PCNT (Sol = Unit 0, Sağ = Unit 1)
    pcnt_init_channel(PCNT_UNIT_0, ENC_L_A, ENC_L_B);
    pcnt_init_channel(PCNT_UNIT_1, ENC_R_A, ENC_R_B);

    // 2. ROS UART (UART0, USB üzerinden)
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    // 3. GPS UART (UART1)
    uart_config_t uart_config_gps = {
        .baud_rate = 9600, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config_gps);
    uart_set_pin(UART_NUM_1, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, 512, 0, 0, NULL, 0);

    // 4. Operatör UART (UART2)
    uart_config_t uart_config_op = uart_config_gps;
    uart_config_op.baud_rate = 115200;
    uart_param_config(UART_NUM_2, &uart_config_op);
    uart_set_pin(UART_NUM_2, OP_TX_PIN, OP_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2, 256, 0, 0, NULL, 0);
}

void Telemetry_Update_Encoders(void) {
    int16_t count_l, count_r;
    pcnt_get_counter_value(PCNT_UNIT_0, &count_l);
    pcnt_get_counter_value(PCNT_UNIT_1, &count_r);
    
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);

    // ESP32 PCNT 16-bit okur ama aralık her 20ms için fazlasıyla yeterli
    robot_encoder_left.current_velocity  = ((float)count_l * DISTANCE_PER_PULSE) / LOOP_PERIOD_S;
    robot_encoder_right.current_velocity = ((float)count_r * DISTANCE_PER_PULSE) / LOOP_PERIOD_S;
    robot_encoder_left.total_pulses  += count_l;
    robot_encoder_right.total_pulses += count_r;
}

static float nmea_to_decimal(const char *field) {
    if (!field || field[0] == '\0') return 0.0f;
    float raw = strtof(field, NULL);
    int degrees = (int)(raw / 100.0f);
    return (float)degrees + (raw - (float)(degrees * 100)) / 60.0f;
}

// Artık direkt buf alıyor, Task içinde güvenle strtok yapılabilir.
void Telemetry_Parse_GPS_Buffer(uint8_t *nmea_buffer) {
    char *buf = (char *)nmea_buffer;
    if (!strstr(buf, "$GPGGA")) return;

    char *fields[15];
    uint8_t count = 0;
    char *tok = strtok(buf, ",");
    while (tok && count < 15) {
        fields[count++] = tok;
        tok = strtok(NULL, ",");
    }

    if (count < 7) return;
    
    float lat = nmea_to_decimal(fields[2]);
    robot_gps.latitude = (fields[3][0] == 'S') ? -lat : lat;
    float lon = nmea_to_decimal(fields[4]);
    robot_gps.longitude = (fields[5][0] == 'W') ? -lon : lon;
    robot_gps.fix_quality = (uint8_t)atoi(fields[6]);
}

static uint16_t calc_checksum(const ROS_Frame_t *f) {
    const uint8_t *p = (const uint8_t *)&f->imu_roll;
    uint16_t sum = 0;
    for (int i=0; i < 29; i++) sum ^= *p++; // 29 bytes from imu_roll to gps_fix
    return sum;
}

ROS_Frame_t Telemetry_Build_ROS_Frame(float roll, float pitch, float yaw) {
    ROS_Frame_t f = {
        .header = ROS_FRAME_HEADER, .imu_roll = roll, .imu_pitch = pitch, .imu_yaw = yaw,
        .encoder_vel_left = robot_encoder_left.current_velocity,
        .encoder_vel_right = robot_encoder_right.current_velocity,
        .gps_lat = robot_gps.latitude, .gps_lon = robot_gps.longitude, .gps_fix = robot_gps.fix_quality
    };
    f.checksum = calc_checksum(&f);
    return f;
}
