/**
 * @file    yrt_daq_api.c
 * @brief   YRT STARDUSTv2 DAQ Kartı Fonksiyon İşlemleri
 * @details Modüler yapıdaki alt donanım görevlerini yürütür.
 */

#include "yrt_daq_api.h"

#if YRT_SYSTEM_ENABLE == 1

/* ==================================================================== */
/* 1. SENSÖR MODÜLÜ KODLARI                                             */
/* ==================================================================== */
#if YRT_USE_SENSOR_MODULE == 1

static I2C_HandleTypeDef *daq_i2c;

YRT_Status_t YRT_DAQ_API_Init_Sensors(I2C_HandleTypeDef *hi2c_ptr) {
    if (hi2c_ptr == NULL) return YRT_ERROR;
    daq_i2c = hi2c_ptr;

    if (ADS1115_Init(daq_i2c, ADS1115_DATA_RATE_128, ADS1115_PGA_TWO) != HAL_OK) {
        return YRT_ERROR;
    }
    return YRT_OK;
}

YRT_Status_t YRT_DAQ_API_Read_PT1_Pressure(float *pressure) {
    float raw_voltage = 0.0f;
    if (ADS1115_readSingleEnded(ADS1115_MUX_AIN1, &raw_voltage) == HAL_OK) {
        *pressure = (raw_voltage * PT_VOLTAGE_TO_BAR_COEF);
        return YRT_OK;
    }
    return YRT_ERROR;
}

YRT_Status_t YRT_DAQ_API_Read_PT2_Pressure(float *pressure) {
    float raw_voltage = 0.0f;
    if (ADS1115_readSingleEnded(ADS1115_MUX_AIN2, &raw_voltage) == HAL_OK) {
        *pressure = (raw_voltage * PT_VOLTAGE_TO_BAR_COEF);
        return YRT_OK;
    }
    return YRT_ERROR;
}

YRT_Status_t YRT_DAQ_API_Read_PT3_Pressure(float *pressure) {
    float raw_voltage = 0.0f;
    if (ADS1115_readSingleEnded(ADS1115_MUX_AIN3, &raw_voltage) == HAL_OK) {
        *pressure = (raw_voltage * PT_VOLTAGE_TO_BAR_COEF);
        return YRT_OK;
    }
    return YRT_ERROR;
}

#endif /* YRT_USE_SENSOR_MODULE */


/* ==================================================================== */
/* 2. VALF VE BİLDİRİM MODÜLÜ KODLARI                                   */
/* ==================================================================== */
#if YRT_USE_VALVE_MODULE == 1

void YRT_DAQ_API_Init_Valves(void) {
    YRT_DAQ_API_IgnitionValve_Close();
    YRT_DAQ_API_ReleaseValve_Close();
    YRT_DAQ_API_Buzzer_Set(1);
    HAL_Delay(100);
    YRT_DAQ_API_Buzzer_Set(0);
}

void YRT_DAQ_API_IgnitionValve_Open(void) { HAL_GPIO_WritePin(IGN_VALVE_PORT, IGN_VALVE_PIN, GPIO_PIN_SET); }
void YRT_DAQ_API_IgnitionValve_Close(void) { HAL_GPIO_WritePin(IGN_VALVE_PORT, IGN_VALVE_PIN, GPIO_PIN_RESET); }
void YRT_DAQ_API_ReleaseValve_Open(void) { HAL_GPIO_WritePin(REL_VALVE_PORT, REL_VALVE_PIN, GPIO_PIN_SET); }
void YRT_DAQ_API_ReleaseValve_Close(void) { HAL_GPIO_WritePin(REL_VALVE_PORT, REL_VALVE_PIN, GPIO_PIN_RESET); }
void YRT_DAQ_API_Buzzer_Set(uint8_t state) { HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, state ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void YRT_DAQ_API_SystemError_Alert(void) {
    HAL_GPIO_WritePin(USR_LED1_PORT, USR_LED1_PIN, GPIO_PIN_SET);
    YRT_DAQ_API_Buzzer_Set(1);
}

void YRT_DAQ_API_SystemError_Clear(void) {
    HAL_GPIO_WritePin(USR_LED1_PORT, USR_LED1_PIN, GPIO_PIN_RESET);
    YRT_DAQ_API_Buzzer_Set(0);
}

#endif /* YRT_USE_VALVE_MODULE */


/* ==================================================================== */
/* 3. SD KART MODÜLÜ KODLARI                                            */
/* ==================================================================== */
#if YRT_USE_SDCARD_MODULE == 1

static FATFS yrt_fs;
static FIL yrt_file;
static FRESULT yrt_fres;

YRT_Status_t YRT_SD_API_Init(void) {
    yrt_fres = f_mount(&yrt_fs, "", 1);
    if (yrt_fres != FR_OK) return YRT_ERROR;

    yrt_fres = f_open(&yrt_file, YRT_LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE);
    if (yrt_fres != FR_OK) return YRT_ERROR;

    char header[] = "ZAMAN(ms),PT1(Bar),PT2(Bar),PT3(Bar)\n";
    UINT bytesWrote;
    yrt_fres = f_write(&yrt_file, header, strlen(header), &bytesWrote);
    f_close(&yrt_file);

    if (yrt_fres != FR_OK || bytesWrote == 0) return YRT_ERROR;
    return YRT_OK;
}

YRT_Status_t YRT_SD_API_LogData(uint32_t timestamp_ms, float pt1, float pt2, float pt3) {
    char log_buffer[100];
    UINT bytesWrote;
    snprintf(log_buffer, sizeof(log_buffer), "%lu,%.2f,%.2f,%.2f\n", timestamp_ms, pt1, pt2, pt3);

    yrt_fres = f_open(&yrt_file, YRT_LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE);
    if (yrt_fres != FR_OK) return YRT_ERROR;

    yrt_fres = f_write(&yrt_file, log_buffer, strlen(log_buffer), &bytesWrote);
    f_close(&yrt_file);

    if (yrt_fres != FR_OK || bytesWrote == 0) return YRT_ERROR;
    return YRT_OK;
}

void YRT_SD_API_DeInit(void) {
    f_mount(NULL, "", 0);
}

#endif /* YRT_USE_SDCARD_MODULE */


/* ==================================================================== */
/* 4. CAN BUS HABERLEŞME MODÜLÜ KODLARI                                 */
/* ==================================================================== */
#if YRT_USE_CAN_MODULE == 1

static CAN_HandleTypeDef *daq_can;
static CAN_TxHeaderTypeDef txHeader;
static uint32_t txMailbox;

YRT_Status_t YRT_CAN_API_Init(CAN_HandleTypeDef *hcan_ptr) {
    if (hcan_ptr == NULL) return YRT_ERROR;
    daq_can = hcan_ptr;

    /* Donanımsal CAN Filtresini Ayarla */
    CAN_FilterTypeDef canFilterConfig = {0};
    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000;
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(daq_can, &canFilterConfig) != HAL_OK) return YRT_ERROR;

    /* CAN Modülünü Başlat */
    if (HAL_CAN_Start(daq_can) != HAL_OK) return YRT_ERROR;

    /* Mesaj Başlığını Ayarla (Supervisor ID: 0x100) */
    txHeader.StdId = 0x100;
    txHeader.ExtId = 0x01;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 6; /* Gönderilecek Veri Uzunluğu (6 Byte) */

    return YRT_OK;
}

YRT_Status_t YRT_CAN_API_SendTelemetry(float pt1, float pt2, float pt3) {
    uint8_t payload[8] = {0};

    /* Verileri virgülden kurtarmak için 100 ile çarpıp tam sayıya çeviriyoruz (Örn: 15.42 Bar -> 1542) */
    uint16_t pt1_scaled = (uint16_t)(pt1 * 100.0f);
    uint16_t pt2_scaled = (uint16_t)(pt2 * 100.0f);
    uint16_t pt3_scaled = (uint16_t)(pt3 * 100.0f);

    /* 6 Byte'lık veriyi payload dizisine yerleştir */
    payload[0] = (pt1_scaled >> 8) & 0xFF; /* PT1 High Byte */
    payload[1] = pt1_scaled & 0xFF;        /* PT1 Low Byte */

    payload[2] = (pt2_scaled >> 8) & 0xFF; /* PT2 High Byte */
    payload[3] = pt2_scaled & 0xFF;        /* PT2 Low Byte */

    payload[4] = (pt3_scaled >> 8) & 0xFF; /* PT3 High Byte */
    payload[5] = pt3_scaled & 0xFF;        /* PT3 Low Byte */

    /* Veriyi hatta bas */
    if (HAL_CAN_AddTxMessage(daq_can, &txHeader, payload, &txMailbox) != HAL_OK) {
        return YRT_ERROR;
    }
    return YRT_OK;
}

#endif /* YRT_USE_CAN_MODULE */


/* ==================================================================== */
/* GENEL SİSTEM BAŞLATMA FONKSİYONU                                     */
/* ==================================================================== */
YRT_Status_t YRT_DAQ_API_Init_System(I2C_HandleTypeDef *hi2c_ptr, CAN_HandleTypeDef *hcan_ptr) {
    YRT_Status_t status = YRT_OK;

    #if YRT_USE_SENSOR_MODULE == 1
    if (YRT_DAQ_API_Init_Sensors(hi2c_ptr) != YRT_OK) status = YRT_ERROR;
    #endif

    #if YRT_USE_VALVE_MODULE == 1
    YRT_DAQ_API_Init_Valves();
    #endif

    #if YRT_USE_SDCARD_MODULE == 1
    if (YRT_SD_API_Init() != YRT_OK) status = YRT_ERROR;
    #endif

    #if YRT_USE_CAN_MODULE == 1
    if (YRT_CAN_API_Init(hcan_ptr) != YRT_OK) status = YRT_ERROR;
    #endif

    return status;
}

#endif /* YRT_SYSTEM_ENABLE == 1 */
