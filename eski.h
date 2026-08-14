/**
 * @file    yrt_daq_api.h
 * @brief   YRT STARDUSTv2 DAQ Kartı Donanım Soyutlama Katmanı (API)
 * @details Modüler yapıda tasarlanmıştır. İhtiyaç duyulmayan alt modüller
 *          anahtarlar üzerinden (0 yapılarak) devre dışı bırakılabilir.
 */

#ifndef YRT_DAQ_API_H
#define YRT_DAQ_API_H

#include "stm32f4xx_hal.h"

/* ==================================================================== */
/* YRT STANDART DURUM BİLDİRİMLERİ          */
/* ==================================================================== */
/**
 * @brief YRT sistemleri için evrensel hata ve durum dönüş tipi.
 * @details HAL kütüphanesine olan bağımlılığı izole etmek için kullanılır.
 */
typedef enum {
    YRT_OK = 0,    /**< İşlem başarıyla tamamlandı */
    YRT_ERROR = 1  /**< İşlem sırasında hata veya zaman aşımı oluştu */
} YRT_Status_t;

/* ==================================================================== */
/* MODÜL AKTİF/PASİF ANAHTARLARI (1 = açık, 0 = kapalı)              */
/* ==================================================================== */
/**
 * @def YRT_SYSTEM_ENABLE
 * @brief Tüm DAQ sistemini komple açar (1) veya kapatır (0).
 */
#define YRT_SYSTEM_ENABLE       1

/**
 * @def YRT_USE_SENSOR_MODULE
 * @brief ADS1115 ve basınç sensörleri modülünü kontrol eder.
 */
#define YRT_USE_SENSOR_MODULE   1

/**
 * @def YRT_USE_VALVE_MODULE
 * @brief Ateşleme, tahliye valfleri ve bildirim (Buzzer/LED) modülünü kontrol eder.
 */
#define YRT_USE_VALVE_MODULE    1

/**
 * @def YRT_USE_SDCARD_MODULE
 * @brief FATFS ve SD Kart veri kaydı (Loglama) modülünü kontrol eder.
 */
#define YRT_USE_SDCARD_MODULE   1

/**
 * @def YRT_USE_CAN_MODULE
 * @brief CAN Bus telemetri haberleşme modülünü kontrol eder.
 */
#define YRT_USE_CAN_MODULE      1

/* ==================================================================== */

#if YRT_SYSTEM_ENABLE == 1

/* --- SENSÖR MODÜLÜ TANIMLAMALARI --- */
#if YRT_USE_SENSOR_MODULE == 1
#include "ads1115.h"

/**
 * @def PT_VOLTAGE_TO_BAR_COEF
 * @brief Basınç sensörü (PT) voltaj değerini fiziksel Bar birimine çeviren katsayı.
 */
#define PT_VOLTAGE_TO_BAR_COEF  10.0f

/**
 * @brief   ADS1115 entegresini I2C üzerinden başlatır.
 * @param   hi2c_ptr Kullanılacak I2C hattının adresi (örn: &hi2c1)
 * @retval  YRT_Status_t Başarılı ise YRT_OK, hata varsa YRT_ERROR döner.
 */
YRT_Status_t YRT_DAQ_API_Init_Sensors(I2C_HandleTypeDef *hi2c_ptr);

/**
 * @brief   PT1 basınç sensöründen veriyi okur ve Bar cinsine çevirir.
 * @param   pressure Okunan değerin yazılacağı float tipindeki değişkenin adresi.
 * @retval  YRT_Status_t İşlem başarılıysa YRT_OK, hata varsa YRT_ERROR döner.
 */
YRT_Status_t YRT_DAQ_API_Read_PT1_Pressure(float *pressure);

/**
 * @brief   PT2 basınç sensöründen veriyi okur ve Bar cinsine çevirir.
 * @param   pressure Okunan değerin yazılacağı float tipindeki değişkenin adresi.
 * @retval  YRT_Status_t İşlem başarılıysa YRT_OK, hata varsa YRT_ERROR döner.
 */
YRT_Status_t YRT_DAQ_API_Read_PT2_Pressure(float *pressure);

/**
 * @brief   PT3 basınç sensöründen veriyi okur ve Bar cinsine çevirir.
 * @param   pressure Okunan değerin yazılacağı float tipindeki değişkenin adresi.
 * @retval  YRT_Status_t İşlem başarılıysa YRT_OK, hata varsa YRT_ERROR döner.
 */
YRT_Status_t YRT_DAQ_API_Read_PT3_Pressure(float *pressure);
#endif /* YRT_USE_SENSOR_MODULE */


/* --- VALF VE BİLDİRİM MODÜLÜ TANIMLAMALARI --- */
#if YRT_USE_VALVE_MODULE == 1
/* CubeMX'in main.h dosyasında ürettiği pin tanımlamalarını kullanır */
#define IGN_VALVE_PORT   YRT_IGN_VALVE_GPIO_Port
#define IGN_VALVE_PIN    YRT_IGN_VALVE_Pin
#define REL_VALVE_PORT   YRT_REL_VALVE_GPIO_Port
#define REL_VALVE_PIN    YRT_REL_VALVE_Pin
#define USR_LED1_PORT    YRT_USR_LED1_GPIO_Port
#define USR_LED1_PIN     YRT_USR_LED1_Pin
#define BUZZER_PORT      YRT_BUZZER_GPIO_Port
#define BUZZER_PIN       YRT_BUZZER_Pin

/**
 * @brief   Valfleri güvenli (kapalı) konuma getirir ve başlangıç uyarısını çalar.
 * @retval  None
 */
void YRT_DAQ_API_Init_Valves(void);

/**
 * @brief   Ateşleme (Ignition) valfini açar.
 * @retval  None
 */
void YRT_DAQ_API_IgnitionValve_Open(void);

/**
 * @brief   Ateşleme (Ignition) valfini kapatır.
 * @retval  None
 */
void YRT_DAQ_API_IgnitionValve_Close(void);

/**
 * @brief   Tahliye (Release) valfini açar.
 * @retval  None
 */
void YRT_DAQ_API_ReleaseValve_Open(void);

/**
 * @brief   Tahliye (Release) valfini kapatır.
 * @retval  None
 */
void YRT_DAQ_API_ReleaseValve_Close(void);

/**
 * @brief   Sesli uyarı (Buzzer) modülünün durumunu ayarlar.
 * @param   state 1 ise buzzer öter, 0 ise susar.
 * @retval  None
 */
void YRT_DAQ_API_Buzzer_Set(uint8_t state);

/**
 * @brief   Kritik sistem hatası durumunda devreye giren alarm bildirim fonksiyonu.
 * @details Sistem LED1'i yakar ve buzzer'ı sürekli aktif hale getirir.
 * @retval  None
 */
void YRT_DAQ_API_SystemError_Alert(void);

/**
 * @brief   Sistem hatası giderildiğinde uyarıları temizler.
 * @retval  None
 */
void YRT_DAQ_API_SystemError_Clear(void);
#endif /* YRT_USE_VALVE_MODULE */


/* --- SD KART LOGLAMA MODÜLÜ TANIMLAMALARI --- */
#if YRT_USE_SDCARD_MODULE == 1
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

/**
 * @def YRT_LOG_FILENAME
 * @brief SD Karta kaydedilecek CSV dosyasının adı.
 */
#define YRT_LOG_FILENAME "FLIGHT_DATA.csv"

/**
 * @brief   SD Kartı bağlar (mount) ve CSV dosyasının başlık satırını yazar.
 * @retval  YRT_Status_t Başarılı ise YRT_OK, kart bulunamazsa YRT_ERROR döner.
 */
YRT_Status_t YRT_SD_API_Init(void);

/**
 * @brief   DAQ telemetri verilerini SD karta yeni bir satır olarak kaydeder.
 * @param   timestamp_ms Geçen süre (milisaniye)
 * @param   pt1 Bar cinsinden PT1 basınç verisi
 * @param   pt2 Bar cinsinden PT2 basınç verisi
 * @param   pt3 Bar cinsinden PT3 basınç verisi
 * @retval  YRT_Status_t Yazma başarılıysa YRT_OK, hata varsa YRT_ERROR döner.
 */
YRT_Status_t YRT_SD_API_LogData(uint32_t timestamp_ms, float pt1, float pt2, float pt3);

/**
 * @brief   Sistem kapanırken SD kartı güvenli şekilde ayırır (unmount).
 * @retval  None
 */
void YRT_SD_API_DeInit(void);
#endif /* YRT_USE_SDCARD_MODULE */


/* --- CAN BUS HABERLEŞME MODÜLÜ TANIMLAMALARI --- */
#if YRT_USE_CAN_MODULE == 1
/**
 * @brief   CAN Donanımı için filtreleri ayarlar ve modülü başlatır.
 * @param   hcan_ptr Kullanılacak CAN hattının adresi (örn: &hcan1)
 * @retval  YRT_Status_t Başarılı ise YRT_OK döner.
 */
YRT_Status_t YRT_CAN_API_Init(CAN_HandleTypeDef *hcan_ptr);

/**
 * @brief   DAQ Telemetri verilerini 8 byte'lık pakete sıkıştırıp CAN hattına yollar.
 * @param   pt1 Bar cinsinden PT1 basınç verisi
 * @param   pt2 Bar cinsinden PT2 basınç verisi
 * @param   pt3 Bar cinsinden PT3 basınç verisi
 * @retval  YRT_Status_t İletim başarılıysa YRT_OK, hatta hata varsa YRT_ERROR döner.
 */
YRT_Status_t YRT_CAN_API_SendTelemetry(float pt1, float pt2, float pt3);
#endif /* YRT_USE_CAN_MODULE */


/* --- GENEL SİSTEM YÖNETİMİ --- */
/**
 * @brief   Aktif edilmiş tüm YRT modüllerini (Sensör, Valf, SD, CAN) tek seferde başlatır.
 * @param   hi2c_ptr Kullanılacak I2C hattının adresi (örn: &hi2c1)
 * @param   hcan_ptr Kullanılacak CAN hattının adresi (örn: &hcan1)
 * @retval  YRT_Status_t Tüm modüller sorunsuz başlarsa YRT_OK döner.
 */
YRT_Status_t YRT_DAQ_API_Init_System(I2C_HandleTypeDef *hi2c_ptr, CAN_HandleTypeDef *hcan_ptr);

#endif /* YRT_SYSTEM_ENABLE */

#endif /* YRT_DAQ_API_H */
