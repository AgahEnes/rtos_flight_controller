#ifndef SERVO_STM32_HAL_PORT_H_
#define SERVO_STM32_HAL_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "servo_driver.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

typedef struct
{
    TIM_HandleTypeDef *pxTimHandle;  /* STM32 HAL timer handle pointer'ı; PWM kanalının bağlı olduğu donanım instance'ını port katmanı için taşır. */
    uint32_t u32Channel;             /* HAL TIM_CHANNEL_x değeri; hangi CCR register'ının servo PWM çıkışı olduğunu belirtir. */
    osMutexId_t xServoMutex;         /* Opsiyonel CMSIS-RTOS2 mutex; actuator task ve test kodu aynı timer kanalına erişirse koruma sağlar. */
    uint32_t u32TimerClockHz;        /* Opsiyonel timer kernel clock override; 0 ise port katmanı HAL RCC bilgisiyle clock değerini hesaplar. */
} ts_Servo_Stm32PortContext;

/**
 * @brief Mikro-saniye servo darbesini STM32 timer compare değerine çevirir ve CCR register'ına yazar.
 * @param u32PulseWidthUs Çekirdek driver tarafından güvenli sınırlara kelepçelenmiş PWM darbe genişliği.
 * @param vpCtx ts_Servo_Stm32PortContext pointer'ı; timer handle ve kanal bilgisini taşır.
 * @return DRIVER_OK başarılı yazmada, aksi halde ortak te_Driver_RetCode hata kodu.
 */
te_Driver_RetCode Servo_Stm32Hal_PulseWrite(uint32_t u32PulseWidthUs, void *vpCtx);

/**
 * @brief Servo PWM kaynağı için opsiyonel CMSIS-RTOS2 mutex kilidi alır.
 * @param u32TimeoutMs Mutex bekleme süresi; çekirdek driver tarafından config'ten gelir.
 * @param vpCtx ts_Servo_Stm32PortContext pointer'ı; xServoMutex alanını taşır.
 * @return DRIVER_OK kilit alındığında veya mutex yoksa, aksi halde timeout/state hatası.
 */
te_Driver_RetCode Servo_Stm32Hal_Lock(uint32_t u32TimeoutMs, void *vpCtx);

/**
 * @brief Servo PWM kaynağı için opsiyonel CMSIS-RTOS2 mutex kilidini bırakır.
 * @param vpCtx ts_Servo_Stm32PortContext pointer'ı; xServoMutex alanını taşır.
 * @return DRIVER_OK kilit bırakıldığında veya mutex yoksa, aksi halde state hatası.
 */
te_Driver_RetCode Servo_Stm32Hal_Unlock(void *vpCtx);

/**
 * @brief STM32 HAL tick değerini milisaniye cinsinden döndürür.
 * @param vpCtx Kullanılmayan bağlam pointer'ı; callback imzasını platform bağımsız tutmak için vardır.
 * @return HAL_GetTick çıktısı; slew-rate hesabında zaman kaynağı olarak kullanılır.
 */
uint32_t Servo_Stm32Hal_GetTickMs(void *vpCtx);

/**
 * @brief ts_Servo_PulseInterface yapısını STM32 HAL PWM callback'i ile doldurur.
 * @param psPulseInterface Doldurulacak çekirdek driver PWM arayüzü.
 * @param psPortContext Timer handle, kanal ve opsiyonel mutex bilgisini taşıyan port context.
 * @return DRIVER_OK başarılı doldurmada, aksi halde argüman/konfigürasyon hatası.
 */
te_Driver_RetCode Servo_Stm32Hal_FillPulseInterface(ts_Servo_PulseInterface *psPulseInterface,
                                                     ts_Servo_Stm32PortContext *psPortContext);

/**
 * @brief ts_Servo_LockInterface yapısını STM32/CMSIS lock callback'leri ile doldurur.
 * @param psLockInterface Doldurulacak çekirdek driver lock arayüzü.
 * @param psPortContext Opsiyonel mutex bilgisini taşıyan port context.
 * @return DRIVER_OK başarılı doldurmada, aksi halde argüman hatası.
 */
te_Driver_RetCode Servo_Stm32Hal_FillLockInterface(ts_Servo_LockInterface *psLockInterface,
                                                    ts_Servo_Stm32PortContext *psPortContext);

/**
 * @brief ts_Servo_TimingInterface yapısını STM32 HAL tick callback'i ile doldurur.
 * @param psTimingInterface Doldurulacak çekirdek driver zaman arayüzü.
 * @return DRIVER_OK başarılı doldurmada, aksi halde argüman hatası.
 */
te_Driver_RetCode Servo_Stm32Hal_FillTimingInterface(ts_Servo_TimingInterface *psTimingInterface);

/**
 * @brief OpenConfig içindeki tüm callback arayüzlerini STM32 HAL port fonksiyonlarıyla doldurur.
 * @param psOpenConfig Servo_Open için hazırlanmış konfigürasyon yapısı; fiziksel limit alanlarına dokunulmaz.
 * @param psPortContext Timer handle, kanal ve opsiyonel mutex bilgisini taşıyan port context.
 * @return DRIVER_OK başarılı doldurmada, aksi halde argüman/konfigürasyon hatası.
 */
te_Driver_RetCode Servo_Stm32Hal_FillInterfaces(ts_Servo_OpenConfig *psOpenConfig,
                                                 ts_Servo_Stm32PortContext *psPortContext);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_STM32_HAL_PORT_H_ */
