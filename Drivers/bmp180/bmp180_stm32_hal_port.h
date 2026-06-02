#ifndef BMP180_STM32_HAL_PORT_H_
#define BMP180_STM32_HAL_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bmp180_driver.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

typedef struct
{
    I2C_HandleTypeDef *pxI2cHandle;                         /* STM32 HAL I2C handle pointer'ıdır; port katmanı gerçek I2C transferlerini bununla yapar. */
    osMutexId_t xBusMutex;                                  /* MPU6050 ve BMP180 aynı I2C bus'ı paylaşıyorsa kullanılan CMSIS-RTOS2 mutex handle'ıdır. */
} ts_Bmp180_Stm32BusContext;

/**
 * @brief BMP180 soyut bus read callback'ini STM32 HAL_I2C_Mem_Read çağrısına uyarlar.
 * @param u8DeviceAddr BMP180 7-bit I2C adresidir; port katmanı STM32 HAL için sola kaydırır.
 * @param u8RegisterAddr Okunacak BMP180 register başlangıç adresidir.
 * @param pu8ReadData Okunan baytların yazılacağı uygulama/driver buffer'ıdır.
 * @param u16ReadLen Okunacak bayt sayısıdır.
 * @param u32TimeoutMs HAL transfer timeout süresidir.
 * @param vpBusContext ts_Bmp180_Stm32BusContext pointer'ıdır.
 * @return DRIVER_OK başarıyı, diğer kodlar HAL/bus hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Stm32Hal_Read(uint8_t u8DeviceAddr,
                                        uint8_t u8RegisterAddr,
                                        uint8_t *pu8ReadData,
                                        uint16_t u16ReadLen,
                                        uint32_t u32TimeoutMs,
                                        void *vpBusContext);

/**
 * @brief BMP180 soyut bus write callback'ini STM32 HAL_I2C_Mem_Write çağrısına uyarlar.
 * @param u8DeviceAddr BMP180 7-bit I2C adresidir; port katmanı STM32 HAL için sola kaydırır.
 * @param u8RegisterAddr Yazılacak BMP180 register başlangıç adresidir.
 * @param pu8WriteData Yazılacak veriyi taşıyan kaynak buffer'dır.
 * @param u16WriteLen Yazılacak bayt sayısıdır.
 * @param u32TimeoutMs HAL transfer timeout süresidir.
 * @param vpBusContext ts_Bmp180_Stm32BusContext pointer'ıdır.
 * @return DRIVER_OK başarıyı, diğer kodlar HAL/bus hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Stm32Hal_Write(uint8_t u8DeviceAddr,
                                         uint8_t u8RegisterAddr,
                                         const uint8_t *pu8WriteData,
                                         uint16_t u16WriteLen,
                                         uint32_t u32TimeoutMs,
                                         void *vpBusContext);

/**
 * @brief BMP180 servis timing callback'idir; scheduler çalışıyorsa osDelay, henüz başlamadıysa HAL_Delay kullanır.
 * @param u32DelayMs Beklenecek milisaniye süresidir.
 * @param vpBusContext Kullanılmayan port bağlamıdır; callback imzası tutarlılığı için vardır.
 * @return DRIVER_OK bekleme tamamlandığında, DRIVER_ERR_IO scheduler beklemesi başarısız olduğunda döner.
 */
te_Driver_RetCode Bmp180_Stm32Hal_DelayMs(uint32_t u32DelayMs, void *vpBusContext);

/**
 * @brief BMP180 timestamp callback'i olarak HAL_GetTick kullanan fonksiyondur.
 * @param vpBusContext Kullanılmayan port bağlamıdır; callback imzası tutarlılığı için vardır.
 * @return HAL tick değerini milisaniye cinsinden döndürür.
 */
uint32_t Bmp180_Stm32Hal_GetTickMs(void *vpBusContext);

/**
 * @brief Paylaşılan I2C bus mutex'ini CMSIS-RTOS2 osMutexAcquire ile alır.
 * @param u32TimeoutMs Mutex bekleme timeout süresidir.
 * @param vpBusContext ts_Bmp180_Stm32BusContext pointer'ıdır.
 * @return DRIVER_OK başarıyı, timeout veya argüman hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Stm32Hal_Lock(uint32_t u32TimeoutMs, void *vpBusContext);

/**
 * @brief Paylaşılan I2C bus mutex'ini CMSIS-RTOS2 osMutexRelease ile bırakır.
 * @param vpBusContext ts_Bmp180_Stm32BusContext pointer'ıdır.
 * @return DRIVER_OK başarıyı, durum veya argüman hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Stm32Hal_Unlock(void *vpBusContext);

/**
 * @brief ts_Bmp180_BusInterface yapısını STM32 HAL callback'leriyle doldurur.
 * @param psBusInterface Doldurulacak platform bağımsız bus interface pointer'ıdır.
 * @param psBusContext I2C handle ve opsiyonel mutex içeren STM32 port bağlamıdır.
 * @return DRIVER_OK başarıyı, geçersiz argüman hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Stm32Hal_FillBusInterface(ts_Bmp180_BusInterface *psBusInterface,
                                                    ts_Bmp180_Stm32BusContext *psBusContext);

/**
 * @brief ts_Bmp180_LockInterface yapısını CMSIS-RTOS2 mutex callback'leriyle doldurur.
 * @param psLockInterface Doldurulacak platform bağımsız lock interface pointer'ıdır.
 * @param psBusContext Opsiyonel mutex içeren STM32 port bağlamıdır.
 * @return DRIVER_OK başarıyı, geçersiz argüman hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Stm32Hal_FillLockInterface(ts_Bmp180_LockInterface *psLockInterface,
                                                     ts_Bmp180_Stm32BusContext *psBusContext);

/**
 * @brief ts_Bmp180_TimingInterface yapısını RTOS-aware delay ve HAL_GetTick callback'leriyle doldurur.
 * @param psTimingInterface Doldurulacak platform bağımsız timing interface pointer'ıdır.
 * @return DRIVER_OK başarıyı, geçersiz argüman hatasını bildirir.
 */
te_Driver_RetCode Bmp180_Stm32Hal_FillTimingInterface(ts_Bmp180_TimingInterface *psTimingInterface);

#ifdef __cplusplus
}
#endif

#endif /* BMP180_STM32_HAL_PORT_H_ */
