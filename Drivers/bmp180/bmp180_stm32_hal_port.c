#include "bmp180_stm32_hal_port.h"

#include <stddef.h>

te_Driver_RetCode Bmp180_Stm32Hal_Read(uint8_t u8DeviceAddr,
                                        uint8_t u8RegisterAddr,
                                        uint8_t *pu8ReadData,
                                        uint16_t u16ReadLen,
                                        uint32_t u32TimeoutMs,
                                        void *vpBusContext)
{
    ts_Bmp180_Stm32BusContext *psContext = (ts_Bmp180_Stm32BusContext *)vpBusContext; /* Soyut ctx pointer'ı STM32 port bağlamına çevrilir. */
    HAL_StatusTypeDef eHalStatus;                            /* HAL_I2C_Mem_Read dönüş değerini BMP180 hata kodlarına map etmek için saklar. */

    if ((psContext == NULL) || (psContext->pxI2cHandle == NULL) || (pu8ReadData == NULL) || (u16ReadLen == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;                       /* I2C handle, hedef buffer ve uzunluk olmadan HAL transferi güvenli değildir. */
    }

    eHalStatus = HAL_I2C_Mem_Read(psContext->pxI2cHandle,
                                  (uint16_t)(u8DeviceAddr << 1U),
                                  u8RegisterAddr,
                                  I2C_MEMADD_SIZE_8BIT,
                                  pu8ReadData,
                                  u16ReadLen,
                                  u32TimeoutMs);             /* Platform bağımsız read callback'i STM32 HAL register okuma fonksiyonuna uyarlanır. */
    if (eHalStatus != HAL_OK)
    {
        return ((eHalStatus == HAL_TIMEOUT) || (psContext->pxI2cHandle->ErrorCode == HAL_I2C_ERROR_TIMEOUT)) ?
                   DRIVER_ERR_TIMEOUT :
                   DRIVER_ERR_BUS;                     /* HAL timeout status/error ayrı, diğer I2C/NACK/bus hataları genel bus error olarak döndürülür. */
    }

    return DRIVER_OK;                                    /* HAL okuma başarıyla tamamlanmıştır. */
}

te_Driver_RetCode Bmp180_Stm32Hal_Write(uint8_t u8DeviceAddr,
                                         uint8_t u8RegisterAddr,
                                         const uint8_t *pu8WriteData,
                                         uint16_t u16WriteLen,
                                         uint32_t u32TimeoutMs,
                                         void *vpBusContext)
{
    ts_Bmp180_Stm32BusContext *psContext = (ts_Bmp180_Stm32BusContext *)vpBusContext; /* Soyut ctx pointer'ı STM32 port bağlamına çevrilir. */
    HAL_StatusTypeDef eHalStatus;                            /* HAL_I2C_Mem_Write dönüş değerini BMP180 hata kodlarına map etmek için saklar. */

    if ((psContext == NULL) || (psContext->pxI2cHandle == NULL) || (pu8WriteData == NULL) || (u16WriteLen == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;                       /* I2C handle, kaynak buffer ve uzunluk olmadan HAL yazması güvenli değildir. */
    }

    eHalStatus = HAL_I2C_Mem_Write(psContext->pxI2cHandle,
                                   (uint16_t)(u8DeviceAddr << 1U),
                                   u8RegisterAddr,
                                   I2C_MEMADD_SIZE_8BIT,
                                   (uint8_t *)pu8WriteData,
                                   u16WriteLen,
                                   u32TimeoutMs);            /* Platform bağımsız write callback'i STM32 HAL register yazma fonksiyonuna uyarlanır. */
    if (eHalStatus != HAL_OK)
    {
        return ((eHalStatus == HAL_TIMEOUT) || (psContext->pxI2cHandle->ErrorCode == HAL_I2C_ERROR_TIMEOUT)) ?
                   DRIVER_ERR_TIMEOUT :
                   DRIVER_ERR_BUS;                     /* HAL timeout status/error ayrı raporlanır; diğer HAL hataları bus error olarak sadeleştirilir. */
    }

    return DRIVER_OK;                                    /* HAL yazma başarıyla tamamlanmıştır. */
}

te_Driver_RetCode Bmp180_Stm32Hal_DelayMs(uint32_t u32DelayMs, void *vpBusContext)
{
    (void)vpBusContext;                                      /* HAL ve CMSIS zaman servisleri ek bağlam istemez; callback imzası için parametre tutulur. */
    if (u32DelayMs == 0U)
    {
        return DRIVER_OK;                                    /* Sıfır süreli servis beklemesi scheduler veya HAL çağrısı oluşturmadan tamamlanır. */
    }

    if (osKernelGetState() == osKernelRunning)
    {
        return (osDelay(u32DelayMs) == osOK) ?
                   DRIVER_OK :
                   DRIVER_ERR_IO;                            /* Scheduler çalışırken task WAITING durumuna alınır; CPU başka task'lara bırakılır. */
    }

    HAL_Delay(u32DelayMs);                                   /* Scheduler başlamadan önce Open gibi servis akışlarında HAL zaman tabanı güvenli fallback'tir. */
    return DRIVER_OK;                                        /* HAL_Delay dönüş değeri olmadığı için pre-kernel bekleme başarılı kabul edilir. */
}

uint32_t Bmp180_Stm32Hal_GetTickMs(void *vpBusContext)
{
    (void)vpBusContext;                                      /* HAL_GetTick ek bağlam istemez; callback imzası için parametre tutulur. */
    return HAL_GetTick();                                    /* Ölçüm timestamp'i STM32 HAL milisaniye tick değerinden üretilir. */
}

te_Driver_RetCode Bmp180_Stm32Hal_Lock(uint32_t u32TimeoutMs, void *vpBusContext)
{
    ts_Bmp180_Stm32BusContext *psContext = (ts_Bmp180_Stm32BusContext *)vpBusContext; /* Lock callback bağlamı STM32 bus context olarak yorumlanır. */

    if (psContext == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;                       /* Mutex olmasa bile context pointer'ın geçerli olması port sözleşmesidir. */
    }

    if (psContext->xBusMutex == NULL)
    {
        return DRIVER_OK;                                /* Tek task veya harici korumalı kullanımda NULL mutex no-op kabul edilir. */
    }

    if (osMutexAcquire(psContext->xBusMutex, u32TimeoutMs) != osOK)
    {
        return DRIVER_ERR_TIMEOUT;                           /* Mutex alınamazsa paylaşılan I2C bus için bounded wait timeout raporlanır. */
    }

    return DRIVER_OK;                                    /* Mutex başarıyla alınmıştır; driver artık tek I2C transferini yapabilir. */
}

te_Driver_RetCode Bmp180_Stm32Hal_Unlock(void *vpBusContext)
{
    ts_Bmp180_Stm32BusContext *psContext = (ts_Bmp180_Stm32BusContext *)vpBusContext; /* Unlock callback bağlamı STM32 bus context olarak yorumlanır. */

    if (psContext == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;                       /* Context yoksa hangi mutex'in bırakılacağı bilinemez. */
    }

    if (psContext->xBusMutex == NULL)
    {
        return DRIVER_OK;                                /* NULL mutex kullanılan sistemlerde unlock no-op kabul edilir. */
    }

    return (osMutexRelease(psContext->xBusMutex) == osOK) ?
               DRIVER_OK :
               DRIVER_ERR_STATE;                     /* Release başarısızsa mutex sahipliği/durum problemi üst katmana taşınır. */
}

te_Driver_RetCode Bmp180_Stm32Hal_FillBusInterface(ts_Bmp180_BusInterface *psBusInterface,
                                                    ts_Bmp180_Stm32BusContext *psBusContext)
{
    if ((psBusInterface == NULL) || (psBusContext == NULL) || (psBusContext->pxI2cHandle == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;                       /* Bus interface doldurmak için hedef struct ve geçerli I2C handle zorunludur. */
    }

    psBusInterface->pfnRead = Bmp180_Stm32Hal_Read;          /* Platform bağımsız driver'ın read çağrısı STM32 HAL port fonksiyonuna bağlanır. */
    psBusInterface->pfnWrite = Bmp180_Stm32Hal_Write;        /* Platform bağımsız driver'ın write çağrısı STM32 HAL port fonksiyonuna bağlanır. */
    psBusInterface->vpCtx = psBusContext;                    /* I2C handle ve mutex bağlamı opaque pointer olarak driver'a verilir. */
    return DRIVER_OK;                                    /* Bus interface doldurma işlemi tamamlanmıştır. */
}

te_Driver_RetCode Bmp180_Stm32Hal_FillLockInterface(ts_Bmp180_LockInterface *psLockInterface,
                                                     ts_Bmp180_Stm32BusContext *psBusContext)
{
    if ((psLockInterface == NULL) || (psBusContext == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;                       /* Lock interface doldurmak için hedef struct ve context zorunludur. */
    }

    psLockInterface->pfnLock = Bmp180_Stm32Hal_Lock;         /* Driver'ın soyut lock çağrısı CMSIS-RTOS2 osMutexAcquire adaptörüne bağlanır. */
    psLockInterface->pfnUnlock = Bmp180_Stm32Hal_Unlock;     /* Driver'ın soyut unlock çağrısı CMSIS-RTOS2 osMutexRelease adaptörüne bağlanır. */
    psLockInterface->vpCtx = psBusContext;                   /* Aynı context bus ve lock callback'leri arasında paylaşılır. */
    return DRIVER_OK;                                    /* Lock interface doldurma işlemi tamamlanmıştır. */
}

te_Driver_RetCode Bmp180_Stm32Hal_FillTimingInterface(ts_Bmp180_TimingInterface *psTimingInterface)
{
    if (psTimingInterface == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;                       /* Timing interface doldurmak için hedef struct zorunludur. */
    }

    psTimingInterface->pfnDelayMs = Bmp180_Stm32Hal_DelayMs; /* Servis bekleme callback'i kernel durumuna göre osDelay veya HAL_Delay seçen adaptöre bağlanır. */
    psTimingInterface->pfnGetTickMs = Bmp180_Stm32Hal_GetTickMs; /* Timestamp callback'i HAL_GetTick adaptörüne bağlanır. */
    psTimingInterface->vpCtx = NULL;                         /* HAL_Delay/HAL_GetTick bağlam istemediği için context NULL bırakılır. */
    return DRIVER_OK;                                    /* Timing interface doldurma işlemi tamamlanmıştır. */
}
