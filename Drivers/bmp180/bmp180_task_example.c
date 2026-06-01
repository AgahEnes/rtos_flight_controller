#if defined(BMP180_ENABLE_TASK_EXAMPLE)

#include "bmp180_driver.h"
#include "bmp180_hal.h"
#include "bmp180_stm32_hal_port.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <stddef.h>

extern I2C_HandleTypeDef hi2c1;                              /* CubeMX tarafından üretilen I2C handle'dır; uygulama kendi instance adını eşleştirmelidir. */
extern QueueHandle_t bmpQueue;                               /* Uygulamanın oluşturduğu BMP180 veri kuyruğudur; task ölçüm çıktısını buraya gönderir. */
extern osMutexId_t xSharedI2cMutex;                          /* MPU6050 ve BMP180 aynı bus'taysa ortak priority-inheritance mutex burada paylaşılır. */

void BMP180_Task(void *pvParameters)
{
    static ts_Bmp180_Stm32BusContext sBusCtx;                /* Task ömrü boyunca yaşayan STM32 port bağlamıdır; stack dışında tutulur. */
    static ts_Bmp180_BusInterface sBusIf;                    /* Driver'a verilecek bus callback paketidir. */
    static ts_Bmp180_LockInterface sLockIf;                  /* Driver'a verilecek lock callback paketidir. */
    static ts_Bmp180_TimingInterface sTimingIf;              /* Driver'a verilecek timing callback paketidir. */
    static ts_Bmp180_OpenConfig sCfg;                        /* Driver Open konfigürasyonudur. */
    static ts_Bmp180_Handle sHandle;                         /* BMP180 instance state'ini taşıyan handle'dır; global singleton driver içinde değil uygulama tarafındadır. */
    ts_Bmp180_Data sData;                                    /* Her periyotta doldurulan ölçüm verisidir. */
    TickType_t xLastWakeTime;                                /* vTaskDelayUntil için son uyanma zamanını tutar. */
    const TickType_t xPeriodTicks = pdMS_TO_TICKS(200U);     /* BMP180 task periyodu 200 ms seçilmiştir; sensör dönüşüm sürelerinden uzundur. */

    (void)pvParameters;                                      /* Bu örnek task parametre kullanmaz; FreeRTOS imzası için argüman tutulur. */
    sBusCtx.pxI2cHandle = &hi2c1;                            /* BMP180'in bağlı olduğu I2C instance'ı port context içine yazılır. */
    sBusCtx.xBusMutex = xSharedI2cMutex;                     /* Aynı I2C bus üzerindeki MPU6050 ve BMP180 erişimleri ortak mutex ile korunur. */
    (void)Bmp180_Stm32Hal_FillBusInterface(&sBusIf, &sBusCtx); /* STM32 HAL read/write adaptörleri driver bus interface'ine bağlanır. */
    (void)Bmp180_Stm32Hal_FillLockInterface(&sLockIf, &sBusCtx); /* CMSIS mutex adaptörleri driver lock interface'ine bağlanır. */
    (void)Bmp180_Stm32Hal_FillTimingInterface(&sTimingIf);   /* HAL_Delay ve HAL_GetTick timing interface'ine bağlanır. */

    sCfg.u8I2cAddress = BMP180_I2C_ADDR_DEFAULT;             /* BMP180 varsayılan 7-bit I2C adresi kullanılır. */
    sCfg.u32BusTimeoutMs = 100U;                             /* Her I2C transferi için bounded timeout verilir. */
    sCfg.u32BusLockTimeoutMs = 100U;                         /* Shared bus mutex için bounded bekleme verilir. */
    sCfg.eOversampling = BMP180_OSS3_ULTRA_HIGH_RESOLUTION;  /* Denge/sensör füzyonu için daha düşük gürültülü basınç modu seçilir. */
    sCfg.sBusInterface = sBusIf;                             /* Bus callback paketi Open config içine kopyalanır. */
    sCfg.sLockInterface = sLockIf;                           /* Lock callback paketi Open config içine kopyalanır. */
    sCfg.sTimingInterface = sTimingIf;                       /* Timing callback paketi Open config içine kopyalanır. */
    sCfg.f32SeaLevelPressurePa = BMP180_SEA_LEVEL_PRESSURE_PA; /* Altitude hesabı için standart referans kullanılır; uygulama Ioctl ile değiştirebilir. */

    if (Bmp180_Open(&sHandle, &sCfg) != BMP180_RET_OK)
    {
        vTaskDelete(NULL);                                   /* Örnek kodda Open başarısızsa task kendini sonlandırır; gerçek projede manager task'a hata bildirilebilir. */
    }

    xLastWakeTime = xTaskGetTickCount();                     /* Periyodik çalışmanın faz referansı alınır. */
    for (;;)
    {
        if (Bmp180_Read(&sHandle, &sData) == BMP180_RET_OK)
        {
            (void)xQueueSend(bmpQueue, &sData, 0U);          /* Ölçüm çıktısı processing task'a kuyrukla aktarılır; driver core queue bilmez. */
        }

        vTaskDelayUntil(&xLastWakeTime, xPeriodTicks);       /* Sabit periyotlu sensör okuma yapılır; jitter basit delay'e göre azalır. */
    }
}

#endif /* BMP180_ENABLE_TASK_EXAMPLE */
