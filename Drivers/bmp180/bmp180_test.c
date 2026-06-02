#if defined(BMP180_ENABLE_TEST)

#include "bmp180_test.h"

#include "bmp180_driver.h"
#include "bmp180_hal.h"
#include "bmp180_stm32_hal_port.h"

#include <stddef.h>

#define BMP180_TEST_LOOP_DELAY_MS                (200U)       /* Test uygulaması ölçümleri 5 Hz civarında yazdırır; BMP180 dönüşüm süreleri için rahattır. */
#define BMP180_TEST_UART_TIMEOUT_MS              (100U)       /* UART log transferi için kısa ve bounded timeout değeridir. */

static osMutexId_t s_xBmp180Test_BusMutex = NULL;             /* Sadece test dosyasına ait fallback mutex'tir; driver çekirdeğinde global state yoktur. */

static uint16_t Bmp180_Test_prvStrLen(const char *szText)
{
    uint16_t u16Len = 0U;                                    /* UART'a verilecek string uzunluğu byte olarak sayılır. */

    if (szText == NULL)
    {
        return 0U;                                           /* NULL string sessizce boş kabul edilir. */
    }

    while (szText[u16Len] != '\0')
    {
        ++u16Len;                                            /* Basit C string uzunluğu hesaplanır; stdlib bağımlılığı eklenmez. */
    }

    return u16Len;                                           /* Hesaplanan uzunluk UART transmit için döndürülür. */
}

static void Bmp180_Test_prvSendText(UART_HandleTypeDef *pxUart, const char *szText)
{
    uint16_t u16Len;                                         /* Gönderilecek metin uzunluğunu tutar. */

    if ((pxUart == NULL) || (szText == NULL))
    {
        return;                                              /* UART opsiyoneldir; NULL ise test sadece dönüş kodlarıyla çalışır. */
    }

    u16Len = Bmp180_Test_prvStrLen(szText);                  /* HAL_UART_Transmit uzunluk istediği için string boyu hesaplanır. */
    if (u16Len == 0U)
    {
        return;                                              /* Boş string için HAL çağrısı yapılmaz. */
    }

    (void)HAL_UART_Transmit(pxUart, (uint8_t *)szText, u16Len, BMP180_TEST_UART_TIMEOUT_MS); /* UART sadece test/uygulama katmanındadır; driver core stdio bilmez. */
}

static char *Bmp180_Test_prvAppendChar(char *pcDst, char cValue)
{
    *pcDst = cValue;                                         /* Hedef buffer'a tek karakter yazılır. */
    return &pcDst[1];                                        /* Caller'ın sonraki karakteri yazacağı konum döndürülür. */
}

static char *Bmp180_Test_prvAppendText(char *pcDst, const char *szText)
{
    while ((szText != NULL) && (*szText != '\0'))
    {
        *pcDst = *szText;                                    /* Kaynak metindeki karakter hedef satıra kopyalanır. */
        ++pcDst;                                             /* Hedef pointer bir sonraki yazma konumuna ilerler. */
        ++szText;                                            /* Kaynak pointer bir sonraki karaktere ilerler. */
    }

    return pcDst;                                            /* Güncel yazma konumu caller'a geri verilir. */
}

static char *Bmp180_Test_prvAppendU32(char *pcDst, uint32_t u32Value)
{
    char acDigits[10];                                       /* 32-bit decimal sayı için maksimum 10 basamaklık geçici buffer'dır. */
    uint8_t u8Count = 0U;                                    /* Geçici buffer'a yazılan basamak sayısını tutar. */

    do
    {
        acDigits[u8Count] = (char)('0' + (u32Value % 10U));  /* Sayının en düşük decimal basamağı ASCII karaktere çevrilir. */
        u32Value /= 10U;                                     /* Sonraki basamağa geçmek için sayı 10'a bölünür. */
        ++u8Count;                                           /* Yazılan basamak sayısı artırılır. */
    } while (u32Value > 0U);

    while (u8Count > 0U)
    {
        --u8Count;                                           /* Basamaklar ters toplandığı için sondan başa doğru yazılır. */
        *pcDst = acDigits[u8Count];                          /* Decimal basamak hedef metne eklenir. */
        ++pcDst;                                             /* Hedef pointer ilerletilir. */
    }

    return pcDst;                                            /* Güncel yazma konumu döndürülür. */
}

static char *Bmp180_Test_prvAppendFixed2(char *pcDst, float f32Value)
{
    int32_t s32Scaled = (int32_t)(f32Value * 100.0f);        /* Float değer iki ondalık basamak için ölçeklenir; printf-float bağımlılığı azaltılır. */
    int32_t s32Whole;                                       /* Ölçeklenmiş sayının tam kısmını tutar. */
    int32_t s32Frac;                                        /* Ölçeklenmiş sayının iki basamaklı kesir kısmını tutar. */

    if (s32Scaled < 0)
    {
        pcDst = Bmp180_Test_prvAppendChar(pcDst, '-');       /* Negatif değerler için işaret açıkça yazılır. */
        s32Scaled = -s32Scaled;                              /* Yazdırma kolaylığı için mutlak ölçeklenmiş değer kullanılır. */
    }

    s32Whole = s32Scaled / 100;                              /* Tam kısım iki ondalık ölçekten ayrılır. */
    s32Frac = s32Scaled % 100;                               /* Kesir kısmı 0..99 aralığında tutulur. */
    pcDst = Bmp180_Test_prvAppendU32(pcDst, (uint32_t)s32Whole); /* Tam kısım decimal olarak yazılır. */
    pcDst = Bmp180_Test_prvAppendChar(pcDst, '.');           /* Ondalık ayırıcı eklenir. */
    if (s32Frac < 10)
    {
        pcDst = Bmp180_Test_prvAppendChar(pcDst, '0');       /* Tek basamaklı kesirler 05 gibi iki basamaklı gösterilir. */
    }
    pcDst = Bmp180_Test_prvAppendU32(pcDst, (uint32_t)s32Frac); /* Kesir kısmı metne eklenir. */
    return pcDst;                                            /* Güncel yazma konumu döndürülür. */
}

static void Bmp180_Test_prvSendSample(UART_HandleTypeDef *pxUart, const ts_Bmp180_Data *psData)
{
    char acLine[128];                                        /* Tek ölçüm satırı için sabit stack buffer'dır; dinamik bellek kullanılmaz. */
    char *pcWrite = acLine;                                  /* Satır oluşturma sırasında güncel yazma konumunu tutar. */

    if ((pxUart == NULL) || (psData == NULL))
    {
        return;                                              /* UART veya veri yoksa log üretimi güvenli no-op olur. */
    }

    pcWrite = Bmp180_Test_prvAppendText(pcWrite, "BMP180 T="); /* Sıcaklık alanı etiketi eklenir. */
    pcWrite = Bmp180_Test_prvAppendFixed2(pcWrite, psData->f32TemperatureC); /* Celsius sıcaklık iki ondalıkla yazılır. */
    pcWrite = Bmp180_Test_prvAppendText(pcWrite, " C P=");   /* Basınç alanı etiketi eklenir. */
    pcWrite = Bmp180_Test_prvAppendFixed2(pcWrite, psData->f32PressurePa); /* Pascal basınç iki ondalıkla yazılır. */
    pcWrite = Bmp180_Test_prvAppendText(pcWrite, " Pa Alt="); /* Yükseklik alanı etiketi eklenir. */
    pcWrite = Bmp180_Test_prvAppendFixed2(pcWrite, psData->f32AltitudeM); /* Metre yükseklik iki ondalıkla yazılır. */
    pcWrite = Bmp180_Test_prvAppendText(pcWrite, " m");       /* Public driver çıktısı kurallar gereği yalnız SI alanlarını içerir. */
    pcWrite = Bmp180_Test_prvAppendText(pcWrite, "\r\n");   /* UART terminal satır sonu eklenir. */
    *pcWrite = '\0';                                         /* C string sonlandırıcısı eklenir. */
    Bmp180_Test_prvSendText(pxUart, acLine);                 /* Hazırlanan satır UART üzerinden gönderilir. */
}

int8_t BMP180_TEST(const ts_Bmp180_TestParams *psX)
{
    ts_Bmp180_Stm32BusContext sBusCtx;                       /* STM32 I2C handle ve mutex bilgisini port callback'lerine taşır. */
    ts_Bmp180_BusInterface sBusIf;                           /* Platform bağımsız driver'a verilecek bus interface paketidir. */
    ts_Bmp180_LockInterface sLockIf;                         /* Platform bağımsız driver'a verilecek lock interface paketidir. */
    ts_Bmp180_TimingInterface sTimingIf;                     /* Platform bağımsız driver'a verilecek timing interface paketidir. */
    ts_Bmp180_OpenConfig sCfg;                               /* BMP180 Open çağrısının konfigürasyon struct'ıdır. */
    ts_Bmp180_Handle sHandle;                                /* Teste özel BMP180 instance handle'ıdır. */
    ts_Bmp180_Data sData;                                    /* Her döngüde doldurulan ölçüm çıktısıdır. */
    ts_Bmp180_HealthStatus sHealth;                          /* Health-check sonucunu taşır. */
    te_Driver_RetCode eRet;                                  /* Driver ve port çağrılarının dönüş kodunu taşır. */
    uint32_t u32ElapsedMs = 0U;                              /* Test döngüsünde geçen süreyi takip eder. */
    const osMutexAttr_t xMutexAttr = {
        .name = "bmp180_test_bus",
        .attr_bits = osMutexPrioInherit,
    };                                                       /* Test kendi mutex'ini yaratırsa priority inheritance açık kullanılır. */

    if ((psX == NULL) || (psX->pxI2c == NULL) || (psX->u8Oversampling > BMP180_OSS_MAX))
    {
        return -1;                                           /* Zorunlu test parametreleri eksikse açma hatası döndürülür. */
    }

    if (psX->xBusMutex != NULL)
    {
        sBusCtx.xBusMutex = psX->xBusMutex;                  /* Uygulama ortak MPU6050/BMP180 bus mutex'i verdiyse aynı mutex kullanılır. */
    }
    else
    {
        if (s_xBmp180Test_BusMutex == NULL)
        {
            s_xBmp180Test_BusMutex = osMutexNew(&xMutexAttr); /* Teste özel fallback mutex yalnızca uygulama mutex vermediyse oluşturulur. */
            if (s_xBmp180Test_BusMutex == NULL)
            {
                Bmp180_Test_prvSendText(psX->pxUart, "BMP180_TEST: bus mutex create failed\r\n"); /* Mutex oluşturma hatası UART'a bildirilir. */
                return -2;                                  /* Kaynak yönetimi hatası konfigürasyon/çalışma hatası olarak döndürülür. */
            }
        }
        sBusCtx.xBusMutex = s_xBmp180Test_BusMutex;          /* Oluşturulan fallback mutex bus context içine bağlanır. */
    }

    sBusCtx.pxI2cHandle = psX->pxI2c;                        /* STM32 HAL I2C handle port context içine yerleştirilir. */
    eRet = Bmp180_Stm32Hal_FillBusInterface(&sBusIf, &sBusCtx); /* HAL read/write callback'leri platform bağımsız interface'e bağlanır. */
    if (eRet != DRIVER_OK)
    {
        Bmp180_Test_prvSendText(psX->pxUart, "BMP180_TEST: bus interface fill failed\r\n"); /* Bus interface hatası loglanır. */
        return -2;                                           /* Konfigürasyon hatası test başarısızlığı olarak döndürülür. */
    }

    (void)Bmp180_Stm32Hal_FillLockInterface(&sLockIf, &sBusCtx); /* Mutex callback'leri interface struct'ına doldurulur. */
    (void)Bmp180_Stm32Hal_FillTimingInterface(&sTimingIf);   /* RTOS-aware delay ve HAL_GetTick callback'leri interface struct'ına doldurulur. */
    sCfg.u8I2cAddress = (psX->u8I2cAddr7bit == 0U) ? BMP180_I2C_ADDR_DEFAULT : psX->u8I2cAddr7bit; /* 0 adres varsayılan 0x77'ye çevrilir. */
    sCfg.u32BusTimeoutMs = 100U;                             /* Test için kısa ve bounded I2C timeout kullanılır. */
    sCfg.u32BusLockTimeoutMs = 100U;                         /* Test için kısa ve bounded mutex timeout kullanılır. */
    sCfg.eOversampling = (te_Bmp180_Oversampling)psX->u8Oversampling; /* Test parametresi driver enum'una çevrilir. */
    sCfg.sBusInterface = sBusIf;                             /* Bus interface Open config içine kopyalanır. */
    sCfg.sLockInterface = sLockIf;                           /* Lock interface Open config içine kopyalanır. */
    sCfg.sTimingInterface = sTimingIf;                       /* Timing interface Open config içine kopyalanır. */
    sCfg.f32SeaLevelPressurePa = BMP180_SEA_LEVEL_PRESSURE_PA; /* Test standart deniz seviyesi basıncını kullanır. */

    eRet = Bmp180_Open(&sHandle, &sCfg);                     /* Platform bağımsız driver API ile sensör açılır. */
    if (eRet != DRIVER_OK)
    {
        Bmp180_Test_prvSendText(psX->pxUart, "BMP180_TEST: Open failed\r\n"); /* Chip-id, bus veya kalibrasyon hatası loglanır. */
        return -1;                                           /* Açma/kimlik doğrulama hatası ayrı dönüş koduyla bildirilir. */
    }

    eRet = Bmp180_Ioctl(&sHandle, BMP180_IOCTL_CHECK_HEALTH, &sHealth); /* Açılış sonrası health-check örnek Ioctl kullanımı gösterilir. */
    if (eRet != DRIVER_OK)
    {
        Bmp180_Test_prvSendText(psX->pxUart, "BMP180_TEST: CHECK_HEALTH failed\r\n"); /* Health-check hatası loglanır. */
        (void)Bmp180_Close(&sHandle);                        /* Hata yolunda handle uygulama açısından kapatılır. */
        return -2;                                           /* Çalışma hatası test başarısızlığı olarak döndürülür. */
    }

    Bmp180_Test_prvSendText(psX->pxUart, "BMP180_TEST: Open OK\r\n"); /* Başarılı açılış UART'a bildirilir. */
    while (u32ElapsedMs < psX->u32TimeoutMs)
    {
        eRet = Bmp180_Test(&sHandle, 100U);                  /* Deployment test API runtime ile aynı bloklamasız state machine'i bounded polling ile doğrular. */
        if (eRet != DRIVER_OK)
        {
            Bmp180_Test_prvSendText(psX->pxUart, "BMP180_TEST: measurement failed\r\n"); /* Ölçüm veya health hatası UART'a bildirilir. */
            (void)Bmp180_Close(&sHandle);                    /* Hata yolunda handle kapatılır. */
            return -2;                                       /* Çalışma hatası test başarısızlığı olarak döndürülür. */
        }

        eRet = Bmp180_Read(&sHandle, &sData);                /* Tamamlanan SI cache örneği bekleme yapmadan alınır. */
        if (eRet != DRIVER_OK)
        {
            Bmp180_Test_prvSendText(psX->pxUart, "BMP180_TEST: cache read failed\r\n"); /* Cache erişim hatası UART'a bildirilir. */
            (void)Bmp180_Close(&sHandle);                    /* Hata yolunda handle kapatılır. */
            return -2;                                       /* Çalışma hatası test başarısızlığı olarak döndürülür. */
        }

        Bmp180_Test_prvSendSample(psX->pxUart, &sData);      /* Ölçüm değerleri UART'a insan okunur formatta yazılır. */
        (void)osDelay(BMP180_TEST_LOOP_DELAY_MS);            /* Test uygulama katmanında periyodik bekleme yapar; driver core RTOS bilmez. */
        u32ElapsedMs += BMP180_TEST_LOOP_DELAY_MS;           /* Toplam test süresi takip edilir. */
    }

    (void)Bmp180_Close(&sHandle);                            /* Test sonunda driver instance uygulama açısından kapatılır. */
    Bmp180_Test_prvSendText(psX->pxUart, "BMP180_TEST: done\r\n"); /* Test tamamlanma mesajı UART'a yazılır. */
    return 0;                                                /* Test başarıyla tamamlanmıştır. */
}

#endif /* BMP180_ENABLE_TEST */
