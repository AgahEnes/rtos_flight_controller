#include "bmp180_driver.h"
#include "bmp180_hal.h"

#include <math.h>

#define BMP180_TIMEOUT_DEFAULT_MS                  (100U)     /* Bus transferleri için platform bağımsız varsayılan timeout değeridir. */
#define BMP180_LOCK_TIMEOUT_DEFAULT_MS             (100U)     /* Paylaşılan I2C mutex'i için varsayılan bekleme süresidir. */
#define BMP180_TEST_TIMEOUT_DEFAULT_MS              (100U)     /* Deployment test state machine'i için caller sıfır verdiğinde kullanılan bounded bütçedir. */
#define BMP180_TEST_POLL_DELAY_MS                   (1U)       /* Yalnız Test API içinde state machine adımları arasında kullanılan kısa servis beklemesidir. */

#ifndef NULL
#define NULL                                       ((void *)0) /* Standart başlık eklemeden NULL kullanımı için yerel korumadır. */
#endif

static te_Driver_RetCode Bmp180_prvMarkError(ts_Bmp180_Handle *psHandle, te_Driver_RetCode eRet); /* Platform bağımsız driver helper'ıdır; ciddi hatalarda handle state'ini ERROR yapar. */
static te_Driver_RetCode Bmp180_prvLock(ts_Bmp180_Handle *psHandle); /* RTOS/resource-management helper'ıdır; enjekte edilen lock callback'i üzerinden paylaşılan bus'ı korur. */
static te_Driver_RetCode Bmp180_prvUnlock(ts_Bmp180_Handle *psHandle); /* RTOS/resource-management helper'ıdır; her bus transferinden sonra lock callback'ini bırakır. */
static uint32_t Bmp180_prvGetBusTimeout(const ts_Bmp180_Handle *psHandle); /* Platform bağımsız driver helper'ıdır; 0 timeout konfigürasyonunu güvenli varsayılana çevirir. */
static te_Driver_RetCode Bmp180_prvReadBlock(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data, uint16_t u16Len); /* Bus erişim helper'ıdır; lock alıp soyut read callback'iyle BMP180 register bloğu okur. */
static te_Driver_RetCode Bmp180_prvWriteBlock(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, const uint8_t *pu8Data, uint16_t u16Len); /* Bus erişim helper'ıdır; lock alıp soyut write callback'iyle BMP180 register bloğu yazar. */
static te_Driver_RetCode Bmp180_prvReadRegister(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data); /* Register erişim helper'ıdır; tek baytlık BMP180 register okumasını blok helper'a indirger. */
static te_Driver_RetCode Bmp180_prvWriteRegister(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t u8Data); /* Register erişim helper'ıdır; tek baytlık BMP180 register yazmasını blok helper'a indirger. */
static te_Driver_RetCode Bmp180_prvCheckChipId(ts_Bmp180_Handle *psHandle, uint8_t *pu8ChipId); /* Class-A health helper'ıdır; BMP180 CHIP_ID register'ını datasheet beklenen değeriyle doğrular. */
static int16_t Bmp180_prvParseBeS16(const uint8_t *pu8Data); /* Endian helper'ıdır; BMP180 MSB-first signed 16-bit katsayılarını taşınabilir biçimde çözer. */
static uint16_t Bmp180_prvParseBeU16(const uint8_t *pu8Data); /* Endian helper'ıdır; BMP180 MSB-first unsigned 16-bit katsayılarını taşınabilir biçimde çözer. */
static te_Driver_RetCode Bmp180_prvReadCalibrationCoeffs(ts_Bmp180_Handle *psHandle); /* Datasheet helper'ıdır; 22 baytlık fabrika kalibrasyon bloğunu okuyup handle içine parse eder. */
static bool Bmp180_prvValidateCalibrationCoeffs(const ts_Bmp180_CalibrationCoeffs *psCalib); /* Kalibrasyon helper'ıdır; all-zero/all-ones ve kritik sıfır katsayılarını ayıklar. */
static uint32_t Bmp180_prvGetPressureConversionTimeMs(te_Bmp180_Oversampling eOss); /* Datasheet timing helper'ıdır; OSS modunu gereken basınç dönüşüm beklemesine çevirir. */
static uint8_t Bmp180_prvBuildPressureConversionCommand(te_Bmp180_Oversampling eOss); /* Datasheet komut helper'ıdır; 0x34+(OSS<<6) basınç komut baytını üretir. */
static te_Driver_RetCode Bmp180_prvDelayMs(ts_Bmp180_Handle *psHandle, uint32_t u32DelayMs); /* Timing helper'ıdır; driver core'un HAL_Delay/osDelay bilmeden beklemesini sağlar. */
static uint32_t Bmp180_prvGetTickMs(const ts_Bmp180_Handle *psHandle); /* Timing helper'ıdır; bloklamasız state machine için enjekte edilen monotonik tick kaynağını okur. */
static bool Bmp180_prvHasDeadlineElapsed(uint32_t u32NowMs, uint32_t u32DeadlineMs); /* Wrap-around güvenli deadline helper'ıdır; uint32_t tick taşmasını deterministik ele alır. */
static te_Driver_RetCode Bmp180_prvIsConversionBusy(ts_Bmp180_Handle *psHandle, bool *pbIsBusy); /* Datasheet helper'ıdır; CONTROL register SCO bitinden fiziksel dönüşümün sürüp sürmediğini okur. */
static te_Driver_RetCode Bmp180_prvReadRawTemperatureResult(ts_Bmp180_Handle *psHandle, int32_t *ps32RawTemperature); /* Ölçüm helper'ıdır; hazır sıcaklık output register'larından UT değerini beklemeden okur. */
static te_Driver_RetCode Bmp180_prvReadRawPressureResult(ts_Bmp180_Handle *psHandle, int32_t *ps32RawPressure); /* Ölçüm helper'ıdır; hazır basınç output register'larından OSS hizalı UP değerini beklemeden okur. */
static te_Driver_RetCode Bmp180_prvStartMeasurement(ts_Bmp180_Handle *psHandle); /* Runtime helper'ıdır; sıcaklık dönüşümünü başlatıp hemen dönen ilk state machine adımıdır. */
static te_Driver_RetCode Bmp180_prvProcessMeasurement(ts_Bmp180_Handle *psHandle); /* Runtime helper'ıdır; deadline ve SCO kontrolüyle state machine'i en fazla tek kısa adım ilerletir. */
static te_Driver_RetCode Bmp180_prvBuildLatestData(ts_Bmp180_Handle *psHandle, int32_t s32RawPressure); /* Runtime helper'ıdır; UT/UP telafisini hesaplayıp Read API cache'ini günceller. */
static te_Driver_RetCode Bmp180_prvComputeTrueValues(const ts_Bmp180_CalibrationCoeffs *psCalib,
                                                      int32_t s32RawTemperature,
                                                      int32_t s32RawPressure,
                                                      te_Bmp180_Oversampling eOss,
                                                      int32_t *ps32TemperatureDeciC,
                                                      int32_t *ps32PressurePa); /* Datasheet telafi helper'ıdır; BMP180 integer algoritmasıyla gerçek sıcaklık ve basıncı hesaplar. */
static float Bmp180_prvComputeAltitudeM(float f32PressurePa, float f32SeaLevelPressurePa); /* Uygulama-facing helper'dır; basınçtan deniz seviyesi referanslı altitude üretir. */
static void Bmp180_prvMemZero(void *vpData, uint32_t u32Len); /* Platform bağımsız bellek helper'ıdır; libc memset zorunluluğu olmadan struct temizler. */
static bool Bmp180_prvIsOversamplingValid(te_Bmp180_Oversampling eOss); /* Argüman doğrulama helper'ıdır; OSS enum değerini datasheet 0..3 aralığına sınar. */
static bool Bmp180_prvIsSeaLevelPressureValid(float f32PressurePa); /* Argüman doğrulama helper'ıdır; altitude referans basıncını makul fiziksel aralıkta tutar. */

static te_Driver_RetCode Bmp180_prvMarkError(ts_Bmp180_Handle *psHandle, te_Driver_RetCode eRet)
{
    if ((psHandle != NULL) && (eRet != DRIVER_OK))
    {
        psHandle->eState = BMP180_STATE_ERROR;              /* Hata durum makinesine yazılır; public API sonraki çağrılarda deterministik davranır. */
        psHandle->eMeasurementState = BMP180_MEASUREMENT_IDLE; /* Hata sonrası yarım dönüşüm state'i korunmaz; toparlanma soft reset ile açıkça yapılır. */
    }

    return eRet;                                             /* Asıl hata kodu korunur; caller gerçek kök nedeni kaybetmez. */
}

static te_Driver_RetCode Bmp180_prvLock(ts_Bmp180_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                          /* Handle yoksa lock bağlamına erişmek güvenli değildir. */
    }

    if (psHandle->sLockInterface.pfnLock == NULL)
    {
        return DRIVER_OK;                                /* Bare-metal veya tek-sensör kullanımında lock opsiyoneldir. */
    }

    return psHandle->sLockInterface.pfnLock(psHandle->u32BusLockTimeoutMs,
                                            psHandle->sLockInterface.vpCtx); /* FreeRTOS/CMSIS mutex ayrıntısı port katmanına bırakılır. */
}

static te_Driver_RetCode Bmp180_prvUnlock(ts_Bmp180_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                          /* Handle yoksa unlock callback bağlamı bilinemez. */
    }

    if (psHandle->sLockInterface.pfnUnlock == NULL)
    {
        return DRIVER_OK;                                /* Lock kullanılmayan sistemlerde unlock no-op kabul edilir. */
    }

    return psHandle->sLockInterface.pfnUnlock(psHandle->sLockInterface.vpCtx); /* Mutex sahipliği ve release kuralı port katmanında uygulanır. */
}

static uint32_t Bmp180_prvGetBusTimeout(const ts_Bmp180_Handle *psHandle)
{
    return (psHandle->u32BusTimeoutMs == 0U) ? BMP180_TIMEOUT_DEFAULT_MS : psHandle->u32BusTimeoutMs; /* 0 konfigürasyonu güvenli varsayılan timeout'a çevrilir. */
}

static te_Driver_RetCode Bmp180_prvReadBlock(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data, uint16_t u16Len)
{
    te_Driver_RetCode eRet;                                  /* Bus callback sonucunu saklar; unlock sonrası değerlendirilir. */
    te_Driver_RetCode eUnlockRet;                            /* Unlock hatasını ayrıca saklar; transfer başarısız olsa bile unlock denenir. */

    if ((psHandle == NULL) || (pu8Data == NULL) || (u16Len == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;                       /* Blok okuma için handle, buffer ve uzunluk zorunludur. */
    }

    if (psHandle->sBusInterface.pfnRead == NULL)
    {
        return DRIVER_ERR_CONFIG;                      /* Platform bağımsız driver bus okuma callback'i olmadan çalışamaz. */
    }

    eRet = Bmp180_prvLock(psHandle);                         /* Paylaşılan I2C bus, MPU6050 ve BMP180 task'ları çakışmasın diye transfer öncesi kilitlenir. */
    if (eRet != DRIVER_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Lock alınamazsa bus kaynağı güvenli değildir ve driver ERROR durumuna geçer. */
    }

    eRet = psHandle->sBusInterface.pfnRead(psHandle->u8I2cAddress,
                                           u8Reg,
                                           pu8Data,
                                           u16Len,
                                           Bmp180_prvGetBusTimeout(psHandle),
                                           psHandle->sBusInterface.vpCtx); /* Soyut okuma STM32 HAL değil, enjekte edilmiş port fonksiyonu üzerinden yapılır. */

    eUnlockRet = Bmp180_prvUnlock(psHandle);                 /* Transfer sonucu ne olursa olsun mutex bırakılır; aksi halde diğer task'lar kilitlenebilir. */
    if (eRet != DRIVER_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Bus hatası Class-A sensör erişimini bozduğu için state ERROR yapılır. */
    }

    return (eUnlockRet == DRIVER_OK) ? DRIVER_OK : Bmp180_prvMarkError(psHandle, eUnlockRet); /* Unlock hatası da kaynak yönetimi hatasıdır. */
}

static te_Driver_RetCode Bmp180_prvWriteBlock(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, const uint8_t *pu8Data, uint16_t u16Len)
{
    te_Driver_RetCode eRet;                                  /* Bus yazma sonucunu saklar. */
    te_Driver_RetCode eUnlockRet;                            /* Mutex release sonucunu saklar. */

    if ((psHandle == NULL) || (pu8Data == NULL) || (u16Len == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;                       /* Yazma için kaynak buffer ve uzunluk geçerli olmalıdır. */
    }

    if (psHandle->sBusInterface.pfnWrite == NULL)
    {
        return DRIVER_ERR_CONFIG;                      /* Platform portu yazma callback'i sağlamadıysa komut gönderilemez. */
    }

    eRet = Bmp180_prvLock(psHandle);                         /* Sadece gerçek I2C transferi sırasında kilit alınır; dönüşüm beklemesinde bus serbest kalır. */
    if (eRet != DRIVER_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Lock timeout, paylaşılan kaynak yönetimi problemidir ve üst katmana bildirilir. */
    }

    eRet = psHandle->sBusInterface.pfnWrite(psHandle->u8I2cAddress,
                                            u8Reg,
                                            pu8Data,
                                            u16Len,
                                            Bmp180_prvGetBusTimeout(psHandle),
                                            psHandle->sBusInterface.vpCtx); /* Register yazımı soyut bus arayüzüyle yapılır; STM32 HAL çekirdek driver'a sızmaz. */

    eUnlockRet = Bmp180_prvUnlock(psHandle);                 /* Hata yolunda bile unlock yapılır; FreeRTOS mutex'in sonsuza kadar tutulması önlenir. */
    if (eRet != DRIVER_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Bus yazma hatası ölçüm komutunun gönderilemediğini gösterir. */
    }

    return (eUnlockRet == DRIVER_OK) ? DRIVER_OK : Bmp180_prvMarkError(psHandle, eUnlockRet); /* Unlock başarısızsa driver güvenli hata durumuna geçer. */
}

static te_Driver_RetCode Bmp180_prvReadRegister(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data)
{
    return Bmp180_prvReadBlock(psHandle, u8Reg, pu8Data, 1U); /* Tek register okuma, blok okuma helper'ının 1 baytlık özel halidir. */
}

static te_Driver_RetCode Bmp180_prvWriteRegister(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t u8Data)
{
    return Bmp180_prvWriteBlock(psHandle, u8Reg, &u8Data, 1U); /* Tek register yazma, blok yazma helper'ının 1 baytlık özel halidir. */
}

static te_Driver_RetCode Bmp180_prvCheckChipId(ts_Bmp180_Handle *psHandle, uint8_t *pu8ChipId)
{
    te_Driver_RetCode eRet;                                  /* CHIP_ID register okuma sonucunu taşır. */

    if ((psHandle == NULL) || (pu8ChipId == NULL))
    {
        return DRIVER_ERR_NULL_PTR;                          /* Health-check için handle ve çıktı pointer'ı zorunludur. */
    }

    eRet = Bmp180_prvReadRegister(psHandle, BMP180_REG_CHIP_ID, pu8ChipId); /* Class-A sensör kimliği datasheet CHIP_ID register'ından okunur. */
    psHandle->u8LastChipId = (eRet == DRIVER_OK) ? *pu8ChipId : 0U; /* Health alanı son okunan kimliği debug için saklar. */
    psHandle->bLastBusOk = (eRet == DRIVER_OK);           /* Health alanı bus erişiminin başarılı olup olmadığını saklar. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Bus okuma hatası chip-id karşılaştırmasından önce üst katmana döndürülür. */
    }

    if (*pu8ChipId != BMP180_CHIP_ID_EXPECTED)
    {
        return Bmp180_prvMarkError(psHandle, DRIVER_ERR_WHOAMI); /* Beklenmeyen kimlik yanlış sensör veya adres problemidir. */
    }

    return DRIVER_OK;                                    /* CHIP_ID beklenen 0x55 değeridir; sensör kimliği doğrulanmıştır. */
}

static int16_t Bmp180_prvParseBeS16(const uint8_t *pu8Data)
{
    return (int16_t)Bmp180_prvParseBeU16(pu8Data);           /* Önce unsigned big-endian birleştirilir, sonra signed katsayıya çevrilerek signed shift riski önlenir. */
}

static uint16_t Bmp180_prvParseBeU16(const uint8_t *pu8Data)
{
    return (uint16_t)(((uint16_t)pu8Data[0] << 8) | (uint16_t)pu8Data[1]); /* Unsigned katsayılar sistem endian'ına güvenmeden big-endian parse edilir. */
}

static te_Driver_RetCode Bmp180_prvReadCalibrationCoeffs(ts_Bmp180_Handle *psHandle)
{
    uint8_t au8Calib[BMP180_CALIB_DATA_LENGTH];              /* Datasheet AC1..MD bloğunu geçici olarak taşıyan ham byte buffer'ıdır. */
    te_Driver_RetCode eRet;                                  /* Blok okuma sonucunu taşır. */

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                          /* Kalibrasyon handle içine yazılacağı için handle zorunludur. */
    }

    Bmp180_prvMemZero(au8Calib, (uint32_t)sizeof(au8Calib));  /* Ham buffer temizlenir; kısmi hata sonrası eski veri kalması önlenir. */
    eRet = Bmp180_prvReadBlock(psHandle, BMP180_REG_CALIB_START, au8Calib, BMP180_CALIB_DATA_LENGTH); /* Kalibrasyon register'ları datasheet sırasıyla tek blok okunur. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Bus hatasında parse yapılmaz; hatalı katsayı üretimi önlenir. */
    }

    psHandle->sCalibrationCoeffs.s16AC1 = Bmp180_prvParseBeS16(&au8Calib[0]);   /* AC1 signed katsayısı register bloğunun 0..1 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.s16AC2 = Bmp180_prvParseBeS16(&au8Calib[2]);   /* AC2 signed katsayısı register bloğunun 2..3 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.s16AC3 = Bmp180_prvParseBeS16(&au8Calib[4]);   /* AC3 signed katsayısı register bloğunun 4..5 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.u16AC4 = Bmp180_prvParseBeU16(&au8Calib[6]);   /* AC4 unsigned katsayısı register bloğunun 6..7 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.u16AC5 = Bmp180_prvParseBeU16(&au8Calib[8]);   /* AC5 unsigned katsayısı register bloğunun 8..9 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.u16AC6 = Bmp180_prvParseBeU16(&au8Calib[10]);  /* AC6 unsigned katsayısı register bloğunun 10..11 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.s16B1 = Bmp180_prvParseBeS16(&au8Calib[12]);   /* B1 signed katsayısı register bloğunun 12..13 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.s16B2 = Bmp180_prvParseBeS16(&au8Calib[14]);   /* B2 signed katsayısı register bloğunun 14..15 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.s16MB = Bmp180_prvParseBeS16(&au8Calib[16]);   /* MB signed katsayısı register bloğunun 16..17 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.s16MC = Bmp180_prvParseBeS16(&au8Calib[18]);   /* MC signed katsayısı register bloğunun 18..19 offsetlerinden çıkarılır. */
    psHandle->sCalibrationCoeffs.s16MD = Bmp180_prvParseBeS16(&au8Calib[20]);   /* MD signed katsayısı register bloğunun 20..21 offsetlerinden çıkarılır. */

    psHandle->bLastCalibrationOk = Bmp180_prvValidateCalibrationCoeffs(&psHandle->sCalibrationCoeffs); /* Tüm sıfır veya tüm 0xFF gibi hatalı okumalar ayıklanır. */
    if (psHandle->bLastCalibrationOk == false)
    {
        return Bmp180_prvMarkError(psHandle, DRIVER_ERR_IO); /* Kalibrasyon mantıksızsa ölçüm hesabı güvenli değildir. */
    }

    return DRIVER_OK;                                    /* Kalibrasyon katsayıları handle içine güvenli şekilde yüklenmiştir. */
}

static bool Bmp180_prvValidateCalibrationCoeffs(const ts_Bmp180_CalibrationCoeffs *psCalib)
{
    bool bAllZero;                                           /* Tüm katsayıların 0 okunup okunmadığını kontrol eder; kopuk bus belirtisi olabilir. */
    bool bAllOnes;                                           /* Tüm katsayıların 0xFF okunup okunmadığını kontrol eder; pull-up/bus hatası belirtisi olabilir. */

    if (psCalib == NULL)
    {
        return false;                                        /* Kalibrasyon pointer'ı yoksa ölçüm algoritması çalıştırılamaz. */
    }

    bAllZero = (psCalib->s16AC1 == 0) &&
               (psCalib->s16AC2 == 0) &&
               (psCalib->s16AC3 == 0) &&
               (psCalib->u16AC4 == 0U) &&
               (psCalib->u16AC5 == 0U) &&
               (psCalib->u16AC6 == 0U) &&
               (psCalib->s16B1 == 0) &&
               (psCalib->s16B2 == 0) &&
               (psCalib->s16MB == 0) &&
               (psCalib->s16MC == 0) &&
               (psCalib->s16MD == 0);                       /* Tüm sıfır kontrolü blok okumanın gerçek kalibrasyon getirmediğini yakalar. */

    bAllOnes = (psCalib->s16AC1 == (int16_t)-1) &&
               (psCalib->s16AC2 == (int16_t)-1) &&
               (psCalib->s16AC3 == (int16_t)-1) &&
               (psCalib->u16AC4 == 0xFFFFU) &&
               (psCalib->u16AC5 == 0xFFFFU) &&
               (psCalib->u16AC6 == 0xFFFFU) &&
               (psCalib->s16B1 == (int16_t)-1) &&
               (psCalib->s16B2 == (int16_t)-1) &&
               (psCalib->s16MB == (int16_t)-1) &&
               (psCalib->s16MC == (int16_t)-1) &&
               (psCalib->s16MD == (int16_t)-1);             /* Tüm 0xFF kontrolü boştaki I2C hattı veya yanlış adres durumunu yakalar. */

    if ((bAllZero == true) || (bAllOnes == true))
    {
        return false;                                        /* Bariz geçersiz kalibrasyonla telafi algoritması çalıştırılmaz. */
    }

    if ((psCalib->u16AC4 == 0U) || (psCalib->u16AC5 == 0U) || (psCalib->s16MD == 0))
    {
        return false;                                        /* Datasheet algoritmasında bölme/ölçekleme için kritik katsayılar sıfır olamaz. */
    }

    return true;                                             /* Kalibrasyon broad sanity-check'i geçti; daha sıkı fabrika aralığı varsayılmaz. */
}

static uint32_t Bmp180_prvGetPressureConversionTimeMs(te_Bmp180_Oversampling eOss)
{
    switch (eOss)
    {
    case BMP180_OSS0_ULTRA_LOW_POWER:
        return BMP180_PRESS_CONV_TIME_OSS0_MS;               /* OSS0 datasheet dönüşüm süresi seçilir. */
    case BMP180_OSS1_STANDARD:
        return BMP180_PRESS_CONV_TIME_OSS1_MS;               /* OSS1 datasheet dönüşüm süresi seçilir. */
    case BMP180_OSS2_HIGH_RESOLUTION:
        return BMP180_PRESS_CONV_TIME_OSS2_MS;               /* OSS2 datasheet dönüşüm süresi seçilir. */
    case BMP180_OSS3_ULTRA_HIGH_RESOLUTION:
        return BMP180_PRESS_CONV_TIME_OSS3_MS;               /* OSS3 datasheet dönüşüm süresi seçilir. */
    default:
        return BMP180_PRESS_CONV_TIME_OSS0_MS;               /* Geçersiz enum savunmacı olarak en kısa güvenli varsayılana düşer. */
    }
}

static uint8_t Bmp180_prvBuildPressureConversionCommand(te_Bmp180_Oversampling eOss)
{
    return BMP180_BUILD_PRESSURE_CMD((uint8_t)eOss);          /* Datasheet formülü 0x34 + (OSS << 6) makro üzerinden uygulanır. */
}

static te_Driver_RetCode Bmp180_prvDelayMs(ts_Bmp180_Handle *psHandle, uint32_t u32DelayMs)
{
    if ((psHandle == NULL) || (psHandle->sTimingInterface.pfnDelayMs == NULL))
    {
        return DRIVER_ERR_CONFIG;                             /* Open, soft-reset ve deployment test gibi servis akışları için timing callback zorunludur. */
    }

    return psHandle->sTimingInterface.pfnDelayMs(u32DelayMs, psHandle->sTimingInterface.vpCtx); /* Runtime ölçüm state machine'i bu helper'ı çağırmaz; bloklu servis seçimi port katmanına bırakılır. */
}

static uint32_t Bmp180_prvGetTickMs(const ts_Bmp180_Handle *psHandle)
{
    return psHandle->sTimingInterface.pfnGetTickMs(psHandle->sTimingInterface.vpCtx); /* Open doğrulaması callback'in varlığını garanti eder; core yalnız soyut tick kaynağını bilir. */
}

static bool Bmp180_prvHasDeadlineElapsed(uint32_t u32NowMs, uint32_t u32DeadlineMs)
{
    return ((int32_t)(u32NowMs - u32DeadlineMs) >= 0);         /* Unsigned farkın signed yorumu uint32_t tick wrap-around durumunda kısa deadline'ları güvenle karşılaştırır. */
}

static te_Driver_RetCode Bmp180_prvIsConversionBusy(ts_Bmp180_Handle *psHandle, bool *pbIsBusy)
{
    uint8_t u8Control = 0U;                                    /* CONTROL register ham değeridir; datasheet SCO biti bu byte içinden okunur. */
    te_Driver_RetCode eRet;                                    /* Tek register okumasının sonucunu taşır. */

    if ((psHandle == NULL) || (pbIsBusy == NULL))
    {
        return DRIVER_ERR_NULL_PTR;                            /* State machine ve çıktı pointer'ı olmadan SCO kontrolü güvenli değildir. */
    }

    eRet = Bmp180_prvReadRegister(psHandle, BMP180_REG_CONTROL, &u8Control); /* Dönüşüm sırasında yalnız kısa bir I2C okuması yapılır; mutex hemen bırakılır. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                           /* Bus hatası state machine'i ilerletmeden üst katmana taşınır. */
    }

    *pbIsBusy = ((u8Control & BMP180_CONTROL_SCO_MASK) != 0U);  /* Datasheet SCO=1 durumunu fiziksel dönüşüm devam ediyor olarak yorumlar. */
    return DRIVER_OK;                                          /* Dönüşüm durumu başarıyla okunmuştur. */
}

static te_Driver_RetCode Bmp180_prvReadRawTemperatureResult(ts_Bmp180_Handle *psHandle, int32_t *ps32RawTemperature)
{
    uint8_t au8Raw[BMP180_RAW_TEMP_DATA_LENGTH];             /* OUT_MSB ve OUT_LSB sıcaklık baytlarını taşır. */
    te_Driver_RetCode eRet;                                  /* Hazır output register okumasının sonucunu taşır. */

    if ((psHandle == NULL) || (ps32RawTemperature == NULL))
    {
        return DRIVER_ERR_NULL_PTR;                          /* Ham sıcaklık çıkışı için geçerli pointer zorunludur. */
    }

    eRet = Bmp180_prvReadBlock(psHandle, BMP180_REG_OUT_MSB, au8Raw, BMP180_RAW_TEMP_DATA_LENGTH); /* Sıcaklık sonucu MSB-first iki bayt olarak okunur. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Output register okuması başarısızsa ham UT üretilmez. */
    }

    *ps32RawTemperature = (int32_t)Bmp180_prvParseBeU16(au8Raw); /* UT unsigned 16-bit sayı olarak datasheet algoritmasına verilir. */
    return DRIVER_OK;                                        /* Ham sıcaklık okuması bekleme yapmadan tamamlanmıştır. */
}

static te_Driver_RetCode Bmp180_prvReadRawPressureResult(ts_Bmp180_Handle *psHandle, int32_t *ps32RawPressure)
{
    uint8_t au8Raw[BMP180_RAW_PRESS_DATA_LENGTH];            /* OUT_MSB, OUT_LSB ve OUT_XLSB basınç baytlarını taşır. */
    te_Driver_RetCode eRet;                                  /* Hazır output register okumasının sonucunu taşır. */

    if ((psHandle == NULL) || (ps32RawPressure == NULL))
    {
        return DRIVER_ERR_NULL_PTR;                          /* Ham basınç çıkışı için geçerli pointer zorunludur. */
    }

    eRet = Bmp180_prvReadBlock(psHandle, BMP180_REG_OUT_MSB, au8Raw, BMP180_RAW_PRESS_DATA_LENGTH); /* Basınç sonucu MSB-first üç bayt olarak okunur. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Bus okuma hatası ham UP üretimini engeller. */
    }

    *ps32RawPressure = (int32_t)((((uint32_t)au8Raw[0] << 16) |
                                  ((uint32_t)au8Raw[1] << 8) |
                                  (uint32_t)au8Raw[2]) >> (8U - (uint8_t)psHandle->eOversampling)); /* Datasheet UP hizalaması OSS'ye göre sağa kaydırılarak yapılır. */
    return DRIVER_OK;                                        /* Ham basınç okuması bekleme yapmadan tamamlanmıştır. */
}

static te_Driver_RetCode Bmp180_prvStartMeasurement(ts_Bmp180_Handle *psHandle)
{
    uint32_t u32NowMs;                                        /* Başlatma anındaki monotonik tick değeridir; deadline hesabının referansıdır. */
    te_Driver_RetCode eRet;                                   /* CONTROL register yazım sonucunu taşır. */

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                            /* State machine handle olmadan başlatılamaz. */
    }
    if (psHandle->eState != BMP180_STATE_READY)
    {
        return DRIVER_ERR_STATE;                               /* Open tamamlanmadan veya hata durumundayken yeni ölçüm başlatılmaz. */
    }
    if (psHandle->eMeasurementState != BMP180_MEASUREMENT_IDLE)
    {
        return DRIVER_OK;                                      /* Periyodik scheduler aktif dönüşüm varken tekrar tetiklerse mevcut ölçüm korunur. */
    }

    eRet = Bmp180_prvWriteRegister(psHandle, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP); /* Datasheet sıcaklık dönüşümü CONTROL register'a 0x2E yazılarak başlatılır. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                           /* Komut gönderilemediyse state değiştirilmez. */
    }

    u32NowMs = Bmp180_prvGetTickMs(psHandle);                  /* Dönüşüm komutu tamamlandıktan sonra gerçek başlangıç tick'i alınır. */
    psHandle->u32ConversionDeadlineMs = u32NowMs + BMP180_TEMP_CONV_TIME_MS; /* Bu zamandan önce output register ve SCO kontrolü yapılmaz. */
    psHandle->u32ConversionTimeoutMs = psHandle->u32ConversionDeadlineMs + BMP180_CONVERSION_TIMEOUT_MARGIN_MS; /* Sensör takılırsa bounded timeout üretilir. */
    psHandle->eMeasurementState = BMP180_MEASUREMENT_WAIT_TEMPERATURE; /* Bir sonraki Process çağrısı sıcaklık dönüşümünü takip eder. */
    return DRIVER_OK;                                          /* Fonksiyon fiziksel dönüşümü beklemeden task'e geri döner. */
}

static te_Driver_RetCode Bmp180_prvProcessMeasurement(ts_Bmp180_Handle *psHandle)
{
    uint32_t u32NowMs;                                        /* Her Process çağrısındaki monotonik tick değeridir. */
    uint32_t u32PressureConversionMs;                          /* Seçili OSS için datasheet basınç dönüşüm süresidir. */
    int32_t s32RawPressure = 0;                                /* Basınç state'i tamamlandığında okunacak ham UP değeridir. */
    uint8_t u8Command;                                        /* OSS bitleri eklenmiş basınç dönüşüm komut baytıdır. */
    bool bIsBusy = false;                                     /* CONTROL SCO bitinin çözümlenmiş değeridir. */
    te_Driver_RetCode eRet;                                   /* Tek state machine adımının hata kodunu taşır. */

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                            /* Process geçerli instance olmadan çalışamaz. */
    }
    if (psHandle->eState != BMP180_STATE_READY)
    {
        return DRIVER_ERR_STATE;                               /* Hatalı veya kapalı instance runtime akışında ilerletilmez. */
    }
    if (psHandle->eMeasurementState == BMP180_MEASUREMENT_IDLE)
    {
        return DRIVER_OK;                                      /* Aktif ölçüm yoksa Process hızlı no-op olarak geri döner. */
    }

    u32NowMs = Bmp180_prvGetTickMs(psHandle);                  /* Deadline karşılaştırması için yalnız soyut tick callback'i kullanılır. */
    if (Bmp180_prvHasDeadlineElapsed(u32NowMs, psHandle->u32ConversionDeadlineMs) == false)
    {
        return DRIVER_OK;                                      /* Fiziksel dönüşüm minimum süresi dolmadıysa hiçbir bus erişimi yapmadan çıkılır. */
    }

    eRet = Bmp180_prvIsConversionBusy(psHandle, &bIsBusy);     /* Minimum süre geçince datasheet SCO biti kısa I2C okumasıyla doğrulanır. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                           /* Bus hatası private bus helper tarafından ERROR durumuna taşınmıştır. */
    }
    if (bIsBusy == true)
    {
        return (Bmp180_prvHasDeadlineElapsed(u32NowMs, psHandle->u32ConversionTimeoutMs) == true) ?
                   Bmp180_prvMarkError(psHandle, DRIVER_ERR_TIMEOUT) :
                   DRIVER_OK;                                  /* Sensör henüz hazır değilse bounded timeout dolana kadar task bloklanmadan çıkılır. */
    }

    if (psHandle->eMeasurementState == BMP180_MEASUREMENT_WAIT_TEMPERATURE)
    {
        eRet = Bmp180_prvReadRawTemperatureResult(psHandle, &psHandle->s32PendingRawTemperature); /* Hazır UT değeri telafi için handle içine alınır. */
        if (eRet != DRIVER_OK)
        {
            return eRet;                                       /* UT okunamazsa basınç dönüşümü başlatılmaz. */
        }

        u8Command = Bmp180_prvBuildPressureConversionCommand(psHandle->eOversampling); /* Datasheet 0x34+(OSS<<6) komutu seçili çözünürlükle oluşturulur. */
        eRet = Bmp180_prvWriteRegister(psHandle, BMP180_REG_CONTROL, u8Command); /* Basınç dönüşümü kısa register yazımıyla başlatılır. */
        if (eRet != DRIVER_OK)
        {
            return eRet;                                       /* Komut yazma hatası state machine'i ilerletmez. */
        }

        u32NowMs = Bmp180_prvGetTickMs(psHandle);              /* Basınç komutu sonrası deadline yeni başlangıç anına göre hesaplanır. */
        u32PressureConversionMs = Bmp180_prvGetPressureConversionTimeMs(psHandle->eOversampling); /* OSS datasheet süresine çevrilir. */
        psHandle->u32ConversionDeadlineMs = u32NowMs + u32PressureConversionMs; /* Basınç hazır olmadan output register okunmaz. */
        psHandle->u32ConversionTimeoutMs = psHandle->u32ConversionDeadlineMs + BMP180_CONVERSION_TIMEOUT_MARGIN_MS; /* Takılı sensör bounded timeout ile yakalanır. */
        psHandle->eMeasurementState = BMP180_MEASUREMENT_WAIT_PRESSURE; /* Sonraki Process çağrısı basınç sonucunu takip eder. */
        return DRIVER_OK;                                      /* Task basınç dönüşümünü beklemeden scheduler'a döner. */
    }

    if (psHandle->eMeasurementState == BMP180_MEASUREMENT_WAIT_PRESSURE)
    {
        eRet = Bmp180_prvReadRawPressureResult(psHandle, &s32RawPressure); /* Hazır UP register'ları tek kısa burst ile okunur. */
        if (eRet != DRIVER_OK)
        {
            return eRet;                                       /* UP okunamadıysa SI örneği güncellenmez. */
        }

        eRet = Bmp180_prvBuildLatestData(psHandle, s32RawPressure); /* Datasheet telafisi cache üzerinde tamamlanır. */
        if (eRet != DRIVER_OK)
        {
            return Bmp180_prvMarkError(psHandle, eRet);        /* Hesap veya kalibrasyon hatası instance'ı güvenli ERROR durumuna alır. */
        }

        psHandle->eMeasurementState = BMP180_MEASUREMENT_IDLE; /* Tamamlanan ölçümden sonra yeni periyodik tetik kabul edilir. */
        return DRIVER_OK;                                      /* SI cache güncellenmiştir; Read çağrısı artık yeni timestamp'i kopyalayabilir. */
    }

    return Bmp180_prvMarkError(psHandle, DRIVER_ERR_STATE);    /* Tanımsız state bellekte bozulma veya programlama hatasıdır. */
}

static te_Driver_RetCode Bmp180_prvComputeTrueValues(const ts_Bmp180_CalibrationCoeffs *psCalib,
                                                      int32_t s32RawTemperature,
                                                      int32_t s32RawPressure,
                                                      te_Bmp180_Oversampling eOss,
                                                      int32_t *ps32TemperatureDeciC,
                                                      int32_t *ps32PressurePa)
{
    int32_t s32X1;                                           /* Datasheet geçici değişkenidir; hem sıcaklık hem basınç telafisinde kullanılır. */
    int32_t s32X2;                                           /* Datasheet geçici değişkenidir; integer hesapla taşınabilirlik sağlar. */
    int32_t s32X3;                                           /* Datasheet geçici değişkenidir; B3/B4 hesaplarında ara toplamdır. */
    int32_t s32B3;                                           /* Datasheet B3 ara değişkenidir; basınç offset telafisini taşır. */
    int32_t s32B5;                                           /* Datasheet B5 ara değişkenidir; gerçek sıcaklık ve basınç için ortak durumdur. */
    int32_t s32B6;                                           /* Datasheet B6 ara değişkenidir; B5'ten 4000 çıkarılarak elde edilir. */
    uint32_t u32B4;                                          /* Datasheet B4 ara değişkenidir; unsigned tutulması algoritma ile uyumludur. */
    uint64_t u64B7;                                          /* Datasheet B7 ara değişkenidir; çarpım taşmasını önlemek için 64-bit tutulur. */
    int32_t s32Pressure;                                     /* Son telafi edilmiş basınç değeridir; Pascal biriminde integer üretilir. */
    int32_t s32Divisor;                                      /* Sıcaklık X2 paydasıdır; sıfır bölme savunması için ayrı tutulur. */
    int64_t s64B3Base;                                       /* B3 hesabının signed ara tabanıdır; negatif signed left-shift riskini önlemek için ayrı tutulur. */
    uint8_t u8Oss;                                           /* Enum değerinin datasheet algoritmasında kullanılan 0..3 integer karşılığıdır. */

    if ((psCalib == NULL) || (ps32TemperatureDeciC == NULL) || (ps32PressurePa == NULL))
    {
        return DRIVER_ERR_NULL_PTR;                          /* Hesap fonksiyonu kalibrasyon ve çıktı pointer'ları olmadan çalışamaz. */
    }

    if (Bmp180_prvValidateCalibrationCoeffs(psCalib) == false)
    {
        return DRIVER_ERR_IO;                 /* Güvenilmeyen kalibrasyon katsayılarıyla datasheet telafisi yapılmaz. */
    }

    u8Oss = (uint8_t)eOss;                                   /* Oversampling enum'u datasheet denklemlerindeki shift değerine çevrilir. */
    s32X1 = (int32_t)(((int64_t)(s32RawTemperature - (int32_t)psCalib->u16AC6) * (int64_t)psCalib->u16AC5) >> 15); /* UT, AC5 ve AC6 ile sıcaklık X1 terimi hesaplanır. */
    s32Divisor = s32X1 + (int32_t)psCalib->s16MD;             /* Datasheet X2 paydasıdır; MD katsayısı sıfır olmasa bile toplam sıfır olabilir. */
    if (s32Divisor == 0)
    {
        return DRIVER_ERR_IO;                 /* Sıfıra bölme ölçüm algoritmasını geçersiz kılar. */
    }

    s32X2 = (int32_t)(((int64_t)psCalib->s16MC * 2048LL) / s32Divisor); /* MC katsayısıyla sıcaklık X2 terimi hesaplanır; negatif değeri sola kaydırmamak için çarpım kullanılır. */
    s32B5 = s32X1 + s32X2;                                   /* B5 gerçek sıcaklık ve basınç denklemlerinde ortak ara değerdir. */
    *ps32TemperatureDeciC = (s32B5 + 8) >> 4;                 /* Datasheet sıcaklık çıktısı 0.1 Celsius biriminde integer olarak üretilir. */

    s32B6 = s32B5 - 4000;                                    /* Basınç algoritması B6 değerini B5'ten türetir. */
    s32X1 = (int32_t)(((int64_t)psCalib->s16B2 * (((int64_t)s32B6 * (int64_t)s32B6) >> 12)) >> 11); /* B2 ikinci derece basınç telafi terimine uygulanır. */
    s32X2 = (int32_t)(((int64_t)psCalib->s16AC2 * (int64_t)s32B6) >> 11); /* AC2 ve B6 ile lineer basınç telafi terimi hesaplanır. */
    s32X3 = s32X1 + s32X2;                                   /* X1 ve X2 toplamı B3 hesabına girer. */
    s64B3Base = (((int64_t)psCalib->s16AC1 * 4LL) + (int64_t)s32X3); /* B3 tabanı AC1 ve X3 ile üretilir; ara değer signed 64-bit tutulur. */
    s32B3 = (int32_t)(((s64B3Base * (int64_t)(1UL << u8Oss)) + 2LL) / 4LL); /* B3, OSS ile ölçeklenir; olası negatif signed left-shift yerine çarpım kullanılır. */

    s32X1 = (int32_t)(((int64_t)psCalib->s16AC3 * (int64_t)s32B6) >> 13); /* AC3 katsayısı B4 hesabının ilk ara terimini üretir. */
    s32X2 = (int32_t)(((int64_t)psCalib->s16B1 * (((int64_t)s32B6 * (int64_t)s32B6) >> 12)) >> 16); /* B1 katsayısı B4 hesabının ikinci ara terimini üretir. */
    s32X3 = (s32X1 + s32X2 + 2) >> 2;                        /* X3 yuvarlanmış ara toplamdır; datasheet fixed-point ölçeğini korur. */
    u32B4 = (uint32_t)(((uint64_t)psCalib->u16AC4 * (uint64_t)(uint32_t)(s32X3 + 32768)) >> 15); /* AC4 unsigned katsayısı B4 ölçek terimini üretir. */
    if (u32B4 == 0U)
    {
        return DRIVER_ERR_IO;                 /* B4 sıfırsa sonraki basınç bölmesi yapılamaz. */
    }

    u64B7 = (uint64_t)((uint32_t)(s32RawPressure - s32B3)) * (uint64_t)(50000UL >> u8Oss); /* B7 çarpımı 32-bit taşabileceği için 64-bit hesaplanır. */
    if (u64B7 < 0x80000000ULL)
    {
        s32Pressure = (int32_t)((u64B7 * 2ULL) / (uint64_t)u32B4); /* Küçük B7 yolunda önce çarpıp sonra bölme datasheet ile uyumludur. */
    }
    else
    {
        s32Pressure = (int32_t)((u64B7 / (uint64_t)u32B4) * 2ULL); /* Büyük B7 yolunda önce bölme taşmayı azaltır. */
    }

    s32X1 = (s32Pressure >> 8) * (s32Pressure >> 8);          /* Son basınç doğrusal olmayan düzeltme için önce ölçeklenmiş kare terim hazırlanır. */
    s32X1 = (int32_t)(((int64_t)s32X1 * 3038) >> 16);         /* Datasheet 3038 katsayısı basınç kare düzeltmesine uygulanır. */
    s32X2 = (int32_t)(((-7357LL) * (int64_t)s32Pressure) >> 16); /* Datasheet -7357 katsayısı basınç lineer düzeltmesine uygulanır. */
    s32Pressure = s32Pressure + ((s32X1 + s32X2 + 3791) >> 4); /* Final Pascal basınç değeri datasheet yuvarlamasıyla üretilir. */

    *ps32PressurePa = s32Pressure;                           /* Telafi edilmiş basınç caller'a Pascal integer olarak verilir. */
    return DRIVER_OK;                                    /* Sıcaklık ve basınç telafisi başarıyla tamamlanmıştır. */
}

static float Bmp180_prvComputeAltitudeM(float f32PressurePa, float f32SeaLevelPressurePa)
{
    return BMP180_ALTITUDE_SCALE_M *
           (1.0f - powf((f32PressurePa / f32SeaLevelPressurePa), BMP180_ALTITUDE_EXPONENT)); /* Barometrik yükseklik formülü kullanılır; link aşamasında libm gerekebilir. */
}

static te_Driver_RetCode Bmp180_prvBuildLatestData(ts_Bmp180_Handle *psHandle, int32_t s32RawPressure)
{
    int32_t s32TemperatureDeciC = 0;                         /* Datasheet gerçek sıcaklık çıktısıdır; 0.1 Celsius birimindedir. */
    int32_t s32PressurePa = 0;                               /* Telafi edilmiş basınç çıktısıdır; Pascal integer birimindedir. */
    te_Driver_RetCode eRet;                                  /* Datasheet telafi algoritmasının sonucunu taşır. */

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                          /* Cache güncellemesi için geçerli instance handle zorunludur. */
    }

    eRet = Bmp180_prvComputeTrueValues(&psHandle->sCalibrationCoeffs,
                                       psHandle->s32PendingRawTemperature,
                                       s32RawPressure,
                                       psHandle->eOversampling,
                                       &s32TemperatureDeciC,
                                       &s32PressurePa);       /* Tamamlanan UT ve UP değerleri platform bağımsız datasheet algoritmasına verilir. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Hatalı kalibrasyon veya aritmetik durumunda cache eski geçerli örneğini korur. */
    }

    psHandle->sLatestData.f32TemperatureC = ((float)s32TemperatureDeciC) / 10.0f; /* 0.1 Celsius integer çıktısı uygulama için float Celsius'a çevrilir. */
    psHandle->sLatestData.f32PressurePa = (float)s32PressurePa; /* Pascal integer çıktısı uygulama-facing SI float değerine çevrilir. */
    psHandle->sLatestData.f32AltitudeM = Bmp180_prvComputeAltitudeM(psHandle->sLatestData.f32PressurePa,
                                                                   psHandle->f32SeaLevelPressurePa); /* Basınç ve QNH referansından yaklaşık yükseklik hesaplanır. */
    psHandle->sLatestData.u32TimestampMs = Bmp180_prvGetTickMs(psHandle); /* Cache timestamp'i fiziksel basınç dönüşümünün tamamlandığı anda alınır. */
    psHandle->sLatestData.bValid = true;                      /* Bütün dönüşüm ve telafi adımları tamamlandığı için cache geçerli işaretlenir. */
    return DRIVER_OK;                                         /* Read API'sinin beklemeden döndüreceği cache güncellenmiştir. */
}

static void Bmp180_prvMemZero(void *vpData, uint32_t u32Len)
{
    uint8_t *pu8Data = (uint8_t *)vpData;                    /* Byte seviyesinde temizleme için generic pointer uint8_t pointer'a çevrilir. */
    uint32_t u32Idx;                                         /* Sabit süreli basit döngü indeksidir; dinamik bellek veya libc bağımlılığı yoktur. */

    if (vpData == NULL)
    {
        return;                                              /* NULL pointer temizlenmeye çalışılmaz; helper güvenli no-op davranır. */
    }

    for (u32Idx = 0U; u32Idx < u32Len; ++u32Idx)
    {
        pu8Data[u32Idx] = 0U;                                /* Her byte sıfırlanır; handle/config yapılarında eski veri kalması önlenir. */
    }
}

static bool Bmp180_prvIsOversamplingValid(te_Bmp180_Oversampling eOss)
{
    return ((uint8_t)eOss <= BMP180_OSS_MAX);                /* BMP180 datasheet yalnızca 0..3 OSS değerlerini tanımlar. */
}

static bool Bmp180_prvIsSeaLevelPressureValid(float f32PressurePa)
{
    return ((f32PressurePa >= BMP180_SEA_LEVEL_PRESSURE_MIN_PA) &&
            (f32PressurePa <= BMP180_SEA_LEVEL_PRESSURE_MAX_PA)); /* Altitude referansı fiziksel olarak makul geniş atmosfer aralığında tutulur. */
}

te_Driver_RetCode Bmp180_Open(ts_Bmp180_Handle *psHandle, const ts_Bmp180_OpenConfig *psConfig)
{
    uint8_t u8ChipId = 0U;                                   /* Open sırasında okunan CHIP_ID değeridir. */
    te_Driver_RetCode eRet;                                  /* Başlatma adımlarının hata kodunu taşır. */

    if ((psHandle == NULL) || (psConfig == NULL))
    {
        return DRIVER_ERR_NULL_PTR;                          /* Open API sınırında handle ve config pointer'ları zorunludur. */
    }

    if ((psConfig->sBusInterface.pfnRead == NULL) ||
        (psConfig->sBusInterface.pfnWrite == NULL) ||
        (psConfig->sTimingInterface.pfnDelayMs == NULL) ||
        (psConfig->sTimingInterface.pfnGetTickMs == NULL))
    {
        return DRIVER_ERR_CONFIG;                             /* Bus erişimi, servis beklemeleri ve bloklamasız deadline takibi için callback paketi eksiksiz olmalıdır. */
    }

    if (Bmp180_prvIsOversamplingValid(psConfig->eOversampling) == false)
    {
        return DRIVER_ERR_INVALID_ARG;                       /* Geçersiz OSS değeri dönüşüm komutunu datasheet dışına çıkarır. */
    }

    if ((psConfig->f32SeaLevelPressurePa != 0.0f) &&
        (Bmp180_prvIsSeaLevelPressureValid(psConfig->f32SeaLevelPressurePa) == false))
    {
        return DRIVER_ERR_INVALID_ARG;                       /* 0 dışındaki deniz seviyesi referansı makul atmosfer aralığında olmalıdır. */
    }

    Bmp180_prvMemZero(psHandle, (uint32_t)sizeof(*psHandle)); /* Handle temizlenir; global singleton yerine instance iç durumu burada başlar. */
    psHandle->eState = BMP180_STATE_UNINIT;                  /* CHIP_ID ve kalibrasyon doğrulanana kadar driver hazır kabul edilmez. */
    psHandle->u8I2cAddress = (psConfig->u8I2cAddress == 0U) ? BMP180_I2C_ADDR_DEFAULT : psConfig->u8I2cAddress; /* 0 adres, uygulama kolaylığı için varsayılan 0x77'ye çevrilir. */
    psHandle->u32BusTimeoutMs = (psConfig->u32BusTimeoutMs == 0U) ? BMP180_TIMEOUT_DEFAULT_MS : psConfig->u32BusTimeoutMs; /* 0 timeout güvenli varsayılan değere çevrilir. */
    psHandle->u32BusLockTimeoutMs = (psConfig->u32BusLockTimeoutMs == 0U) ? BMP180_LOCK_TIMEOUT_DEFAULT_MS : psConfig->u32BusLockTimeoutMs; /* 0 lock timeout varsayılan değer alır. */
    psHandle->eOversampling = psConfig->eOversampling;       /* Basınç dönüşüm çözünürlüğü handle içinde instance ayarı olarak saklanır. */
    psHandle->f32SeaLevelPressurePa = (psConfig->f32SeaLevelPressurePa == 0.0f) ? BMP180_SEA_LEVEL_PRESSURE_PA : psConfig->f32SeaLevelPressurePa; /* Altitude için varsayılan referans atanır. */
    psHandle->sBusInterface = psConfig->sBusInterface;       /* Bus callback'leri kopyalanır; driver platform bağımsız kalır. */
    psHandle->sLockInterface = psConfig->sLockInterface;     /* Lock callback'leri kopyalanır; shared I2C mutex uygulama/port tarafından sahiplenilir. */
    psHandle->sTimingInterface = psConfig->sTimingInterface; /* Timing callback'leri kopyalanır; conversion delay port üzerinden yapılır. */
    psHandle->eMeasurementState = BMP180_MEASUREMENT_IDLE;   /* Runtime state machine aktif dönüşüm olmadan başlar; ilk tetik Ioctl üzerinden gelir. */

    eRet = Bmp180_prvDelayMs(psHandle, BMP180_POWER_ON_DELAY_MS); /* Başlangıçta kısa bekleme, güçlenme sonrası register cevaplarını kararlı hale getirir. */
    if (eRet != DRIVER_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Timing altyapısı çalışmıyorsa Open güvenli şekilde başarısız olur. */
    }

    eRet = Bmp180_prvCheckChipId(psHandle, &u8ChipId);       /* Class-A sensör kuralı gereği kimlik doğrulaması Open içinde yapılır. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Yanlış kimlik veya bus hatası Open'ı durdurur. */
    }

    eRet = Bmp180_prvReadCalibrationCoeffs(psHandle);        /* BMP180 telafi algoritması için fabrika kalibrasyonu Open sırasında yüklenir. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Kalibrasyon yoksa Read anlamlı SI çıktısı üretemez. */
    }

    psHandle->eState = BMP180_STATE_READY;                   /* Kimlik ve kalibrasyon doğrulandıktan sonra public Read çağrıları açılır. */
    return DRIVER_OK;                                    /* Instance başarıyla kullanıma hazırdır. */
}

te_Driver_RetCode Bmp180_Close(ts_Bmp180_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                          /* Close için handle pointer'ı zorunludur. */
    }

    if (psHandle->eState == BMP180_STATE_UNINIT)
    {
        return DRIVER_ERR_STATE;                     /* Zaten kapalı instance tekrar kapatılmaz; durum hatası açıkça bildirilir. */
    }

    psHandle->eState = BMP180_STATE_UNINIT;                  /* BMP180'de ayrı sleep register olmadığı için uygulama düzeyinde kapalı durum işaretlenir. */
    psHandle->eMeasurementState = BMP180_MEASUREMENT_IDLE;   /* Kapanan instance üzerinde yarım kalan fiziksel dönüşüm yazılım açısından iptal edilir. */
    return DRIVER_OK;                                    /* Close, donanımda kalıcı bir konfigürasyon değişikliği yapmadan tamamlanır. */
}

te_Driver_RetCode Bmp180_Read(ts_Bmp180_Handle *psHandle, ts_Bmp180_Data *psOutData)
{
    if ((psHandle == NULL) || (psOutData == NULL))
    {
        return DRIVER_ERR_NULL_PTR;                          /* Read için handle ve çıktı buffer'ı zorunludur. */
    }

    if (psHandle->eState != BMP180_STATE_READY)
    {
        return DRIVER_ERR_STATE;                     /* Open başarıyla tamamlanmadan veya ERROR durumundayken ölçüm yapılmaz. */
    }

    *psOutData = psHandle->sLatestData;                       /* MPU6050 async Read modeline benzer biçimde yalnız son tamamlanmış cache örneği kopyalanır. */
    return DRIVER_OK;                                        /* Read hiçbir dönüşüm başlatmaz, delay yapmaz ve task'i fiziksel sensör süresince tutmaz. */
}

te_Driver_RetCode Bmp180_Write(ts_Bmp180_Handle *psHandle, const void *vpInData)
{
    (void)psHandle;                                          /* BMP180 generic data write modeli sunmadığı için handle kullanılmaz. */
    (void)vpInData;                                          /* Konfigürasyon ve register erişimi Ioctl üzerinden yürütüldüğü için input kullanılmaz. */
    return DRIVER_ERR_NOT_SUPPORTED;                         /* POSIX ilhamlı API tutarlılığı korunur ama anlamsız işlem sessizce başarılı sayılmaz. */
}

te_Driver_RetCode Bmp180_Ioctl(ts_Bmp180_Handle *psHandle, te_Bmp180_IoctlCmd eCmd, void *vpArg)
{
    te_Driver_RetCode eRet = DRIVER_OK;                  /* Switch içindeki komut sonucunu taşır. */

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                          /* Ioctl her zaman geçerli instance handle ister. */
    }

    if ((psHandle->eState == BMP180_STATE_UNINIT) && (eCmd != BMP180_IOCTL_GET_STATE))
    {
        return DRIVER_ERR_STATE;                     /* Open öncesi yalnızca durum sorgusu güvenli kabul edilir. */
    }

    switch (eCmd)
    {
    case BMP180_IOCTL_GET_VERSION:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Versiyon döndürmek için uint16_t çıktı pointer'ı gerekir. */
            break;
        }
        *(uint16_t *)vpArg = BMP180_DRIVER_API_VERSION;      /* Uygulama derlenen driver API versiyonunu runtime'da okuyabilir. */
        break;

    case BMP180_IOCTL_GET_STATE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Durum döndürmek için te_Bmp180_State pointer'ı gerekir. */
            break;
        }
        *(te_Bmp180_State *)vpArg = psHandle->eState;         /* Handle içindeki durum makinesi uygulamaya kopyalanır. */
        break;

    case BMP180_IOCTL_CHECK_HEALTH:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Health-check sonucu için çıktı struct pointer'ı gerekir. */
            break;
        }
        else
        {
            ts_Bmp180_HealthStatus *psHealth = (ts_Bmp180_HealthStatus *)vpArg; /* Void argüman beklenen health struct tipine çevrilir. */
            Bmp180_prvMemZero(psHealth, (uint32_t)sizeof(*psHealth)); /* Health sonucu temizlenir; kısmi hata alanları eski kalmaz. */
            psHealth->eState = psHandle->eState;             /* Driver state her durumda rapora yazılır. */
            eRet = Bmp180_prvReadRegister(psHandle, BMP180_REG_CHIP_ID, &psHealth->u8ChipIdValue); /* Bus sağlığı CHIP_ID register okumasıyla sınanır. */
            psHealth->bBusOk = (eRet == DRIVER_OK);      /* Okuma başarılıysa bus path çalışıyor kabul edilir. */
            psHealth->bChipIdOk = ((eRet == DRIVER_OK) && (psHealth->u8ChipIdValue == BMP180_CHIP_ID_EXPECTED)); /* Kimlik beklenen 0x55 ile karşılaştırılır. */
            psHealth->bCalibrationOk = Bmp180_prvValidateCalibrationCoeffs(&psHandle->sCalibrationCoeffs); /* Mevcut kalibrasyon katsayıları yeniden sanity-check edilir. */
            psHandle->u8LastChipId = psHealth->u8ChipIdValue; /* Son chip-id değeri handle sağlık alanına yazılır. */
            psHandle->bLastBusOk = psHealth->bBusOk;         /* Son bus sonucu handle içinde saklanır. */
            psHandle->bLastCalibrationOk = psHealth->bCalibrationOk; /* Son kalibrasyon sonucu handle içinde saklanır. */
            if ((eRet == DRIVER_OK) && (psHealth->bChipIdOk == false))
            {
                eRet = DRIVER_ERR_WHOAMI;             /* Bus çalışsa bile yanlış chip-id ayrı hata koduyla raporlanır. */
            }
            if ((eRet == DRIVER_OK) && (psHealth->bCalibrationOk == false))
            {
                eRet = DRIVER_ERR_IO;         /* Kimlik doğru olsa bile kalibrasyon geçersizse health başarısızdır. */
            }
        }
        break;

    case BMP180_IOCTL_SOFT_RESET:
        eRet = Bmp180_prvWriteRegister(psHandle, BMP180_REG_SOFT_RESET, BMP180_CMD_SOFT_RESET); /* Datasheet soft reset komutu 0xE0 register'ına 0xB6 yazılarak verilir. */
        if (eRet == DRIVER_OK)
        {
            eRet = Bmp180_prvDelayMs(psHandle, BMP180_SOFT_RESET_DELAY_MS); /* Reset sonrası kalibrasyon register'ları hazır olsun diye beklenir. */
        }
        if (eRet == DRIVER_OK)
        {
            uint8_t u8ChipId = 0U;                           /* Reset sonrası sensör kimliği tekrar okunur. */
            eRet = Bmp180_prvCheckChipId(psHandle, &u8ChipId); /* Resetin ardından aynı cihazın cevap verdiği doğrulanır. */
        }
        if (eRet == DRIVER_OK)
        {
            eRet = Bmp180_prvReadCalibrationCoeffs(psHandle); /* Reset sonrası kalibrasyon katsayıları yeniden yüklenir. */
        }
        if (eRet == DRIVER_OK)
        {
            psHandle->eState = BMP180_STATE_READY;           /* Reset toparlanması başarılıysa ERROR durumundan READY durumuna dönülebilir. */
            psHandle->eMeasurementState = BMP180_MEASUREMENT_IDLE; /* Reset sonrası yarım dönüşüm yazılım durumundan temizlenir. */
            Bmp180_prvMemZero(&psHandle->sLatestData, (uint32_t)sizeof(psHandle->sLatestData)); /* Reset öncesi cache yeni kalibrasyonla karıştırılmaz. */
        }
        break;

    case BMP180_IOCTL_SET_OVERSAMPLING:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* OSS ayarı için te_Bmp180_Oversampling pointer'ı gerekir. */
            break;
        }
        if (Bmp180_prvIsOversamplingValid(*(te_Bmp180_Oversampling *)vpArg) == false)
        {
            eRet = DRIVER_ERR_INVALID_ARG;                   /* Datasheet dışı OSS değeri komut baytını geçersiz yapar. */
            break;
        }
        if (psHandle->eMeasurementState != BMP180_MEASUREMENT_IDLE)
        {
            eRet = DRIVER_ERR_STATE;                         /* Aktif basınç döngüsünde OSS değiştirmek komut ile UP hizalamasını tutarsız yapabilir. */
            break;
        }
        psHandle->eOversampling = *(te_Bmp180_Oversampling *)vpArg; /* Yeni OSS handle içine yazılır; BMP180'de kalıcı config register yoktur. */
        break;

    case BMP180_IOCTL_GET_OVERSAMPLING:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* OSS okumak için te_Bmp180_Oversampling pointer'ı gerekir. */
            break;
        }
        *(te_Bmp180_Oversampling *)vpArg = psHandle->eOversampling; /* Mevcut instance OSS ayarı uygulamaya kopyalanır. */
        break;

    case BMP180_IOCTL_SET_SEA_LEVEL_PRESSURE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Referans basınç ayarı için float pointer'ı gerekir. */
            break;
        }
        if (Bmp180_prvIsSeaLevelPressureValid(*(float *)vpArg) == false)
        {
            eRet = DRIVER_ERR_INVALID_ARG;                   /* Altitude hesabında fiziksel olmayan referans basınç reddedilir. */
            break;
        }
        psHandle->f32SeaLevelPressurePa = *(float *)vpArg;   /* Uygulama yerel QNH/deniz seviyesi referansını güncelleyebilir. */
        break;

    case BMP180_IOCTL_GET_SEA_LEVEL_PRESSURE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Referans basıncı okumak için float pointer'ı gerekir. */
            break;
        }
        *(float *)vpArg = psHandle->f32SeaLevelPressurePa;    /* Mevcut altitude referansı uygulamaya kopyalanır. */
        break;

    case BMP180_IOCTL_START_MEASUREMENT:
        if (vpArg != NULL)
        {
            eRet = DRIVER_ERR_INVALID_ARG;                   /* START komutu ek argüman almaz; yanlış sözleşme sessizce kabul edilmez. */
            break;
        }
        eRet = Bmp180_prvStartMeasurement(psHandle);          /* Sıcaklık dönüşümü başlatılır ve fiziksel dönüşüm süresi beklenmeden task'e dönülür. */
        break;

    case BMP180_IOCTL_PROCESS_MEASUREMENT:
        if (vpArg != NULL)
        {
            eRet = DRIVER_ERR_INVALID_ARG;                   /* PROCESS komutu ek argüman almaz; handle içindeki state üzerinden ilerler. */
            break;
        }
        eRet = Bmp180_prvProcessMeasurement(psHandle);        /* Her çağrı state machine'i en fazla tek kısa I2C adımı ilerletir. */
        break;

    case BMP180_IOCTL_GET_MEASUREMENT_STATE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Ölçüm state'ini döndürmek için te_Bmp180_MeasurementState pointer'ı gerekir. */
            break;
        }
        *(te_Bmp180_MeasurementState *)vpArg = psHandle->eMeasurementState; /* Runtime gözlemleme ve deployment test için güncel aşama kopyalanır. */
        break;

    case BMP180_IOCTL_GET_CALIBRATION:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Kalibrasyon kopyalamak için struct pointer'ı gerekir. */
            break;
        }
        *(ts_Bmp180_CalibrationCoeffs *)vpArg = psHandle->sCalibrationCoeffs; /* Fabrika katsayıları debug/test için uygulamaya verilir. */
        break;

    case BMP180_IOCTL_REG_READ:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Tek register okuma için access struct pointer'ı gerekir. */
            break;
        }
        eRet = Bmp180_prvReadRegister(psHandle,
                                      ((ts_Bmp180_RegisterAccess *)vpArg)->u8RegisterAddr,
                                      &((ts_Bmp180_RegisterAccess *)vpArg)->u8Value); /* Debug register okuma da lock'lu private helper üzerinden yapılır. */
        break;

    case BMP180_IOCTL_REG_WRITE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;                      /* Tek register yazma için access struct pointer'ı gerekir. */
            break;
        }
        eRet = Bmp180_prvWriteRegister(psHandle,
                                      ((ts_Bmp180_RegisterAccess *)vpArg)->u8RegisterAddr,
                                      ((ts_Bmp180_RegisterAccess *)vpArg)->u8Value); /* Debug register yazımı güncel MPU6050 modeline benzer biçimde doğrudan private write helper kullanır. */
        break;

    case BMP180_IOCTL_REG_READ_BLOCK:
        if ((vpArg == NULL) ||
            (((ts_Bmp180_RegisterBlockAccess *)vpArg)->pu8Buffer == NULL) ||
            (((ts_Bmp180_RegisterBlockAccess *)vpArg)->u16Length == 0U))
        {
            eRet = DRIVER_ERR_INVALID_ARG;                   /* Blok okuma için struct, buffer ve uzunluk zorunludur. */
            break;
        }
        eRet = Bmp180_prvReadBlock(psHandle,
                                   ((ts_Bmp180_RegisterBlockAccess *)vpArg)->u8RegisterAddr,
                                   ((ts_Bmp180_RegisterBlockAccess *)vpArg)->pu8Buffer,
                                   ((ts_Bmp180_RegisterBlockAccess *)vpArg)->u16Length); /* Blok register okuma bus mutex korumalı helper ile yapılır. */
        break;

    case BMP180_IOCTL_REG_WRITE_BLOCK:
        if ((vpArg == NULL) ||
            (((ts_Bmp180_RegisterBlockAccess *)vpArg)->pu8Buffer == NULL) ||
            (((ts_Bmp180_RegisterBlockAccess *)vpArg)->u16Length == 0U))
        {
            eRet = DRIVER_ERR_INVALID_ARG;                   /* Blok yazma için struct, buffer ve uzunluk zorunludur. */
            break;
        }
        eRet = Bmp180_prvWriteBlock(psHandle,
                                    ((ts_Bmp180_RegisterBlockAccess *)vpArg)->u8RegisterAddr,
                                    ((ts_Bmp180_RegisterBlockAccess *)vpArg)->pu8Buffer,
                                    ((ts_Bmp180_RegisterBlockAccess *)vpArg)->u16Length); /* Blok register yazma port callback üzerinden yapılır. */
        break;

    default:
        eRet = DRIVER_ERR_NOT_SUPPORTED;                     /* Tanımlanmayan ioctl komutları sessiz başarıya çevrilmez. */
        break;
    }

    if ((eRet == DRIVER_ERR_BUS) ||
        (eRet == DRIVER_ERR_TIMEOUT) ||
        (eRet == DRIVER_ERR_WHOAMI) ||
        (eRet == DRIVER_ERR_IO))
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Sağlık veya bus kökenli ciddi hatalar durum makinesini ERROR yapar. */
    }

    return eRet;                                             /* Parametre/unsupported gibi sözleşme hataları state'i bozmadan döner. */
}

te_Driver_RetCode Bmp180_Test(ts_Bmp180_Handle *psHandle, uint32_t u32TimeoutMs)
{
    ts_Bmp180_HealthStatus sHealth;                          /* Health-check çıktısını yerel olarak tutar. */
    ts_Bmp180_Data sData;                                    /* Örnek ölçüm çıktısını yerel olarak tutar. */
    te_Bmp180_MeasurementState eMeasurementState;            /* Deployment test sırasında bloklamasız state machine aşamasını izler. */
    te_Driver_RetCode eRet;                                  /* Test adımlarının sonucunu taşır. */
    uint32_t u32ElapsedMs = 0U;                              /* Test içindeki kontrollü polling süresini bounded tutar. */
    uint32_t u32TimeoutBudgetMs;                             /* Caller sıfır verirse güvenli varsayılan bütçeyi taşır. */

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;                          /* Test için geçerli handle zorunludur. */
    }

    eRet = Bmp180_Ioctl(psHandle, BMP180_IOCTL_CHECK_HEALTH, &sHealth); /* Önce bus, chip-id ve kalibrasyon sağlığı doğrulanır. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Health-check başarısızsa ölçüm testine geçilmez. */
    }

    u32TimeoutBudgetMs = (u32TimeoutMs == 0U) ? BMP180_TEST_TIMEOUT_DEFAULT_MS : u32TimeoutMs; /* Test sonsuz beklemeye dönüşmez. */
    eRet = Bmp180_Ioctl(psHandle, BMP180_IOCTL_START_MEASUREMENT, NULL); /* Runtime ile aynı state machine ölçümü kontrollü test akışında başlatılır. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Ölçüm tetiklenemiyorsa deployment testi başarısızdır. */
    }

    do
    {
        eRet = Bmp180_Ioctl(psHandle, BMP180_IOCTL_PROCESS_MEASUREMENT, NULL); /* Test API state machine'i task bağlamında adım adım ilerletir. */
        if (eRet != DRIVER_OK)
        {
            return eRet;                                     /* Bus, timeout veya state hatası doğrudan raporlanır. */
        }

        eRet = Bmp180_Ioctl(psHandle, BMP180_IOCTL_GET_MEASUREMENT_STATE, &eMeasurementState); /* Tamamlanma durumu handle içinden okunur. */
        if (eRet != DRIVER_OK)
        {
            return eRet;                                     /* State sorgusu başarısızsa test güvenilir biçimde devam edemez. */
        }

        if (eMeasurementState != BMP180_MEASUREMENT_IDLE)
        {
            eRet = Bmp180_prvDelayMs(psHandle, BMP180_TEST_POLL_DELAY_MS); /* Bloklama yalnız açıkça deployment amaçlı Test API içinde ve enjekte callback ile yapılır. */
            if (eRet != DRIVER_OK)
            {
                return eRet;                                 /* Test timing servisi başarısızsa ölçüm sonucu beklenmez. */
            }
            u32ElapsedMs += BMP180_TEST_POLL_DELAY_MS;       /* Bounded test bütçesi her kontrollü polling adımında azaltılır. */
            if (u32ElapsedMs >= u32TimeoutBudgetMs)
            {
                return Bmp180_prvMarkError(psHandle, DRIVER_ERR_TIMEOUT); /* Deployment test fiziksel sensör cevap vermezse deterministik sonlanır. */
            }
        }
    } while (eMeasurementState != BMP180_MEASUREMENT_IDLE);

    eRet = Bmp180_Read(psHandle, &sData);                    /* Tamamlanan cache örneği bekleme yapmadan public Read API üzerinden alınır. */
    if (eRet != DRIVER_OK)
    {
        return eRet;                                         /* Read cache erişim hatası doğrudan test hatasıdır. */
    }
    if (sData.bValid == false)
    {
        return Bmp180_prvMarkError(psHandle, DRIVER_ERR_IO); /* Başarılı Read sonrası valid false ise API sözleşmesi bozulmuştur. */
    }

    if ((sData.f32TemperatureC < BMP180_TEMP_MIN_C) ||
        (sData.f32TemperatureC > BMP180_TEMP_MAX_C) ||
        (sData.f32PressurePa < BMP180_PRESSURE_MIN_PA) ||
        (sData.f32PressurePa > BMP180_PRESSURE_MAX_PA))
    {
        return Bmp180_prvMarkError(psHandle, DRIVER_ERR_IO); /* Geniş fiziksel aralık dışı veri byte sırası veya kalibrasyon sorununa işaret eder. */
    }

    return DRIVER_OK;                                    /* Health-check ve örnek ölçüm başarıyla tamamlanmıştır. */
}
