#include "bmp180_driver.h"
#include "bmp180_hal.h"

#include <math.h>

#define BMP180_TIMEOUT_DEFAULT_MS                  (100U)     /* Bus transferleri için platform bağımsız varsayılan timeout değeridir. */
#define BMP180_LOCK_TIMEOUT_DEFAULT_MS             (100U)     /* Paylaşılan I2C mutex'i için varsayılan bekleme süresidir. */

#ifndef NULL
#define NULL                                       ((void *)0) /* Standart başlık eklemeden NULL kullanımı için yerel korumadır. */
#endif

static te_Bmp180_RetCode Bmp180_prvMarkError(ts_Bmp180_Handle *psHandle, te_Bmp180_RetCode eRet); /* Platform bağımsız driver helper'ıdır; ciddi hatalarda handle state'ini ERROR yapar. */
static te_Bmp180_RetCode Bmp180_prvLock(ts_Bmp180_Handle *psHandle); /* RTOS/resource-management helper'ıdır; enjekte edilen lock callback'i üzerinden paylaşılan bus'ı korur. */
static te_Bmp180_RetCode Bmp180_prvUnlock(ts_Bmp180_Handle *psHandle); /* RTOS/resource-management helper'ıdır; her bus transferinden sonra lock callback'ini bırakır. */
static uint32_t Bmp180_prvGetBusTimeout(const ts_Bmp180_Handle *psHandle); /* Platform bağımsız driver helper'ıdır; 0 timeout konfigürasyonunu güvenli varsayılana çevirir. */
static te_Bmp180_RetCode Bmp180_prvReadBlock(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data, uint16_t u16Len); /* Bus erişim helper'ıdır; lock alıp soyut read callback'iyle BMP180 register bloğu okur. */
static te_Bmp180_RetCode Bmp180_prvWriteBlock(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, const uint8_t *pu8Data, uint16_t u16Len); /* Bus erişim helper'ıdır; lock alıp soyut write callback'iyle BMP180 register bloğu yazar. */
static te_Bmp180_RetCode Bmp180_prvReadRegister(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data); /* Register erişim helper'ıdır; tek baytlık BMP180 register okumasını blok helper'a indirger. */
static te_Bmp180_RetCode Bmp180_prvWriteRegister(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t u8Data); /* Register erişim helper'ıdır; tek baytlık BMP180 register yazmasını blok helper'a indirger. */
static te_Bmp180_RetCode Bmp180_prvUpdateRegisterIfNeeded(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t u8Mask, uint8_t u8Value); /* Register güvenliği helper'ıdır; maskeli alanlarda reserved bitleri koruyarak yazım yapar. */
static te_Bmp180_RetCode Bmp180_prvCheckChipId(ts_Bmp180_Handle *psHandle, uint8_t *pu8ChipId); /* Class-A health helper'ıdır; BMP180 CHIP_ID register'ını datasheet beklenen değeriyle doğrular. */
static int16_t Bmp180_prvParseBeS16(const uint8_t *pu8Data); /* Endian helper'ıdır; BMP180 MSB-first signed 16-bit katsayılarını taşınabilir biçimde çözer. */
static uint16_t Bmp180_prvParseBeU16(const uint8_t *pu8Data); /* Endian helper'ıdır; BMP180 MSB-first unsigned 16-bit katsayılarını taşınabilir biçimde çözer. */
static te_Bmp180_RetCode Bmp180_prvReadCalibrationCoeffs(ts_Bmp180_Handle *psHandle); /* Datasheet helper'ıdır; 22 baytlık fabrika kalibrasyon bloğunu okuyup handle içine parse eder. */
static bool Bmp180_prvValidateCalibrationCoeffs(const ts_Bmp180_CalibrationCoeffs *psCalib); /* Kalibrasyon helper'ıdır; all-zero/all-ones ve kritik sıfır katsayılarını ayıklar. */
static uint32_t Bmp180_prvGetPressureConversionTimeMs(te_Bmp180_Oversampling eOss); /* Datasheet timing helper'ıdır; OSS modunu gereken basınç dönüşüm beklemesine çevirir. */
static uint8_t Bmp180_prvBuildPressureConversionCommand(te_Bmp180_Oversampling eOss); /* Datasheet komut helper'ıdır; 0x34+(OSS<<6) basınç komut baytını üretir. */
static te_Bmp180_RetCode Bmp180_prvDelayMs(ts_Bmp180_Handle *psHandle, uint32_t u32DelayMs); /* Timing helper'ıdır; driver core'un HAL_Delay/osDelay bilmeden beklemesini sağlar. */
static te_Bmp180_RetCode Bmp180_prvReadRawTemperature(ts_Bmp180_Handle *psHandle, int32_t *ps32RawTemperature); /* Ölçüm helper'ıdır; sıcaklık dönüşümünü başlatıp ham UT değerini okur. */
static te_Bmp180_RetCode Bmp180_prvReadRawPressure(ts_Bmp180_Handle *psHandle, int32_t *ps32RawPressure); /* Ölçüm helper'ıdır; basınç dönüşümünü başlatıp OSS hizalı ham UP değerini okur. */
static te_Bmp180_RetCode Bmp180_prvComputeTrueValues(const ts_Bmp180_CalibrationCoeffs *psCalib,
                                                      int32_t s32RawTemperature,
                                                      int32_t s32RawPressure,
                                                      te_Bmp180_Oversampling eOss,
                                                      int32_t *ps32TemperatureDeciC,
                                                      int32_t *ps32PressurePa); /* Datasheet telafi helper'ıdır; BMP180 integer algoritmasıyla gerçek sıcaklık ve basıncı hesaplar. */
static float Bmp180_prvComputeAltitudeM(float f32PressurePa, float f32SeaLevelPressurePa); /* Uygulama-facing helper'dır; basınçtan deniz seviyesi referanslı altitude üretir. */
static void Bmp180_prvMemZero(void *vpData, uint32_t u32Len); /* Platform bağımsız bellek helper'ıdır; libc memset zorunluluğu olmadan struct temizler. */
static bool Bmp180_prvIsOversamplingValid(te_Bmp180_Oversampling eOss); /* Argüman doğrulama helper'ıdır; OSS enum değerini datasheet 0..3 aralığına sınar. */
static bool Bmp180_prvIsSeaLevelPressureValid(float f32PressurePa); /* Argüman doğrulama helper'ıdır; altitude referans basıncını makul fiziksel aralıkta tutar. */

static te_Bmp180_RetCode Bmp180_prvMarkError(ts_Bmp180_Handle *psHandle, te_Bmp180_RetCode eRet)
{
    if ((psHandle != NULL) && (eRet != BMP180_RET_OK))
    {
        psHandle->eState = BMP180_STATE_ERROR;              /* Hata durum makinesine yazılır; public API sonraki çağrılarda deterministik davranır. */
    }

    return eRet;                                             /* Asıl hata kodu korunur; caller gerçek kök nedeni kaybetmez. */
}

static te_Bmp180_RetCode Bmp180_prvLock(ts_Bmp180_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return BMP180_RET_NULL_PTR;                          /* Handle yoksa lock bağlamına erişmek güvenli değildir. */
    }

    if (psHandle->sLockInterface.pfnLock == NULL)
    {
        return BMP180_RET_OK;                                /* Bare-metal veya tek-sensör kullanımında lock opsiyoneldir. */
    }

    return psHandle->sLockInterface.pfnLock(psHandle->u32BusLockTimeoutMs,
                                            psHandle->sLockInterface.vpCtx); /* FreeRTOS/CMSIS mutex ayrıntısı port katmanına bırakılır. */
}

static te_Bmp180_RetCode Bmp180_prvUnlock(ts_Bmp180_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return BMP180_RET_NULL_PTR;                          /* Handle yoksa unlock callback bağlamı bilinemez. */
    }

    if (psHandle->sLockInterface.pfnUnlock == NULL)
    {
        return BMP180_RET_OK;                                /* Lock kullanılmayan sistemlerde unlock no-op kabul edilir. */
    }

    return psHandle->sLockInterface.pfnUnlock(psHandle->sLockInterface.vpCtx); /* Mutex sahipliği ve release kuralı port katmanında uygulanır. */
}

static uint32_t Bmp180_prvGetBusTimeout(const ts_Bmp180_Handle *psHandle)
{
    return (psHandle->u32BusTimeoutMs == 0U) ? BMP180_TIMEOUT_DEFAULT_MS : psHandle->u32BusTimeoutMs; /* 0 konfigürasyonu güvenli varsayılan timeout'a çevrilir. */
}

static te_Bmp180_RetCode Bmp180_prvReadBlock(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data, uint16_t u16Len)
{
    te_Bmp180_RetCode eRet;                                  /* Bus callback sonucunu saklar; unlock sonrası değerlendirilir. */
    te_Bmp180_RetCode eUnlockRet;                            /* Unlock hatasını ayrıca saklar; transfer başarısız olsa bile unlock denenir. */

    if ((psHandle == NULL) || (pu8Data == NULL) || (u16Len == 0U))
    {
        return BMP180_RET_INVALID_ARG;                       /* Blok okuma için handle, buffer ve uzunluk zorunludur. */
    }

    if (psHandle->sBusInterface.pfnRead == NULL)
    {
        return BMP180_RET_CONFIG_ERROR;                      /* Platform bağımsız driver bus okuma callback'i olmadan çalışamaz. */
    }

    eRet = Bmp180_prvLock(psHandle);                         /* Paylaşılan I2C bus, MPU6050 ve BMP180 task'ları çakışmasın diye transfer öncesi kilitlenir. */
    if (eRet != BMP180_RET_OK)
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
    if (eRet != BMP180_RET_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Bus hatası Class-A sensör erişimini bozduğu için state ERROR yapılır. */
    }

    return (eUnlockRet == BMP180_RET_OK) ? BMP180_RET_OK : Bmp180_prvMarkError(psHandle, eUnlockRet); /* Unlock hatası da kaynak yönetimi hatasıdır. */
}

static te_Bmp180_RetCode Bmp180_prvWriteBlock(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, const uint8_t *pu8Data, uint16_t u16Len)
{
    te_Bmp180_RetCode eRet;                                  /* Bus yazma sonucunu saklar. */
    te_Bmp180_RetCode eUnlockRet;                            /* Mutex release sonucunu saklar. */

    if ((psHandle == NULL) || (pu8Data == NULL) || (u16Len == 0U))
    {
        return BMP180_RET_INVALID_ARG;                       /* Yazma için kaynak buffer ve uzunluk geçerli olmalıdır. */
    }

    if (psHandle->sBusInterface.pfnWrite == NULL)
    {
        return BMP180_RET_CONFIG_ERROR;                      /* Platform portu yazma callback'i sağlamadıysa komut gönderilemez. */
    }

    eRet = Bmp180_prvLock(psHandle);                         /* Sadece gerçek I2C transferi sırasında kilit alınır; dönüşüm beklemesinde bus serbest kalır. */
    if (eRet != BMP180_RET_OK)
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
    if (eRet != BMP180_RET_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Bus yazma hatası ölçüm komutunun gönderilemediğini gösterir. */
    }

    return (eUnlockRet == BMP180_RET_OK) ? BMP180_RET_OK : Bmp180_prvMarkError(psHandle, eUnlockRet); /* Unlock başarısızsa driver güvenli hata durumuna geçer. */
}

static te_Bmp180_RetCode Bmp180_prvReadRegister(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data)
{
    return Bmp180_prvReadBlock(psHandle, u8Reg, pu8Data, 1U); /* Tek register okuma, blok okuma helper'ının 1 baytlık özel halidir. */
}

static te_Bmp180_RetCode Bmp180_prvWriteRegister(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t u8Data)
{
    return Bmp180_prvWriteBlock(psHandle, u8Reg, &u8Data, 1U); /* Tek register yazma, blok yazma helper'ının 1 baytlık özel halidir. */
}

static te_Bmp180_RetCode Bmp180_prvUpdateRegisterIfNeeded(ts_Bmp180_Handle *psHandle, uint8_t u8Reg, uint8_t u8Mask, uint8_t u8Value)
{
    uint8_t u8RegValue = 0U;                                 /* Register'ın mevcut değerini tutar; reserved bitler korunarak güncelleme yapılır. */
    uint8_t u8NewValue;                                      /* Maskelenmiş yeni register değerini tutar; gereksiz yazmayı engeller. */
    te_Bmp180_RetCode eRet;                                  /* Okuma/yazma helper sonucunu taşır. */

    if (u8Mask == 0xFFU)
    {
        return Bmp180_prvWriteRegister(psHandle, u8Reg, u8Value); /* Tam bayt debug/komut yazımlarında write-only register'lar okunamayabileceği için doğrudan yazılır. */
    }

    eRet = Bmp180_prvReadRegister(psHandle, u8Reg, &u8RegValue); /* RMW akışı önce register'ı okur; reserved bitleri korumak için bu zorunludur. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Okuma başarısızsa yazma denenmez; ilk hata korunur. */
    }

    u8NewValue = (uint8_t)((u8RegValue & (uint8_t)(~u8Mask)) | (u8Value & u8Mask)); /* Maskeli alan değiştirilir, maskenin dışındaki bitler aynen kalır. */
    if (u8NewValue == u8RegValue)
    {
        return BMP180_RET_OK;                                /* Değer zaten istenen haldeyse I2C yazması yapılmaz; bus yükü azaltılır. */
    }

    return Bmp180_prvWriteRegister(psHandle, u8Reg, u8NewValue); /* Gerçek değişiklik varsa güvenli tek register yazma helper'ı kullanılır. */
}

static te_Bmp180_RetCode Bmp180_prvCheckChipId(ts_Bmp180_Handle *psHandle, uint8_t *pu8ChipId)
{
    te_Bmp180_RetCode eRet;                                  /* CHIP_ID register okuma sonucunu taşır. */

    if ((psHandle == NULL) || (pu8ChipId == NULL))
    {
        return BMP180_RET_NULL_PTR;                          /* Health-check için handle ve çıktı pointer'ı zorunludur. */
    }

    eRet = Bmp180_prvReadRegister(psHandle, BMP180_REG_CHIP_ID, pu8ChipId); /* Class-A sensör kimliği datasheet CHIP_ID register'ından okunur. */
    psHandle->u8LastChipId = (eRet == BMP180_RET_OK) ? *pu8ChipId : 0U; /* Health alanı son okunan kimliği debug için saklar. */
    psHandle->bLastBusOk = (eRet == BMP180_RET_OK);           /* Health alanı bus erişiminin başarılı olup olmadığını saklar. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Bus okuma hatası chip-id karşılaştırmasından önce üst katmana döndürülür. */
    }

    if (*pu8ChipId != BMP180_CHIP_ID_EXPECTED)
    {
        return Bmp180_prvMarkError(psHandle, BMP180_RET_CHIP_ID_ERROR); /* Beklenmeyen kimlik yanlış sensör veya adres problemidir. */
    }

    return BMP180_RET_OK;                                    /* CHIP_ID beklenen 0x55 değeridir; sensör kimliği doğrulanmıştır. */
}

static int16_t Bmp180_prvParseBeS16(const uint8_t *pu8Data)
{
    return (int16_t)Bmp180_prvParseBeU16(pu8Data);           /* Önce unsigned big-endian birleştirilir, sonra signed katsayıya çevrilerek signed shift riski önlenir. */
}

static uint16_t Bmp180_prvParseBeU16(const uint8_t *pu8Data)
{
    return (uint16_t)(((uint16_t)pu8Data[0] << 8) | (uint16_t)pu8Data[1]); /* Unsigned katsayılar sistem endian'ına güvenmeden big-endian parse edilir. */
}

static te_Bmp180_RetCode Bmp180_prvReadCalibrationCoeffs(ts_Bmp180_Handle *psHandle)
{
    uint8_t au8Calib[BMP180_CALIB_DATA_LENGTH];              /* Datasheet AC1..MD bloğunu geçici olarak taşıyan ham byte buffer'ıdır. */
    te_Bmp180_RetCode eRet;                                  /* Blok okuma sonucunu taşır. */

    if (psHandle == NULL)
    {
        return BMP180_RET_NULL_PTR;                          /* Kalibrasyon handle içine yazılacağı için handle zorunludur. */
    }

    Bmp180_prvMemZero(au8Calib, (uint32_t)sizeof(au8Calib));  /* Ham buffer temizlenir; kısmi hata sonrası eski veri kalması önlenir. */
    eRet = Bmp180_prvReadBlock(psHandle, BMP180_REG_CALIB_START, au8Calib, BMP180_CALIB_DATA_LENGTH); /* Kalibrasyon register'ları datasheet sırasıyla tek blok okunur. */
    if (eRet != BMP180_RET_OK)
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
        return Bmp180_prvMarkError(psHandle, BMP180_RET_CALIBRATION_ERROR); /* Kalibrasyon mantıksızsa ölçüm hesabı güvenli değildir. */
    }

    return BMP180_RET_OK;                                    /* Kalibrasyon katsayıları handle içine güvenli şekilde yüklenmiştir. */
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

static te_Bmp180_RetCode Bmp180_prvDelayMs(ts_Bmp180_Handle *psHandle, uint32_t u32DelayMs)
{
    if ((psHandle == NULL) || (psHandle->sTimingInterface.pfnDelayMs == NULL))
    {
        return BMP180_RET_CONFIG_ERROR;                      /* BMP180 dönüşüm sonucu için bekleme zorunludur; timing callback olmadan güvenli ölçüm yoktur. */
    }

    return psHandle->sTimingInterface.pfnDelayMs(u32DelayMs, psHandle->sTimingInterface.vpCtx); /* HAL_Delay/osDelay seçimi port veya uygulama katmanına bırakılır. */
}

static te_Bmp180_RetCode Bmp180_prvReadRawTemperature(ts_Bmp180_Handle *psHandle, int32_t *ps32RawTemperature)
{
    uint8_t au8Raw[BMP180_RAW_TEMP_DATA_LENGTH];             /* OUT_MSB ve OUT_LSB sıcaklık baytlarını taşır. */
    te_Bmp180_RetCode eRet;                                  /* Komut, bekleme ve okuma adımlarının sonucunu taşır. */

    if ((psHandle == NULL) || (ps32RawTemperature == NULL))
    {
        return BMP180_RET_NULL_PTR;                          /* Ham sıcaklık çıkışı için geçerli pointer zorunludur. */
    }

    eRet = Bmp180_prvWriteRegister(psHandle, BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP); /* Datasheet sıcaklık dönüşümü CONTROL register'a 0x2E yazılarak başlatılır. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Komut yazılamadıysa dönüşüm beklemek anlamsızdır. */
    }

    eRet = Bmp180_prvDelayMs(psHandle, BMP180_TEMP_CONV_TIME_MS); /* Dönüşüm beklemesi sırasında I2C mutex tutulmaz; diğer sensörler bus'ı kullanabilir. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Timing callback başarısızsa ölçüm geçerli kabul edilmez. */
    }

    eRet = Bmp180_prvReadBlock(psHandle, BMP180_REG_OUT_MSB, au8Raw, BMP180_RAW_TEMP_DATA_LENGTH); /* Sıcaklık sonucu MSB-first iki bayt olarak okunur. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Output register okuması başarısızsa ham UT üretilmez. */
    }

    *ps32RawTemperature = (int32_t)Bmp180_prvParseBeU16(au8Raw); /* UT unsigned 16-bit sayı olarak datasheet algoritmasına verilir. */
    return BMP180_RET_OK;                                    /* Ham sıcaklık okuması tamamlanmıştır. */
}

static te_Bmp180_RetCode Bmp180_prvReadRawPressure(ts_Bmp180_Handle *psHandle, int32_t *ps32RawPressure)
{
    uint8_t au8Raw[BMP180_RAW_PRESS_DATA_LENGTH];            /* OUT_MSB, OUT_LSB ve OUT_XLSB basınç baytlarını taşır. */
    uint8_t u8Command;                                       /* OSS değerini içeren basınç dönüşüm komut baytıdır. */
    te_Bmp180_RetCode eRet;                                  /* Komut, bekleme ve okuma adımlarının sonucunu taşır. */

    if ((psHandle == NULL) || (ps32RawPressure == NULL))
    {
        return BMP180_RET_NULL_PTR;                          /* Ham basınç çıkışı için geçerli pointer zorunludur. */
    }

    u8Command = Bmp180_prvBuildPressureConversionCommand(psHandle->eOversampling); /* Seçili OSS, datasheet komut formatına dönüştürülür. */
    eRet = Bmp180_prvWriteRegister(psHandle, BMP180_REG_CONTROL, u8Command); /* Basınç dönüşümü CONTROL register'a 0x34+(OSS<<6) yazılarak başlatılır. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Komut gönderilemediyse basınç ölçümü başlatılamamıştır. */
    }

    eRet = Bmp180_prvDelayMs(psHandle, Bmp180_prvGetPressureConversionTimeMs(psHandle->eOversampling)); /* OSS'ye göre datasheet bekleme süresi uygulanır. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Bekleme başarısızsa output register'ların hazır olduğu varsayılmaz. */
    }

    eRet = Bmp180_prvReadBlock(psHandle, BMP180_REG_OUT_MSB, au8Raw, BMP180_RAW_PRESS_DATA_LENGTH); /* Basınç sonucu MSB-first üç bayt olarak okunur. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Bus okuma hatası ham UP üretimini engeller. */
    }

    *ps32RawPressure = (int32_t)((((uint32_t)au8Raw[0] << 16) |
                                  ((uint32_t)au8Raw[1] << 8) |
                                  (uint32_t)au8Raw[2]) >> (8U - (uint8_t)psHandle->eOversampling)); /* Datasheet UP hizalaması OSS'ye göre sağa kaydırılarak yapılır. */
    return BMP180_RET_OK;                                    /* Ham basınç okuması tamamlanmıştır. */
}

static te_Bmp180_RetCode Bmp180_prvComputeTrueValues(const ts_Bmp180_CalibrationCoeffs *psCalib,
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
        return BMP180_RET_NULL_PTR;                          /* Hesap fonksiyonu kalibrasyon ve çıktı pointer'ları olmadan çalışamaz. */
    }

    if (Bmp180_prvValidateCalibrationCoeffs(psCalib) == false)
    {
        return BMP180_RET_CALIBRATION_ERROR;                 /* Güvenilmeyen kalibrasyon katsayılarıyla datasheet telafisi yapılmaz. */
    }

    u8Oss = (uint8_t)eOss;                                   /* Oversampling enum'u datasheet denklemlerindeki shift değerine çevrilir. */
    s32X1 = (int32_t)(((int64_t)(s32RawTemperature - (int32_t)psCalib->u16AC6) * (int64_t)psCalib->u16AC5) >> 15); /* UT, AC5 ve AC6 ile sıcaklık X1 terimi hesaplanır. */
    s32Divisor = s32X1 + (int32_t)psCalib->s16MD;             /* Datasheet X2 paydasıdır; MD katsayısı sıfır olmasa bile toplam sıfır olabilir. */
    if (s32Divisor == 0)
    {
        return BMP180_RET_CALIBRATION_ERROR;                 /* Sıfıra bölme ölçüm algoritmasını geçersiz kılar. */
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
        return BMP180_RET_CALIBRATION_ERROR;                 /* B4 sıfırsa sonraki basınç bölmesi yapılamaz. */
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
    return BMP180_RET_OK;                                    /* Sıcaklık ve basınç telafisi başarıyla tamamlanmıştır. */
}

static float Bmp180_prvComputeAltitudeM(float f32PressurePa, float f32SeaLevelPressurePa)
{
    return BMP180_ALTITUDE_SCALE_M *
           (1.0f - powf((f32PressurePa / f32SeaLevelPressurePa), BMP180_ALTITUDE_EXPONENT)); /* Barometrik yükseklik formülü kullanılır; link aşamasında libm gerekebilir. */
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

te_Bmp180_RetCode Bmp180_Open(ts_Bmp180_Handle *psHandle, const ts_Bmp180_OpenConfig *psConfig)
{
    uint8_t u8ChipId = 0U;                                   /* Open sırasında okunan CHIP_ID değeridir. */
    te_Bmp180_RetCode eRet;                                  /* Başlatma adımlarının hata kodunu taşır. */

    if ((psHandle == NULL) || (psConfig == NULL))
    {
        return BMP180_RET_NULL_PTR;                          /* Open API sınırında handle ve config pointer'ları zorunludur. */
    }

    if ((psConfig->sBusInterface.pfnRead == NULL) ||
        (psConfig->sBusInterface.pfnWrite == NULL) ||
        (psConfig->sTimingInterface.pfnDelayMs == NULL))
    {
        return BMP180_RET_CONFIG_ERROR;                      /* BMP180 ölçümü için bus read/write ve dönüşüm bekleme callback'i zorunludur. */
    }

    if (Bmp180_prvIsOversamplingValid(psConfig->eOversampling) == false)
    {
        return BMP180_RET_INVALID_ARG;                       /* Geçersiz OSS değeri dönüşüm komutunu datasheet dışına çıkarır. */
    }

    if ((psConfig->f32SeaLevelPressurePa != 0.0f) &&
        (Bmp180_prvIsSeaLevelPressureValid(psConfig->f32SeaLevelPressurePa) == false))
    {
        return BMP180_RET_INVALID_ARG;                       /* 0 dışındaki deniz seviyesi referansı makul atmosfer aralığında olmalıdır. */
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

    eRet = Bmp180_prvDelayMs(psHandle, BMP180_POWER_ON_DELAY_MS); /* Başlangıçta kısa bekleme, güçlenme sonrası register cevaplarını kararlı hale getirir. */
    if (eRet != BMP180_RET_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Timing altyapısı çalışmıyorsa Open güvenli şekilde başarısız olur. */
    }

    eRet = Bmp180_prvCheckChipId(psHandle, &u8ChipId);       /* Class-A sensör kuralı gereği kimlik doğrulaması Open içinde yapılır. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Yanlış kimlik veya bus hatası Open'ı durdurur. */
    }

    eRet = Bmp180_prvReadCalibrationCoeffs(psHandle);        /* BMP180 telafi algoritması için fabrika kalibrasyonu Open sırasında yüklenir. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Kalibrasyon yoksa Read anlamlı SI çıktısı üretemez. */
    }

    psHandle->eState = BMP180_STATE_READY;                   /* Kimlik ve kalibrasyon doğrulandıktan sonra public Read çağrıları açılır. */
    return BMP180_RET_OK;                                    /* Instance başarıyla kullanıma hazırdır. */
}

te_Bmp180_RetCode Bmp180_Close(ts_Bmp180_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return BMP180_RET_NULL_PTR;                          /* Close için handle pointer'ı zorunludur. */
    }

    if (psHandle->eState == BMP180_STATE_UNINIT)
    {
        return BMP180_RET_INVALID_STATE;                     /* Zaten kapalı instance tekrar kapatılmaz; durum hatası açıkça bildirilir. */
    }

    psHandle->eState = BMP180_STATE_UNINIT;                  /* BMP180'de ayrı sleep register olmadığı için uygulama düzeyinde kapalı durum işaretlenir. */
    return BMP180_RET_OK;                                    /* Close, donanımda kalıcı bir konfigürasyon değişikliği yapmadan tamamlanır. */
}

te_Bmp180_RetCode Bmp180_Read(ts_Bmp180_Handle *psHandle, ts_Bmp180_Data *psOutData)
{
    int32_t s32RawTemperature = 0;                           /* Datasheet UT ham sıcaklık değeridir. */
    int32_t s32RawPressure = 0;                              /* Datasheet UP ham basınç değeridir. */
    int32_t s32TemperatureDeciC = 0;                         /* Datasheet gerçek sıcaklık çıktısıdır; 0.1 Celsius birimindedir. */
    int32_t s32PressurePa = 0;                               /* Telafi edilmiş basınç çıktısıdır; Pascal integer birimindedir. */
    te_Bmp180_RetCode eRet;                                  /* Her ölçüm adımının hata kodunu taşır. */

    if ((psHandle == NULL) || (psOutData == NULL))
    {
        return BMP180_RET_NULL_PTR;                          /* Read için handle ve çıktı buffer'ı zorunludur. */
    }

    if (psHandle->eState != BMP180_STATE_READY)
    {
        return BMP180_RET_INVALID_STATE;                     /* Open başarıyla tamamlanmadan veya ERROR durumundayken ölçüm yapılmaz. */
    }

    Bmp180_prvMemZero(psOutData, (uint32_t)sizeof(*psOutData)); /* Çıktı baştan temizlenir; hata dönüşünde eski valid veri yanlışlıkla kullanılmaz. */
    eRet = Bmp180_prvReadRawTemperature(psHandle, &s32RawTemperature); /* BMP180 sıcaklık dönüşümü basınç telafisi için önce yapılmalıdır. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Ham sıcaklık olmadan B5 hesaplanamaz. */
    }

    eRet = Bmp180_prvReadRawPressure(psHandle, &s32RawPressure); /* Seçili OSS ile basınç dönüşümü yapılır ve ham UP okunur. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Ham basınç okunamadıysa SI çıktısı üretilemez. */
    }

    eRet = Bmp180_prvComputeTrueValues(&psHandle->sCalibrationCoeffs,
                                       s32RawTemperature,
                                       s32RawPressure,
                                       psHandle->eOversampling,
                                       &s32TemperatureDeciC,
                                       &s32PressurePa);       /* Datasheet integer telafi algoritması platform bağımsız çekirdekte uygulanır. */
    if (eRet != BMP180_RET_OK)
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Kalibrasyon veya hesap hatası driver'ı güvenli ERROR durumuna alır. */
    }

    psOutData->f32TemperatureC = ((float)s32TemperatureDeciC) / 10.0f; /* 0.1 Celsius integer çıktısı uygulama için float Celsius'a çevrilir. */
    psOutData->f32PressurePa = (float)s32PressurePa;          /* Pascal integer çıktısı uygulama-facing SI float değerine çevrilir. */
    psOutData->f32AltitudeM = Bmp180_prvComputeAltitudeM(psOutData->f32PressurePa,
                                                         psHandle->f32SeaLevelPressurePa); /* Basınç ve referans basınçtan yaklaşık yükseklik hesaplanır. */
    psOutData->u32TimestampMs = (psHandle->sTimingInterface.pfnGetTickMs != NULL) ?
                                    psHandle->sTimingInterface.pfnGetTickMs(psHandle->sTimingInterface.vpCtx) :
                                    0U;                       /* Tick callback opsiyoneldir; yoksa timestamp 0 bırakılır. */
    psOutData->bValid = true;                                /* Bütün ölçüm ve telafi adımları başarıyla tamamlandığı için veri geçerli işaretlenir. */
    psOutData->s32RawTemperature = s32RawTemperature;         /* Debug/öğrenme amacıyla UT değeri saklanır; ana uygulama SI alanlarını kullanmalıdır. */
    psOutData->s32RawPressure = s32RawPressure;               /* Debug/öğrenme amacıyla UP değeri saklanır; kontrol algoritması doğrudan bunu kullanmamalıdır. */
    return BMP180_RET_OK;                                    /* Ölçüm başarıyla tamamlanmıştır. */
}

te_Bmp180_RetCode Bmp180_Write(ts_Bmp180_Handle *psHandle, const void *vpInData)
{
    (void)psHandle;                                          /* BMP180 generic data write modeli sunmadığı için handle kullanılmaz. */
    (void)vpInData;                                          /* Konfigürasyon ve register erişimi Ioctl üzerinden yürütüldüğü için input kullanılmaz. */
    return BMP180_RET_NOT_SUPPORTED;                         /* POSIX ilhamlı API tutarlılığı korunur ama anlamsız işlem sessizce başarılı sayılmaz. */
}

te_Bmp180_RetCode Bmp180_Ioctl(ts_Bmp180_Handle *psHandle, te_Bmp180_IoctlCmd eCmd, void *vpArg)
{
    te_Bmp180_RetCode eRet = BMP180_RET_OK;                  /* Switch içindeki komut sonucunu taşır. */

    if (psHandle == NULL)
    {
        return BMP180_RET_NULL_PTR;                          /* Ioctl her zaman geçerli instance handle ister. */
    }

    if ((psHandle->eState == BMP180_STATE_UNINIT) && (eCmd != BMP180_IOCTL_GET_STATE))
    {
        return BMP180_RET_INVALID_STATE;                     /* Open öncesi yalnızca durum sorgusu güvenli kabul edilir. */
    }

    switch (eCmd)
    {
    case BMP180_IOCTL_GET_VERSION:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* Versiyon döndürmek için uint16_t çıktı pointer'ı gerekir. */
            break;
        }
        *(uint16_t *)vpArg = BMP180_DRIVER_API_VERSION;      /* Uygulama derlenen driver API versiyonunu runtime'da okuyabilir. */
        break;

    case BMP180_IOCTL_GET_STATE:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* Durum döndürmek için te_Bmp180_State pointer'ı gerekir. */
            break;
        }
        *(te_Bmp180_State *)vpArg = psHandle->eState;         /* Handle içindeki durum makinesi uygulamaya kopyalanır. */
        break;

    case BMP180_IOCTL_CHECK_HEALTH:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* Health-check sonucu için çıktı struct pointer'ı gerekir. */
            break;
        }
        else
        {
            ts_Bmp180_HealthStatus *psHealth = (ts_Bmp180_HealthStatus *)vpArg; /* Void argüman beklenen health struct tipine çevrilir. */
            Bmp180_prvMemZero(psHealth, (uint32_t)sizeof(*psHealth)); /* Health sonucu temizlenir; kısmi hata alanları eski kalmaz. */
            psHealth->eState = psHandle->eState;             /* Driver state her durumda rapora yazılır. */
            eRet = Bmp180_prvReadRegister(psHandle, BMP180_REG_CHIP_ID, &psHealth->u8ChipIdValue); /* Bus sağlığı CHIP_ID register okumasıyla sınanır. */
            psHealth->bBusOk = (eRet == BMP180_RET_OK);      /* Okuma başarılıysa bus path çalışıyor kabul edilir. */
            psHealth->bChipIdOk = ((eRet == BMP180_RET_OK) && (psHealth->u8ChipIdValue == BMP180_CHIP_ID_EXPECTED)); /* Kimlik beklenen 0x55 ile karşılaştırılır. */
            psHealth->bCalibrationOk = Bmp180_prvValidateCalibrationCoeffs(&psHandle->sCalibrationCoeffs); /* Mevcut kalibrasyon katsayıları yeniden sanity-check edilir. */
            psHandle->u8LastChipId = psHealth->u8ChipIdValue; /* Son chip-id değeri handle sağlık alanına yazılır. */
            psHandle->bLastBusOk = psHealth->bBusOk;         /* Son bus sonucu handle içinde saklanır. */
            psHandle->bLastCalibrationOk = psHealth->bCalibrationOk; /* Son kalibrasyon sonucu handle içinde saklanır. */
            if ((eRet == BMP180_RET_OK) && (psHealth->bChipIdOk == false))
            {
                eRet = BMP180_RET_CHIP_ID_ERROR;             /* Bus çalışsa bile yanlış chip-id ayrı hata koduyla raporlanır. */
            }
            if ((eRet == BMP180_RET_OK) && (psHealth->bCalibrationOk == false))
            {
                eRet = BMP180_RET_CALIBRATION_ERROR;         /* Kimlik doğru olsa bile kalibrasyon geçersizse health başarısızdır. */
            }
        }
        break;

    case BMP180_IOCTL_SOFT_RESET:
        eRet = Bmp180_prvWriteRegister(psHandle, BMP180_REG_SOFT_RESET, BMP180_CMD_SOFT_RESET); /* Datasheet soft reset komutu 0xE0 register'ına 0xB6 yazılarak verilir. */
        if (eRet == BMP180_RET_OK)
        {
            eRet = Bmp180_prvDelayMs(psHandle, BMP180_SOFT_RESET_DELAY_MS); /* Reset sonrası kalibrasyon register'ları hazır olsun diye beklenir. */
        }
        if (eRet == BMP180_RET_OK)
        {
            uint8_t u8ChipId = 0U;                           /* Reset sonrası sensör kimliği tekrar okunur. */
            eRet = Bmp180_prvCheckChipId(psHandle, &u8ChipId); /* Resetin ardından aynı cihazın cevap verdiği doğrulanır. */
        }
        if (eRet == BMP180_RET_OK)
        {
            eRet = Bmp180_prvReadCalibrationCoeffs(psHandle); /* Reset sonrası kalibrasyon katsayıları yeniden yüklenir. */
        }
        if (eRet == BMP180_RET_OK)
        {
            psHandle->eState = BMP180_STATE_READY;           /* Reset toparlanması başarılıysa ERROR durumundan READY durumuna dönülebilir. */
        }
        break;

    case BMP180_IOCTL_SET_OVERSAMPLING:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* OSS ayarı için te_Bmp180_Oversampling pointer'ı gerekir. */
            break;
        }
        if (Bmp180_prvIsOversamplingValid(*(te_Bmp180_Oversampling *)vpArg) == false)
        {
            eRet = BMP180_RET_INVALID_ARG;                   /* Datasheet dışı OSS değeri komut baytını geçersiz yapar. */
            break;
        }
        psHandle->eOversampling = *(te_Bmp180_Oversampling *)vpArg; /* Yeni OSS handle içine yazılır; BMP180'de kalıcı config register yoktur. */
        break;

    case BMP180_IOCTL_GET_OVERSAMPLING:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* OSS okumak için te_Bmp180_Oversampling pointer'ı gerekir. */
            break;
        }
        *(te_Bmp180_Oversampling *)vpArg = psHandle->eOversampling; /* Mevcut instance OSS ayarı uygulamaya kopyalanır. */
        break;

    case BMP180_IOCTL_SET_SEA_LEVEL_PRESSURE:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* Referans basınç ayarı için float pointer'ı gerekir. */
            break;
        }
        if (Bmp180_prvIsSeaLevelPressureValid(*(float *)vpArg) == false)
        {
            eRet = BMP180_RET_INVALID_ARG;                   /* Altitude hesabında fiziksel olmayan referans basınç reddedilir. */
            break;
        }
        psHandle->f32SeaLevelPressurePa = *(float *)vpArg;   /* Uygulama yerel QNH/deniz seviyesi referansını güncelleyebilir. */
        break;

    case BMP180_IOCTL_GET_SEA_LEVEL_PRESSURE:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* Referans basıncı okumak için float pointer'ı gerekir. */
            break;
        }
        *(float *)vpArg = psHandle->f32SeaLevelPressurePa;    /* Mevcut altitude referansı uygulamaya kopyalanır. */
        break;

    case BMP180_IOCTL_GET_CALIBRATION:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* Kalibrasyon kopyalamak için struct pointer'ı gerekir. */
            break;
        }
        *(ts_Bmp180_CalibrationCoeffs *)vpArg = psHandle->sCalibrationCoeffs; /* Fabrika katsayıları debug/test için uygulamaya verilir. */
        break;

    case BMP180_IOCTL_REG_READ:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* Tek register okuma için access struct pointer'ı gerekir. */
            break;
        }
        eRet = Bmp180_prvReadRegister(psHandle,
                                      ((ts_Bmp180_RegisterAccess *)vpArg)->u8RegisterAddr,
                                      &((ts_Bmp180_RegisterAccess *)vpArg)->u8Value); /* Debug register okuma da lock'lu private helper üzerinden yapılır. */
        break;

    case BMP180_IOCTL_REG_WRITE:
        if (vpArg == NULL)
        {
            eRet = BMP180_RET_NULL_PTR;                      /* Tek register yazma için access struct pointer'ı gerekir. */
            break;
        }
        eRet = Bmp180_prvUpdateRegisterIfNeeded(psHandle,
                                                ((ts_Bmp180_RegisterAccess *)vpArg)->u8RegisterAddr,
                                                0xFFU,
                                                ((ts_Bmp180_RegisterAccess *)vpArg)->u8Value); /* Debug yazmada tüm byte hedeflenir ama helper gereksiz yazmayı önler. */
        break;

    case BMP180_IOCTL_REG_READ_BLOCK:
        if ((vpArg == NULL) ||
            (((ts_Bmp180_RegisterBlockAccess *)vpArg)->pu8Buffer == NULL) ||
            (((ts_Bmp180_RegisterBlockAccess *)vpArg)->u16Length == 0U))
        {
            eRet = BMP180_RET_INVALID_ARG;                   /* Blok okuma için struct, buffer ve uzunluk zorunludur. */
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
            eRet = BMP180_RET_INVALID_ARG;                   /* Blok yazma için struct, buffer ve uzunluk zorunludur. */
            break;
        }
        eRet = Bmp180_prvWriteBlock(psHandle,
                                    ((ts_Bmp180_RegisterBlockAccess *)vpArg)->u8RegisterAddr,
                                    ((ts_Bmp180_RegisterBlockAccess *)vpArg)->pu8Buffer,
                                    ((ts_Bmp180_RegisterBlockAccess *)vpArg)->u16Length); /* Blok register yazma port callback üzerinden yapılır. */
        break;

    default:
        eRet = BMP180_RET_NOT_SUPPORTED;                     /* Tanımlanmayan ioctl komutları sessiz başarıya çevrilmez. */
        break;
    }

    if ((eRet == BMP180_RET_BUS_ERROR) ||
        (eRet == BMP180_RET_TIMEOUT) ||
        (eRet == BMP180_RET_CHIP_ID_ERROR) ||
        (eRet == BMP180_RET_CALIBRATION_ERROR) ||
        (eRet == BMP180_RET_IO_ERROR))
    {
        return Bmp180_prvMarkError(psHandle, eRet);          /* Sağlık veya bus kökenli ciddi hatalar durum makinesini ERROR yapar. */
    }

    return eRet;                                             /* Parametre/unsupported gibi sözleşme hataları state'i bozmadan döner. */
}

te_Bmp180_RetCode Bmp180_Test(ts_Bmp180_Handle *psHandle, uint32_t u32TimeoutMs)
{
    ts_Bmp180_HealthStatus sHealth;                          /* Health-check çıktısını yerel olarak tutar. */
    ts_Bmp180_Data sData;                                    /* Örnek ölçüm çıktısını yerel olarak tutar. */
    te_Bmp180_RetCode eRet;                                  /* Test adımlarının sonucunu taşır. */

    if (psHandle == NULL)
    {
        return BMP180_RET_NULL_PTR;                          /* Test için geçerli handle zorunludur. */
    }

    eRet = Bmp180_Ioctl(psHandle, BMP180_IOCTL_CHECK_HEALTH, &sHealth); /* Önce bus, chip-id ve kalibrasyon sağlığı doğrulanır. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Health-check başarısızsa ölçüm testine geçilmez. */
    }

    eRet = Bmp180_Read(psHandle, &sData);                    /* En az bir gerçek sıcaklık/basınç ölçümü yapılır. */
    if (eRet != BMP180_RET_OK)
    {
        return eRet;                                         /* Read hatası doğrudan test hatasıdır. */
    }

    if (sData.bValid == false)
    {
        return Bmp180_prvMarkError(psHandle, BMP180_RET_IO_ERROR); /* Başarılı Read sonrası valid false ise API sözleşmesi bozulmuştur. */
    }

    if ((sData.f32TemperatureC < BMP180_TEMP_MIN_C) ||
        (sData.f32TemperatureC > BMP180_TEMP_MAX_C) ||
        (sData.f32PressurePa < BMP180_PRESSURE_MIN_PA) ||
        (sData.f32PressurePa > BMP180_PRESSURE_MAX_PA))
    {
        return Bmp180_prvMarkError(psHandle, BMP180_RET_IO_ERROR); /* Geniş fiziksel aralık dışı veri byte sırası veya kalibrasyon sorununa işaret eder. */
    }

    if ((u32TimeoutMs > 0U) && (psHandle->sTimingInterface.pfnDelayMs != NULL))
    {
        (void)psHandle->sTimingInterface.pfnDelayMs(u32TimeoutMs, psHandle->sTimingInterface.vpCtx); /* Test akışını gözlemlemek için opsiyonel bekleme uygulanır. */
    }

    return BMP180_RET_OK;                                    /* Health-check ve örnek ölçüm başarıyla tamamlanmıştır. */
}
