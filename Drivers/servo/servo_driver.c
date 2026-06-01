#include "servo_driver.h"

#ifndef NULL
#define NULL ((void *)0)
#endif

/* Servo test akışındaki komut sayısı; min/center/max/center sırası ile actuator yolunu doğrular. */
#define SERVO_TEST_STEP_COUNT                    (4U)

static te_Driver_RetCode Servo_prvMarkError(ts_Servo_Handle *psHandle, te_Driver_RetCode eRet);
/* Hata durumunda state makinesini ERROR'a taşır; çekirdek driver hata yönetimi yardımcısıdır. */

static void Servo_prvMemZero(void *vpData, uint32_t u32Len);
/* Belleği byte byte sıfırlar; dinamik bellek veya libc bağımlılığı olmadan handle temizliği sağlar. */

static float Servo_prvAbsF32(float f32Value);
/* Float mutlak değer hesabı yapar; math.h bağımlılığı olmadan konfigürasyon doğrulama katmanında kullanılır. */

static bool Servo_prvIsValidFloat(float f32Value);
/* NaN benzeri geçersiz float değerlerini eler; dış API'den gelen fiziksel komutları savunmacı şekilde doğrular. */

static bool Servo_prvIsNearZero(float f32Value);
/* Varsayılan konfigürasyon seçimini yaparken float sıfır karşılaştırmasını küçük eşikle güvenli hale getirir. */

static float Servo_prvClampF32(float f32Value, float f32Min, float f32Max);
/* Açı ve yüzde komutlarını güvenli aralığa kelepçeler; mekanik sınır korumasının çekirdek yardımcısıdır. */

static uint32_t Servo_prvClampU32(uint32_t u32Value, uint32_t u32Min, uint32_t u32Max);
/* Ham PWM darbe komutlarını güvenli mikro-saniye aralığına kelepçeler; dişli ve timer koruması sağlar. */

static te_Driver_RetCode Servo_prvValidateLimits(float f32MinAngleRad,
                                                  float f32MaxAngleRad,
                                                  uint32_t u32MinPulseUs,
                                                  uint32_t u32MaxPulseUs);
/* Açı ve pulse sınırlarını doğrular; Open ve Ioctl SET_LIMITS için ortak konfigürasyon kontrolüdür. */

static te_Driver_RetCode Servo_prvValidateSlewConfig(bool bEnableSlewRate,
                                                      float f32MaxSlewRateRadPerSec,
                                                      const ts_Servo_TimingInterface *psTimingInterface);
/* Slew-rate konfigürasyonunu doğrular; zaman callback'i olmadan hız sınırlamasının açılmasını engeller. */

static te_Driver_RetCode Servo_prvValidateOpenConfig(const ts_Servo_OpenConfig *psConfig,
                                                      float *pf32MinAngleRad,
                                                      float *pf32MaxAngleRad,
                                                      uint32_t *pu32MinPulseUs,
                                                      uint32_t *pu32MaxPulseUs);
/* Open konfigürasyonunu doğrular ve sıfır bırakılan SG90 varsayılanlarını somut değerlere çevirir. */

static te_Driver_RetCode Servo_prvLock(ts_Servo_Handle *psHandle);
/* Opsiyonel RTOS kilidini alır; çekirdek driver mutex tipini bilmeden kaynak koruması yapar. */

static te_Driver_RetCode Servo_prvUnlock(ts_Servo_Handle *psHandle);
/* Opsiyonel RTOS kilidini bırakır; hata yollarında bile port kaynağının kilitli kalmasını önler. */

static uint32_t Servo_prvGetTickMs(const ts_Servo_Handle *psHandle);
/* Enjekte edilen tick callback'ini çağırır; yoksa sıfır döndürerek zaman bağımlılığını opsiyonel bırakır. */

static bool Servo_prvHasTimedOut(const ts_Servo_Handle *psHandle, uint32_t u32StartTickMs, uint32_t u32TimeoutMs);
/* Test fonksiyonunda unsigned tick farkı ile zaman bütçesini kontrol eder; wrap-around davranışı güvenlidir. */

static uint32_t Servo_prvAngleToPulseUs(const ts_Servo_Handle *psHandle, float f32AngleRad);
/* Kelepçelenmiş fiziksel açıyı mikro-saniye PWM darbesine dönüştürür; sürücünün ana fiziksel eşleme fonksiyonudur. */

static float Servo_prvPulseToAngleRad(const ts_Servo_Handle *psHandle, uint32_t u32PulseUs);
/* Ham PWM override sonrası status alanını tutarlı yapmak için pulse değerini yaklaşık açıya geri dönüştürür. */

static float Servo_prvApplySlewRate(ts_Servo_Handle *psHandle, float f32TargetAngleRad, uint32_t u32NowTickMs);
/* Hedef açıyı maksimum açısal hıza göre sınırlar; kanatçıkların ani hareketini yazılımda yumuşatır. */

static te_Driver_RetCode Servo_prvWritePulse(ts_Servo_Handle *psHandle, uint32_t u32PulseUs);
/* Hesaplanan PWM darbesini lock/write/unlock sırası ile port callback'ine gönderir. */

static te_Driver_RetCode Servo_prvApplyAngleCommand(ts_Servo_Handle *psHandle, float f32TargetAngleRad);
/* Uygulama hedef açısını offset, saturasyon, slew-rate ve PWM yazma adımlarından geçirir. */

static void Servo_prvFillStatus(const ts_Servo_Handle *psHandle, ts_Servo_Status *psStatus);
/* Handle içindeki son servo durumunu uygulamaya okunabilir status yapısı olarak kopyalar. */

static void Servo_prvFillHealth(const ts_Servo_Handle *psHandle, ts_Servo_HealthStatus *psHealth);
/* Class-B health-check sonucunu doldurur; timer/PWM callback ve konfigürasyon sağlığını uygulamaya bildirir. */

static te_Driver_RetCode Servo_prvMarkError(ts_Servo_Handle *psHandle, te_Driver_RetCode eRet)
{
    if ((psHandle != NULL) && (eRet != DRIVER_OK))
    {
        psHandle->eState = SERVO_STATE_ERROR;
        psHandle->bLastWriteOk = false;
    }

    return eRet;
}

static void Servo_prvMemZero(void *vpData, uint32_t u32Len)
{
    uint8_t *pu8Data;
    uint32_t u32Idx;

    if (vpData == NULL)
    {
        return;
    }

    pu8Data = (uint8_t *)vpData;
    for (u32Idx = 0U; u32Idx < u32Len; ++u32Idx)
    {
        pu8Data[u32Idx] = 0U;
    }
}

static float Servo_prvAbsF32(float f32Value)
{
    return (f32Value < 0.0F) ? (-f32Value) : f32Value;
}

static bool Servo_prvIsValidFloat(float f32Value)
{
    return (f32Value == f32Value) ? true : false;
}

static bool Servo_prvIsNearZero(float f32Value)
{
    return (Servo_prvAbsF32(f32Value) <= SERVO_FLOAT_EPSILON) ? true : false;
}

static float Servo_prvClampF32(float f32Value, float f32Min, float f32Max)
{
    float f32ClampedValue;

    if (f32Value < f32Min)
    {
        f32ClampedValue = f32Min;
    }
    else if (f32Value > f32Max)
    {
        f32ClampedValue = f32Max;
    }
    else
    {
        f32ClampedValue = f32Value;
    }

    return f32ClampedValue;
}

static uint32_t Servo_prvClampU32(uint32_t u32Value, uint32_t u32Min, uint32_t u32Max)
{
    uint32_t u32ClampedValue;

    if (u32Value < u32Min)
    {
        u32ClampedValue = u32Min;
    }
    else if (u32Value > u32Max)
    {
        u32ClampedValue = u32Max;
    }
    else
    {
        u32ClampedValue = u32Value;
    }

    return u32ClampedValue;
}

static te_Driver_RetCode Servo_prvValidateLimits(float f32MinAngleRad,
                                                  float f32MaxAngleRad,
                                                  uint32_t u32MinPulseUs,
                                                  uint32_t u32MaxPulseUs)
{
    if ((Servo_prvIsValidFloat(f32MinAngleRad) == false) ||
        (Servo_prvIsValidFloat(f32MaxAngleRad) == false))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if (f32MaxAngleRad <= f32MinAngleRad)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if ((Servo_prvAbsF32(f32MinAngleRad) > SERVO_ABSOLUTE_MAX_ANGLE_RAD) ||
        (Servo_prvAbsF32(f32MaxAngleRad) > SERVO_ABSOLUTE_MAX_ANGLE_RAD))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if ((u32MinPulseUs < SERVO_SAFE_MIN_PULSE_US) ||
        (u32MaxPulseUs > SERVO_SAFE_MAX_PULSE_US) ||
        (u32MaxPulseUs <= u32MinPulseUs))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    return DRIVER_OK;
}

static te_Driver_RetCode Servo_prvValidateSlewConfig(bool bEnableSlewRate,
                                                      float f32MaxSlewRateRadPerSec,
                                                      const ts_Servo_TimingInterface *psTimingInterface)
{
    if (bEnableSlewRate == false)
    {
        return DRIVER_OK;
    }

    if (Servo_prvIsValidFloat(f32MaxSlewRateRadPerSec) == false)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if (f32MaxSlewRateRadPerSec <= SERVO_FLOAT_EPSILON)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if ((psTimingInterface == NULL) || (psTimingInterface->pfnGetTickMs == NULL))
    {
        return DRIVER_ERR_CONFIG;
    }

    return DRIVER_OK;
}

static te_Driver_RetCode Servo_prvValidateOpenConfig(const ts_Servo_OpenConfig *psConfig,
                                                      float *pf32MinAngleRad,
                                                      float *pf32MaxAngleRad,
                                                      uint32_t *pu32MinPulseUs,
                                                      uint32_t *pu32MaxPulseUs)
{
    te_Driver_RetCode eRet;

    if ((psConfig == NULL) ||
        (pf32MinAngleRad == NULL) ||
        (pf32MaxAngleRad == NULL) ||
        (pu32MinPulseUs == NULL) ||
        (pu32MaxPulseUs == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psConfig->sPulseInterface.pfnPulseWrite == NULL)
    {
        return DRIVER_ERR_CONFIG;
    }

    if (((psConfig->sLockInterface.pfnLock == NULL) && (psConfig->sLockInterface.pfnUnlock != NULL)) ||
        ((psConfig->sLockInterface.pfnLock != NULL) && (psConfig->sLockInterface.pfnUnlock == NULL)))
    {
        return DRIVER_ERR_CONFIG;
    }

    if ((Servo_prvIsNearZero(psConfig->f32MinAngleRad) == true) &&
        (Servo_prvIsNearZero(psConfig->f32MaxAngleRad) == true))
    {
        *pf32MinAngleRad = SERVO_SG90_MIN_ANGLE_RAD;
        *pf32MaxAngleRad = SERVO_SG90_MAX_ANGLE_RAD;
    }
    else
    {
        *pf32MinAngleRad = psConfig->f32MinAngleRad;
        *pf32MaxAngleRad = psConfig->f32MaxAngleRad;
    }

    if ((psConfig->u32MinPulseUs == 0U) && (psConfig->u32MaxPulseUs == 0U))
    {
        *pu32MinPulseUs = SERVO_SG90_MIN_PULSE_US;
        *pu32MaxPulseUs = SERVO_SG90_MAX_PULSE_US;
    }
    else
    {
        *pu32MinPulseUs = psConfig->u32MinPulseUs;
        *pu32MaxPulseUs = psConfig->u32MaxPulseUs;
    }

    eRet = Servo_prvValidateLimits(*pf32MinAngleRad,
                                   *pf32MaxAngleRad,
                                   *pu32MinPulseUs,
                                   *pu32MaxPulseUs);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    return Servo_prvValidateSlewConfig(psConfig->bEnableSlewRate,
                                       psConfig->f32MaxSlewRateRadPerSec,
                                       &psConfig->sTimingInterface);
}

static te_Driver_RetCode Servo_prvLock(ts_Servo_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->sLockInterface.pfnLock == NULL)
    {
        return DRIVER_OK;
    }

    return psHandle->sLockInterface.pfnLock(psHandle->u32LockTimeoutMs, psHandle->sLockInterface.vpCtx);
}

static te_Driver_RetCode Servo_prvUnlock(ts_Servo_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->sLockInterface.pfnUnlock == NULL)
    {
        return DRIVER_OK;
    }

    return psHandle->sLockInterface.pfnUnlock(psHandle->sLockInterface.vpCtx);
}

static uint32_t Servo_prvGetTickMs(const ts_Servo_Handle *psHandle)
{
    if ((psHandle == NULL) || (psHandle->sTimingInterface.pfnGetTickMs == NULL))
    {
        return 0U;
    }

    return psHandle->sTimingInterface.pfnGetTickMs(psHandle->sTimingInterface.vpCtx);
}

static bool Servo_prvHasTimedOut(const ts_Servo_Handle *psHandle, uint32_t u32StartTickMs, uint32_t u32TimeoutMs)
{
    uint32_t u32NowTickMs;

    if ((psHandle == NULL) || (psHandle->sTimingInterface.pfnGetTickMs == NULL))
    {
        return false;
    }

    u32NowTickMs = Servo_prvGetTickMs(psHandle);
    return (((uint32_t)(u32NowTickMs - u32StartTickMs)) > u32TimeoutMs) ? true : false;
}

static uint32_t Servo_prvAngleToPulseUs(const ts_Servo_Handle *psHandle, float f32AngleRad)
{
    float f32Ratio;
    float f32PulseUs;
    uint32_t u32PulseUs;

    f32Ratio = (f32AngleRad - psHandle->f32MinAngleRad) /
               (psHandle->f32MaxAngleRad - psHandle->f32MinAngleRad);
    f32Ratio = Servo_prvClampF32(f32Ratio, 0.0F, 1.0F);

    f32PulseUs = (float)psHandle->u32MinPulseUs +
                 (f32Ratio * (float)(psHandle->u32MaxPulseUs - psHandle->u32MinPulseUs));

    if (f32PulseUs <= 0.0F)
    {
        u32PulseUs = psHandle->u32MinPulseUs;
    }
    else
    {
        u32PulseUs = (uint32_t)(f32PulseUs + 0.5F);
    }

    return Servo_prvClampU32(u32PulseUs, psHandle->u32MinPulseUs, psHandle->u32MaxPulseUs);
}

static float Servo_prvPulseToAngleRad(const ts_Servo_Handle *psHandle, uint32_t u32PulseUs)
{
    float f32Ratio;
    uint32_t u32SafePulseUs;

    u32SafePulseUs = Servo_prvClampU32(u32PulseUs, psHandle->u32MinPulseUs, psHandle->u32MaxPulseUs);
    f32Ratio = ((float)(u32SafePulseUs - psHandle->u32MinPulseUs)) /
               ((float)(psHandle->u32MaxPulseUs - psHandle->u32MinPulseUs));

    return psHandle->f32MinAngleRad +
           (f32Ratio * (psHandle->f32MaxAngleRad - psHandle->f32MinAngleRad));
}

static float Servo_prvApplySlewRate(ts_Servo_Handle *psHandle, float f32TargetAngleRad, uint32_t u32NowTickMs)
{
    uint32_t u32DeltaMs;
    float f32MaxDeltaRad;
    float f32DeltaRad;
    float f32LimitedAngleRad;

    if ((psHandle->bEnableSlewRate == false) || (psHandle->bHasLastWriteTick == false))
    {
        return f32TargetAngleRad;
    }

    u32DeltaMs = (uint32_t)(u32NowTickMs - psHandle->u32LastWriteTickMs);
    f32MaxDeltaRad = psHandle->f32MaxSlewRateRadPerSec * ((float)u32DeltaMs / 1000.0F);
    f32DeltaRad = f32TargetAngleRad - psHandle->f32CurrentAngleRad;

    if (f32MaxDeltaRad <= SERVO_FLOAT_EPSILON)
    {
        f32LimitedAngleRad = psHandle->f32CurrentAngleRad;
    }
    else if (f32DeltaRad > f32MaxDeltaRad)
    {
        f32LimitedAngleRad = psHandle->f32CurrentAngleRad + f32MaxDeltaRad;
    }
    else if (f32DeltaRad < (-f32MaxDeltaRad))
    {
        f32LimitedAngleRad = psHandle->f32CurrentAngleRad - f32MaxDeltaRad;
    }
    else
    {
        f32LimitedAngleRad = f32TargetAngleRad;
    }

    return Servo_prvClampF32(f32LimitedAngleRad, psHandle->f32MinAngleRad, psHandle->f32MaxAngleRad);
}

static te_Driver_RetCode Servo_prvWritePulse(ts_Servo_Handle *psHandle, uint32_t u32PulseUs)
{
    te_Driver_RetCode eRet;
    te_Driver_RetCode eUnlockRet;

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->sPulseInterface.pfnPulseWrite == NULL)
    {
        return Servo_prvMarkError(psHandle, DRIVER_ERR_CONFIG);
    }

    eRet = Servo_prvLock(psHandle);
    if (eRet != DRIVER_OK)
    {
        return Servo_prvMarkError(psHandle, eRet);
    }

    psHandle->eState = SERVO_STATE_BUSY;
    eRet = psHandle->sPulseInterface.pfnPulseWrite(u32PulseUs, psHandle->sPulseInterface.vpCtx);
    eUnlockRet = Servo_prvUnlock(psHandle);

    if ((eRet == DRIVER_OK) && (eUnlockRet != DRIVER_OK))
    {
        eRet = eUnlockRet;
    }

    if (eRet == DRIVER_OK)
    {
        psHandle->eState = SERVO_STATE_READY;
        psHandle->bLastWriteOk = true;
        return DRIVER_OK;
    }

    return Servo_prvMarkError(psHandle, eRet);
}

static te_Driver_RetCode Servo_prvApplyAngleCommand(ts_Servo_Handle *psHandle, float f32TargetAngleRad)
{
    float f32PhysicalAngleRad;
    float f32LimitedAngleRad;
    uint32_t u32NowTickMs;
    uint32_t u32PulseUs;
    te_Driver_RetCode eRet;

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (Servo_prvIsValidFloat(f32TargetAngleRad) == false)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    u32NowTickMs = Servo_prvGetTickMs(psHandle);
    psHandle->f32TargetAngleRad = f32TargetAngleRad;

    f32PhysicalAngleRad = f32TargetAngleRad + psHandle->f32AngleOffsetRad;
    f32PhysicalAngleRad = Servo_prvClampF32(f32PhysicalAngleRad, psHandle->f32MinAngleRad, psHandle->f32MaxAngleRad);
    f32LimitedAngleRad = Servo_prvApplySlewRate(psHandle, f32PhysicalAngleRad, u32NowTickMs);
    u32PulseUs = Servo_prvAngleToPulseUs(psHandle, f32LimitedAngleRad);

    eRet = Servo_prvWritePulse(psHandle, u32PulseUs);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    psHandle->f32CurrentAngleRad = f32LimitedAngleRad;
    psHandle->u32CurrentPulseUs = u32PulseUs;
    psHandle->u32LastWriteTickMs = u32NowTickMs;
    psHandle->bHasLastWriteTick = true;

    return DRIVER_OK;
}

static void Servo_prvFillStatus(const ts_Servo_Handle *psHandle, ts_Servo_Status *psStatus)
{
    if ((psHandle == NULL) || (psStatus == NULL))
    {
        return;
    }

    psStatus->bPeripheralConfigured = ((psHandle->sPulseInterface.pfnPulseWrite != NULL) &&
                                       (psHandle->u32MaxPulseUs > psHandle->u32MinPulseUs)) ? true : false;
    psStatus->bLastWriteOk = psHandle->bLastWriteOk;
    psStatus->eState = psHandle->eState;
    psStatus->f32CurrentAngleRad = psHandle->f32CurrentAngleRad;
    psStatus->f32TargetAngleRad = psHandle->f32TargetAngleRad;
    psStatus->u32CurrentPulseUs = psHandle->u32CurrentPulseUs;
    psStatus->u32LastWriteTickMs = psHandle->u32LastWriteTickMs;
}

static void Servo_prvFillHealth(const ts_Servo_Handle *psHandle, ts_Servo_HealthStatus *psHealth)
{
    te_Driver_RetCode eLimitRet;
    te_Driver_RetCode eSlewRet;

    if ((psHandle == NULL) || (psHealth == NULL))
    {
        return;
    }

    eLimitRet = Servo_prvValidateLimits(psHandle->f32MinAngleRad,
                                        psHandle->f32MaxAngleRad,
                                        psHandle->u32MinPulseUs,
                                        psHandle->u32MaxPulseUs);
    eSlewRet = Servo_prvValidateSlewConfig(psHandle->bEnableSlewRate,
                                           psHandle->f32MaxSlewRateRadPerSec,
                                           &psHandle->sTimingInterface);

    psHealth->bPulseCallbackOk = (psHandle->sPulseInterface.pfnPulseWrite != NULL) ? true : false;
    psHealth->bLimitConfigOk = (eLimitRet == DRIVER_OK) ? true : false;
    psHealth->bSlewConfigOk = (eSlewRet == DRIVER_OK) ? true : false;
    psHealth->bLastWriteOk = psHandle->bLastWriteOk;
    psHealth->eState = psHandle->eState;
}

te_Driver_RetCode Servo_Open(ts_Servo_Handle *psHandle, const ts_Servo_OpenConfig *psConfig)
{
    float f32MinAngleRad;
    float f32MaxAngleRad;
    uint32_t u32MinPulseUs;
    uint32_t u32MaxPulseUs;
    float f32CenterAngleRad;
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (psConfig == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    eRet = Servo_prvValidateOpenConfig(psConfig,
                                       &f32MinAngleRad,
                                       &f32MaxAngleRad,
                                       &u32MinPulseUs,
                                       &u32MaxPulseUs);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    if (Servo_prvIsValidFloat(psConfig->f32AngleOffsetRad) == false)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    Servo_prvMemZero(psHandle, (uint32_t)sizeof(*psHandle));
    psHandle->eState = SERVO_STATE_UNINIT;
    psHandle->f32MinAngleRad = f32MinAngleRad;
    psHandle->f32MaxAngleRad = f32MaxAngleRad;
    psHandle->u32MinPulseUs = u32MinPulseUs;
    psHandle->u32MaxPulseUs = u32MaxPulseUs;
    psHandle->f32AngleOffsetRad = psConfig->f32AngleOffsetRad;
    psHandle->u32LockTimeoutMs = (psConfig->u32LockTimeoutMs == 0U) ?
                                 SERVO_DEFAULT_LOCK_TIMEOUT_MS :
                                 psConfig->u32LockTimeoutMs;
    psHandle->bEnableSlewRate = psConfig->bEnableSlewRate;
    psHandle->f32MaxSlewRateRadPerSec = psConfig->f32MaxSlewRateRadPerSec;
    psHandle->sPulseInterface = psConfig->sPulseInterface;
    psHandle->sLockInterface = psConfig->sLockInterface;
    psHandle->sTimingInterface = psConfig->sTimingInterface;

    f32CenterAngleRad = (psHandle->f32MinAngleRad + psHandle->f32MaxAngleRad) * 0.5F;
    psHandle->f32CurrentAngleRad = Servo_prvClampF32(f32CenterAngleRad,
                                                     psHandle->f32MinAngleRad,
                                                     psHandle->f32MaxAngleRad);
    psHandle->f32TargetAngleRad = psHandle->f32CurrentAngleRad - psHandle->f32AngleOffsetRad;
    psHandle->u32CurrentPulseUs = Servo_prvAngleToPulseUs(psHandle, psHandle->f32CurrentAngleRad);
    psHandle->u32LastWriteTickMs = 0U;
    psHandle->bHasLastWriteTick = false;
    psHandle->bLastWriteOk = false;
    psHandle->eState = SERVO_STATE_READY;

    return DRIVER_OK;
}

te_Driver_RetCode Servo_Close(ts_Servo_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->eState == SERVO_STATE_UNINIT)
    {
        return DRIVER_ERR_STATE;
    }

    psHandle->eState = SERVO_STATE_UNINIT;
    psHandle->bLastWriteOk = false;

    return DRIVER_OK;
}

te_Driver_RetCode Servo_Read(ts_Servo_Handle *psHandle, ts_Servo_Status *psOutStatus)
{
    if ((psHandle == NULL) || (psOutStatus == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->eState == SERVO_STATE_UNINIT)
    {
        return DRIVER_ERR_STATE;
    }

    Servo_prvMemZero(psOutStatus, (uint32_t)sizeof(*psOutStatus));
    Servo_prvFillStatus(psHandle, psOutStatus);

    return DRIVER_OK;
}

te_Driver_RetCode Servo_Write(ts_Servo_Handle *psHandle, const void *vpInData)
{
    const float *pf32TargetAngleRad;

    if ((psHandle == NULL) || (vpInData == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->eState != SERVO_STATE_READY)
    {
        return DRIVER_ERR_STATE;
    }

    pf32TargetAngleRad = (const float *)vpInData;
    return Servo_prvApplyAngleCommand(psHandle, *pf32TargetAngleRad);
}

te_Driver_RetCode Servo_Ioctl(ts_Servo_Handle *psHandle, te_Servo_IoctlCmd eCmd, void *vpArg)
{
    te_Driver_RetCode eRet;

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if ((psHandle->eState == SERVO_STATE_UNINIT) &&
        (eCmd != SERVO_IOCTL_GET_STATE) &&
        (eCmd != SERVO_IOCTL_GET_VERSION))
    {
        return DRIVER_ERR_STATE;
    }

    eRet = DRIVER_OK;

    switch (eCmd)
    {
    case SERVO_IOCTL_GET_VERSION:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        *(uint16_t *)vpArg = SERVO_DRIVER_API_VERSION;
        break;

    case SERVO_IOCTL_GET_STATE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        *(te_Servo_State *)vpArg = psHandle->eState;
        break;

    case SERVO_IOCTL_GET_STATUS:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        Servo_prvMemZero(vpArg, (uint32_t)sizeof(ts_Servo_Status));
        Servo_prvFillStatus(psHandle, (ts_Servo_Status *)vpArg);
        if (((ts_Servo_Status *)vpArg)->bPeripheralConfigured == false)
        {
            eRet = DRIVER_ERR_CONFIG;
        }
        break;

    case SERVO_IOCTL_CHECK_HEALTH:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        Servo_prvMemZero(vpArg, (uint32_t)sizeof(ts_Servo_HealthStatus));
        Servo_prvFillHealth(psHandle, (ts_Servo_HealthStatus *)vpArg);
        if ((((ts_Servo_HealthStatus *)vpArg)->bPulseCallbackOk == false) ||
            (((ts_Servo_HealthStatus *)vpArg)->bLimitConfigOk == false) ||
            (((ts_Servo_HealthStatus *)vpArg)->bSlewConfigOk == false))
        {
            eRet = DRIVER_ERR_CONFIG;
        }
        break;

    case SERVO_IOCTL_SET_LIMITS:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        else
        {
            const ts_Servo_Limits *psLimits = (const ts_Servo_Limits *)vpArg;
            eRet = Servo_prvValidateLimits(psLimits->f32MinAngleRad,
                                           psLimits->f32MaxAngleRad,
                                           psLimits->u32MinPulseUs,
                                           psLimits->u32MaxPulseUs);
            if (eRet == DRIVER_OK)
            {
                psHandle->f32MinAngleRad = psLimits->f32MinAngleRad;
                psHandle->f32MaxAngleRad = psLimits->f32MaxAngleRad;
                psHandle->u32MinPulseUs = psLimits->u32MinPulseUs;
                psHandle->u32MaxPulseUs = psLimits->u32MaxPulseUs;
                psHandle->f32CurrentAngleRad = Servo_prvClampF32(psHandle->f32CurrentAngleRad,
                                                                 psHandle->f32MinAngleRad,
                                                                 psHandle->f32MaxAngleRad);
                psHandle->u32CurrentPulseUs = Servo_prvAngleToPulseUs(psHandle, psHandle->f32CurrentAngleRad);
            }
        }
        break;

    case SERVO_IOCTL_GET_LIMITS:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        else
        {
            ts_Servo_Limits *psLimits = (ts_Servo_Limits *)vpArg;
            psLimits->f32MinAngleRad = psHandle->f32MinAngleRad;
            psLimits->f32MaxAngleRad = psHandle->f32MaxAngleRad;
            psLimits->u32MinPulseUs = psHandle->u32MinPulseUs;
            psLimits->u32MaxPulseUs = psHandle->u32MaxPulseUs;
        }
        break;

    case SERVO_IOCTL_SET_SLEW_RATE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        else
        {
            const ts_Servo_SlewRateConfig *psSlew = (const ts_Servo_SlewRateConfig *)vpArg;
            eRet = Servo_prvValidateSlewConfig(psSlew->bEnable,
                                               psSlew->f32MaxSlewRateRadPerSec,
                                               &psHandle->sTimingInterface);
            if (eRet == DRIVER_OK)
            {
                psHandle->bEnableSlewRate = psSlew->bEnable;
                psHandle->f32MaxSlewRateRadPerSec = psSlew->f32MaxSlewRateRadPerSec;
                psHandle->u32LastWriteTickMs = Servo_prvGetTickMs(psHandle);
                psHandle->bHasLastWriteTick = false;
            }
        }
        break;

    case SERVO_IOCTL_GET_SLEW_RATE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        else
        {
            ts_Servo_SlewRateConfig *psSlew = (ts_Servo_SlewRateConfig *)vpArg;
            psSlew->bEnable = psHandle->bEnableSlewRate;
            psSlew->f32MaxSlewRateRadPerSec = psHandle->f32MaxSlewRateRadPerSec;
        }
        break;

    case SERVO_IOCTL_OVERRIDE_RAW_PULSE:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        else
        {
            const ts_Servo_RawPulseCommand *psRawPulse = (const ts_Servo_RawPulseCommand *)vpArg;
            uint32_t u32SafePulseUs = Servo_prvClampU32(psRawPulse->u32PulseWidthUs,
                                                        psHandle->u32MinPulseUs,
                                                        psHandle->u32MaxPulseUs);
            eRet = Servo_prvWritePulse(psHandle, u32SafePulseUs);
            if (eRet == DRIVER_OK)
            {
                psHandle->u32CurrentPulseUs = u32SafePulseUs;
                psHandle->f32CurrentAngleRad = Servo_prvPulseToAngleRad(psHandle, u32SafePulseUs);
                psHandle->f32TargetAngleRad = psHandle->f32CurrentAngleRad - psHandle->f32AngleOffsetRad;
                psHandle->u32LastWriteTickMs = Servo_prvGetTickMs(psHandle);
                psHandle->bHasLastWriteTick = true;
            }
        }
        break;

    case SERVO_IOCTL_WRITE_PERCENT:
        if (vpArg == NULL)
        {
            eRet = DRIVER_ERR_NULL_PTR;
            break;
        }
        else
        {
            const ts_Servo_PercentCommand *psPercentCommand = (const ts_Servo_PercentCommand *)vpArg;
            float f32Percent;
            float f32Ratio;
            float f32PhysicalAngleRad;
            float f32CommandAngleRad;

            if (Servo_prvIsValidFloat(psPercentCommand->f32Percent) == false)
            {
                eRet = DRIVER_ERR_INVALID_ARG;
                break;
            }

            f32Percent = Servo_prvClampF32(psPercentCommand->f32Percent,
                                           SERVO_PERCENT_MIN,
                                           SERVO_PERCENT_MAX);
            f32Ratio = (f32Percent - SERVO_PERCENT_MIN) / (SERVO_PERCENT_MAX - SERVO_PERCENT_MIN);
            f32PhysicalAngleRad = psHandle->f32MinAngleRad +
                                  (f32Ratio * (psHandle->f32MaxAngleRad - psHandle->f32MinAngleRad));
            f32CommandAngleRad = f32PhysicalAngleRad - psHandle->f32AngleOffsetRad;
            eRet = Servo_prvApplyAngleCommand(psHandle, f32CommandAngleRad);
        }
        break;

    default:
        eRet = DRIVER_ERR_INVALID_ARG;
        break;
    }

    return eRet;
}

te_Driver_RetCode Servo_Test(ts_Servo_Handle *psHandle, uint32_t u32TimeoutMs)
{
    float af32PhysicalAngles[SERVO_TEST_STEP_COUNT];
    float f32CommandAngleRad;
    uint32_t u32StartTickMs;
    uint32_t u32EffectiveTimeoutMs;
    uint32_t u32Idx;
    te_Driver_RetCode eRet;

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->eState != SERVO_STATE_READY)
    {
        return DRIVER_ERR_STATE;
    }

    u32EffectiveTimeoutMs = (u32TimeoutMs == 0U) ? SERVO_DEFAULT_TEST_TIMEOUT_MS : u32TimeoutMs;
    u32StartTickMs = Servo_prvGetTickMs(psHandle);

    af32PhysicalAngles[0] = psHandle->f32MinAngleRad;
    af32PhysicalAngles[1] = (psHandle->f32MinAngleRad + psHandle->f32MaxAngleRad) * 0.5F;
    af32PhysicalAngles[2] = psHandle->f32MaxAngleRad;
    af32PhysicalAngles[3] = af32PhysicalAngles[1];

    for (u32Idx = 0U; u32Idx < SERVO_TEST_STEP_COUNT; ++u32Idx)
    {
        if (Servo_prvHasTimedOut(psHandle, u32StartTickMs, u32EffectiveTimeoutMs) == true)
        {
            return Servo_prvMarkError(psHandle, DRIVER_ERR_TIMEOUT);
        }

        f32CommandAngleRad = af32PhysicalAngles[u32Idx] - psHandle->f32AngleOffsetRad;
        eRet = Servo_prvApplyAngleCommand(psHandle, f32CommandAngleRad);
        if (eRet != DRIVER_OK)
        {
            return eRet;
        }
    }

    return DRIVER_OK;
}
