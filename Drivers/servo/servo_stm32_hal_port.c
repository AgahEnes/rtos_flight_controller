#include "servo_stm32_hal_port.h"

static bool Servo_Stm32Hal_prvIsApb2Timer(const TIM_TypeDef *pxTimInstance);
/* Timer instance'ının APB2 domaininde olup olmadığını belirler; STM32 timer clock çarpanı hesabı için port katmanı yardımcısıdır. */

static uint32_t Servo_Stm32Hal_prvGetTimerKernelClockHz(const ts_Servo_Stm32PortContext *psContext);
/* Timer kernel clock değerini HAL RCC bilgisinden veya context override alanından hesaplar. */

static uint32_t Servo_Stm32Hal_prvGetCounterClockHz(const ts_Servo_Stm32PortContext *psContext);
/* Timer prescaler değerini hesaba katarak counter'ın saniyedeki tick sayısını üretir. */

static bool Servo_Stm32Hal_prvIsApb2Timer(const TIM_TypeDef *pxTimInstance)
{
    bool bIsApb2Timer;

    bIsApb2Timer = false;

#if defined(TIM1)
    if (pxTimInstance == TIM1)
    {
        bIsApb2Timer = true;
    }
#endif

#if defined(TIM8)
    if (pxTimInstance == TIM8)
    {
        bIsApb2Timer = true;
    }
#endif

#if defined(TIM9)
    if (pxTimInstance == TIM9)
    {
        bIsApb2Timer = true;
    }
#endif

#if defined(TIM10)
    if (pxTimInstance == TIM10)
    {
        bIsApb2Timer = true;
    }
#endif

#if defined(TIM11)
    if (pxTimInstance == TIM11)
    {
        bIsApb2Timer = true;
    }
#endif

    return bIsApb2Timer;
}

static uint32_t Servo_Stm32Hal_prvGetTimerKernelClockHz(const ts_Servo_Stm32PortContext *psContext)
{
    RCC_ClkInitTypeDef sClockConfig;
    uint32_t u32FlashLatency;
    uint32_t u32PclkHz;
    uint32_t u32TimerClockHz;

    if ((psContext == NULL) || (psContext->pxTimHandle == NULL) || (psContext->pxTimHandle->Instance == NULL))
    {
        return 0U;
    }

    if (psContext->u32TimerClockHz != 0U)
    {
        return psContext->u32TimerClockHz;
    }

    u32FlashLatency = 0U;
    u32TimerClockHz = 0U;
    HAL_RCC_GetClockConfig(&sClockConfig, &u32FlashLatency);

    if (Servo_Stm32Hal_prvIsApb2Timer(psContext->pxTimHandle->Instance) == true)
    {
        u32PclkHz = HAL_RCC_GetPCLK2Freq();
        if (sClockConfig.APB2CLKDivider == RCC_HCLK_DIV1)
        {
            u32TimerClockHz = u32PclkHz;
        }
        else
        {
            u32TimerClockHz = u32PclkHz * 2U;
        }
    }
    else
    {
        u32PclkHz = HAL_RCC_GetPCLK1Freq();
        if (sClockConfig.APB1CLKDivider == RCC_HCLK_DIV1)
        {
            u32TimerClockHz = u32PclkHz;
        }
        else
        {
            u32TimerClockHz = u32PclkHz * 2U;
        }
    }

    return u32TimerClockHz;
}

static uint32_t Servo_Stm32Hal_prvGetCounterClockHz(const ts_Servo_Stm32PortContext *psContext)
{
    uint32_t u32TimerClockHz;
    uint32_t u32PrescalerPlusOne;

    if ((psContext == NULL) || (psContext->pxTimHandle == NULL) || (psContext->pxTimHandle->Instance == NULL))
    {
        return 0U;
    }

    u32TimerClockHz = Servo_Stm32Hal_prvGetTimerKernelClockHz(psContext);
    if (u32TimerClockHz == 0U)
    {
        return 0U;
    }

    u32PrescalerPlusOne = (uint32_t)(psContext->pxTimHandle->Instance->PSC + 1U);
    if (u32PrescalerPlusOne == 0U)
    {
        return 0U;
    }

    return u32TimerClockHz / u32PrescalerPlusOne;
}

te_Driver_RetCode Servo_Stm32Hal_PulseWrite(uint32_t u32PulseWidthUs, void *vpCtx)
{
    ts_Servo_Stm32PortContext *psContext;
    uint32_t u32CounterClockHz;
    uint32_t u32AutoReload;
    uint32_t u32CompareValue;
    uint64_t u64CompareValue;

    psContext = (ts_Servo_Stm32PortContext *)vpCtx;
    if ((psContext == NULL) || (psContext->pxTimHandle == NULL) || (psContext->pxTimHandle->Instance == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if (u32PulseWidthUs == 0U)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    u32CounterClockHz = Servo_Stm32Hal_prvGetCounterClockHz(psContext);
    if (u32CounterClockHz == 0U)
    {
        return DRIVER_ERR_CONFIG;
    }

    u32AutoReload = (uint32_t)__HAL_TIM_GET_AUTORELOAD(psContext->pxTimHandle);
    if (u32AutoReload == 0U)
    {
        return DRIVER_ERR_CONFIG;
    }

    u64CompareValue = (((uint64_t)u32PulseWidthUs * (uint64_t)u32CounterClockHz) + 500000ULL) / 1000000ULL;
    if (u64CompareValue > (uint64_t)u32AutoReload)
    {
        u64CompareValue = (uint64_t)u32AutoReload;
    }

    u32CompareValue = (uint32_t)u64CompareValue;
    __HAL_TIM_SET_COMPARE(psContext->pxTimHandle, psContext->u32Channel, u32CompareValue);

    return DRIVER_OK;
}

te_Driver_RetCode Servo_Stm32Hal_Lock(uint32_t u32TimeoutMs, void *vpCtx)
{
    ts_Servo_Stm32PortContext *psContext;

    psContext = (ts_Servo_Stm32PortContext *)vpCtx;
    if (psContext == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if (psContext->xServoMutex == NULL)
    {
        return DRIVER_OK;
    }

    if (osMutexAcquire(psContext->xServoMutex, u32TimeoutMs) != osOK)
    {
        return DRIVER_ERR_TIMEOUT;
    }

    return DRIVER_OK;
}

te_Driver_RetCode Servo_Stm32Hal_Unlock(void *vpCtx)
{
    ts_Servo_Stm32PortContext *psContext;

    psContext = (ts_Servo_Stm32PortContext *)vpCtx;
    if (psContext == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if (psContext->xServoMutex == NULL)
    {
        return DRIVER_OK;
    }

    return (osMutexRelease(psContext->xServoMutex) == osOK) ? DRIVER_OK : DRIVER_ERR_STATE;
}

uint32_t Servo_Stm32Hal_GetTickMs(void *vpCtx)
{
    (void)vpCtx;
    return HAL_GetTick();
}

te_Driver_RetCode Servo_Stm32Hal_FillPulseInterface(ts_Servo_PulseInterface *psPulseInterface,
                                                     ts_Servo_Stm32PortContext *psPortContext)
{
    if ((psPulseInterface == NULL) ||
        (psPortContext == NULL) ||
        (psPortContext->pxTimHandle == NULL) ||
        (psPortContext->pxTimHandle->Instance == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    psPulseInterface->pfnPulseWrite = Servo_Stm32Hal_PulseWrite;
    psPulseInterface->vpCtx = psPortContext;

    return DRIVER_OK;
}

te_Driver_RetCode Servo_Stm32Hal_FillLockInterface(ts_Servo_LockInterface *psLockInterface,
                                                    ts_Servo_Stm32PortContext *psPortContext)
{
    if ((psLockInterface == NULL) || (psPortContext == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    psLockInterface->pfnLock = Servo_Stm32Hal_Lock;
    psLockInterface->pfnUnlock = Servo_Stm32Hal_Unlock;
    psLockInterface->vpCtx = psPortContext;

    return DRIVER_OK;
}

te_Driver_RetCode Servo_Stm32Hal_FillTimingInterface(ts_Servo_TimingInterface *psTimingInterface)
{
    if (psTimingInterface == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    psTimingInterface->pfnGetTickMs = Servo_Stm32Hal_GetTickMs;
    psTimingInterface->vpCtx = NULL;

    return DRIVER_OK;
}

te_Driver_RetCode Servo_Stm32Hal_FillInterfaces(ts_Servo_OpenConfig *psOpenConfig,
                                                 ts_Servo_Stm32PortContext *psPortContext)
{
    te_Driver_RetCode eRet;

    if ((psOpenConfig == NULL) || (psPortContext == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    eRet = Servo_Stm32Hal_FillPulseInterface(&psOpenConfig->sPulseInterface, psPortContext);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Servo_Stm32Hal_FillLockInterface(&psOpenConfig->sLockInterface, psPortContext);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    return Servo_Stm32Hal_FillTimingInterface(&psOpenConfig->sTimingInterface);
}
