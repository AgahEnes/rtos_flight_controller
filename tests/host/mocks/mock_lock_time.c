#include "mock_lock_time.h"

static uint32_t gu32TickMs = 0U;

void MockTime_Reset(void)
{
    gu32TickMs = 0U;
}

te_Driver_RetCode MockLock_Lock(uint32_t u32TimeoutMs, void *vpCtx)
{
    (void)u32TimeoutMs;
    (void)vpCtx;
    return DRIVER_OK;
}

te_Driver_RetCode MockLock_Unlock(void *vpCtx)
{
    (void)vpCtx;
    return DRIVER_OK;
}

te_Driver_RetCode MockTime_DelayMs(uint32_t u32DelayMs, void *vpCtx)
{
    (void)vpCtx;
    gu32TickMs += u32DelayMs;
    return DRIVER_OK;
}

uint32_t MockTime_GetTickMs(void *vpCtx)
{
    (void)vpCtx;
    return gu32TickMs;
}
