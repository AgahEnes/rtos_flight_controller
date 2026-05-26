#include "mock_bus.h"

#include <string.h>

#include "mpu6050_hal.h"

static uint8_t gau8RegisterImage[256];

void MockBus_Reset(void)
{
    (void)memset(gau8RegisterImage, 0, sizeof(gau8RegisterImage));
    gau8RegisterImage[MPU6050_REG_WHO_AM_I] = MPU6050_WHO_AM_I_EXPECTED_LOW;
}

void MockBus_SetWhoAmI(uint8_t u8WhoAmI)
{
    gau8RegisterImage[MPU6050_REG_WHO_AM_I] = u8WhoAmI;
}

te_Driver_RetCode MockBus_Read(uint8_t u8DeviceAddr,
                               uint8_t u8RegisterAddr,
                               uint8_t *pu8ReadData,
                               uint16_t u16ReadLen,
                               uint32_t u32TimeoutMs,
                               void *vpCtx)
{
    (void)u8DeviceAddr;
    (void)u32TimeoutMs;
    (void)vpCtx;

    if ((pu8ReadData == NULL) || (u16ReadLen == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    (void)memcpy(pu8ReadData, &gau8RegisterImage[u8RegisterAddr], u16ReadLen);
    return DRIVER_OK;
}

te_Driver_RetCode MockBus_Write(uint8_t u8DeviceAddr,
                                uint8_t u8RegisterAddr,
                                const uint8_t *pu8WriteData,
                                uint16_t u16WriteLen,
                                uint32_t u32TimeoutMs,
                                void *vpCtx)
{
    (void)u8DeviceAddr;
    (void)u32TimeoutMs;
    (void)vpCtx;

    if ((pu8WriteData == NULL) || (u16WriteLen == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    (void)memcpy(&gau8RegisterImage[u8RegisterAddr], pu8WriteData, u16WriteLen);
    return DRIVER_OK;
}
