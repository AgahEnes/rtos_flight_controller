#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bmp180_driver.h"

static uint8_t gau8Registers[256];
static uint32_t gu32TickMs;
static uint32_t gu32DelayCallCount;

static void Bmp180Smoke_prvWriteBe16(uint8_t u8Register, int32_t s32Value)
{
    uint16_t u16Value = (uint16_t)s32Value;

    gau8Registers[u8Register] = (uint8_t)(u16Value >> 8U);
    gau8Registers[(uint8_t)(u8Register + 1U)] = (uint8_t)u16Value;
}

static void Bmp180Smoke_prvLoadCalibration(void)
{
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_AC1_MSB, 408);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_AC2_MSB, -72);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_AC3_MSB, -14383);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_AC4_MSB, 32741);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_AC5_MSB, 32757);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_AC6_MSB, 23153);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_B1_MSB, 6190);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_B2_MSB, 4);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_MB_MSB, -32768);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_MC_MSB, -8711);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_CALIB_MD_MSB, 2868);
}

static te_Driver_RetCode Bmp180Smoke_prvRead(uint8_t u8DeviceAddr,
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

    (void)memcpy(pu8ReadData, &gau8Registers[u8RegisterAddr], u16ReadLen);
    return DRIVER_OK;
}

static te_Driver_RetCode Bmp180Smoke_prvWrite(uint8_t u8DeviceAddr,
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

    (void)memcpy(&gau8Registers[u8RegisterAddr], pu8WriteData, u16WriteLen);
    return DRIVER_OK;
}

static te_Driver_RetCode Bmp180Smoke_prvDelay(uint32_t u32DelayMs, void *vpCtx)
{
    (void)vpCtx;
    gu32TickMs += u32DelayMs;
    ++gu32DelayCallCount;
    return DRIVER_OK;
}

static uint32_t Bmp180Smoke_prvGetTick(void *vpCtx)
{
    (void)vpCtx;
    return gu32TickMs;
}

static void Bmp180Smoke_prvCompleteTemperature(void)
{
    gau8Registers[BMP180_REG_CONTROL] &= (uint8_t)(~BMP180_CONTROL_SCO_MASK);
    Bmp180Smoke_prvWriteBe16(BMP180_REG_OUT_MSB, 27898);
}

static void Bmp180Smoke_prvCompletePressure(void)
{
    uint32_t u32RawPressure = (uint32_t)23843U << 8U;

    gau8Registers[BMP180_REG_CONTROL] &= (uint8_t)(~BMP180_CONTROL_SCO_MASK);
    gau8Registers[BMP180_REG_OUT_MSB] = (uint8_t)(u32RawPressure >> 16U);
    gau8Registers[BMP180_REG_OUT_LSB] = (uint8_t)(u32RawPressure >> 8U);
    gau8Registers[BMP180_REG_OUT_XLSB] = (uint8_t)u32RawPressure;
}

int main(void)
{
    ts_Bmp180_Handle sHandle;
    ts_Bmp180_OpenConfig sConfig;
    ts_Bmp180_Data sData;
    te_Bmp180_MeasurementState eMeasurementState;

    (void)memset(&gau8Registers, 0, sizeof(gau8Registers));
    (void)memset(&sHandle, 0, sizeof(sHandle));
    (void)memset(&sConfig, 0, sizeof(sConfig));
    gau8Registers[BMP180_REG_CHIP_ID] = BMP180_CHIP_ID_EXPECTED;
    Bmp180Smoke_prvLoadCalibration();

    sConfig.u8I2cAddress = BMP180_I2C_ADDR_DEFAULT;
    sConfig.eOversampling = BMP180_OSS0_ULTRA_LOW_POWER;
    sConfig.sBusInterface.pfnRead = Bmp180Smoke_prvRead;
    sConfig.sBusInterface.pfnWrite = Bmp180Smoke_prvWrite;
    sConfig.sTimingInterface.pfnDelayMs = Bmp180Smoke_prvDelay;
    sConfig.sTimingInterface.pfnGetTickMs = Bmp180Smoke_prvGetTick;
    sConfig.f32SeaLevelPressurePa = BMP180_SEA_LEVEL_PRESSURE_PA;

    assert(Bmp180_Open(&sHandle, &sConfig) == DRIVER_OK);
    assert(gu32DelayCallCount == 1U);
    assert(Bmp180_Read(&sHandle, &sData) == DRIVER_OK);
    assert(sData.bValid == false);

    assert(Bmp180_Ioctl(&sHandle, BMP180_IOCTL_START_MEASUREMENT, NULL) == DRIVER_OK);
    assert(Bmp180_Ioctl(&sHandle, BMP180_IOCTL_PROCESS_MEASUREMENT, NULL) == DRIVER_OK);
    assert(gu32DelayCallCount == 1U);

    gu32TickMs += BMP180_TEMP_CONV_TIME_MS;
    Bmp180Smoke_prvCompleteTemperature();
    assert(Bmp180_Ioctl(&sHandle, BMP180_IOCTL_PROCESS_MEASUREMENT, NULL) == DRIVER_OK);
    assert(Bmp180_Ioctl(&sHandle, BMP180_IOCTL_GET_MEASUREMENT_STATE, &eMeasurementState) == DRIVER_OK);
    assert(eMeasurementState == BMP180_MEASUREMENT_WAIT_PRESSURE);

    gu32TickMs += BMP180_PRESS_CONV_TIME_OSS0_MS;
    Bmp180Smoke_prvCompletePressure();
    assert(Bmp180_Ioctl(&sHandle, BMP180_IOCTL_PROCESS_MEASUREMENT, NULL) == DRIVER_OK);
    assert(Bmp180_Ioctl(&sHandle, BMP180_IOCTL_GET_MEASUREMENT_STATE, &eMeasurementState) == DRIVER_OK);
    assert(eMeasurementState == BMP180_MEASUREMENT_IDLE);
    assert(gu32DelayCallCount == 1U);

    assert(Bmp180_Read(&sHandle, &sData) == DRIVER_OK);
    assert(sData.bValid == true);
    assert(fabsf(sData.f32TemperatureC - 15.0f) < 0.1f);
    assert(fabsf(sData.f32PressurePa - 69964.0f) < 1.0f);
    return 0;
}
