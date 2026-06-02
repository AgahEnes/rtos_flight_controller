#include "gtest/gtest.h"
#include <cstring>

extern "C" {
#include "global_data_space.h"
#include "sensor_manager.h"
#include "telemetry_task.h"
}

namespace {
bool g_bUartTxCalled = false;
uint16_t g_u16LastLen = 0U;
uint8_t g_au8LastFrame[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH] = {0};
uint8_t g_u8TxCallCount = 0U;
uint8_t g_u8LastMsgId = 0U;
uint32_t g_u32FakeTxTickMs = 0U;
uint16_t g_u16StateMsgCount = 0U;
uint16_t g_u16CalMsgCount = 0U;
uint8_t g_u8ForceTxFailCount = 0U;
constexpr uint16_t k_u16CalTelemetryTimestampOffset = TELEMETRY_TASK_FRAME_HEADER_LENGTH;
constexpr uint16_t k_u16CalUpdateCounterOffset = TELEMETRY_TASK_FRAME_HEADER_LENGTH + 4U + 12U + 12U + 4U;
constexpr uint16_t k_u16CalIsValidOffset = k_u16CalUpdateCounterOffset + 4U;

uint32_t prvReadU32Le(const uint8_t *pu8Data)
{
    return static_cast<uint32_t>(pu8Data[0]) |
           (static_cast<uint32_t>(pu8Data[1]) << 8U) |
           (static_cast<uint32_t>(pu8Data[2]) << 16U) |
           (static_cast<uint32_t>(pu8Data[3]) << 24U);
}

float prvReadF32Le(const uint8_t *pu8Data)
{
    uint32_t u32Raw = prvReadU32Le(pu8Data);
    float f32Value = 0.0F;

    (void)memcpy(&f32Value, &u32Raw, sizeof(f32Value));
    return f32Value;
}

uint16_t prvCrc16CcittFalse(const uint8_t *pu8Data, uint16_t u16Length)
{
    uint16_t u16Crc = 0xFFFFU;
    uint16_t u16Idx;
    uint8_t u8Bit;

    for (u16Idx = 0U; u16Idx < u16Length; ++u16Idx)
    {
        u16Crc ^= static_cast<uint16_t>(static_cast<uint16_t>(pu8Data[u16Idx]) << 8U);
        for (u8Bit = 0U; u8Bit < 8U; ++u8Bit)
        {
            if ((u16Crc & 0x8000U) != 0U)
            {
                u16Crc = static_cast<uint16_t>((u16Crc << 1U) ^ 0x1021U);
            }
            else
            {
                u16Crc <<= 1U;
            }
        }
    }

    return u16Crc;
}

uint16_t prvReadCrc16Le(const uint8_t *pu8Frame, uint16_t u16CrcOffset)
{
    return static_cast<uint16_t>(pu8Frame[u16CrcOffset]) |
           (static_cast<uint16_t>(pu8Frame[u16CrcOffset + 1U]) << 8U);
}

uint32_t prvReadCalUpdateCounter(const uint8_t *pu8Frame)
{
    return prvReadU32Le(&pu8Frame[k_u16CalUpdateCounterOffset]);
}

struct ts_FakeImuContext
{
    te_ImuDriverRetCode eInitRet;
    te_ImuDriverRetCode eReadRet;
    te_ImuDriverRetCode eSetBiasRet;
    uint32_t u32SetBiasCallCount;
    ts_TopicImuCalibration sLastCalibration;
    ts_TopicRawImu sSample;
};

te_ImuDriverRetCode FakeImu_Init(void *vpContext)
{
    ts_FakeImuContext *psContext = static_cast<ts_FakeImuContext *>(vpContext);

    if (psContext == nullptr)
    {
        return IMU_DRIVER_ERR_ARG;
    }

    return psContext->eInitRet;
}

te_ImuDriverRetCode FakeImu_Read(void *vpContext, ts_TopicRawImu *psRawImu)
{
    ts_FakeImuContext *psContext = static_cast<ts_FakeImuContext *>(vpContext);

    if ((psContext == nullptr) || (psRawImu == nullptr))
    {
        return IMU_DRIVER_ERR_ARG;
    }
    if (psContext->eReadRet != IMU_DRIVER_OK)
    {
        return psContext->eReadRet;
    }

    *psRawImu = psContext->sSample;
    return IMU_DRIVER_OK;
}

te_ImuDriverRetCode FakeImu_SetBias(void *vpContext, const ts_TopicImuCalibration *psCalibration)
{
    ts_FakeImuContext *psContext = static_cast<ts_FakeImuContext *>(vpContext);

    if ((psContext == nullptr) || (psCalibration == nullptr))
    {
        return IMU_DRIVER_ERR_ARG;
    }
    if (psContext->eSetBiasRet != IMU_DRIVER_OK)
    {
        return psContext->eSetBiasRet;
    }
    if (psContext->u32SetBiasCallCount < UINT32_MAX)
    {
        psContext->u32SetBiasCallCount++;
    }
    psContext->sLastCalibration = *psCalibration;

    return IMU_DRIVER_OK;
}

bool TestUartSend(const uint8_t *pu8Data, uint16_t u16Length, void *vpContext)
{
    (void)vpContext;
    g_bUartTxCalled = (pu8Data != nullptr);
    if ((pu8Data != nullptr) && (pu8Data[2] == TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE) && (g_u16StateMsgCount < UINT16_MAX))
    {
        g_u16StateMsgCount++;
    }
    if ((pu8Data != nullptr) && (pu8Data[2] == TELEMETRY_TASK_MSG_ID_IMU_CALIBRATION) && (g_u16CalMsgCount < UINT16_MAX))
    {
        g_u16CalMsgCount++;
    }
    if ((pu8Data != nullptr) && (g_u8TxCallCount < 255U))
    {
        g_u8TxCallCount++;
        g_u8LastMsgId = pu8Data[2];
    }
    g_u16LastLen = u16Length;
    if ((pu8Data != nullptr) && (u16Length <= TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH))
    {
        (void)memset(g_au8LastFrame, 0, sizeof(g_au8LastFrame));
        (void)memcpy(g_au8LastFrame, pu8Data, u16Length);
    }

    if (g_u8ForceTxFailCount > 0U)
    {
        g_u8ForceTxFailCount--;
        return false;
    }

    return true;
}

uint32_t TestGetTickMs(void *vpContext)
{
    (void)vpContext;
    return g_u32FakeTxTickMs;
}
}  // namespace

TEST(SensorManagerTest, StepPublishesValidImuToGds)
{
    static const ts_ImuDriverVTable ksVtable = {FakeImu_Init, FakeImu_Read, FakeImu_SetBias};
    ts_SensorManagerContext sContext {};
    ts_SensorManagerConfig sConfig {};
    ts_ImuDevice sImuDevice {};
    ts_FakeImuContext sImuContext {};
    ts_TopicRawImu sReadBack {};

    sImuContext.eInitRet = IMU_DRIVER_OK;
    sImuContext.eReadRet = IMU_DRIVER_OK;
    sImuContext.eSetBiasRet = IMU_DRIVER_OK;
    sImuContext.sSample.sAccel.f32X = 1.0F;
    sImuContext.sSample.sAccel.f32Y = 2.0F;
    sImuContext.sSample.sAccel.f32Z = 3.0F;
    sImuContext.sSample.sGyro.f32X = 4.0F;
    sImuContext.sSample.sGyro.f32Y = 5.0F;
    sImuContext.sSample.sGyro.f32Z = 6.0F;
    sImuContext.sSample.f32TempC = 26.5F;
    sImuContext.sSample.u32TimestampMs = 77U;
    sImuContext.sSample.bIsValid = true;

    sImuDevice.psVTable = &ksVtable;
    sImuDevice.vpContext = &sImuContext;
    sConfig.psImuDevices = &sImuDevice;
    sConfig.u8ImuDeviceCount = 1U;

    Gds_ResetRawImu();
    ASSERT_EQ(SensorManager_Init(&sContext, &sConfig), SENSOR_MANAGER_OK);
    ASSERT_EQ(SensorManager_Step(&sContext), SENSOR_MANAGER_OK);

    ASSERT_EQ(Gds_ReadRawImu(&sReadBack), GDS_OK);
    EXPECT_TRUE(sReadBack.bIsValid);
    EXPECT_FLOAT_EQ(sReadBack.sAccel.f32X, 1.0F);
    EXPECT_FLOAT_EQ(sReadBack.sGyro.f32Z, 6.0F);
    EXPECT_FLOAT_EQ(sReadBack.f32TempC, 26.5F);
    EXPECT_EQ(sReadBack.u32TimestampMs, 77U);
}

TEST(SensorManagerTest, StepSkipsPublishWhenTimestampUnchanged)
{
    static const ts_ImuDriverVTable ksVtable = {FakeImu_Init, FakeImu_Read, FakeImu_SetBias};
    ts_SensorManagerContext sContext {};
    ts_SensorManagerConfig sConfig {};
    ts_ImuDevice sImuDevice {};
    ts_FakeImuContext sImuContext {};
    ts_TopicRawImu sReadBack {};

    sImuContext.eInitRet = IMU_DRIVER_OK;
    sImuContext.eReadRet = IMU_DRIVER_OK;
    sImuContext.eSetBiasRet = IMU_DRIVER_OK;
    sImuContext.sSample.bIsValid = true;
    sImuContext.sSample.u32TimestampMs = 100U;

    sImuDevice.psVTable = &ksVtable;
    sImuDevice.vpContext = &sImuContext;
    sConfig.psImuDevices = &sImuDevice;
    sConfig.u8ImuDeviceCount = 1U;

    Gds_ResetRawImu();
    ASSERT_EQ(SensorManager_Init(&sContext, &sConfig), SENSOR_MANAGER_OK);

    ASSERT_EQ(SensorManager_Step(&sContext), SENSOR_MANAGER_OK);
    ASSERT_EQ(Gds_ReadRawImu(&sReadBack), GDS_OK);
    EXPECT_EQ(sReadBack.u32TimestampMs, 100U);

    ASSERT_EQ(SensorManager_Step(&sContext), SENSOR_MANAGER_OK);
    ASSERT_EQ(Gds_ReadRawImu(&sReadBack), GDS_OK);
    EXPECT_EQ(sReadBack.u32TimestampMs, 100U);

    sImuContext.sSample.u32TimestampMs = 110U;
    ASSERT_EQ(SensorManager_Step(&sContext), SENSOR_MANAGER_OK);
    ASSERT_EQ(Gds_ReadRawImu(&sReadBack), GDS_OK);
    EXPECT_EQ(sReadBack.u32TimestampMs, 110U);
}

TEST(SensorManagerTest, StepAppliesOnlyFreshCalibrationCounters)
{
    static const ts_ImuDriverVTable ksVtable = {FakeImu_Init, FakeImu_Read, FakeImu_SetBias};
    ts_SensorManagerContext sContext {};
    ts_SensorManagerConfig sConfig {};
    ts_ImuDevice sImuDevice {};
    ts_FakeImuContext sImuContext {};
    ts_TopicImuCalibration sCalibration {};

    sImuContext.eInitRet = IMU_DRIVER_OK;
    sImuContext.eReadRet = IMU_DRIVER_OK;
    sImuContext.eSetBiasRet = IMU_DRIVER_OK;
    sImuContext.sSample.sAccel.f32Z = 9.81F;
    sImuContext.sSample.bIsValid = true;
    sImuContext.sSample.u32TimestampMs = 100U;

    sImuDevice.psVTable = &ksVtable;
    sImuDevice.vpContext = &sImuContext;
    sConfig.psImuDevices = &sImuDevice;
    sConfig.u8ImuDeviceCount = 1U;

    Gds_ResetRawImu();
    Gds_ResetImuCalibration();
    ASSERT_EQ(SensorManager_Init(&sContext, &sConfig), SENSOR_MANAGER_OK);

    sCalibration.sAccelBiasMps2.f32X = 0.10F;
    sCalibration.sGyroBiasRadS.f32Y = 0.20F;
    sCalibration.u32TimestampMs = 101U;
    sCalibration.u32UpdateCounter = 1U;
    sCalibration.bIsValid = true;
    ASSERT_EQ(Gds_PublishImuCalibration(&sCalibration), GDS_OK);

    ASSERT_EQ(SensorManager_Step(&sContext), SENSOR_MANAGER_OK);
    EXPECT_EQ(sImuContext.u32SetBiasCallCount, 1U);
    EXPECT_EQ(sImuContext.sLastCalibration.u32UpdateCounter, 1U);

    sImuContext.sSample.u32TimestampMs = 110U;
    ASSERT_EQ(SensorManager_Step(&sContext), SENSOR_MANAGER_OK);
    EXPECT_EQ(sImuContext.u32SetBiasCallCount, 1U);

    sCalibration.u32UpdateCounter = 2U;
    sCalibration.sGyroBiasRadS.f32Z = 0.30F;
    ASSERT_EQ(Gds_PublishImuCalibration(&sCalibration), GDS_OK);
    sImuContext.sSample.u32TimestampMs = 120U;
    ASSERT_EQ(SensorManager_Step(&sContext), SENSOR_MANAGER_OK);
    EXPECT_EQ(sImuContext.u32SetBiasCallCount, 2U);
    EXPECT_FLOAT_EQ(sImuContext.sLastCalibration.sGyroBiasRadS.f32Z, 0.30F);
}

TEST(TelemetryTaskTest, StepPacksAndTransmitsWhenValidTopicExists)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH] = {0};

    g_bUartTxCalled = false;
    g_u16LastLen = 0U;
    g_u8TxCallCount = 0U;
    g_u8LastMsgId = 0U;
    g_u16StateMsgCount = 0U;
    g_u16CalMsgCount = 0U;
    g_u8ForceTxFailCount = 0U;
    (void)memset(g_au8LastFrame, 0, sizeof(g_au8LastFrame));

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pfnGetTickMs = TestGetTickMs;
    sConfig.vpTickContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.sAccel.f32X = 9.0F;
    sRawImu.sGyro.f32Y = 8.0F;
    sRawImu.u32TimestampMs = 1234U;
    sRawImu.bIsValid = true;
    g_u32FakeTxTickMs = 5000U;

    Gds_ResetRawImu();
    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(TelemetryTask_Init(&sContext, &sConfig), TELEMETRY_TASK_OK);
    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);

    EXPECT_TRUE(g_bUartTxCalled);
    EXPECT_EQ(g_u8TxCallCount, 1U);
    EXPECT_EQ(g_u16LastLen, TELEMETRY_TASK_FRAME_LENGTH);
    EXPECT_EQ(g_u16LastLen, TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH);
    EXPECT_EQ(g_au8LastFrame[0], TELEMETRY_TASK_SYNC_BYTE_0);
    EXPECT_EQ(g_au8LastFrame[1], TELEMETRY_TASK_SYNC_BYTE_1);
    EXPECT_EQ(g_au8LastFrame[2], TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE);
    EXPECT_EQ(prvReadU32Le(&g_au8LastFrame[4]), g_u32FakeTxTickMs);
    EXPECT_EQ(prvReadCrc16Le(g_au8LastFrame,
                             TELEMETRY_TASK_FRAME_LENGTH - TELEMETRY_TASK_FRAME_CRC_LENGTH),
              prvCrc16CcittFalse(g_au8LastFrame,
                                 TELEMETRY_TASK_FRAME_LENGTH - TELEMETRY_TASK_FRAME_CRC_LENGTH));
}

TEST(TelemetryTaskTest, StepAlwaysTransmitsEvenWhenImuTopicInvalid)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH] = {0};

    g_bUartTxCalled = false;
    g_u8TxCallCount = 0U;
    g_u16StateMsgCount = 0U;
    g_u16CalMsgCount = 0U;
    g_u8ForceTxFailCount = 0U;
    (void)memset(g_au8LastFrame, 0, sizeof(g_au8LastFrame));

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pfnGetTickMs = TestGetTickMs;
    sConfig.vpTickContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.sAccel.f32X = 7.0F;
    sRawImu.bIsValid = false;
    g_u32FakeTxTickMs = 3000U;

    Gds_ResetRawImu();
    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(TelemetryTask_Init(&sContext, &sConfig), TELEMETRY_TASK_OK);
    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);

    EXPECT_EQ(g_u8TxCallCount, 1U);
    EXPECT_FLOAT_EQ(prvReadF32Le(&g_au8LastFrame[TELEMETRY_TASK_FRAME_HEADER_LENGTH +
                                                   TELEMETRY_TASK_PACKET_TIMESTAMP_LENGTH]),
                    7.0F);
}

TEST(TelemetryTaskTest, StepTransmitsCombinedFrameIncludingEstimatedVehicleState)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    ts_TopicVehicleState sVehicleState {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH] = {0};

    g_bUartTxCalled = false;
    g_u16LastLen = 0U;
    g_u8TxCallCount = 0U;
    g_u8LastMsgId = 0U;
    g_u16StateMsgCount = 0U;
    g_u16CalMsgCount = 0U;
    g_u8ForceTxFailCount = 0U;
    (void)memset(g_au8LastFrame, 0, sizeof(g_au8LastFrame));

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pfnGetTickMs = TestGetTickMs;
    sConfig.vpTickContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.sAccel.f32X = 1.0F;
    sRawImu.sGyro.f32Y = 2.0F;
    sRawImu.u32TimestampMs = 2000U;
    sRawImu.bIsValid = true;
    g_u32FakeTxTickMs = 9000U;

    sVehicleState.f32RollRad = 0.12F;
    sVehicleState.f32PitchRad = 0.34F;
    sVehicleState.f32YawRad = 0.56F;
    sVehicleState.f32RollRateRadS = 0.78F;
    sVehicleState.f32PitchRateRadS = 0.91F;
    sVehicleState.f32YawRateRadS = 1.23F;
    sVehicleState.u32TimestampMs = 2000U;
    sVehicleState.bIsEstimated = true;

    Gds_ResetRawImu();
    Gds_ResetVehicleState();
    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(Gds_PublishVehicleState(&sVehicleState), GDS_OK);
    ASSERT_EQ(TelemetryTask_Init(&sContext, &sConfig), TELEMETRY_TASK_OK);
    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);

    EXPECT_TRUE(g_bUartTxCalled);
    EXPECT_EQ(g_u8TxCallCount, 1U);
    EXPECT_EQ(g_u8LastMsgId, TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE);
    EXPECT_EQ(g_u16LastLen, TELEMETRY_TASK_FRAME_LENGTH);
    EXPECT_EQ(g_au8LastFrame[0], TELEMETRY_TASK_SYNC_BYTE_0);
    EXPECT_EQ(g_au8LastFrame[1], TELEMETRY_TASK_SYNC_BYTE_1);
    EXPECT_EQ(g_au8LastFrame[2], TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE);
    EXPECT_EQ(prvReadU32Le(&g_au8LastFrame[4]), g_u32FakeTxTickMs);
    EXPECT_EQ(g_au8LastFrame[TELEMETRY_TASK_FRAME_HEADER_LENGTH +
                            TELEMETRY_TASK_PACKET_TIMESTAMP_LENGTH +
                            TELEMETRY_TASK_IMU_PACKET_LENGTH +
                            TELEMETRY_TASK_VEHICLE_PACKET_LENGTH - 1U],
              1U);
    EXPECT_EQ(prvReadCrc16Le(g_au8LastFrame,
                             TELEMETRY_TASK_FRAME_LENGTH - TELEMETRY_TASK_FRAME_CRC_LENGTH),
              prvCrc16CcittFalse(g_au8LastFrame,
                                 TELEMETRY_TASK_FRAME_LENGTH - TELEMETRY_TASK_FRAME_CRC_LENGTH));
}

TEST(TelemetryTaskTest, StepEventSlotTransmitsCalibrationWhenPending)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    ts_TopicImuCalibration sCalibration {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH] = {0};

    g_bUartTxCalled = false;
    g_u16LastLen = 0U;
    g_u8TxCallCount = 0U;
    g_u8LastMsgId = 0U;
    g_u16StateMsgCount = 0U;
    g_u16CalMsgCount = 0U;
    g_u8ForceTxFailCount = 0U;
    (void)memset(g_au8LastFrame, 0, sizeof(g_au8LastFrame));

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pfnGetTickMs = TestGetTickMs;
    sConfig.vpTickContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.bIsValid = true;
    sRawImu.sAccel.f32X = 3.0F;
    g_u32FakeTxTickMs = 1000U;

    sCalibration.sAccelBiasMps2.f32X = 0.1F;
    sCalibration.sAccelBiasMps2.f32Y = -0.2F;
    sCalibration.sAccelBiasMps2.f32Z = 0.3F;
    sCalibration.sGyroBiasRadS.f32X = 0.4F;
    sCalibration.sGyroBiasRadS.f32Y = -0.5F;
    sCalibration.sGyroBiasRadS.f32Z = 0.6F;
    sCalibration.u32TimestampMs = 555U;
    sCalibration.u32UpdateCounter = 1U;
    sCalibration.bIsValid = true;

    Gds_ResetRawImu();
    Gds_ResetImuCalibration();
    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(Gds_PublishImuCalibration(&sCalibration), GDS_OK);
    ASSERT_EQ(TelemetryTask_Init(&sContext, &sConfig), TELEMETRY_TASK_OK);

    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);
    EXPECT_EQ(g_u8LastMsgId, TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE);
    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);
    EXPECT_EQ(g_u8LastMsgId, TELEMETRY_TASK_MSG_ID_IMU_CALIBRATION);
    EXPECT_EQ(g_u16LastLen, TELEMETRY_TASK_CALIBRATION_FRAME_LENGTH);
    EXPECT_EQ(prvReadU32Le(&g_au8LastFrame[k_u16CalTelemetryTimestampOffset]), g_u32FakeTxTickMs);
    EXPECT_EQ(prvReadCalUpdateCounter(g_au8LastFrame), sCalibration.u32UpdateCounter);
    EXPECT_EQ(g_au8LastFrame[k_u16CalIsValidOffset], 1U);
    EXPECT_EQ(g_u16StateMsgCount, 1U);
    EXPECT_EQ(g_u16CalMsgCount, 1U);
}

TEST(TelemetryTaskTest, StepEventSlotNoTxWhenNoPending)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH] = {0};

    g_u8TxCallCount = 0U;
    g_u16StateMsgCount = 0U;
    g_u16CalMsgCount = 0U;
    g_u8ForceTxFailCount = 0U;

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pfnGetTickMs = TestGetTickMs;
    sConfig.vpTickContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.bIsValid = true;
    Gds_ResetRawImu();
    Gds_ResetImuCalibration();
    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(TelemetryTask_Init(&sContext, &sConfig), TELEMETRY_TASK_OK);

    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);
    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);
    EXPECT_EQ(g_u8TxCallCount, 1U);
    EXPECT_EQ(g_u16StateMsgCount, 1U);
    EXPECT_EQ(g_u16CalMsgCount, 0U);
}

TEST(TelemetryTaskTest, StepEventSlotRetriesCalibrationOnNextEventSlot)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    ts_TopicImuCalibration sCalibration {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH] = {0};

    g_u8TxCallCount = 0U;
    g_u16StateMsgCount = 0U;
    g_u16CalMsgCount = 0U;
    g_u8ForceTxFailCount = 0U;
    g_u8LastMsgId = 0U;

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pfnGetTickMs = TestGetTickMs;
    sConfig.vpTickContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.bIsValid = true;
    sCalibration.bIsValid = true;
    sCalibration.u32TimestampMs = 123U;
    sCalibration.u32UpdateCounter = 7U;

    Gds_ResetRawImu();
    Gds_ResetImuCalibration();
    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(Gds_PublishImuCalibration(&sCalibration), GDS_OK);
    ASSERT_EQ(TelemetryTask_Init(&sContext, &sConfig), TELEMETRY_TASK_OK);

    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);
    g_u8ForceTxFailCount = 1U;
    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_ERR_TX);
    EXPECT_EQ(g_u8LastMsgId, TELEMETRY_TASK_MSG_ID_IMU_CALIBRATION);

    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);
    EXPECT_EQ(g_u8LastMsgId, TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE);

    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);
    EXPECT_EQ(g_u8LastMsgId, TELEMETRY_TASK_MSG_ID_IMU_CALIBRATION);
    EXPECT_EQ(prvReadCalUpdateCounter(g_au8LastFrame), sCalibration.u32UpdateCounter);
    EXPECT_EQ(g_u16StateMsgCount, 2U);
    EXPECT_EQ(g_u16CalMsgCount, 2U);
}

TEST(TelemetryTaskTest, AlternatingCadenceProducesTenHzSync)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH] = {0};
    uint8_t u8Step;

    g_u8TxCallCount = 0U;
    g_u16StateMsgCount = 0U;
    g_u16CalMsgCount = 0U;
    g_u8ForceTxFailCount = 0U;

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pfnGetTickMs = TestGetTickMs;
    sConfig.vpTickContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.bIsValid = true;
    Gds_ResetRawImu();
    Gds_ResetImuCalibration();
    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(TelemetryTask_Init(&sContext, &sConfig), TELEMETRY_TASK_OK);

    for (u8Step = 0U; u8Step < 10U; u8Step++)
    {
        ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);
    }

    EXPECT_EQ(g_u16StateMsgCount, 5U);
    EXPECT_EQ(g_u16CalMsgCount, 0U);
    EXPECT_EQ(g_u8TxCallCount, 5U);
}

TEST(GlobalDataSpaceTest, PublishAndReadRejectNullPointers)
{
    ts_TopicRawImu sRawImu {};

    Gds_ResetRawImu();
    EXPECT_EQ(Gds_PublishRawImu(nullptr), GDS_ERR_ARG);
    EXPECT_EQ(Gds_ReadRawImu(nullptr), GDS_ERR_ARG);
    EXPECT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
}

TEST(GlobalDataSpaceVehicleStateTest, PublishAndReadRoundTripAndNullChecks)
{
    ts_TopicVehicleState sVehicleState {};
    ts_TopicVehicleState sReadBack {};

    sVehicleState.f32RollRad = 0.10F;
    sVehicleState.f32PitchRad = -0.20F;
    sVehicleState.f32YawRad = 1.50F;
    sVehicleState.f32RollRateRadS = 2.10F;
    sVehicleState.f32PitchRateRadS = -3.20F;
    sVehicleState.f32YawRateRadS = 4.30F;
    sVehicleState.u32TimestampMs = 321U;
    sVehicleState.bIsEstimated = true;

    Gds_ResetVehicleState();
    EXPECT_EQ(Gds_PublishVehicleState(nullptr), GDS_ERR_ARG);
    EXPECT_EQ(Gds_ReadVehicleState(nullptr), GDS_ERR_ARG);

    ASSERT_EQ(Gds_PublishVehicleState(&sVehicleState), GDS_OK);
    ASSERT_EQ(Gds_ReadVehicleState(&sReadBack), GDS_OK);
    EXPECT_FLOAT_EQ(sReadBack.f32RollRad, sVehicleState.f32RollRad);
    EXPECT_FLOAT_EQ(sReadBack.f32PitchRad, sVehicleState.f32PitchRad);
    EXPECT_FLOAT_EQ(sReadBack.f32YawRad, sVehicleState.f32YawRad);
    EXPECT_FLOAT_EQ(sReadBack.f32RollRateRadS, sVehicleState.f32RollRateRadS);
    EXPECT_FLOAT_EQ(sReadBack.f32PitchRateRadS, sVehicleState.f32PitchRateRadS);
    EXPECT_FLOAT_EQ(sReadBack.f32YawRateRadS, sVehicleState.f32YawRateRadS);
    EXPECT_EQ(sReadBack.u32TimestampMs, sVehicleState.u32TimestampMs);
    EXPECT_EQ(sReadBack.bIsEstimated, true);
}

TEST(GlobalDataSpaceCalibrationTest, PublishAndReadRoundTripAndNullChecks)
{
    ts_TopicImuCalibration sCalibration {};
    ts_TopicImuCalibration sReadBack {};

    sCalibration.sAccelBiasMps2.f32X = 0.11F;
    sCalibration.sAccelBiasMps2.f32Y = -0.22F;
    sCalibration.sAccelBiasMps2.f32Z = 0.33F;
    sCalibration.sGyroBiasRadS.f32X = 0.44F;
    sCalibration.sGyroBiasRadS.f32Y = -0.55F;
    sCalibration.sGyroBiasRadS.f32Z = 0.66F;
    sCalibration.u32TimestampMs = 567U;
    sCalibration.u32UpdateCounter = 42U;
    sCalibration.bIsValid = true;

    Gds_ResetImuCalibration();
    EXPECT_EQ(Gds_PublishImuCalibration(nullptr), GDS_ERR_ARG);
    EXPECT_EQ(Gds_ReadImuCalibration(nullptr), GDS_ERR_ARG);

    ASSERT_EQ(Gds_PublishImuCalibration(&sCalibration), GDS_OK);
    ASSERT_EQ(Gds_ReadImuCalibration(&sReadBack), GDS_OK);
    EXPECT_FLOAT_EQ(sReadBack.sAccelBiasMps2.f32X, sCalibration.sAccelBiasMps2.f32X);
    EXPECT_FLOAT_EQ(sReadBack.sAccelBiasMps2.f32Y, sCalibration.sAccelBiasMps2.f32Y);
    EXPECT_FLOAT_EQ(sReadBack.sAccelBiasMps2.f32Z, sCalibration.sAccelBiasMps2.f32Z);
    EXPECT_FLOAT_EQ(sReadBack.sGyroBiasRadS.f32X, sCalibration.sGyroBiasRadS.f32X);
    EXPECT_FLOAT_EQ(sReadBack.sGyroBiasRadS.f32Y, sCalibration.sGyroBiasRadS.f32Y);
    EXPECT_FLOAT_EQ(sReadBack.sGyroBiasRadS.f32Z, sCalibration.sGyroBiasRadS.f32Z);
    EXPECT_EQ(sReadBack.u32TimestampMs, sCalibration.u32TimestampMs);
    EXPECT_EQ(sReadBack.u32UpdateCounter, sCalibration.u32UpdateCounter);
    EXPECT_TRUE(sReadBack.bIsValid);
}

TEST(GlobalDataSpaceNavCommandTest, PublishAndReadRoundTripAndNullChecks)
{
    ts_TopicNavCommand sCommand {};
    ts_TopicNavCommand sReadBack {};

    sCommand.eCommand = NAV_CMD_REINIT;
    sCommand.u32Sequence = 7U;
    sCommand.u32TimestampMs = 777U;

    Gds_ResetNavCommand();
    EXPECT_EQ(Gds_PublishNavCommand(nullptr), GDS_ERR_ARG);
    EXPECT_EQ(Gds_ReadNavCommand(nullptr), GDS_ERR_ARG);

    ASSERT_EQ(Gds_PublishNavCommand(&sCommand), GDS_OK);
    ASSERT_EQ(Gds_ReadNavCommand(&sReadBack), GDS_OK);
    EXPECT_EQ(sReadBack.eCommand, NAV_CMD_REINIT);
    EXPECT_EQ(sReadBack.u32Sequence, sCommand.u32Sequence);
    EXPECT_EQ(sReadBack.u32TimestampMs, sCommand.u32TimestampMs);
}

TEST(GlobalDataSpaceActuatorCmdTest, PublishAndReadRoundTripAndNullChecks)
{
    ts_TopicActuatorCmd sCommand {};
    ts_TopicActuatorCmd sReadBack {};

    sCommand.f32FinAngleRad[0] = 0.10F;
    sCommand.f32FinAngleRad[1] = -0.20F;
    sCommand.f32FinAngleRad[2] = 0.30F;
    sCommand.f32FinAngleRad[3] = -0.40F;
    sCommand.u32TimestampMs = 555U;
    sCommand.u32Sequence = 3U;
    sCommand.bIsActive = true;

    Gds_ResetActuatorCmd();
    EXPECT_EQ(Gds_PublishActuatorCmd(nullptr), GDS_ERR_ARG);
    EXPECT_EQ(Gds_ReadActuatorCmd(nullptr), GDS_ERR_ARG);

    ASSERT_EQ(Gds_PublishActuatorCmd(&sCommand), GDS_OK);
    ASSERT_EQ(Gds_ReadActuatorCmd(&sReadBack), GDS_OK);
    EXPECT_FLOAT_EQ(sReadBack.f32FinAngleRad[0], sCommand.f32FinAngleRad[0]);
    EXPECT_FLOAT_EQ(sReadBack.f32FinAngleRad[1], sCommand.f32FinAngleRad[1]);
    EXPECT_FLOAT_EQ(sReadBack.f32FinAngleRad[2], sCommand.f32FinAngleRad[2]);
    EXPECT_FLOAT_EQ(sReadBack.f32FinAngleRad[3], sCommand.f32FinAngleRad[3]);
    EXPECT_EQ(sReadBack.u32TimestampMs, sCommand.u32TimestampMs);
    EXPECT_EQ(sReadBack.u32Sequence, sCommand.u32Sequence);
    EXPECT_TRUE(sReadBack.bIsActive);
}
