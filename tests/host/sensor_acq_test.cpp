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
uint8_t g_au8LastFrame[TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH] = {0};
uint8_t g_u8TxCallCount = 0U;
uint8_t g_u8LastMsgId = 0U;

struct ts_FakeImuContext
{
    te_ImuDriverRetCode eInitRet;
    te_ImuDriverRetCode eReadRet;
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

bool TestUartSend(const uint8_t *pu8Data, uint16_t u16Length, void *vpContext)
{
    (void)vpContext;
    g_bUartTxCalled = (pu8Data != nullptr);
    if ((pu8Data != nullptr) && (g_u8TxCallCount < 255U))
    {
        g_u8TxCallCount++;
        g_u8LastMsgId = pu8Data[2];
    }
    g_u16LastLen = u16Length;
    if ((pu8Data != nullptr) && (u16Length <= TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH))
    {
        (void)memset(g_au8LastFrame, 0, sizeof(g_au8LastFrame));
        (void)memcpy(g_au8LastFrame, pu8Data, u16Length);
    }

    return true;
}
}  // namespace

TEST(SensorManagerTest, StepPublishesValidImuToGds)
{
    static const ts_ImuDriverVTable ksVtable = {FakeImu_Init, FakeImu_Read};
    ts_SensorManagerContext sContext {};
    ts_SensorManagerConfig sConfig {};
    ts_ImuDevice sImuDevice {};
    ts_FakeImuContext sImuContext {};
    ts_TopicRawImu sReadBack {};

    sImuContext.eInitRet = IMU_DRIVER_OK;
    sImuContext.eReadRet = IMU_DRIVER_OK;
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

TEST(TelemetryTaskTest, StepPacksAndTransmitsWhenValidTopicExists)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH] = {0};

    g_bUartTxCalled = false;
    g_u16LastLen = 0U;
    g_u8TxCallCount = 0U;
    g_u8LastMsgId = 0U;
    (void)memset(g_au8LastFrame, 0, sizeof(g_au8LastFrame));

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.sAccel.f32X = 9.0F;
    sRawImu.sGyro.f32Y = 8.0F;
    sRawImu.u32TimestampMs = 1234U;
    sRawImu.bIsValid = true;

    Gds_ResetRawImu();
    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(TelemetryTask_Init(&sContext, &sConfig), TELEMETRY_TASK_OK);
    ASSERT_EQ(TelemetryTask_Step(&sContext), TELEMETRY_TASK_OK);

    EXPECT_TRUE(g_bUartTxCalled);
    EXPECT_EQ(g_u8TxCallCount, 1U);
    EXPECT_EQ(g_u16LastLen, TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH);
    EXPECT_EQ(g_au8LastFrame[0], TELEMETRY_TASK_SYNC_BYTE_0);
    EXPECT_EQ(g_au8LastFrame[1], TELEMETRY_TASK_SYNC_BYTE_1);
    EXPECT_EQ(g_au8LastFrame[2], TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE);
}

TEST(TelemetryTaskTest, StepTransmitsCombinedFrameIncludingEstimatedVehicleState)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    ts_TopicVehicleState sVehicleState {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH] = {0};

    g_bUartTxCalled = false;
    g_u16LastLen = 0U;
    g_u8TxCallCount = 0U;
    g_u8LastMsgId = 0U;
    (void)memset(g_au8LastFrame, 0, sizeof(g_au8LastFrame));

    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    sRawImu.sAccel.f32X = 1.0F;
    sRawImu.sGyro.f32Y = 2.0F;
    sRawImu.u32TimestampMs = 2000U;
    sRawImu.bIsValid = true;

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
    EXPECT_EQ(g_u16LastLen, TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH);
    EXPECT_EQ(g_au8LastFrame[0], TELEMETRY_TASK_SYNC_BYTE_0);
    EXPECT_EQ(g_au8LastFrame[1], TELEMETRY_TASK_SYNC_BYTE_1);
    EXPECT_EQ(g_au8LastFrame[2], TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE);
    EXPECT_EQ(g_au8LastFrame[TELEMETRY_TASK_FRAME_HEADER_LENGTH + TELEMETRY_TASK_IMU_PACKET_LENGTH +
                            TELEMETRY_TASK_VEHICLE_PACKET_LENGTH - 1U],
              1U);
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
