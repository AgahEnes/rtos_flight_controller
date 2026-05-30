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
uint8_t g_au8LastFrame[TELEMETRY_TASK_IMU_PACKET_LENGTH] = {0};

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
    g_u16LastLen = u16Length;
    if ((pu8Data != nullptr) && (u16Length == TELEMETRY_TASK_IMU_PACKET_LENGTH))
    {
        (void)memcpy(g_au8LastFrame, pu8Data, TELEMETRY_TASK_IMU_PACKET_LENGTH);
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
    EXPECT_EQ(sReadBack.u32TimestampMs, 77U);
}

TEST(TelemetryTaskTest, StepPacksAndTransmitsWhenValidTopicExists)
{
    ts_TelemetryTaskContext sContext {};
    ts_TelemetryTaskConfig sConfig {};
    ts_TopicRawImu sRawImu {};
    uint8_t au8TxBuffer[TELEMETRY_TASK_IMU_PACKET_LENGTH] = {0};

    g_bUartTxCalled = false;
    g_u16LastLen = 0U;
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
    EXPECT_EQ(g_u16LastLen, TELEMETRY_TASK_IMU_PACKET_LENGTH);
    EXPECT_EQ(g_au8LastFrame[0], TELEMETRY_TASK_SYNC_BYTE_0);
    EXPECT_EQ(g_au8LastFrame[1], TELEMETRY_TASK_SYNC_BYTE_1);
    EXPECT_EQ(g_au8LastFrame[2], TELEMETRY_TASK_MSG_ID_IMU);
}

TEST(GlobalDataSpaceTest, PublishAndReadRejectNullPointers)
{
    ts_TopicRawImu sRawImu {};

    Gds_ResetRawImu();
    EXPECT_EQ(Gds_PublishRawImu(nullptr), GDS_ERR_ARG);
    EXPECT_EQ(Gds_ReadRawImu(nullptr), GDS_ERR_ARG);
    EXPECT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
}
