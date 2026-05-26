#include "gtest/gtest.h"

extern "C" {
#include "sensor_acq.h"
#include "gnc_telemetry.h"
#include "mock_mpu6050_api.h"
}

namespace {
bool g_bTxCalled = false;
uint16_t g_u16LastLen = 0U;

bool TestUartSend(const uint8_t *pu8Data, uint16_t u16Length, void *vpContext)
{
    (void)vpContext;
    g_bTxCalled = (pu8Data != nullptr);
    g_u16LastLen = u16Length;
    return true;
}
}  // namespace

TEST(SensorAcqTest, StepSendsPacketWhenValidImuAvailable)
{
    ts_SensorAcqContext sContext {};
    ts_SensorAcqConfig sConfig {};
    ts_Mpu6050_Handle sHandle {};
    ts_Mpu6050_Data sMockData {};
    uint8_t au8TxBuffer[GNC_TELEM_IMU_PACKET_LENGTH] = {0};

    g_bTxCalled = false;
    g_u16LastLen = 0U;

    sMockData.bValid = true;
    sMockData.u32TimestampMs = 42U;
    MockMpu6050_SetReadResponse(DRIVER_OK, &sMockData);

    sConfig.psMpuHandle = &sHandle;
    sConfig.pfnUartSend = TestUartSend;
    sConfig.vpUartContext = nullptr;
    sConfig.pu8TxBuffer = au8TxBuffer;
    sConfig.u16TxBufferLength = sizeof(au8TxBuffer);

    ASSERT_EQ(SensorAcq_Init(&sContext, &sConfig), SENSOR_ACQ_OK);
    EXPECT_EQ(SensorAcq_Step(&sContext), SENSOR_ACQ_OK);
    EXPECT_TRUE(g_bTxCalled);
    EXPECT_EQ(g_u16LastLen, GNC_TELEM_IMU_PACKET_LENGTH);
}
