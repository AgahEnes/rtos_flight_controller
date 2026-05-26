#include "gtest/gtest.h"

extern "C" {
#include "gnc_telemetry.h"
}

TEST(GncTelemetryTest, PacksImuFrameWithExpectedHeaderAndLength)
{
    ts_GncTelemImuPayload sPayload {};
    uint8_t au8Frame[GNC_TELEM_IMU_PACKET_LENGTH] = {0};

    sPayload.u8Sequence = 7U;
    sPayload.u32TimestampMs = 1234U;
    sPayload.f32AccelXMps2 = 1.0F;
    sPayload.f32AccelYMps2 = 2.0F;
    sPayload.f32AccelZMps2 = 3.0F;
    sPayload.f32GyroXRadS = 4.0F;
    sPayload.f32GyroYRadS = 5.0F;
    sPayload.f32GyroZRadS = 6.0F;
    sPayload.f32TempC = 35.5F;

    const uint16_t u16Len = GncTelemetry_PackImu(&sPayload, au8Frame, sizeof(au8Frame));

    EXPECT_EQ(u16Len, GNC_TELEM_IMU_PACKET_LENGTH);
    EXPECT_EQ(au8Frame[0], GNC_TELEM_SYNC_BYTE_0);
    EXPECT_EQ(au8Frame[1], GNC_TELEM_SYNC_BYTE_1);
    EXPECT_EQ(au8Frame[2], GNC_TELEM_MSG_ID_IMU);
    EXPECT_EQ(au8Frame[3], sPayload.u8Sequence);
}
