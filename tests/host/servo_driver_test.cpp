#include "gtest/gtest.h"

extern "C" {
#include "servo_driver.h"
#include "servo_hal.h"
}

namespace {

struct ServoMockContext
{
    uint32_t u32LastPulseUs;
    uint32_t u32TickMs;
};

te_Driver_RetCode MockPulseWrite(uint32_t u32PulseWidthUs, void *vpCtx)
{
    ServoMockContext *psContext;

    psContext = static_cast<ServoMockContext *>(vpCtx);
    if (psContext == nullptr)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    psContext->u32LastPulseUs = u32PulseWidthUs;
    return DRIVER_OK;
}

uint32_t MockGetTickMs(void *vpCtx)
{
    ServoMockContext *psContext;

    psContext = static_cast<ServoMockContext *>(vpCtx);
    if (psContext == nullptr)
    {
        return 0U;
    }

    return psContext->u32TickMs;
}

ts_Servo_OpenConfig MakeDefaultConfig(ServoMockContext *psContext)
{
    ts_Servo_OpenConfig sConfig {};

    sConfig.sPulseInterface.pfnPulseWrite = MockPulseWrite;
    sConfig.sPulseInterface.vpCtx = psContext;
    sConfig.sTimingInterface.pfnGetTickMs = MockGetTickMs;
    sConfig.sTimingInterface.vpCtx = psContext;

    return sConfig;
}

}

TEST(ServoDriverTest, OpenUsesSg90Defaults)
{
    ServoMockContext sMock {};
    ts_Servo_Handle sHandle {};
    ts_Servo_OpenConfig sConfig = MakeDefaultConfig(&sMock);

    ASSERT_EQ(Servo_Open(&sHandle, &sConfig), DRIVER_OK);
    EXPECT_EQ(sHandle.eState, SERVO_STATE_READY);
    EXPECT_EQ(sHandle.u32MinPulseUs, SERVO_SG90_MIN_PULSE_US);
    EXPECT_EQ(sHandle.u32MaxPulseUs, SERVO_SG90_MAX_PULSE_US);
}

TEST(ServoDriverTest, WriteCenterMapsToCenterPulse)
{
    ServoMockContext sMock {};
    ts_Servo_Handle sHandle {};
    ts_Servo_OpenConfig sConfig = MakeDefaultConfig(&sMock);
    float f32AngleRad = 0.0F;

    ASSERT_EQ(Servo_Open(&sHandle, &sConfig), DRIVER_OK);
    ASSERT_EQ(Servo_Write(&sHandle, &f32AngleRad), DRIVER_OK);
    EXPECT_EQ(sMock.u32LastPulseUs, SERVO_SG90_CENTER_PULSE_US);
}

TEST(ServoDriverTest, WriteOverRangeClampsToSafePulse)
{
    ServoMockContext sMock {};
    ts_Servo_Handle sHandle {};
    ts_Servo_OpenConfig sConfig = MakeDefaultConfig(&sMock);
    float f32AngleRad = 10.0F;

    ASSERT_EQ(Servo_Open(&sHandle, &sConfig), DRIVER_OK);
    ASSERT_EQ(Servo_Write(&sHandle, &f32AngleRad), DRIVER_OK);
    EXPECT_EQ(sMock.u32LastPulseUs, SERVO_SG90_MAX_PULSE_US);
}

TEST(ServoDriverTest, RawPulseOverrideStillClamps)
{
    ServoMockContext sMock {};
    ts_Servo_Handle sHandle {};
    ts_Servo_OpenConfig sConfig = MakeDefaultConfig(&sMock);
    ts_Servo_RawPulseCommand sRaw {};

    ASSERT_EQ(Servo_Open(&sHandle, &sConfig), DRIVER_OK);
    sRaw.u32PulseWidthUs = 9999U;
    ASSERT_EQ(Servo_Ioctl(&sHandle, SERVO_IOCTL_OVERRIDE_RAW_PULSE, &sRaw), DRIVER_OK);
    EXPECT_EQ(sMock.u32LastPulseUs, SERVO_SG90_MAX_PULSE_US);
}

TEST(ServoDriverTest, SlewRateLimitsLargeStep)
{
    ServoMockContext sMock {};
    ts_Servo_Handle sHandle {};
    ts_Servo_OpenConfig sConfig = MakeDefaultConfig(&sMock);
    float f32AngleRad = SERVO_SG90_MAX_ANGLE_RAD;

    sConfig.bEnableSlewRate = true;
    sConfig.f32MaxSlewRateRadPerSec = 1.0F;
    ASSERT_EQ(Servo_Open(&sHandle, &sConfig), DRIVER_OK);

    sMock.u32TickMs = 0U;
    ASSERT_EQ(Servo_Write(&sHandle, &f32AngleRad), DRIVER_OK);
    EXPECT_EQ(sMock.u32LastPulseUs, SERVO_SG90_MAX_PULSE_US);

    sMock.u32TickMs = 20U;
    f32AngleRad = SERVO_SG90_MIN_ANGLE_RAD;
    ASSERT_EQ(Servo_Write(&sHandle, &f32AngleRad), DRIVER_OK);
    EXPECT_GT(sMock.u32LastPulseUs, SERVO_SG90_MIN_PULSE_US);
}
