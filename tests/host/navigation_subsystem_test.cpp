#include "gtest/gtest.h"

extern "C" {
#include "navigation_subsystem.h"
#include "global_data_space.h"
}

namespace {
constexpr float kTwoPiRad = 6.28318530717958647692F;

ts_TopicRawImu CreateValidImuSample(uint32_t u32TimestampMs)
{
    ts_TopicRawImu sRawImu {};

    sRawImu.sAccel.f32X = 0.0F;
    sRawImu.sAccel.f32Y = 0.0F;
    sRawImu.sAccel.f32Z = 9.81F;
    sRawImu.sGyro.f32X = 0.01F;
    sRawImu.sGyro.f32Y = -0.02F;
    sRawImu.sGyro.f32Z = 0.03F;
    sRawImu.u32TimestampMs = u32TimestampMs;
    sRawImu.bIsValid = true;

    return sRawImu;
}

ts_NavConfig CreateDefaultConfig(void)
{
    ts_NavConfig sConfig {};

    sConfig.f32Alpha = NAV_CFG_DEFAULT_ALPHA;
    sConfig.f32DtS = NAV_CFG_DEFAULT_DT_S;
    sConfig.f32ZeroEpsilon = NAV_CFG_ZERO_EPSILON;
    sConfig.u8StuckThresholdCycles = NAV_CFG_STUCK_THRESHOLD_CYCLES;
    sConfig.sInitialAttitude.u8IsValid = 0U;

    return sConfig;
}
}  // namespace

TEST(NavigationSubsystemTest, InitAndStepRejectInvalidArguments)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(10U);
    ts_TopicVehicleState sVehicleState {};

    EXPECT_EQ(Navigation_Init(nullptr, &sConfig), NAV_RET_ERR_ARG);
    EXPECT_EQ(Navigation_Init(&sContext, nullptr), NAV_RET_ERR_ARG);
    EXPECT_EQ(NavigationTask_Step(nullptr, &sRawImu, &sVehicleState), NAV_RET_ERR_ARG);
    EXPECT_EQ(NavigationTask_Step(&sContext, nullptr, &sVehicleState), NAV_RET_ERR_ARG);
    EXPECT_EQ(NavigationTask_Step(&sContext, &sRawImu, nullptr), NAV_RET_ERR_ARG);
    EXPECT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_ERR_STATE);
}

TEST(NavigationSubsystemTest, ValidImuSampleProducesEstimatedState)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(100U);
    ts_TopicVehicleState sVehicleState {};

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);

    EXPECT_EQ(sContext.eLastDataStatus, NAV_STATUS_OK);
    EXPECT_TRUE(sVehicleState.bIsEstimated);
    EXPECT_EQ(sVehicleState.u32TimestampMs, 100U);
}

TEST(NavigationSubsystemTest, StaleTimestampIsDetectedAndBypassesUpdate)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(200U);
    ts_TopicVehicleState sVehicleState {};
    float f32YawAfterFirstStep;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    f32YawAfterFirstStep = sVehicleState.f32YawRad;

    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    EXPECT_EQ(sContext.eLastDataStatus, NAV_STATUS_STALE);
    EXPECT_FLOAT_EQ(sVehicleState.f32YawRad, f32YawAfterFirstStep);
}

TEST(NavigationSubsystemTest, ZeroVectorFaultIsDetected)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(300U);
    ts_TopicVehicleState sVehicleState {};

    sRawImu.sGyro.f32X = 0.0F;
    sRawImu.sGyro.f32Y = 0.0F;
    sRawImu.sGyro.f32Z = 0.0F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    EXPECT_EQ(sContext.eLastDataStatus, NAV_STATUS_ZERO_FAULT);
}

TEST(NavigationSubsystemTest, OutOfBoundsFaultIsDetected)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(400U);
    ts_TopicVehicleState sVehicleState {};

    sRawImu.sGyro.f32X = 40.0F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    EXPECT_EQ(sContext.eLastDataStatus, NAV_STATUS_OUT_OF_BOUNDS);
}

TEST(NavigationSubsystemTest, StuckFaultIsDetectedAfterConsecutiveBitEqualSamples)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(500U);
    ts_TopicVehicleState sVehicleState {};

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);

    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    sRawImu.u32TimestampMs = 510U;
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    sRawImu.u32TimestampMs = 520U;
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    sRawImu.u32TimestampMs = 530U;
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);

    EXPECT_EQ(sContext.eLastDataStatus, NAV_STATUS_STUCK_FAULT);
}

TEST(NavigationSubsystemTest, ComplementaryFilterUsesAccelWhenAlphaIsZero)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(600U);
    ts_TopicVehicleState sVehicleState {};

    sConfig.f32Alpha = 0.0F;
    sRawImu.sAccel.f32X = 0.0F;
    sRawImu.sAccel.f32Y = 1.0F;
    sRawImu.sAccel.f32Z = 0.0F;
    sRawImu.sGyro.f32X = 0.001F;
    sRawImu.sGyro.f32Y = 0.001F;
    sRawImu.sGyro.f32Z = 0.001F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);

    EXPECT_NEAR(sVehicleState.f32RollRad, 1.570796F, 1.0e-3F);
    EXPECT_NEAR(sVehicleState.f32PitchRad, 0.0F, 1.0e-4F);
}

TEST(NavigationSubsystemTest, YawDeadReckoningIntegratesGyroZ)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(700U);
    ts_TopicVehicleState sVehicleState {};

    sConfig.f32Alpha = 1.0F;
    sRawImu.sGyro.f32Z = 1.0F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    EXPECT_NEAR(sVehicleState.f32YawRad, 0.01F, 1.0e-6F);
}

TEST(NavigationSubsystemTest, InitSeedsInitialAttitudeWhenConfigRequestsIt)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();

    sConfig.sInitialAttitude.f32RollRad = -0.10F;
    sConfig.sInitialAttitude.f32PitchRad = 0.20F;
    sConfig.sInitialAttitude.f32YawRad = kTwoPiRad + 0.30F;
    sConfig.sInitialAttitude.u8IsValid = 1U;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    EXPECT_NEAR(sContext.sEstimatedState.f32RollRad, kTwoPiRad - 0.10F, 1.0e-6F);
    EXPECT_NEAR(sContext.sEstimatedState.f32PitchRad, 0.20F, 1.0e-6F);
    EXPECT_NEAR(sContext.sEstimatedState.f32YawRad, 0.30F, 1.0e-6F);
    EXPECT_TRUE(sContext.sEstimatedState.bIsEstimated);
}

TEST(NavigationSubsystemTest, YawIntegrationWrapsToZeroToTwoPiRange)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(710U);
    ts_TopicVehicleState sVehicleState {};

    sConfig.f32Alpha = 1.0F;
    sConfig.sInitialAttitude.f32RollRad = 0.0F;
    sConfig.sInitialAttitude.f32PitchRad = 0.0F;
    sConfig.sInitialAttitude.f32YawRad = kTwoPiRad - 0.002F;
    sConfig.sInitialAttitude.u8IsValid = 1U;
    sRawImu.sGyro.f32X = 0.0F;
    sRawImu.sGyro.f32Y = 0.0F;
    sRawImu.sGyro.f32Z = 1.0F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    EXPECT_NEAR(sVehicleState.f32YawRad, 0.008F, 1.0e-6F);
}

TEST(NavigationSubsystemTest, IntegrationPathReadsImuAndPublishesVehicleState)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(800U);
    ts_TopicRawImu sReadRawImu {};
    ts_TopicVehicleState sVehicleState {};
    ts_TopicVehicleState sReadVehicleState {};

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    Gds_ResetRawImu();
    Gds_ResetVehicleState();

    ASSERT_EQ(Gds_PublishRawImu(&sRawImu), GDS_OK);
    ASSERT_EQ(Gds_ReadRawImu(&sReadRawImu), GDS_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sReadRawImu, &sVehicleState), NAV_RET_OK);
    ASSERT_EQ(Gds_PublishVehicleState(&sVehicleState), GDS_OK);
    ASSERT_EQ(Gds_ReadVehicleState(&sReadVehicleState), GDS_OK);

    EXPECT_TRUE(sReadVehicleState.bIsEstimated);
    EXPECT_EQ(sReadVehicleState.u32TimestampMs, 800U);
}
