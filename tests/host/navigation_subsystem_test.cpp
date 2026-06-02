#include "gtest/gtest.h"
#include <cmath>

extern "C" {
#include "navigation_subsystem.h"
#include "global_data_space.h"
}

namespace {
constexpr float kTwoPiRad = 6.28318530717958647692F;
constexpr float kPiRad = 3.14159265358979323846F;

struct Quaternion
{
    float f32q0;
    float f32q1;
    float f32q2;
    float f32q3;
};

float WrapMinusPiToPi(float f32AngleRad)
{
    while (f32AngleRad > kPiRad)
    {
        f32AngleRad -= kTwoPiRad;
    }

    while (f32AngleRad < (-kPiRad))
    {
        f32AngleRad += kTwoPiRad;
    }

    return f32AngleRad;
}

Quaternion EulerToQuaternion(float f32RollRad, float f32PitchRad, float f32YawRad)
{
    const float f32HalfRoll = 0.5F * f32RollRad;
    const float f32HalfPitch = 0.5F * f32PitchRad;
    const float f32HalfYaw = 0.5F * f32YawRad;
    const float f32Cr = cosf(f32HalfRoll);
    const float f32Sr = sinf(f32HalfRoll);
    const float f32Cp = cosf(f32HalfPitch);
    const float f32Sp = sinf(f32HalfPitch);
    const float f32Cy = cosf(f32HalfYaw);
    const float f32Sy = sinf(f32HalfYaw);

    Quaternion sQuaternion {};
    sQuaternion.f32q0 = (f32Cr * f32Cp * f32Cy) + (f32Sr * f32Sp * f32Sy);
    sQuaternion.f32q1 = (f32Sr * f32Cp * f32Cy) - (f32Cr * f32Sp * f32Sy);
    sQuaternion.f32q2 = (f32Cr * f32Sp * f32Cy) + (f32Sr * f32Cp * f32Sy);
    sQuaternion.f32q3 = (f32Cr * f32Cp * f32Sy) - (f32Sr * f32Sp * f32Cy);
    return sQuaternion;
}

float QuaternionNorm(const ts_NavQuaternion &sQuaternion)
{
    return sqrtf((sQuaternion.f32q0 * sQuaternion.f32q0) +
                 (sQuaternion.f32q1 * sQuaternion.f32q1) +
                 (sQuaternion.f32q2 * sQuaternion.f32q2) +
                 (sQuaternion.f32q3 * sQuaternion.f32q3));
}

float QuaternionSimilarityAbsDot(const ts_NavQuaternion &sLeft, const Quaternion &sRight)
{
    const float f32Dot = (sLeft.f32q0 * sRight.f32q0) +
                         (sLeft.f32q1 * sRight.f32q1) +
                         (sLeft.f32q2 * sRight.f32q2) +
                         (sLeft.f32q3 * sRight.f32q3);
    return fabsf(f32Dot);
}

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

TEST(NavigationSubsystemTest, MahonyFilterPureGyroWhenKpIsZero)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(600U);
    ts_TopicVehicleState sVehicleState {};

    sConfig.f32Alpha = 0.0F;
    sRawImu.sAccel.f32X = 0.0F;
    sRawImu.sAccel.f32Y = 0.0F;
    sRawImu.sAccel.f32Z = 9.81F;
    sRawImu.sGyro.f32X = 0.0F;
    sRawImu.sGyro.f32Y = 0.0F;
    sRawImu.sGyro.f32Z = 1.0F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);

    EXPECT_NEAR(sVehicleState.f32RollRad, 0.0F, 1.0e-5F);
    EXPECT_NEAR(sVehicleState.f32PitchRad, 0.0F, 1.0e-5F);
    EXPECT_NEAR(sVehicleState.f32YawRad, 0.01F, 1.0e-6F);
}

TEST(NavigationSubsystemTest, MahonyFilterConvergesNearVerticalPitch)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(740U);
    ts_TopicVehicleState sVehicleState {};

    sConfig.f32Alpha = 3.0F;
    sConfig.sInitialAttitude.f32RollRad = 0.0F;
    sConfig.sInitialAttitude.f32PitchRad = 1.20F;
    sConfig.sInitialAttitude.f32YawRad = 0.0F;
    sConfig.sInitialAttitude.u8IsValid = 1U;

    sRawImu.sAccel.f32X = -9.81F;
    sRawImu.sAccel.f32Y = 0.0F;
    sRawImu.sAccel.f32Z = 0.0F;
    sRawImu.sGyro.f32X = 0.001F;
    sRawImu.sGyro.f32Y = 0.001F;
    sRawImu.sGyro.f32Z = 0.001F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    for (uint32_t u32StepIdx = 0U; u32StepIdx < 200U; ++u32StepIdx)
    {
        sRawImu.u32TimestampMs = 740U + (u32StepIdx * 10U);
        ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    }

    EXPECT_TRUE(std::isfinite(sVehicleState.f32PitchRad));
    EXPECT_GT(sVehicleState.f32PitchRad, 1.2F);
    EXPECT_LT(sVehicleState.f32PitchRad, 1.95F);
}

TEST(NavigationSubsystemTest, YawDeadReckoningIntegratesGyroZ)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(700U);
    ts_TopicVehicleState sVehicleState {};

    sConfig.f32Alpha = 0.0F;
    sRawImu.sGyro.f32X = 0.0F;
    sRawImu.sGyro.f32Y = 0.0F;
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
    EXPECT_NEAR(sContext.sEstimatedState.f32RollRad, -0.10F, 1.0e-6F);
    EXPECT_NEAR(sContext.sEstimatedState.f32PitchRad, 0.20F, 1.0e-6F);
    EXPECT_NEAR(sContext.sEstimatedState.f32YawRad, 0.30F, 1.0e-6F);
    EXPECT_TRUE(sContext.sEstimatedState.bIsEstimated);
    EXPECT_NEAR(QuaternionNorm(sContext.sQuaternion), 1.0F, 1.0e-6F);
    {
        const Quaternion sExpectedQuat = EulerToQuaternion(-0.10F, 0.20F, 0.30F);
        EXPECT_GT(QuaternionSimilarityAbsDot(sContext.sQuaternion, sExpectedQuat), 1.0F - 1.0e-5F);
    }
}

TEST(NavigationSubsystemTest, MahonyFilterInvalidInitUsesIdentity)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();

    sConfig.sInitialAttitude.u8IsValid = 0U;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    EXPECT_FLOAT_EQ(sContext.sEstimatedState.f32RollRad, 0.0F);
    EXPECT_FLOAT_EQ(sContext.sEstimatedState.f32PitchRad, 0.0F);
    EXPECT_FLOAT_EQ(sContext.sEstimatedState.f32YawRad, 0.0F);
    EXPECT_FALSE(sContext.sEstimatedState.bIsEstimated);
    EXPECT_FLOAT_EQ(sContext.sQuaternion.f32q0, 1.0F);
    EXPECT_FLOAT_EQ(sContext.sQuaternion.f32q1, 0.0F);
    EXPECT_FLOAT_EQ(sContext.sQuaternion.f32q2, 0.0F);
    EXPECT_FLOAT_EQ(sContext.sQuaternion.f32q3, 0.0F);
}

TEST(NavigationSubsystemTest, MahonyFilterRejectsKpAboveMax)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();

    sConfig.f32Alpha = 10.1F;
    EXPECT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_ERR_ARG);
}

TEST(NavigationSubsystemTest, YawIntegrationWrapsToZeroToTwoPiRange)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(710U);
    ts_TopicVehicleState sVehicleState {};

    sConfig.f32Alpha = 0.0F;
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

TEST(NavigationSubsystemTest, MahonyFilterBoundaryRemainsContinuousAcrossZeroToTwoPi)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(720U);
    ts_TopicVehicleState sVehicleState {};
    float f32RollError;

    sConfig.f32Alpha = 3.0F;
    sConfig.sInitialAttitude.f32RollRad = kTwoPiRad - 0.010F;
    sConfig.sInitialAttitude.f32PitchRad = 0.0F;
    sConfig.sInitialAttitude.f32YawRad = 0.0F;
    sConfig.sInitialAttitude.u8IsValid = 1U;

    sRawImu.sAccel.f32X = 0.0F;
    sRawImu.sAccel.f32Y = sinf(0.010F) * 9.81F;
    sRawImu.sAccel.f32Z = cosf(0.010F) * 9.81F;
    sRawImu.sGyro.f32X = 0.001F;
    sRawImu.sGyro.f32Y = 0.001F;
    sRawImu.sGyro.f32Z = 0.001F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    for (uint32_t u32StepIdx = 0U; u32StepIdx < 200U; ++u32StepIdx)
    {
        sRawImu.u32TimestampMs = 720U + (u32StepIdx * 10U);
        ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    }

    f32RollError = WrapMinusPiToPi(sVehicleState.f32RollRad - 0.010F);
    EXPECT_NEAR(f32RollError, 0.0F, 0.05F);
}

TEST(NavigationSubsystemTest, MahonyFilterHighKpTracksAccel)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(760U);
    ts_TopicVehicleState sVehicleState {};
    float f32RollError;
    float f32PitchError;

    sConfig.f32Alpha = 3.0F;
    sConfig.sInitialAttitude.f32RollRad = 0.40F;
    sConfig.sInitialAttitude.f32PitchRad = 0.30F;
    sConfig.sInitialAttitude.f32YawRad = 0.0F;
    sConfig.sInitialAttitude.u8IsValid = 1U;

    sRawImu.sAccel.f32X = 0.0F;
    sRawImu.sAccel.f32Y = 0.0F;
    sRawImu.sAccel.f32Z = 9.81F;
    sRawImu.sGyro.f32X = 0.001F;
    sRawImu.sGyro.f32Y = 0.001F;
    sRawImu.sGyro.f32Z = 0.001F;

    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    for (uint32_t u32StepIdx = 0U; u32StepIdx < 400U; ++u32StepIdx)
    {
        sRawImu.u32TimestampMs = 760U + (u32StepIdx * 10U);
        ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    }

    f32RollError = WrapMinusPiToPi(sVehicleState.f32RollRad);
    f32PitchError = WrapMinusPiToPi(sVehicleState.f32PitchRad);
    EXPECT_LT(fabsf(f32RollError), 0.40F);
    EXPECT_LT(fabsf(f32PitchError), 0.30F);
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

TEST(NavigationSubsystemTest, ResetFilterCommandIsEdgeTriggeredBySequence)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(1000U);
    ts_TopicVehicleState sVehicleState {};
    ts_TopicNavCommand sCommand {};

    Gds_ResetNavCommand();
    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    ASSERT_TRUE(sVehicleState.bIsEstimated);

    sCommand.eCommand = NAV_CMD_RESET_FILTER;
    sCommand.u32Sequence = 10U;
    sCommand.u32TimestampMs = 1000U;
    ASSERT_EQ(Gds_PublishNavCommand(&sCommand), GDS_OK);

    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    EXPECT_EQ(sContext.u32LastExecutedCmdSeq, 10U);
    EXPECT_EQ(sContext.eLastDataStatus, NAV_STATUS_STALE);
    EXPECT_FALSE(sVehicleState.bIsEstimated);

    sContext.sEstimatedState.bIsEstimated = true;
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    EXPECT_TRUE(sVehicleState.bIsEstimated);
    EXPECT_EQ(sContext.u32LastExecutedCmdSeq, 10U);
}

TEST(NavigationSubsystemTest, ReinitCommandClearsInternalTracking)
{
    ts_NavContext sContext {};
    ts_NavConfig sConfig = CreateDefaultConfig();
    ts_TopicRawImu sRawImu = CreateValidImuSample(1100U);
    ts_TopicVehicleState sVehicleState {};
    ts_TopicNavCommand sCommand {};
    ts_TopicImuCalibration sCalibration {};

    Gds_ResetNavCommand();
    ASSERT_EQ(Navigation_Init(&sContext, &sConfig), NAV_RET_OK);
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    ASSERT_TRUE(sContext.u8HasLastRawImu != 0U);
    ASSERT_TRUE(sContext.sEstimatedState.bIsEstimated);

    Gds_ResetImuCalibration();
    sCalibration.bIsValid = true;
    sCalibration.sNavInitialAttitude.f32RollRad = -0.40F;
    sCalibration.sNavInitialAttitude.f32PitchRad = 0.50F;
    sCalibration.sNavInitialAttitude.f32YawRad = kTwoPiRad + 0.60F;
    sCalibration.sNavInitialAttitude.u8IsValid = 1U;
    ASSERT_EQ(Gds_PublishImuCalibration(&sCalibration), GDS_OK);

    sCommand.eCommand = NAV_CMD_REINIT;
    sCommand.u32Sequence = 21U;
    sCommand.u32TimestampMs = 1100U;
    ASSERT_EQ(Gds_PublishNavCommand(&sCommand), GDS_OK);

    sRawImu.u32TimestampMs = 1110U;
    ASSERT_EQ(NavigationTask_Step(&sContext, &sRawImu, &sVehicleState), NAV_RET_OK);
    EXPECT_EQ(sContext.u32LastExecutedCmdSeq, 21U);
    EXPECT_EQ(sContext.u8StuckCycleCount, 0U);
    EXPECT_TRUE(sContext.u8HasLastRawImu != 0U);
    EXPECT_NEAR(sVehicleState.f32RollRad, -0.40F, 1.0e-6F);
    EXPECT_NEAR(sVehicleState.f32PitchRad, 0.50F, 1.0e-6F);
    EXPECT_NEAR(sVehicleState.f32YawRad, 0.60F, 1.0e-6F);
    EXPECT_TRUE(sVehicleState.bIsEstimated);
    EXPECT_NEAR(QuaternionNorm(sContext.sQuaternion), 1.0F, 1.0e-6F);
    {
        const Quaternion sExpectedQuat = EulerToQuaternion(-0.40F, 0.50F, 0.60F);
        EXPECT_GT(QuaternionSimilarityAbsDot(sContext.sQuaternion, sExpectedQuat), 1.0F - 1.0e-5F);
    }
}
