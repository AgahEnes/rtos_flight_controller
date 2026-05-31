#include "gtest/gtest.h"

extern "C" {
#include "flight_control.h"
#include "global_data_space.h"
}

namespace {

ts_TopicRawImu CreateStableImuSample(uint32_t u32TimestampMs)
{
    ts_TopicRawImu sSample {};

    sSample.sAccel.f32X = 0.0F;
    sSample.sAccel.f32Y = 0.0F;
    sSample.sAccel.f32Z = 9.81F;
    sSample.sGyro.f32X = 0.0F;
    sSample.sGyro.f32Y = 0.0F;
    sSample.sGyro.f32Z = 0.0F;
    sSample.u32TimestampMs = u32TimestampMs;
    sSample.bIsValid = true;

    return sSample;
}

void PublishStableVehicleState(uint32_t u32TimestampMs)
{
    ts_TopicVehicleState sVehicleState {};

    sVehicleState.u32TimestampMs = u32TimestampMs;
    sVehicleState.bIsEstimated = true;
    ASSERT_EQ(Gds_PublishVehicleState(&sVehicleState), GDS_OK);
}

void PublishImuAndRunStep(ts_FlightControlContext *psContext, const ts_TopicRawImu *psSample)
{
    ASSERT_NE(psContext, nullptr);
    ASSERT_NE(psSample, nullptr);
    ASSERT_EQ(Gds_PublishRawImu(psSample), GDS_OK);
    PublishStableVehicleState(psSample->u32TimestampMs);
    psContext->u32ModuleTimestampMs = psSample->u32TimestampMs;
    ASSERT_EQ(FlightControl_Step(psContext), FLIGHT_CONTROL_OK);
}

}  // namespace

TEST(FlightControlTest, InitRejectsInvalidArguments)
{
    ts_FlightControlContext sContext {};
    ts_FlightControlConfig sConfig {};

    FlightControl_InitDefaultConfig(&sConfig);
    EXPECT_EQ(FlightControl_Init(nullptr, &sConfig), FLIGHT_CONTROL_ERR_ARG);
    EXPECT_EQ(FlightControl_Init(&sContext, nullptr), FLIGHT_CONTROL_ERR_ARG);
}

TEST(FlightControlTest, ConsecutiveInvalidImuDuringPbitMovesToFailsafe)
{
    ts_FlightControlContext sContext {};
    ts_FlightControlConfig sConfig {};
    ts_TopicRawImu sInvalidImu {};

    FlightControl_InitDefaultConfig(&sConfig);
    ASSERT_EQ(FlightControl_Init(&sContext, &sConfig), FLIGHT_CONTROL_OK);

    Gds_ResetRawImu();
    Gds_ResetVehicleState();
    Gds_ResetImuCalibration();
    Gds_ResetNavCommand();

    sInvalidImu.u32TimestampMs = 10U;
    sInvalidImu.bIsValid = false;
    PublishImuAndRunStep(&sContext, &sInvalidImu);
    EXPECT_EQ(sContext.eMode, FLIGHT_MODE_PREFLIGHT);
    sInvalidImu.u32TimestampMs = 20U;
    PublishImuAndRunStep(&sContext, &sInvalidImu);

    EXPECT_EQ(sContext.eMode, FLIGHT_MODE_FAILSAFE);
}

TEST(FlightControlTest, FullNominalPreflightPublishesCalibrationThenTransitionsToBoost)
{
    ts_FlightControlContext sContext {};
    ts_FlightControlConfig sConfig {};
    ts_TopicRawImu sSample {};
    ts_TopicImuCalibration sCalibration {};
    uint32_t u32Cycle;
    uint32_t u32TimestampMs = 100U;
    uint32_t u32TotalNominalCycles;
    const uint32_t u32MaxNominalCyclesMargin = 5U;

    FlightControl_InitDefaultConfig(&sConfig);
    u32TotalNominalCycles = sConfig.u32PbitPassCycles +
                            sConfig.u32CalibrationCycles +
                            1U +
                            sConfig.u32PostCalibrationWaitCycles +
                            sConfig.u32SettleMinCycles;
    ASSERT_EQ(FlightControl_Init(&sContext, &sConfig), FLIGHT_CONTROL_OK);

    Gds_ResetRawImu();
    Gds_ResetVehicleState();
    Gds_ResetImuCalibration();
    Gds_ResetNavCommand();

    for (u32Cycle = 0U;
         (u32Cycle < (u32TotalNominalCycles + u32MaxNominalCyclesMargin)) &&
         (sContext.eMode != FLIGHT_MODE_READY_FOR_IGNITION);
         u32Cycle++)
    {
        sSample = CreateStableImuSample(u32TimestampMs);
        PublishImuAndRunStep(&sContext, &sSample);
        u32TimestampMs += 10U;
    }

    ASSERT_EQ(Gds_ReadImuCalibration(&sCalibration), GDS_OK);
    EXPECT_TRUE(sCalibration.bIsValid);
    EXPECT_EQ(sCalibration.u32UpdateCounter, 1U);
    EXPECT_NEAR(sCalibration.sGyroBiasRadS.f32X, 0.0F, 1.0e-6F);
    EXPECT_NEAR(sCalibration.sGyroBiasRadS.f32Y, 0.0F, 1.0e-6F);
    EXPECT_NEAR(sCalibration.sGyroBiasRadS.f32Z, 0.0F, 1.0e-6F);
    EXPECT_EQ(sCalibration.sNavInitialAttitude.u8IsValid, 1U);
    EXPECT_NEAR(sCalibration.sNavInitialAttitude.f32RollRad, 0.0F, 1.0e-4F);
    EXPECT_NEAR(sCalibration.sNavInitialAttitude.f32PitchRad, 0.0F, 1.0e-4F);
    EXPECT_NEAR(sCalibration.sNavInitialAttitude.f32YawRad, 0.0F, 1.0e-6F);
    EXPECT_EQ(sContext.eMode, FLIGHT_MODE_READY_FOR_IGNITION);

    sSample = CreateStableImuSample(u32TimestampMs);
    sSample.sAccel.f32Z = 25.0F;
    PublishImuAndRunStep(&sContext, &sSample);
    EXPECT_EQ(sContext.eMode, FLIGHT_MODE_BOOST);
}

TEST(FlightControlTest, PreflightPublishesNavReinitAfterSuccessfulSettle)
{
    ts_FlightControlContext sContext {};
    ts_FlightControlConfig sConfig {};
    ts_TopicRawImu sSample {};
    ts_TopicNavCommand sCommand {};
    uint32_t u32TimestampMs = 200U;
    uint32_t u32Cycle;

    FlightControl_InitDefaultConfig(&sConfig);
    ASSERT_EQ(FlightControl_Init(&sContext, &sConfig), FLIGHT_CONTROL_OK);

    Gds_ResetRawImu();
    Gds_ResetVehicleState();
    Gds_ResetImuCalibration();
    Gds_ResetNavCommand();

    sSample = CreateStableImuSample(u32TimestampMs);
    PublishImuAndRunStep(&sContext, &sSample);
    ASSERT_EQ(Gds_ReadNavCommand(&sCommand), GDS_OK);
    EXPECT_EQ(sCommand.eCommand, NAV_CMD_NONE);
    EXPECT_EQ(sCommand.u32Sequence, 0U);

    for (u32Cycle = 1U;
         u32Cycle < (sConfig.u32PbitPassCycles +
                     sConfig.u32CalibrationCycles +
                     1U +
                     sConfig.u32PostCalibrationWaitCycles +
                     sConfig.u32SettleMinCycles);
         u32Cycle++)
    {
        u32TimestampMs += 10U;
        sSample = CreateStableImuSample(u32TimestampMs);
        PublishImuAndRunStep(&sContext, &sSample);
    }

    ASSERT_EQ(Gds_ReadNavCommand(&sCommand), GDS_OK);
    EXPECT_EQ(sCommand.eCommand, NAV_CMD_REINIT);
    EXPECT_EQ(sCommand.u32Sequence, 1U);
}
