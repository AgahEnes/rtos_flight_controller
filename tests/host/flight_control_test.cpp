#include "gtest/gtest.h"

extern "C" {
#include "flight_control.h"
#include "global_data_space.h"
}

namespace {

constexpr float k_f32FinLimitRad = 0.78539816F;

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

ts_TopicVehicleState CreateVehicleStateSample(uint32_t u32TimestampMs,
                                              float f32RollRad,
                                              float f32PitchRad,
                                              float f32YawRad,
                                              float f32RollRateRadS,
                                              float f32PitchRateRadS,
                                              float f32YawRateRadS)
{
    ts_TopicVehicleState sVehicleState {};

    sVehicleState.f32RollRad = f32RollRad;
    sVehicleState.f32PitchRad = f32PitchRad;
    sVehicleState.f32YawRad = f32YawRad;
    sVehicleState.f32RollRateRadS = f32RollRateRadS;
    sVehicleState.f32PitchRateRadS = f32PitchRateRadS;
    sVehicleState.f32YawRateRadS = f32YawRateRadS;
    sVehicleState.u32TimestampMs = u32TimestampMs;
    sVehicleState.bIsEstimated = true;

    return sVehicleState;
}

void PublishStableVehicleState(uint32_t u32TimestampMs)
{
    ts_TopicVehicleState sVehicleState = CreateVehicleStateSample(u32TimestampMs,
                                                                  0.0F,
                                                                  0.0F,
                                                                  0.0F,
                                                                  0.0F,
                                                                  0.0F,
                                                                  0.0F);
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

void PublishImuVehicleAndRunStep(ts_FlightControlContext *psContext,
                                 const ts_TopicRawImu *psSample,
                                 const ts_TopicVehicleState *psVehicleState)
{
    ASSERT_NE(psContext, nullptr);
    ASSERT_NE(psSample, nullptr);
    ASSERT_NE(psVehicleState, nullptr);
    ASSERT_EQ(Gds_PublishRawImu(psSample), GDS_OK);
    ASSERT_EQ(Gds_PublishVehicleState(psVehicleState), GDS_OK);
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
    Gds_ResetActuatorCmd();

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
    Gds_ResetActuatorCmd();

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
    Gds_ResetActuatorCmd();

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

TEST(FlightControlTest, BoostTransitionsToStabilizeAfterConfiguredDelay)
{
    ts_FlightControlContext sContext {};
    ts_FlightControlConfig sConfig {};
    ts_TopicRawImu sSample = CreateStableImuSample(1000U);

    FlightControl_InitDefaultConfig(&sConfig);
    ASSERT_EQ(FlightControl_Init(&sContext, &sConfig), FLIGHT_CONTROL_OK);

    Gds_ResetRawImu();
    Gds_ResetVehicleState();
    Gds_ResetActuatorCmd();

    sContext.eMode = FLIGHT_MODE_BOOST;
    PublishImuAndRunStep(&sContext, &sSample);
    EXPECT_EQ(sContext.eMode, FLIGHT_MODE_STABILIZE);
}

TEST(FlightControlTest, StabilizePublishesMixedAndSaturatedFinCommands)
{
    ts_FlightControlContext sContext {};
    ts_FlightControlConfig sConfig {};
    ts_TopicRawImu sSample = CreateStableImuSample(2000U);
    ts_TopicVehicleState sVehicleState = CreateVehicleStateSample(2000U,
                                                                  0.2F,
                                                                  -0.1F,
                                                                  0.05F,
                                                                  0.4F,
                                                                  -0.2F,
                                                                  0.1F);
    ts_TopicActuatorCmd sCommand {};

    FlightControl_InitDefaultConfig(&sConfig);
    ASSERT_EQ(FlightControl_Init(&sContext, &sConfig), FLIGHT_CONTROL_OK);

    Gds_ResetRawImu();
    Gds_ResetVehicleState();
    Gds_ResetActuatorCmd();

    sContext.eMode = FLIGHT_MODE_STABILIZE;
    PublishImuVehicleAndRunStep(&sContext, &sSample, &sVehicleState);

    ASSERT_EQ(Gds_ReadActuatorCmd(&sCommand), GDS_OK);
    EXPECT_TRUE(sCommand.bIsActive);
    EXPECT_EQ(sCommand.u32TimestampMs, sSample.u32TimestampMs);
    EXPECT_EQ(sCommand.u32Sequence, 1U);
    EXPECT_NEAR(sCommand.f32FinAngleRad[0], -0.2124F, 1.0e-4F);
    EXPECT_NEAR(sCommand.f32FinAngleRad[1], -0.63845F, 1.0e-4F);
    EXPECT_NEAR(sCommand.f32FinAngleRad[2], -k_f32FinLimitRad, 1.0e-5F);
    EXPECT_NEAR(sCommand.f32FinAngleRad[3], -0.42795F, 1.0e-4F);
}

TEST(FlightControlTest, StabilizeIntegralTermIsClampedForAntiWindup)
{
    ts_FlightControlContext sContext {};
    ts_FlightControlConfig sConfig {};
    ts_TopicRawImu sSample = CreateStableImuSample(3000U);
    ts_TopicVehicleState sVehicleState = CreateVehicleStateSample(3000U,
                                                                  0.0F,
                                                                  -1.0F,
                                                                  0.0F,
                                                                  0.0F,
                                                                  0.0F,
                                                                  0.0F);
    uint32_t u32Cycle;

    FlightControl_InitDefaultConfig(&sConfig);
    ASSERT_EQ(FlightControl_Init(&sContext, &sConfig), FLIGHT_CONTROL_OK);

    Gds_ResetRawImu();
    Gds_ResetVehicleState();
    Gds_ResetActuatorCmd();

    sContext.eMode = FLIGHT_MODE_STABILIZE;
    for (u32Cycle = 0U; u32Cycle < 100U; u32Cycle++)
    {
        sSample.u32TimestampMs = 3000U + (u32Cycle * 10U);
        sVehicleState.u32TimestampMs = sSample.u32TimestampMs;
        PublishImuVehicleAndRunStep(&sContext, &sSample, &sVehicleState);
    }

    EXPECT_NEAR(sContext.sPitchPid.f32Integral, 0.30F, 1.0e-4F);
    EXPECT_LE(sContext.sPitchPid.f32Integral, 0.30F);
}
