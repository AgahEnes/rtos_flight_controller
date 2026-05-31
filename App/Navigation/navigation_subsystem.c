#include "navigation_subsystem.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define NAV_PI_F      (3.14159265358979323846F)
#define NAV_TWO_PI_F  (2.0F * NAV_PI_F)

static float Navigation_prvAbsF32(float f32Value)
{
    return (f32Value < 0.0F) ? (-f32Value) : f32Value;
}

static float Navigation_prvWrapAngle0To2Pi(float f32AngleRad)
{
    if ((f32AngleRad >= 0.0F) && (f32AngleRad < NAV_TWO_PI_F))
    {
        return f32AngleRad;
    }

    if (f32AngleRad < 0.0F)
    {
        while (f32AngleRad < 0.0F)
        {
            f32AngleRad += NAV_TWO_PI_F;
        }
    }
    else
    {
        while (f32AngleRad >= NAV_TWO_PI_F)
        {
            f32AngleRad -= NAV_TWO_PI_F;
        }
    }

    return f32AngleRad;
}

static float Navigation_prvWrapAngleMinusPiToPi(float f32AngleRad)
{
    if ((f32AngleRad >= (-NAV_PI_F)) && (f32AngleRad <= NAV_PI_F))
    {
        return f32AngleRad;
    }

    if (f32AngleRad < (-NAV_PI_F))
    {
        while (f32AngleRad < (-NAV_PI_F))
        {
            f32AngleRad += NAV_TWO_PI_F;
        }
    }
    else
    {
        while (f32AngleRad > NAV_PI_F)
        {
            f32AngleRad -= NAV_TWO_PI_F;
        }
    }

    return f32AngleRad;
}

static float Navigation_prvComputePitchAccelFromAccel(const ts_Vector3d *psAccel)
{
    float f32PitchAccel;
    float f32Denom;

    if (psAccel == NULL)
    {
        return 0.0F;
    }

    f32Denom = sqrtf((psAccel->f32Y * psAccel->f32Y) + (psAccel->f32Z * psAccel->f32Z));
    f32PitchAccel = atan2f(-psAccel->f32X, f32Denom);

    if (psAccel->f32Z < 0.0F)
    {
        if (f32PitchAccel >= 0.0F)
        {
            f32PitchAccel = NAV_PI_F - f32PitchAccel;
        }
        else
        {
            f32PitchAccel = -NAV_PI_F - f32PitchAccel;
        }
    }

    return f32PitchAccel;
}

static uint8_t Navigation_prvIsVectorNearZero(const ts_Vector3d *psVector, float f32Epsilon)
{
    if (psVector == NULL)
    {
        return 0U;
    }

    if ((Navigation_prvAbsF32(psVector->f32X) <= f32Epsilon) &&
        (Navigation_prvAbsF32(psVector->f32Y) <= f32Epsilon) &&
        (Navigation_prvAbsF32(psVector->f32Z) <= f32Epsilon))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t Navigation_prvIsOutOfBounds(const ts_TopicRawImu *psRawImu)
{
    if (psRawImu == NULL)
    {
        return 0U;
    }

    if ((Navigation_prvAbsF32(psRawImu->sAccel.f32X) > NAV_ACCEL_MAX_MPS2) ||
        (Navigation_prvAbsF32(psRawImu->sAccel.f32Y) > NAV_ACCEL_MAX_MPS2) ||
        (Navigation_prvAbsF32(psRawImu->sAccel.f32Z) > NAV_ACCEL_MAX_MPS2) ||
        (Navigation_prvAbsF32(psRawImu->sGyro.f32X) > NAV_GYRO_MAX_RADS) ||
        (Navigation_prvAbsF32(psRawImu->sGyro.f32Y) > NAV_GYRO_MAX_RADS) ||
        (Navigation_prvAbsF32(psRawImu->sGyro.f32Z) > NAV_GYRO_MAX_RADS))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t Navigation_prvFloatBitsEqual(float f32Left, float f32Right)
{
    uint32_t u32LeftBits = 0U;
    uint32_t u32RightBits = 0U;

    (void)memcpy(&u32LeftBits, &f32Left, sizeof(u32LeftBits));
    (void)memcpy(&u32RightBits, &f32Right, sizeof(u32RightBits));

    return (u32LeftBits == u32RightBits) ? 1U : 0U;
}

static uint8_t Navigation_prvRawImuBitsEqual(const ts_TopicRawImu *psLeft, const ts_TopicRawImu *psRight)
{
    if ((psLeft == NULL) || (psRight == NULL))
    {
        return 0U;
    }

    if ((Navigation_prvFloatBitsEqual(psLeft->sAccel.f32X, psRight->sAccel.f32X) == 1U) &&
        (Navigation_prvFloatBitsEqual(psLeft->sAccel.f32Y, psRight->sAccel.f32Y) == 1U) &&
        (Navigation_prvFloatBitsEqual(psLeft->sAccel.f32Z, psRight->sAccel.f32Z) == 1U) &&
        (Navigation_prvFloatBitsEqual(psLeft->sGyro.f32X, psRight->sGyro.f32X) == 1U) &&
        (Navigation_prvFloatBitsEqual(psLeft->sGyro.f32Y, psRight->sGyro.f32Y) == 1U) &&
        (Navigation_prvFloatBitsEqual(psLeft->sGyro.f32Z, psRight->sGyro.f32Z) == 1U))
    {
        return 1U;
    }

    return 0U;
}

static void Navigation_prvUpdateStuckCounter(ts_NavContext *psContext, const ts_TopicRawImu *psRawImu)
{
    uint8_t bIsSameAsPrevious = 0U;

    if ((psContext == NULL) || (psRawImu == NULL))
    {
        return;
    }

    if (psContext->u8HasLastRawImu != 0U)
    {
        bIsSameAsPrevious = Navigation_prvRawImuBitsEqual(&psContext->sLastRawImu, psRawImu);
    }

    if (bIsSameAsPrevious == 1U)
    {
        if (psContext->u8StuckCycleCount < UINT8_MAX)
        {
            psContext->u8StuckCycleCount++;
        }
    }
    else
    {
        psContext->u8StuckCycleCount = 0U;
    }

    psContext->sLastRawImu = *psRawImu;
    psContext->u8HasLastRawImu = 1U;
}

static void Navigation_prvEstimateComplementary(ts_NavContext *psContext, const ts_TopicRawImu *psRawImu)
{
    float f32RollAccel;
    float f32PitchAccel;
    float f32RollGyroIntegrated;
    float f32PitchGyroIntegrated;
    float f32YawGyroIntegrated;
    float f32RollInnovation;
    float f32PitchInnovation;
    float f32OneMinusAlpha;
    float f32RollEstimated;
    float f32PitchEstimated;
    float f32YawEstimated;

    if ((psContext == NULL) || (psRawImu == NULL))
    {
        return;
    }

    f32RollAccel = atan2f(psRawImu->sAccel.f32Y, psRawImu->sAccel.f32Z);
    f32PitchAccel = Navigation_prvComputePitchAccelFromAccel(&psRawImu->sAccel);

    f32RollGyroIntegrated = psContext->sEstimatedState.f32RollRad + (psRawImu->sGyro.f32X * psContext->sConfig.f32DtS);
    f32PitchGyroIntegrated = psContext->sEstimatedState.f32PitchRad + (psRawImu->sGyro.f32Y * psContext->sConfig.f32DtS);
    f32YawGyroIntegrated = psContext->sEstimatedState.f32YawRad + (psRawImu->sGyro.f32Z * psContext->sConfig.f32DtS);

    f32RollInnovation = Navigation_prvWrapAngleMinusPiToPi(f32RollAccel - f32RollGyroIntegrated);
    f32PitchInnovation = Navigation_prvWrapAngleMinusPiToPi(f32PitchAccel - f32PitchGyroIntegrated);
    f32OneMinusAlpha = 1.0F - psContext->sConfig.f32Alpha;

    f32RollEstimated = f32RollGyroIntegrated + (f32OneMinusAlpha * f32RollInnovation);
    f32PitchEstimated = f32PitchGyroIntegrated + (f32OneMinusAlpha * f32PitchInnovation);
    f32YawEstimated = f32YawGyroIntegrated;

    psContext->sEstimatedState.f32RollRad = Navigation_prvWrapAngle0To2Pi(f32RollEstimated);
    psContext->sEstimatedState.f32PitchRad = Navigation_prvWrapAngle0To2Pi(f32PitchEstimated);
    psContext->sEstimatedState.f32YawRad = Navigation_prvWrapAngle0To2Pi(f32YawEstimated);
    psContext->sEstimatedState.f32RollRateRadS = psRawImu->sGyro.f32X;
    psContext->sEstimatedState.f32PitchRateRadS = psRawImu->sGyro.f32Y;
    psContext->sEstimatedState.f32YawRateRadS = psRawImu->sGyro.f32Z;
    psContext->sEstimatedState.u32TimestampMs = psRawImu->u32TimestampMs;
    psContext->sEstimatedState.bIsEstimated = true;
}

static void Navigation_prvApplyInitialAttitude(ts_NavContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->sEstimatedState.f32RollRateRadS = 0.0F;
    psContext->sEstimatedState.f32PitchRateRadS = 0.0F;
    psContext->sEstimatedState.f32YawRateRadS = 0.0F;
    psContext->sEstimatedState.u32TimestampMs = 0U;
    psContext->sEstimatedState.bIsEstimated = false;
    if (psContext->sConfig.sInitialAttitude.u8IsValid == 1U)
    {
        psContext->sEstimatedState.f32RollRad =
            Navigation_prvWrapAngle0To2Pi(psContext->sConfig.sInitialAttitude.f32RollRad);
        psContext->sEstimatedState.f32PitchRad =
            Navigation_prvWrapAngle0To2Pi(psContext->sConfig.sInitialAttitude.f32PitchRad);
        psContext->sEstimatedState.f32YawRad =
            Navigation_prvWrapAngle0To2Pi(psContext->sConfig.sInitialAttitude.f32YawRad);
    }
    else
    {
        psContext->sEstimatedState.f32RollRad = 180.0F;
        psContext->sEstimatedState.f32PitchRad = 180.0F;
        psContext->sEstimatedState.f32YawRad = 180.0F;
    }
}

static void Navigation_prvResetFilter(ts_NavContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    Navigation_prvApplyInitialAttitude(psContext);
    psContext->eLastDataStatus = NAV_STATUS_STALE;
}

static void Navigation_prvReinit(ts_NavContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    Navigation_prvApplyInitialAttitude(psContext);
    (void)memset(&psContext->sLastRawImu, 0, sizeof(psContext->sLastRawImu));
    psContext->u32LastProcessedTimestampMs = 0U;
    psContext->u8HasLastRawImu = 0U;
    psContext->u8StuckCycleCount = 0U;
    psContext->eLastDataStatus = NAV_STATUS_STALE;
}

static void Navigation_prvApplyCalibrationInitialAttitude(ts_NavContext *psContext)
{
    ts_TopicImuCalibration sCalibration;

    if (psContext == NULL)
    {
        return;
    }

    (void)memset(&sCalibration, 0, sizeof(sCalibration));
    if (Gds_ReadImuCalibration(&sCalibration) != GDS_OK)
    {
        return;
    }

    if ((sCalibration.bIsValid == true) && (sCalibration.sNavInitialAttitude.u8IsValid == 1U))
    {
        psContext->sConfig.sInitialAttitude.f32RollRad = sCalibration.sNavInitialAttitude.f32RollRad;
        psContext->sConfig.sInitialAttitude.f32PitchRad = sCalibration.sNavInitialAttitude.f32PitchRad;
        psContext->sConfig.sInitialAttitude.f32YawRad = sCalibration.sNavInitialAttitude.f32YawRad;
        psContext->sConfig.sInitialAttitude.u8IsValid = 1U;
    }
}

static uint8_t Navigation_prvHandleNavCommand(ts_NavContext *psContext)
{
    ts_TopicNavCommand sCommand;
    uint8_t u8CommandApplied = 0U;

    if (psContext == NULL)
    {
        return 0U;
    }

    if (Gds_ReadNavCommand(&sCommand) != GDS_OK)
    {
        return 0U;
    }

    if (sCommand.u32Sequence == psContext->u32LastExecutedCmdSeq)
    {
        return 0U;
    }

    switch (sCommand.eCommand)
    {
        case NAV_CMD_RESET_FILTER:
        {
            Navigation_prvApplyCalibrationInitialAttitude(psContext);
            Navigation_prvResetFilter(psContext);
            u8CommandApplied = 1U;
            break;
        }
        case NAV_CMD_REINIT:
        {
            Navigation_prvApplyCalibrationInitialAttitude(psContext);
            Navigation_prvReinit(psContext);
            u8CommandApplied = 1U;
            break;
        }
        case NAV_CMD_NONE:
        default:
        {
            break;
        }
    }

    psContext->u32LastExecutedCmdSeq = sCommand.u32Sequence;
    return u8CommandApplied;
}

static te_NavDataStatus Nav_ValidateImuInput(const ts_NavContext *psContext, const ts_TopicRawImu *psNewImu)
{
    uint8_t bIsSameAsPrevious;

    if ((psContext == NULL) || (psNewImu == NULL))
    {
        return NAV_STATUS_OUT_OF_BOUNDS;
    }

    if ((psContext->sEstimatedState.bIsEstimated == true) &&
        (psNewImu->u32TimestampMs == psContext->u32LastProcessedTimestampMs))
    {
        return NAV_STATUS_STALE;
    }

    if ((Navigation_prvIsVectorNearZero(&psNewImu->sAccel, psContext->sConfig.f32ZeroEpsilon) == 1U) ||
        (Navigation_prvIsVectorNearZero(&psNewImu->sGyro, psContext->sConfig.f32ZeroEpsilon) == 1U))
    {
        return NAV_STATUS_ZERO_FAULT;
    }

    bIsSameAsPrevious = 0U;
    if (psContext->u8HasLastRawImu != 0U)
    {
        bIsSameAsPrevious = Navigation_prvRawImuBitsEqual(&psContext->sLastRawImu, psNewImu);
    }
    if ((bIsSameAsPrevious == 1U) &&
        ((uint32_t)psContext->u8StuckCycleCount + 1U >= (uint32_t)psContext->sConfig.u8StuckThresholdCycles))
    {
        return NAV_STATUS_STUCK_FAULT;
    }

    if (Navigation_prvIsOutOfBounds(psNewImu) == 1U)
    {
        return NAV_STATUS_OUT_OF_BOUNDS;
    }

    return NAV_STATUS_OK;
}

te_NavigationRetCode Navigation_Init(ts_NavContext *psContext, const ts_NavConfig *psConfig)
{
    if ((psContext == NULL) || (psConfig == NULL))
    {
        return NAV_RET_ERR_ARG;
    }

    if ((psConfig->f32DtS <= 0.0F) ||
        (psConfig->f32Alpha < 0.0F) ||
        (psConfig->f32Alpha > 1.0F) ||
        (psConfig->f32ZeroEpsilon < 0.0F) ||
        (psConfig->u8StuckThresholdCycles == 0U) ||
        ((psConfig->sInitialAttitude.u8IsValid != 0U) && (psConfig->sInitialAttitude.u8IsValid != 1U)))
    {
        return NAV_RET_ERR_ARG;
    }

    (void)memset(psContext, 0, sizeof(*psContext));
    psContext->sConfig = *psConfig;
    if (psConfig->sInitialAttitude.u8IsValid == 1U)
    {
        psContext->sEstimatedState.f32RollRad =
            Navigation_prvWrapAngle0To2Pi(psConfig->sInitialAttitude.f32RollRad);
        psContext->sEstimatedState.f32PitchRad =
            Navigation_prvWrapAngle0To2Pi(psConfig->sInitialAttitude.f32PitchRad);
        psContext->sEstimatedState.f32YawRad =
            Navigation_prvWrapAngle0To2Pi(psConfig->sInitialAttitude.f32YawRad);
        psContext->sEstimatedState.bIsEstimated = true;
    }
    psContext->eLastDataStatus = NAV_STATUS_STALE;
    psContext->u8IsInitialized = 1U;

    return NAV_RET_OK;
}

te_NavigationRetCode NavigationTask_Step(ts_NavContext *psContext,
                                     const ts_TopicRawImu *psRawImu,
                                     ts_TopicVehicleState *psVehicleState)
{
    te_NavDataStatus eDataStatus;
    uint8_t u8CommandApplied;

    if ((psContext == NULL) || (psRawImu == NULL) || (psVehicleState == NULL))
    {
        return NAV_RET_ERR_ARG;
    }

    if (psContext->u8IsInitialized == 0U)
    {
        return NAV_RET_ERR_STATE;
    }

    u8CommandApplied = Navigation_prvHandleNavCommand(psContext);
    if (u8CommandApplied == 0U)
    {
        eDataStatus = Nav_ValidateImuInput(psContext, psRawImu);
        psContext->eLastDataStatus = eDataStatus;

        if (eDataStatus == NAV_STATUS_OK)
        {
            Navigation_prvEstimateComplementary(psContext, psRawImu);
            psContext->u32LastProcessedTimestampMs = psRawImu->u32TimestampMs;
        }
    }

    Navigation_prvUpdateStuckCounter(psContext, psRawImu);
    *psVehicleState = psContext->sEstimatedState;

    return NAV_RET_OK;
}
