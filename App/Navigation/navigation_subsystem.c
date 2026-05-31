#include "navigation_subsystem.h"

#include <math.h>
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

    if ((psContext == NULL) || (psRawImu == NULL))
    {
        return;
    }

    f32RollAccel = atan2f(psRawImu->sAccel.f32Y, psRawImu->sAccel.f32Z);
    f32PitchAccel = atan2f(-psRawImu->sAccel.f32X,
                           sqrtf((psRawImu->sAccel.f32Y * psRawImu->sAccel.f32Y) +
                                 (psRawImu->sAccel.f32Z * psRawImu->sAccel.f32Z)));

    f32RollGyroIntegrated = psContext->sEstimatedState.f32RollRad + (psRawImu->sGyro.f32X * psContext->sConfig.f32DtS);
    f32PitchGyroIntegrated = psContext->sEstimatedState.f32PitchRad + (psRawImu->sGyro.f32Y * psContext->sConfig.f32DtS);

    psContext->sEstimatedState.f32RollRad =
        (psContext->sConfig.f32Alpha * f32RollGyroIntegrated) +
        ((1.0F - psContext->sConfig.f32Alpha) * f32RollAccel);
    psContext->sEstimatedState.f32PitchRad =
        (psContext->sConfig.f32Alpha * f32PitchGyroIntegrated) +
        ((1.0F - psContext->sConfig.f32Alpha) * f32PitchAccel);
    psContext->sEstimatedState.f32YawRad += (psRawImu->sGyro.f32Z * psContext->sConfig.f32DtS);
    psContext->sEstimatedState.f32RollRad =
        Navigation_prvWrapAngle0To2Pi(psContext->sEstimatedState.f32RollRad);
    psContext->sEstimatedState.f32PitchRad =
        Navigation_prvWrapAngle0To2Pi(psContext->sEstimatedState.f32PitchRad);
    psContext->sEstimatedState.f32YawRad =
        Navigation_prvWrapAngle0To2Pi(psContext->sEstimatedState.f32YawRad);

    psContext->sEstimatedState.f32RollRateRadS = psRawImu->sGyro.f32X;
    psContext->sEstimatedState.f32PitchRateRadS = psRawImu->sGyro.f32Y;
    psContext->sEstimatedState.f32YawRateRadS = psRawImu->sGyro.f32Z;
    psContext->sEstimatedState.u32TimestampMs = psRawImu->u32TimestampMs;
    psContext->sEstimatedState.bIsEstimated = true;
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

    if ((psContext == NULL) || (psRawImu == NULL) || (psVehicleState == NULL))
    {
        return NAV_RET_ERR_ARG;
    }

    if (psContext->u8IsInitialized == 0U)
    {
        return NAV_RET_ERR_STATE;
    }

    eDataStatus = Nav_ValidateImuInput(psContext, psRawImu);
    psContext->eLastDataStatus = eDataStatus;

    if (eDataStatus == NAV_STATUS_OK)
    {
        Navigation_prvEstimateComplementary(psContext, psRawImu);
        psContext->u32LastProcessedTimestampMs = psRawImu->u32TimestampMs;
    }

    Navigation_prvUpdateStuckCounter(psContext, psRawImu);
    *psVehicleState = psContext->sEstimatedState;

    return NAV_RET_OK;
}
