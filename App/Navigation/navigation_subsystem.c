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

    while (f32AngleRad > NAV_PI_F)
    {
        f32AngleRad -= NAV_TWO_PI_F;
    }

    while (f32AngleRad < (-NAV_PI_F))
    {
        f32AngleRad += NAV_TWO_PI_F;
    }

    return f32AngleRad;
}

static void Navigation_prvSetIdentityQuaternion(ts_NavQuaternion *psQuaternion)
{
    if (psQuaternion == NULL)
    {
        return;
    }

    psQuaternion->f32q0 = 1.0F;
    psQuaternion->f32q1 = 0.0F;
    psQuaternion->f32q2 = 0.0F;
    psQuaternion->f32q3 = 0.0F;
}

static void Navigation_prvNormalizeQuaternion(ts_NavQuaternion *psQuaternion, float f32ZeroEpsilon)
{
    float f32Norm;

    if (psQuaternion == NULL)
    {
        return;
    }

    f32Norm = sqrtf((psQuaternion->f32q0 * psQuaternion->f32q0) +
                    (psQuaternion->f32q1 * psQuaternion->f32q1) +
                    (psQuaternion->f32q2 * psQuaternion->f32q2) +
                    (psQuaternion->f32q3 * psQuaternion->f32q3));

    if (f32Norm < f32ZeroEpsilon)
    {
        Navigation_prvSetIdentityQuaternion(psQuaternion);
        return;
    }

    psQuaternion->f32q0 /= f32Norm;
    psQuaternion->f32q1 /= f32Norm;
    psQuaternion->f32q2 /= f32Norm;
    psQuaternion->f32q3 /= f32Norm;
}

static void Navigation_prvEulerToQuaternion(float f32RollRad,
                                            float f32PitchRad,
                                            float f32YawRad,
                                            ts_NavQuaternion *psQuaternion)
{
    float f32HalfRoll;
    float f32HalfPitch;
    float f32HalfYaw;
    float f32Cr;
    float f32Sr;
    float f32Cp;
    float f32Sp;
    float f32Cy;
    float f32Sy;

    if (psQuaternion == NULL)
    {
        return;
    }

    f32HalfRoll = 0.5F * f32RollRad;
    f32HalfPitch = 0.5F * f32PitchRad;
    f32HalfYaw = 0.5F * f32YawRad;

    f32Cr = cosf(f32HalfRoll);
    f32Sr = sinf(f32HalfRoll);
    f32Cp = cosf(f32HalfPitch);
    f32Sp = sinf(f32HalfPitch);
    f32Cy = cosf(f32HalfYaw);
    f32Sy = sinf(f32HalfYaw);

    psQuaternion->f32q0 = (f32Cr * f32Cp * f32Cy) + (f32Sr * f32Sp * f32Sy);
    psQuaternion->f32q1 = (f32Sr * f32Cp * f32Cy) - (f32Cr * f32Sp * f32Sy);
    psQuaternion->f32q2 = (f32Cr * f32Sp * f32Cy) + (f32Sr * f32Cp * f32Sy);
    psQuaternion->f32q3 = (f32Cr * f32Cp * f32Sy) - (f32Sr * f32Sp * f32Cy);
}

static void Navigation_prvQuaternionToEuler(const ts_NavQuaternion *psQuaternion,
                                            float *pf32RollRad,
                                            float *pf32PitchRad,
                                            float *pf32YawRad)
{
    float f32AsinArg;

    if ((psQuaternion == NULL) || (pf32RollRad == NULL) || (pf32PitchRad == NULL) || (pf32YawRad == NULL))
    {
        return;
    }

    *pf32RollRad = atan2f(2.0F * ((psQuaternion->f32q0 * psQuaternion->f32q1) + (psQuaternion->f32q2 * psQuaternion->f32q3)),
                          (psQuaternion->f32q0 * psQuaternion->f32q0) -
                              (psQuaternion->f32q1 * psQuaternion->f32q1) -
                              (psQuaternion->f32q2 * psQuaternion->f32q2) +
                              (psQuaternion->f32q3 * psQuaternion->f32q3));

    f32AsinArg = -2.0F * ((psQuaternion->f32q1 * psQuaternion->f32q3) - (psQuaternion->f32q0 * psQuaternion->f32q2));
    f32AsinArg = fmaxf(-1.0F, fminf(1.0F, f32AsinArg));
    *pf32PitchRad = asinf(f32AsinArg);

    *pf32YawRad = atan2f(2.0F * ((psQuaternion->f32q0 * psQuaternion->f32q3) + (psQuaternion->f32q1 * psQuaternion->f32q2)),
                         (psQuaternion->f32q0 * psQuaternion->f32q0) +
                             (psQuaternion->f32q1 * psQuaternion->f32q1) -
                             (psQuaternion->f32q2 * psQuaternion->f32q2) -
                             (psQuaternion->f32q3 * psQuaternion->f32q3));
}

static void Navigation_prvSyncAttitudeFromQuaternion(ts_NavContext *psContext)
{
    float f32RollRad = 0.0F;
    float f32PitchRad = 0.0F;
    float f32YawRad = 0.0F;

    if (psContext == NULL)
    {
        return;
    }

    Navigation_prvQuaternionToEuler(&psContext->sQuaternion, &f32RollRad, &f32PitchRad, &f32YawRad);
    psContext->sEstimatedState.f32RollRad = Navigation_prvWrapAngleMinusPiToPi(f32RollRad);
    psContext->sEstimatedState.f32PitchRad = Navigation_prvWrapAngleMinusPiToPi(f32PitchRad);
    psContext->sEstimatedState.f32YawRad = Navigation_prvWrapAngle0To2Pi(f32YawRad);
}

static void Navigation_prvSyncQuaternionFromEuler(ts_NavContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    Navigation_prvEulerToQuaternion(psContext->sEstimatedState.f32RollRad,
                                    psContext->sEstimatedState.f32PitchRad,
                                    psContext->sEstimatedState.f32YawRad,
                                    &psContext->sQuaternion);
    Navigation_prvNormalizeQuaternion(&psContext->sQuaternion, psContext->sConfig.f32ZeroEpsilon);
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
    float f32q0;
    float f32q1;
    float f32q2;
    float f32q3;
    float f32Ax;
    float f32Ay;
    float f32Az;
    float f32AccelNorm;
    float f32Vx = 0.0F;
    float f32Vy = 0.0F;
    float f32Vz = 0.0F;
    float f32Ex = 0.0F;
    float f32Ey = 0.0F;
    float f32Ez = 0.0F;
    float f32WxCorr;
    float f32WyCorr;
    float f32WzCorr;
    float f32Q0Dot;
    float f32Q1Dot;
    float f32Q2Dot;
    float f32Q3Dot;

    if ((psContext == NULL) || (psRawImu == NULL))
    {
        return;
    }

    f32q0 = psContext->sQuaternion.f32q0;
    f32q1 = psContext->sQuaternion.f32q1;
    f32q2 = psContext->sQuaternion.f32q2;
    f32q3 = psContext->sQuaternion.f32q3;

    f32Ax = psRawImu->sAccel.f32X;
    f32Ay = psRawImu->sAccel.f32Y;
    f32Az = psRawImu->sAccel.f32Z;
    f32AccelNorm = sqrtf((f32Ax * f32Ax) + (f32Ay * f32Ay) + (f32Az * f32Az));

    if (f32AccelNorm >= psContext->sConfig.f32ZeroEpsilon)
    {
        f32Ax /= f32AccelNorm;
        f32Ay /= f32AccelNorm;
        f32Az /= f32AccelNorm;

        f32Vx = 2.0F * ((f32q1 * f32q3) - (f32q0 * f32q2));
        f32Vy = 2.0F * ((f32q0 * f32q1) + (f32q2 * f32q3));
        f32Vz = (f32q0 * f32q0) - (f32q1 * f32q1) - (f32q2 * f32q2) + (f32q3 * f32q3);

        f32Ex = (f32Ay * f32Vz) - (f32Az * f32Vy);
        f32Ey = (f32Az * f32Vx) - (f32Ax * f32Vz);
        f32Ez = (f32Ax * f32Vy) - (f32Ay * f32Vx);
    }

    f32WxCorr = psRawImu->sGyro.f32X + (psContext->sConfig.f32Alpha * f32Ex);
    f32WyCorr = psRawImu->sGyro.f32Y + (psContext->sConfig.f32Alpha * f32Ey);
    f32WzCorr = psRawImu->sGyro.f32Z + (psContext->sConfig.f32Alpha * f32Ez);

    f32Q0Dot = 0.5F * ((-f32q1 * f32WxCorr) - (f32q2 * f32WyCorr) - (f32q3 * f32WzCorr));
    f32Q1Dot = 0.5F * ((f32q0 * f32WxCorr) + (f32q2 * f32WzCorr) - (f32q3 * f32WyCorr));
    f32Q2Dot = 0.5F * ((f32q0 * f32WyCorr) - (f32q1 * f32WzCorr) + (f32q3 * f32WxCorr));
    f32Q3Dot = 0.5F * ((f32q0 * f32WzCorr) + (f32q1 * f32WyCorr) - (f32q2 * f32WxCorr));

    psContext->sQuaternion.f32q0 = f32q0 + (f32Q0Dot * psContext->sConfig.f32DtS);
    psContext->sQuaternion.f32q1 = f32q1 + (f32Q1Dot * psContext->sConfig.f32DtS);
    psContext->sQuaternion.f32q2 = f32q2 + (f32Q2Dot * psContext->sConfig.f32DtS);
    psContext->sQuaternion.f32q3 = f32q3 + (f32Q3Dot * psContext->sConfig.f32DtS);
    Navigation_prvNormalizeQuaternion(&psContext->sQuaternion, psContext->sConfig.f32ZeroEpsilon);
    Navigation_prvSyncAttitudeFromQuaternion(psContext);

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
    if (psContext->sConfig.sInitialAttitude.u8IsValid == 1U)
    {
        psContext->sEstimatedState.f32RollRad =
            Navigation_prvWrapAngleMinusPiToPi(psContext->sConfig.sInitialAttitude.f32RollRad);
        psContext->sEstimatedState.f32PitchRad =
            Navigation_prvWrapAngleMinusPiToPi(psContext->sConfig.sInitialAttitude.f32PitchRad);
        psContext->sEstimatedState.f32YawRad =
            Navigation_prvWrapAngle0To2Pi(psContext->sConfig.sInitialAttitude.f32YawRad);
        Navigation_prvSyncQuaternionFromEuler(psContext);
        psContext->sEstimatedState.bIsEstimated = true;
    }
    else
    {
        Navigation_prvSetIdentityQuaternion(&psContext->sQuaternion);
        Navigation_prvSyncAttitudeFromQuaternion(psContext);
        psContext->sEstimatedState.bIsEstimated = false;
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
        (psConfig->f32Alpha > 10.0F) ||
        (psConfig->f32ZeroEpsilon < 0.0F) ||
        (psConfig->u8StuckThresholdCycles == 0U) ||
        ((psConfig->sInitialAttitude.u8IsValid != 0U) && (psConfig->sInitialAttitude.u8IsValid != 1U)))
    {
        return NAV_RET_ERR_ARG;
    }

    (void)memset(psContext, 0, sizeof(*psContext));
    psContext->sConfig = *psConfig;
    Navigation_prvSetIdentityQuaternion(&psContext->sQuaternion);
    Navigation_prvApplyInitialAttitude(psContext);
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
