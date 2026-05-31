#include "flight_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "global_data_space.h"

#define FLIGHT_CONTROL_DEFAULT_PBIT_PASS_CYCLES                     (30U)
#define FLIGHT_CONTROL_DEFAULT_MAX_CONSECUTIVE_INVALID_IMU_CYCLES     (2U)
#define FLIGHT_CONTROL_DEFAULT_STATIONARY_WINDOW_CYCLES             (50U)
#define FLIGHT_CONTROL_DEFAULT_STATIONARY_GLOBAL_TIMEOUT_CYCLES     (500U)
#define FLIGHT_CONTROL_DEFAULT_CALIBRATION_CYCLES                   (200U)
#define FLIGHT_CONTROL_DEFAULT_SETTLE_MIN_CYCLES                    (30U)
#define FLIGHT_CONTROL_DEFAULT_SETTLE_MAX_CYCLES                    (50U)
#define FLIGHT_CONTROL_DEFAULT_STATIONARY_GYRO_MAX_RADS             (0.05F)
#define FLIGHT_CONTROL_DEFAULT_STATIONARY_ACCEL_TARGET_MPS2         (9.81F)
#define FLIGHT_CONTROL_DEFAULT_STATIONARY_ACCEL_TOL_MPS2            (0.20F)
#define FLIGHT_CONTROL_DEFAULT_STATIONARY_VARIANCE_MAX              (0.005F)
#define FLIGHT_CONTROL_DEFAULT_SETTLE_GYRO_MAX_RADS                 (0.01F)
#define FLIGHT_CONTROL_DEFAULT_SETTLE_ACCEL_TOL_MPS2                (0.05F)
#define FLIGHT_CONTROL_DEFAULT_SETTLE_VARIANCE_MAX                  (0.001F)
#define FLIGHT_CONTROL_DEFAULT_LAUNCH_ACCEL_TRIGGER_MPS2            (15.0F)
#define FLIGHT_CONTROL_DEFAULT_INIT_MODE                            (FLIGHT_MODE_PREFLIGHT)
#define FLIGHT_CONTROL_DEFAULT_INIT_SUB_STATE                       (PREFLIGHT_SUB_PBIT_CHECK)

typedef enum
{
    PREFLIGHT_SUB_PBIT_CHECK = 0,
    PREFLIGHT_SUB_STATIONARY_CHECK,
    PREFLIGHT_SUB_CALIBRATING,
    PREFLIGHT_SUB_SETTLE_CHECK,
    PREFLIGHT_SUB_FAIL_ABORT
} te_PreflightSubState;

typedef struct
{
    const ts_TopicRawImu *psRawImu;
    bool bFreshImu;
    float f32GyroNorm;
    float f32AccelNorm;
} ts_FlightControlStepInputs;

static float FlightControl_prvAbsF32(float f32Value)
{
    return fabsf(f32Value);
}

static float FlightControl_prvNorm3f(float f32X, float f32Y, float f32Z)
{
    return sqrtf((f32X * f32X) + (f32Y * f32Y) + (f32Z * f32Z));
}

static void FlightControl_prvResetWelford(ts_FlightControlWelford *psStats)
{
    if (psStats == NULL)
    {
        return;
    }

    psStats->u32Count = 0U;
    psStats->f32Mean = 0.0F;
    psStats->f32M2 = 0.0F;
}

static void FlightControl_prvUpdateWelford(ts_FlightControlWelford *psStats, float f32Sample)
{
    float f32Delta;
    float f32Delta2;
    float f32CountAsFloat;

    if (psStats == NULL)
    {
        return;
    }

    if (psStats->u32Count < UINT32_MAX)
    {
        psStats->u32Count++;
    }

    f32CountAsFloat = (float)psStats->u32Count;
    if (f32CountAsFloat <= 0.0F)
    {
        return;
    }

    f32Delta = f32Sample - psStats->f32Mean;
    psStats->f32Mean += f32Delta / f32CountAsFloat;
    f32Delta2 = f32Sample - psStats->f32Mean;
    psStats->f32M2 += f32Delta * f32Delta2;
}

static float FlightControl_prvComputeVariance(const ts_FlightControlWelford *psStats)
{
    float f32Denom;

    if ((psStats == NULL) || (psStats->u32Count < 2U))
    {
        return 0.0F;
    }

    f32Denom = (float)(psStats->u32Count - 1U);
    if (f32Denom <= 0.0F)
    {
        return 0.0F;
    }

    return psStats->f32M2 / f32Denom;
}

static bool FlightControl_prvPublishNavCommand(ts_FlightControlContext *psContext,
                                               te_NavCmdType eCommand)
{
    ts_TopicNavCommand sCommand;

    if (psContext == NULL)
    {
        return false;
    }

    if (psContext->u32NavCommandSequence < UINT32_MAX)
    {
        psContext->u32NavCommandSequence++;
    }

    (void)memset(&sCommand, 0, sizeof(sCommand));
    sCommand.eCommand = eCommand;
    sCommand.u32Sequence = psContext->u32NavCommandSequence;
    sCommand.u32TimestampMs = psContext->u32ModuleTimestampMs;

    return (Gds_PublishNavCommand(&sCommand) == GDS_OK);
}

static bool FlightControl_prvIsFreshValidImu(ts_FlightControlContext *psContext,
                                             const ts_TopicRawImu *psRawImu)
{
    if ((psContext == NULL) || (psRawImu == NULL))
    {
        return false;
    }

    if (psRawImu->bIsValid == false)
    {
        return false;
    }

    if ((psContext->u8HasLastImuTimestamp != 0U) &&
        (psRawImu->u32TimestampMs <= psContext->u32LastImuTimestampMs))
    {
        return false;
    }

    psContext->u8HasLastImuTimestamp = 1U;
    psContext->u32LastImuTimestampMs = psRawImu->u32TimestampMs;

    return true;
}

static void FlightControl_prvResetStationaryCollection(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->u32StationaryWindowAcceptedCycles = 0U;
    psContext->u32CalibrationAcceptedCycles = 0U;
    FlightControl_prvResetWelford(&psContext->sStationaryGyroNormStats);
    FlightControl_prvResetWelford(&psContext->sStationaryAccelNormStats);
    FlightControl_prvResetWelford(&psContext->sCalibrationAccelXStats);
    FlightControl_prvResetWelford(&psContext->sCalibrationAccelYStats);
    FlightControl_prvResetWelford(&psContext->sCalibrationAccelZStats);
    FlightControl_prvResetWelford(&psContext->sCalibrationGyroXStats);
    FlightControl_prvResetWelford(&psContext->sCalibrationGyroYStats);
    FlightControl_prvResetWelford(&psContext->sCalibrationGyroZStats);
}

static void FlightControl_prvEnterStationaryCheck(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->u8PreflightSubState = (uint8_t)PREFLIGHT_SUB_STATIONARY_CHECK;
    psContext->u32StationaryElapsedCycles = 0U;
    FlightControl_prvResetStationaryCollection(psContext);
}

static void FlightControl_prvAccumulateStationarySample(ts_FlightControlContext *psContext,
                                                        const ts_TopicRawImu *psRawImu,
                                                        float f32GyroNorm,
                                                        float f32AccelNorm)
{
    if ((psContext == NULL) || (psRawImu == NULL))
    {
        return;
    }

    FlightControl_prvUpdateWelford(&psContext->sStationaryGyroNormStats, f32GyroNorm);
    FlightControl_prvUpdateWelford(&psContext->sStationaryAccelNormStats, f32AccelNorm);
    FlightControl_prvUpdateWelford(&psContext->sCalibrationAccelXStats, psRawImu->sAccel.f32X);
    FlightControl_prvUpdateWelford(&psContext->sCalibrationAccelYStats, psRawImu->sAccel.f32Y);
    FlightControl_prvUpdateWelford(&psContext->sCalibrationAccelZStats, psRawImu->sAccel.f32Z);
    FlightControl_prvUpdateWelford(&psContext->sCalibrationGyroXStats, psRawImu->sGyro.f32X);
    FlightControl_prvUpdateWelford(&psContext->sCalibrationGyroYStats, psRawImu->sGyro.f32Y);
    FlightControl_prvUpdateWelford(&psContext->sCalibrationGyroZStats, psRawImu->sGyro.f32Z);
    if (psContext->u32StationaryWindowAcceptedCycles < UINT32_MAX)
    {
        psContext->u32StationaryWindowAcceptedCycles++;
    }
    if (psContext->u32CalibrationAcceptedCycles < UINT32_MAX)
    {
        psContext->u32CalibrationAcceptedCycles++;
    }
}

static te_FlightControlRetCode FlightControl_prvPublishCalibrationFromContext(ts_FlightControlContext *psContext)
{
    ts_TopicImuCalibration sCalibration;

    if (psContext == NULL)
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    (void)memset(&sCalibration, 0, sizeof(sCalibration));
    sCalibration.sAccelBiasMps2.f32X = psContext->sCalibrationAccelXStats.f32Mean;
    sCalibration.sAccelBiasMps2.f32Y = psContext->sCalibrationAccelYStats.f32Mean;
    sCalibration.sAccelBiasMps2.f32Z = psContext->sCalibrationAccelZStats.f32Mean -
                                       psContext->sConfig.f32StationaryAccelNormTarget;
    sCalibration.sGyroBiasRadS.f32X = psContext->sCalibrationGyroXStats.f32Mean;
    sCalibration.sGyroBiasRadS.f32Y = psContext->sCalibrationGyroYStats.f32Mean;
    sCalibration.sGyroBiasRadS.f32Z = psContext->sCalibrationGyroZStats.f32Mean;
    sCalibration.u32TimestampMs = psContext->u32ModuleTimestampMs;
    if (psContext->u32ImuCalibrationSequence < UINT32_MAX)
    {
        psContext->u32ImuCalibrationSequence++;
    }
    sCalibration.u32UpdateCounter = psContext->u32ImuCalibrationSequence;
    sCalibration.bIsValid = true;

    if (Gds_PublishImuCalibration(&sCalibration) != GDS_OK)
    {
        return FLIGHT_CONTROL_ERR_GDS;
    }

    return FLIGHT_CONTROL_OK;
}

static void FlightControl_prvEnterCalibrating(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->u8PreflightSubState = (uint8_t)PREFLIGHT_SUB_CALIBRATING;
}

static void FlightControl_prvEnterSettleCheck(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->u8PreflightSubState = (uint8_t)PREFLIGHT_SUB_SETTLE_CHECK;
    psContext->u32SettleAcceptedCycles = 0U;
    FlightControl_prvResetWelford(&psContext->sSettleGyroNormStats);
    FlightControl_prvResetWelford(&psContext->sSettleAccelNormStats);
}

static void FlightControl_prvEnterReadyForIgnition(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->eMode = FLIGHT_MODE_READY_FOR_IGNITION;
}

static void FlightControl_prvEnterFailAbort(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->u8PreflightSubState = (uint8_t)PREFLIGHT_SUB_FAIL_ABORT;
    psContext->eMode = FLIGHT_MODE_FAILSAFE;
}

static bool FlightControl_prvHasExceededInvalidImuThreshold(const ts_FlightControlContext *psContext,
                                                            bool bFreshImu)
{
    if (psContext == NULL)
    {
        return false;
    }

    return ((bFreshImu == false) &&
            (psContext->u32ConsecutiveInvalidImuCycles >=
             psContext->sConfig.u32MaxConsecutiveInvalidImuCycles));
}

static te_FlightControlRetCode FlightControl_prvStepPreflightPbitCheck(ts_FlightControlContext *psContext,
                                                                       const ts_FlightControlStepInputs *psInputs)
{
    if ((psContext == NULL) || (psInputs == NULL))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    if (FlightControl_prvHasExceededInvalidImuThreshold(psContext, psInputs->bFreshImu) == true)
    {
        FlightControl_prvEnterFailAbort(psContext);
        return FLIGHT_CONTROL_OK;
    }
    if (psInputs->bFreshImu == false)
    {
        return FLIGHT_CONTROL_OK;
    }

    if (psContext->u32PbitFreshCycles < UINT32_MAX)
    {
        psContext->u32PbitFreshCycles++;
    }

    if (psContext->u32PbitFreshCycles >= psContext->sConfig.u32PbitPassCycles)
    {
        FlightControl_prvEnterStationaryCheck(psContext);
    }

    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepPreflightStationaryCheck(ts_FlightControlContext *psContext,
                                                                             const ts_FlightControlStepInputs *psInputs)
{
    bool bSampleWithinEnvelope;
    float f32GyroVariance;
    float f32AccelVariance;

    if ((psContext == NULL) || (psInputs == NULL) || (psInputs->psRawImu == NULL))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    if (psContext->u32StationaryElapsedCycles < UINT32_MAX)
    {
        psContext->u32StationaryElapsedCycles++;
    }

    bSampleWithinEnvelope = (psInputs->bFreshImu == true) &&
                            (psInputs->f32GyroNorm < psContext->sConfig.f32StationaryGyroMaxRadS) &&
                            (FlightControl_prvAbsF32(psInputs->f32AccelNorm -
                                                     psContext->sConfig.f32StationaryAccelNormTarget) <=
                             psContext->sConfig.f32StationaryAccelTolerance);

    if (bSampleWithinEnvelope == true)
    {
        FlightControl_prvAccumulateStationarySample(psContext,
                                                    psInputs->psRawImu,
                                                    psInputs->f32GyroNorm,
                                                    psInputs->f32AccelNorm);
    }
    else
    {
        FlightControl_prvResetStationaryCollection(psContext);
    }

    if ((psContext->u32CalibrationAcceptedCycles >= psContext->sConfig.u32CalibrationCycles) &&
        (psContext->u32StationaryWindowAcceptedCycles >= psContext->sConfig.u32StationaryWindowCycles))
    {
        f32GyroVariance = FlightControl_prvComputeVariance(&psContext->sStationaryGyroNormStats);
        f32AccelVariance = FlightControl_prvComputeVariance(&psContext->sStationaryAccelNormStats);
        if ((f32GyroVariance <= psContext->sConfig.f32StationaryVarianceThreshold) &&
            (f32AccelVariance <= psContext->sConfig.f32StationaryVarianceThreshold))
        {
            FlightControl_prvEnterCalibrating(psContext);
        }
        else
        {
            FlightControl_prvResetStationaryCollection(psContext);
        }
    }

    if (psContext->u32StationaryElapsedCycles >= psContext->sConfig.u32StationaryGlobalTimeoutCycles)
    {
        FlightControl_prvEnterFailAbort(psContext);
    }

    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepPreflightCalibrating(ts_FlightControlContext *psContext)
{
    te_FlightControlRetCode ePublishRet;

    if (psContext == NULL)
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    ePublishRet = FlightControl_prvPublishCalibrationFromContext(psContext);
    if (ePublishRet != FLIGHT_CONTROL_OK)
    {
        return ePublishRet;
    }

    FlightControl_prvEnterSettleCheck(psContext);
    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepPreflightSettleCheck(ts_FlightControlContext *psContext,
                                                                         const ts_FlightControlStepInputs *psInputs)
{
    float f32GyroVariance;
    float f32AccelVariance;

    if ((psContext == NULL) || (psInputs == NULL))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    if (FlightControl_prvHasExceededInvalidImuThreshold(psContext, psInputs->bFreshImu) == true)
    {
        FlightControl_prvEnterFailAbort(psContext);
        return FLIGHT_CONTROL_OK;
    }
    if (psInputs->bFreshImu == false)
    {
        return FLIGHT_CONTROL_OK;
    }

    if ((psInputs->f32GyroNorm > psContext->sConfig.f32SettleGyroMaxRadS) ||
        (FlightControl_prvAbsF32(psInputs->f32AccelNorm - psContext->sConfig.f32StationaryAccelNormTarget) >
         psContext->sConfig.f32SettleAccelTolerance))
    {
        FlightControl_prvEnterFailAbort(psContext);
        return FLIGHT_CONTROL_OK;
    }

    FlightControl_prvUpdateWelford(&psContext->sSettleGyroNormStats, psInputs->f32GyroNorm);
    FlightControl_prvUpdateWelford(&psContext->sSettleAccelNormStats, psInputs->f32AccelNorm);
    if (psContext->u32SettleAcceptedCycles < UINT32_MAX)
    {
        psContext->u32SettleAcceptedCycles++;
    }

    f32GyroVariance = FlightControl_prvComputeVariance(&psContext->sSettleGyroNormStats);
    f32AccelVariance = FlightControl_prvComputeVariance(&psContext->sSettleAccelNormStats);
    if ((f32GyroVariance > psContext->sConfig.f32SettleVarianceThreshold) ||
        (f32AccelVariance > psContext->sConfig.f32SettleVarianceThreshold))
    {
        FlightControl_prvEnterFailAbort(psContext);
        return FLIGHT_CONTROL_OK;
    }

    if (psContext->u32SettleAcceptedCycles >= psContext->sConfig.u32SettleMinCycles)
    {
        if (psContext->u8PreflightReinitIssued == 0U)
        {
            if (FlightControl_prvPublishNavCommand(psContext, NAV_CMD_REINIT) == false)
            {
                return FLIGHT_CONTROL_ERR_GDS;
            }
            psContext->u8PreflightReinitIssued = 1U;
        }
        FlightControl_prvEnterReadyForIgnition(psContext);
        return FLIGHT_CONTROL_OK;
    }

    if (psContext->u32SettleAcceptedCycles > psContext->sConfig.u32SettleMaxCycles)
    {
        FlightControl_prvEnterFailAbort(psContext);
    }

    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepPreflightFailAbort(ts_FlightControlContext *psContext,
                                                                       const ts_FlightControlStepInputs *psInputs)
{
    if ((psContext == NULL) || (psInputs == NULL))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    psContext->eMode = FLIGHT_MODE_FAILSAFE;
    if ((psContext->u8FailsafeReinitIssued == 0U) && (psInputs->bFreshImu == true))
    {
        if (FlightControl_prvPublishNavCommand(psContext, NAV_CMD_REINIT) == false)
        {
            return FLIGHT_CONTROL_ERR_GDS;
        }
        psContext->u8FailsafeReinitIssued = 1U;
    }

    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepPreflight(ts_FlightControlContext *psContext,
                                                               const ts_FlightControlStepInputs *psInputs)
{
    if ((psContext == NULL) || (psInputs == NULL))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    switch ((te_PreflightSubState)psContext->u8PreflightSubState)
    {
        case PREFLIGHT_SUB_PBIT_CHECK:
            return FlightControl_prvStepPreflightPbitCheck(psContext, psInputs);
        case PREFLIGHT_SUB_STATIONARY_CHECK:
            return FlightControl_prvStepPreflightStationaryCheck(psContext, psInputs);
        case PREFLIGHT_SUB_CALIBRATING:
            return FlightControl_prvStepPreflightCalibrating(psContext);
        case PREFLIGHT_SUB_SETTLE_CHECK:
            return FlightControl_prvStepPreflightSettleCheck(psContext, psInputs);
        case PREFLIGHT_SUB_FAIL_ABORT:
            return FlightControl_prvStepPreflightFailAbort(psContext, psInputs);
        default:
            return FLIGHT_CONTROL_ERR_STATE;
    }
}

static te_FlightControlRetCode FlightControl_prvStepReadyForIgnition(ts_FlightControlContext *psContext,
                                                                     const ts_FlightControlStepInputs *psInputs)
{
    if ((psContext == NULL) || (psInputs == NULL))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    if ((psInputs->bFreshImu == true) &&
        (psInputs->f32AccelNorm >= psContext->sConfig.f32LaunchAccelThresholdMps2))
    {
        psContext->eMode = FLIGHT_MODE_BOOST;
    }

    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepBoost(ts_FlightControlContext *psContext)
{
    (void)psContext;
    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepStabilize(ts_FlightControlContext *psContext)
{
    (void)psContext;
    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepFailsafe(ts_FlightControlContext *psContext,
                                                              const ts_FlightControlStepInputs *psInputs)
{
    if ((psContext == NULL) || (psInputs == NULL))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    if ((psContext->u8FailsafeReinitIssued == 0U) && (psInputs->bFreshImu == true))
    {
        if (FlightControl_prvPublishNavCommand(psContext, NAV_CMD_REINIT) == false)
        {
            return FLIGHT_CONTROL_ERR_GDS;
        }
        psContext->u8FailsafeReinitIssued = 1U;
    }

    return FLIGHT_CONTROL_OK;
}

void FlightControl_InitDefaultConfig(ts_FlightControlConfig *psConfig)
{
    if (psConfig == NULL)
    {
        return;
    }

    psConfig->u32PbitPassCycles = FLIGHT_CONTROL_DEFAULT_PBIT_PASS_CYCLES;
    psConfig->u32MaxConsecutiveInvalidImuCycles = FLIGHT_CONTROL_DEFAULT_MAX_CONSECUTIVE_INVALID_IMU_CYCLES;
    psConfig->u32StationaryWindowCycles = FLIGHT_CONTROL_DEFAULT_STATIONARY_WINDOW_CYCLES;
    psConfig->u32StationaryGlobalTimeoutCycles = FLIGHT_CONTROL_DEFAULT_STATIONARY_GLOBAL_TIMEOUT_CYCLES;
    psConfig->u32CalibrationCycles = FLIGHT_CONTROL_DEFAULT_CALIBRATION_CYCLES;
    psConfig->u32SettleMinCycles = FLIGHT_CONTROL_DEFAULT_SETTLE_MIN_CYCLES;
    psConfig->u32SettleMaxCycles = FLIGHT_CONTROL_DEFAULT_SETTLE_MAX_CYCLES;
    psConfig->f32StationaryGyroMaxRadS = FLIGHT_CONTROL_DEFAULT_STATIONARY_GYRO_MAX_RADS;
    psConfig->f32StationaryAccelNormTarget = FLIGHT_CONTROL_DEFAULT_STATIONARY_ACCEL_TARGET_MPS2;
    psConfig->f32StationaryAccelTolerance = FLIGHT_CONTROL_DEFAULT_STATIONARY_ACCEL_TOL_MPS2;
    psConfig->f32StationaryVarianceThreshold = FLIGHT_CONTROL_DEFAULT_STATIONARY_VARIANCE_MAX;
    psConfig->f32SettleGyroMaxRadS = FLIGHT_CONTROL_DEFAULT_SETTLE_GYRO_MAX_RADS;
    psConfig->f32SettleAccelTolerance = FLIGHT_CONTROL_DEFAULT_SETTLE_ACCEL_TOL_MPS2;
    psConfig->f32SettleVarianceThreshold = FLIGHT_CONTROL_DEFAULT_SETTLE_VARIANCE_MAX;
    psConfig->f32LaunchAccelThresholdMps2 = FLIGHT_CONTROL_DEFAULT_LAUNCH_ACCEL_TRIGGER_MPS2;
}

te_FlightControlRetCode FlightControl_Init(ts_FlightControlContext *psContext,
                                           const ts_FlightControlConfig *psConfig)
{
    if ((psContext == NULL) || (psConfig == NULL))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    if ((psConfig->u32PbitPassCycles == 0U) ||
        (psConfig->u32MaxConsecutiveInvalidImuCycles == 0U) ||
        (psConfig->u32StationaryWindowCycles == 0U) ||
        (psConfig->u32StationaryGlobalTimeoutCycles < psConfig->u32StationaryWindowCycles) ||
        (psConfig->u32StationaryGlobalTimeoutCycles < psConfig->u32CalibrationCycles) ||
        (psConfig->u32StationaryWindowCycles > psConfig->u32CalibrationCycles) ||
        (psConfig->u32CalibrationCycles == 0U) ||
        (psConfig->u32SettleMinCycles == 0U) ||
        (psConfig->u32SettleMaxCycles < psConfig->u32SettleMinCycles) ||
        (psConfig->f32StationaryGyroMaxRadS <= 0.0F) ||
        (psConfig->f32StationaryAccelNormTarget <= 0.0F) ||
        (psConfig->f32StationaryAccelTolerance <= 0.0F) ||
        (psConfig->f32StationaryVarianceThreshold <= 0.0F) ||
        (psConfig->f32SettleGyroMaxRadS <= 0.0F) ||
        (psConfig->f32SettleAccelTolerance <= 0.0F) ||
        (psConfig->f32SettleVarianceThreshold <= 0.0F) ||
        (psConfig->f32LaunchAccelThresholdMps2 <= 0.0F))
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    (void)memset(psContext, 0, sizeof(*psContext));
    psContext->sConfig = *psConfig;
    psContext->eMode = FLIGHT_CONTROL_DEFAULT_INIT_MODE;
    psContext->u8PreflightSubState = (uint8_t)FLIGHT_CONTROL_DEFAULT_INIT_SUB_STATE;
    psContext->u8IsInitialized = 1U;

    return FLIGHT_CONTROL_OK;
}

te_FlightControlRetCode FlightControl_Step(ts_FlightControlContext *psContext)
{
    ts_TopicRawImu sRawImu;
    ts_TopicVehicleState sVehicleState;
    ts_FlightControlStepInputs sInputs;
    te_GdsRetCode eRawReadRet;
    te_FlightControlRetCode eStepRet;

    if (psContext == NULL)
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    if (psContext->u8IsInitialized == 0U)
    {
        return FLIGHT_CONTROL_ERR_STATE;
    }

    eRawReadRet = Gds_ReadRawImu(&sRawImu);
    if (eRawReadRet != GDS_OK)
    {
        return FLIGHT_CONTROL_ERR_GDS;
    }

    (void)Gds_ReadVehicleState(&sVehicleState);

    sInputs.psRawImu = &sRawImu;
    sInputs.bFreshImu = FlightControl_prvIsFreshValidImu(psContext, &sRawImu);
    if (sInputs.bFreshImu == true)
    {
        psContext->u32ConsecutiveInvalidImuCycles = 0U;
    }
    else if (psContext->u32ConsecutiveInvalidImuCycles < UINT32_MAX)
    {
        psContext->u32ConsecutiveInvalidImuCycles++;
    }
    sInputs.f32GyroNorm = FlightControl_prvNorm3f(sRawImu.sGyro.f32X, sRawImu.sGyro.f32Y, sRawImu.sGyro.f32Z);
    sInputs.f32AccelNorm = FlightControl_prvNorm3f(sRawImu.sAccel.f32X, sRawImu.sAccel.f32Y, sRawImu.sAccel.f32Z);

    switch (psContext->eMode)
    {
        case FLIGHT_MODE_PREFLIGHT:
            eStepRet = FlightControl_prvStepPreflight(psContext, &sInputs);
            break;
        case FLIGHT_MODE_READY_FOR_IGNITION:
            eStepRet = FlightControl_prvStepReadyForIgnition(psContext, &sInputs);
            break;
        case FLIGHT_MODE_BOOST:
            eStepRet = FlightControl_prvStepBoost(psContext);
            break;
        case FLIGHT_MODE_STABILIZE:
            eStepRet = FlightControl_prvStepStabilize(psContext);
            break;
        case FLIGHT_MODE_FAILSAFE:
        default:
            eStepRet = FlightControl_prvStepFailsafe(psContext, &sInputs);
            break;
    }

    return eStepRet;
}
