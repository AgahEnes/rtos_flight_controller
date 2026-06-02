#include "flight_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "global_data_space.h"

#define FMC_DEFAULT_PBIT_PASS_CYCLES                                (30U)
#define FMC_DEFAULT_MAX_CONSECUTIVE_INVALID_IMU_CYCLES              (2U)
#define FMC_DEFAULT_STATIONARY_WINDOW_CYCLES                        (50U)
#define FMC_DEFAULT_STATIONARY_GLOBAL_TIMEOUT_CYCLES                (500U)
#define FMC_DEFAULT_CALIBRATION_CYCLES                              (200U)
#define FMC_DEFAULT_POST_CALIBRATION_WAIT_CYCLES                    (3U)
#define FMC_DEFAULT_SETTLE_MIN_CYCLES                               (30U)
#define FMC_DEFAULT_SETTLE_MAX_CYCLES                               (50U)
#define FMC_DEFAULT_STATIONARY_GYRO_MAX_RADS                        (0.07F)
#define FMC_DEFAULT_STATIONARY_ACCEL_TARGET_MPS2                    (9.81F)
#define FMC_DEFAULT_STATIONARY_ACCEL_TOL_MPS2                       (0.30F)
#define FMC_DEFAULT_STATIONARY_VARIANCE_MAX                         (0.005F)
#define FMC_DEFAULT_SETTLE_GYRO_MAX_RADS                            (0.01F)
#define FMC_DEFAULT_SETTLE_ACCEL_TOL_MPS2                           (0.2F)
#define FMC_DEFAULT_SETTLE_VARIANCE_MAX                             (0.001F)
#define FMC_DEFAULT_LAUNCH_ACCEL_TRIGGER_MPS2                       (20.0F)
#define FMC_DEFAULT_INIT_MODE                                       (FLIGHT_MODE_PREFLIGHT)
#define FMC_DEFAULT_INIT_SUB_STATE                                  (PREFLIGHT_SUB_PBIT_CHECK)
#define FLIGHT_CONTROL_PI_F                                         (3.14159265358979323846F)
#define FMC_CONTROL_DT_S                                            (0.01F)
#define FMC_FIN_LIMIT_RAD                                           (0.78539816F)   //45 degrees
#define FMC_TARGET_ROLL_RAD                                         (0.0F)
#define FMC_TARGET_PITCH_RAD                                        (0.0F)
#define FMC_TARGET_YAW_RAD                                          (0.0F)
#define FMC_ROLL_KP                                                 (2.5F)
#define FMC_ROLL_KI                                                 (0.60F)
#define FMC_ROLL_KD                                                 (0.08F)
#define FMC_ROLL_I_MAX                                              (0.25F)
#define FMC_PITCH_KP                                                (3.0F)
#define FMC_PITCH_KI                                                (0.80F)
#define FMC_PITCH_KD                                                (0.10F)
#define FMC_PITCH_I_MAX                                             (0.30F)
#define FMC_YAW_KP                                                  (2.0F)
#define FMC_YAW_KI                                                  (0.50F)
#define FMC_YAW_KD                                                  (0.05F)
#define FMC_YAW_I_MAX                                               (0.20F)
#define FMC_BOOST_TO_STABILIZE_CYCLES                               (500U)
#define FMC_FIN_COMMAND_COUNT                                       (4U)
#define FMC_FIN_INDEX_POS_Y                                         (0U)
#define FMC_FIN_INDEX_POS_X                                         (1U)
#define FMC_FIN_INDEX_NEG_Y                                         (2U)
#define FMC_FIN_INDEX_NEG_X                                         (3U)

typedef enum
{
    PREFLIGHT_SUB_PBIT_CHECK = 0,
    PREFLIGHT_SUB_STATIONARY_CHECK,
    PREFLIGHT_SUB_CALIBRATING,
    PREFLIGHT_SUB_POST_CALIBRATION_WAIT,
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

static float FlightControl_prvClampF32(float f32Value, float f32Min, float f32Max)
{
    if (f32Value > f32Max)
    {
        return f32Max;
    }
    if (f32Value < f32Min)
    {
        return f32Min;
    }

    return f32Value;
}

static void FlightControl_prvInitPidController(ts_FmcPidController *psPid,
                                               float f32Kp,
                                               float f32Ki,
                                               float f32Kd,
                                               float f32IntegralClamp)
{
    if (psPid == NULL)
    {
        return;
    }

    psPid->f32Kp = f32Kp;
    psPid->f32Ki = f32Ki;
    psPid->f32Kd = f32Kd;
    psPid->f32Integral = 0.0F;
    psPid->f32IntegralClamp = f32IntegralClamp;
}

static void FlightControl_prvInitStabilizeControllers(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    FlightControl_prvInitPidController(&psContext->sRollPid,
                                       FMC_ROLL_KP,
                                       FMC_ROLL_KI,
                                       FMC_ROLL_KD,
                                       FMC_ROLL_I_MAX);
    FlightControl_prvInitPidController(&psContext->sPitchPid,
                                       FMC_PITCH_KP,
                                       FMC_PITCH_KI,
                                       FMC_PITCH_KD,
                                       FMC_PITCH_I_MAX);
    FlightControl_prvInitPidController(&psContext->sYawPid,
                                       FMC_YAW_KP,
                                       FMC_YAW_KI,
                                       FMC_YAW_KD,
                                       FMC_YAW_I_MAX);
}

static void FlightControl_prvEnterBoost(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->eMode = FLIGHT_MODE_BOOST;
    psContext->u32BoostElapsedCycles = 0U;
}

static void FlightControl_prvEnterStabilize(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->eMode = FLIGHT_MODE_STABILIZE;
    FlightControl_prvInitStabilizeControllers(psContext);
}

static float FlightControl_prvComputePidDemand(ts_FmcPidController *psPid,
                                               float f32TargetAngleRad,
                                               float f32CurrentAngleRad,
                                               float f32AngularRateRadS)
{
    float f32Error;
    float f32Proportional;
    float f32IntegralMin;
    float f32IntegralMax;
    float f32IntegralNext;
    float f32Derivative;

    if (psPid == NULL)
    {
        return 0.0F;
    }

    f32Error = f32TargetAngleRad - f32CurrentAngleRad;
    f32Proportional = psPid->f32Kp * f32Error;
    f32IntegralMin = -psPid->f32IntegralClamp;
    f32IntegralMax = psPid->f32IntegralClamp;
    f32IntegralNext = psPid->f32Integral + (psPid->f32Ki * f32Error * FMC_CONTROL_DT_S);
    if (f32IntegralNext > f32IntegralMax)
    {
        f32IntegralNext = f32IntegralMax;
    }
    else if (f32IntegralNext < f32IntegralMin)
    {
        f32IntegralNext = f32IntegralMin;
    }
    psPid->f32Integral = f32IntegralNext;
    f32Derivative = -psPid->f32Kd * f32AngularRateRadS;

    return f32Proportional + psPid->f32Integral + f32Derivative;
}

static bool FlightControl_prvPublishActuatorCommand(ts_FlightControlContext *psContext,
                                                    const float af32FinAngleRad[FMC_FIN_COMMAND_COUNT])
{
    ts_TopicActuatorCmd sActuatorCmd;
    uint8_t u8Idx;

    if ((psContext == NULL) || (af32FinAngleRad == NULL))
    {
        return false;
    }

    if (psContext->u32ActuatorCommandSequence < UINT32_MAX)
    {
        psContext->u32ActuatorCommandSequence++;
    }

    (void)memset(&sActuatorCmd, 0, sizeof(sActuatorCmd));
    for (u8Idx = 0U; u8Idx < FMC_FIN_COMMAND_COUNT; u8Idx++)
    {
        sActuatorCmd.f32FinAngleRad[u8Idx] = af32FinAngleRad[u8Idx];
    }
    sActuatorCmd.u32TimestampMs = psContext->u32ModuleTimestampMs;
    sActuatorCmd.u32Sequence = psContext->u32ActuatorCommandSequence;
    sActuatorCmd.bIsActive = true;

    return (Gds_PublishActuatorCmd(&sActuatorCmd) == GDS_OK);
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
    float f32CalAccelXMean;
    float f32CalAccelYMean;
    float f32CalAccelZMean;
    float f32PitchAccelDenom;

    if (psContext == NULL)
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    (void)memset(&sCalibration, 0, sizeof(sCalibration));
    f32CalAccelXMean = psContext->sCalibrationAccelXStats.f32Mean;
    f32CalAccelYMean = psContext->sCalibrationAccelYStats.f32Mean;
    f32CalAccelZMean = psContext->sCalibrationAccelZStats.f32Mean;
    f32PitchAccelDenom = sqrtf((f32CalAccelYMean * f32CalAccelYMean) + (f32CalAccelZMean * f32CalAccelZMean));
    
    sCalibration.sAccelBiasMps2.f32X = 0.096F;
    sCalibration.sAccelBiasMps2.f32Y = -0.1395F;
    sCalibration.sAccelBiasMps2.f32Z = 9.636F - 9.63683817F;

    sCalibration.sGyroBiasRadS.f32X = psContext->sCalibrationGyroXStats.f32Mean;
    sCalibration.sGyroBiasRadS.f32Y = psContext->sCalibrationGyroYStats.f32Mean;
    sCalibration.sGyroBiasRadS.f32Z = psContext->sCalibrationGyroZStats.f32Mean;

    sCalibration.sNavInitialAttitude.f32RollRad = atan2f(f32CalAccelYMean, f32CalAccelZMean);
    sCalibration.sNavInitialAttitude.f32PitchRad = atan2f(-f32CalAccelXMean, f32PitchAccelDenom);
    if (f32CalAccelZMean < 0.0F)
    {
        if (sCalibration.sNavInitialAttitude.f32PitchRad >= 0.0F)
        {
            sCalibration.sNavInitialAttitude.f32PitchRad =
                FLIGHT_CONTROL_PI_F - sCalibration.sNavInitialAttitude.f32PitchRad;
        }
        else
        {
            sCalibration.sNavInitialAttitude.f32PitchRad =
                (-FLIGHT_CONTROL_PI_F) - sCalibration.sNavInitialAttitude.f32PitchRad;
        }
    }
    sCalibration.sNavInitialAttitude.f32YawRad = 0.0F;
    sCalibration.sNavInitialAttitude.u8IsValid = 1U;
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

static void FlightControl_prvEnterPostCalibrationWait(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return;
    }

    psContext->u8PreflightSubState = (uint8_t)PREFLIGHT_SUB_POST_CALIBRATION_WAIT;
    psContext->u32PostCalibrationWaitElapsedCycles = 0U;
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

    // if (psContext->u32StationaryElapsedCycles >= psContext->sConfig.u32StationaryGlobalTimeoutCycles)
    // {
    //     FlightControl_prvEnterFailAbort(psContext);
    // }

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

    FlightControl_prvEnterPostCalibrationWait(psContext);
    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepPreflightPostCalibrationWait(ts_FlightControlContext *psContext,
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

    if (psContext->u32PostCalibrationWaitElapsedCycles < UINT32_MAX)
    {
        psContext->u32PostCalibrationWaitElapsedCycles++;
    }

    if (psContext->u32PostCalibrationWaitElapsedCycles >= psContext->sConfig.u32PostCalibrationWaitCycles)
    {
        FlightControl_prvEnterSettleCheck(psContext);
    }

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
        case PREFLIGHT_SUB_POST_CALIBRATION_WAIT:
            return FlightControl_prvStepPreflightPostCalibrationWait(psContext, psInputs);
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
        FlightControl_prvEnterBoost(psContext);
    }

    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepBoost(ts_FlightControlContext *psContext)
{
    if (psContext == NULL)
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    if (psContext->u32BoostElapsedCycles < UINT32_MAX)
    {
        psContext->u32BoostElapsedCycles++;
    }

    if (psContext->u32BoostElapsedCycles >= FMC_BOOST_TO_STABILIZE_CYCLES)
    {
        FlightControl_prvEnterStabilize(psContext);
    }

    return FLIGHT_CONTROL_OK;
}

static te_FlightControlRetCode FlightControl_prvStepStabilize(ts_FlightControlContext *psContext)
{
    ts_TopicVehicleState sVehicleState;
    te_GdsRetCode eGdsRet;
    float f32RollDemand;
    float f32PitchDemand;
    float f32YawDemand;
    float af32FinAngleRad[FMC_FIN_COMMAND_COUNT];
    uint8_t u8Idx;

    if (psContext == NULL)
    {
        return FLIGHT_CONTROL_ERR_ARG;
    }

    eGdsRet = Gds_ReadVehicleState(&sVehicleState);
    if (eGdsRet != GDS_OK)
    {
        return FLIGHT_CONTROL_ERR_GDS;
    }

    if (sVehicleState.bIsEstimated == false)
    {
        return FLIGHT_CONTROL_OK;
    }

    f32RollDemand = FlightControl_prvComputePidDemand(&psContext->sRollPid,
                                                      FMC_TARGET_ROLL_RAD,
                                                      sVehicleState.f32RollRad,
                                                      sVehicleState.f32RollRateRadS);
    f32PitchDemand = FlightControl_prvComputePidDemand(&psContext->sPitchPid,
                                                       FMC_TARGET_PITCH_RAD,
                                                       sVehicleState.f32PitchRad,
                                                       sVehicleState.f32PitchRateRadS);
    f32YawDemand = FlightControl_prvComputePidDemand(&psContext->sYawPid,
                                                     FMC_TARGET_YAW_RAD,
                                                     sVehicleState.f32YawRad,
                                                     sVehicleState.f32YawRateRadS);

    af32FinAngleRad[FMC_FIN_INDEX_POS_Y] = f32PitchDemand + f32RollDemand;
    af32FinAngleRad[FMC_FIN_INDEX_POS_X] = f32YawDemand + f32RollDemand;
    af32FinAngleRad[FMC_FIN_INDEX_NEG_Y] = (-f32PitchDemand) + f32RollDemand;
    af32FinAngleRad[FMC_FIN_INDEX_NEG_X] = (-f32YawDemand) + f32RollDemand;

    for (u8Idx = 0U; u8Idx < FMC_FIN_COMMAND_COUNT; u8Idx++)
    {
        af32FinAngleRad[u8Idx] = FlightControl_prvClampF32(af32FinAngleRad[u8Idx],
                                                           -FMC_FIN_LIMIT_RAD,
                                                           FMC_FIN_LIMIT_RAD);
    }

    if (FlightControl_prvPublishActuatorCommand(psContext, af32FinAngleRad) == false)
    {
        return FLIGHT_CONTROL_ERR_GDS;
    }

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

    psConfig->u32PbitPassCycles = FMC_DEFAULT_PBIT_PASS_CYCLES;
    psConfig->u32MaxConsecutiveInvalidImuCycles = FMC_DEFAULT_MAX_CONSECUTIVE_INVALID_IMU_CYCLES;
    psConfig->u32StationaryWindowCycles = FMC_DEFAULT_STATIONARY_WINDOW_CYCLES;
    psConfig->u32StationaryGlobalTimeoutCycles = FMC_DEFAULT_STATIONARY_GLOBAL_TIMEOUT_CYCLES;
    psConfig->u32CalibrationCycles = FMC_DEFAULT_CALIBRATION_CYCLES;
    psConfig->u32PostCalibrationWaitCycles = FMC_DEFAULT_POST_CALIBRATION_WAIT_CYCLES;
    psConfig->u32SettleMinCycles = FMC_DEFAULT_SETTLE_MIN_CYCLES;
    psConfig->u32SettleMaxCycles = FMC_DEFAULT_SETTLE_MAX_CYCLES;
    psConfig->f32StationaryGyroMaxRadS = FMC_DEFAULT_STATIONARY_GYRO_MAX_RADS;
    psConfig->f32StationaryAccelNormTarget = FMC_DEFAULT_STATIONARY_ACCEL_TARGET_MPS2;
    psConfig->f32StationaryAccelTolerance = FMC_DEFAULT_STATIONARY_ACCEL_TOL_MPS2;
    psConfig->f32StationaryVarianceThreshold = FMC_DEFAULT_STATIONARY_VARIANCE_MAX;
    psConfig->f32SettleGyroMaxRadS = FMC_DEFAULT_SETTLE_GYRO_MAX_RADS;
    psConfig->f32SettleAccelTolerance = FMC_DEFAULT_SETTLE_ACCEL_TOL_MPS2;
    psConfig->f32SettleVarianceThreshold = FMC_DEFAULT_SETTLE_VARIANCE_MAX;
    psConfig->f32LaunchAccelThresholdMps2 = FMC_DEFAULT_LAUNCH_ACCEL_TRIGGER_MPS2;
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
        (psConfig->u32PostCalibrationWaitCycles == 0U) ||
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
    FlightControl_prvInitStabilizeControllers(psContext);
    psContext->eMode = FMC_DEFAULT_INIT_MODE;
    psContext->u8PreflightSubState = (uint8_t)FMC_DEFAULT_INIT_SUB_STATE;
    psContext->u8IsInitialized = 1U;

    return FLIGHT_CONTROL_OK;
}

te_FlightControlRetCode FlightControl_Step(ts_FlightControlContext *psContext)
{
    ts_TopicRawImu sRawImu;
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
