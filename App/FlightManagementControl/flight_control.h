#ifndef APP_FLIGHT_CONTROL_FLIGHT_CONTROL_H_
#define APP_FLIGHT_CONTROL_FLIGHT_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    FLIGHT_CONTROL_OK = 0,
    FLIGHT_CONTROL_ERR_ARG,
    FLIGHT_CONTROL_ERR_STATE,
    FLIGHT_CONTROL_ERR_GDS
} te_FlightControlRetCode;

typedef enum
{
    FLIGHT_MODE_PREFLIGHT = 0,
    FLIGHT_MODE_READY_FOR_IGNITION,
    FLIGHT_MODE_BOOST,
    FLIGHT_MODE_STABILIZE,
    FLIGHT_MODE_FAILSAFE
} te_FlightMode;

typedef struct
{
    uint32_t u32Count;
    float f32Mean;
    float f32M2;
} ts_FlightControlWelford;

typedef struct
{
    float f32Kp;
    float f32Ki;
    float f32Kd;
    float f32Integral;
    float f32IntegralClamp;
} ts_FmcPidController;

typedef struct
{
    uint32_t u32PbitPassCycles;
    uint32_t u32MaxConsecutiveInvalidImuCycles;
    uint32_t u32StationaryWindowCycles;
    uint32_t u32StationaryGlobalTimeoutCycles;
    uint32_t u32CalibrationCycles;
    uint32_t u32PostCalibrationWaitCycles;
    uint32_t u32SettleMinCycles;
    uint32_t u32SettleMaxCycles;
    float f32StationaryGyroMaxRadS;
    float f32StationaryAccelNormTarget;
    float f32StationaryAccelTolerance;
    float f32StationaryVarianceThreshold;
    float f32SettleGyroMaxRadS;
    float f32SettleAccelTolerance;
    float f32SettleVarianceThreshold;
    float f32LaunchAccelThresholdMps2;
} ts_FlightControlConfig;

typedef struct
{
    ts_FlightControlConfig sConfig;
    te_FlightMode eMode;
    uint32_t u32ModuleTimestampMs;
    uint32_t u32ImuCalibrationSequence;
    uint32_t u32NavCommandSequence;
    uint32_t u32LastImuTimestampMs;
    uint32_t u32ConsecutiveInvalidImuCycles;
    uint32_t u32PbitFreshCycles;
    uint32_t u32StationaryElapsedCycles;
    uint32_t u32StationaryWindowAcceptedCycles;
    uint32_t u32CalibrationAcceptedCycles;
    uint32_t u32PostCalibrationWaitElapsedCycles;
    uint32_t u32SettleAcceptedCycles;
    ts_FlightControlWelford sStationaryGyroNormStats;
    ts_FlightControlWelford sStationaryAccelNormStats;
    ts_FlightControlWelford sSettleGyroNormStats;
    ts_FlightControlWelford sSettleAccelNormStats;
    ts_FlightControlWelford sCalibrationAccelXStats;
    ts_FlightControlWelford sCalibrationAccelYStats;
    ts_FlightControlWelford sCalibrationAccelZStats;
    ts_FlightControlWelford sCalibrationGyroXStats;
    ts_FlightControlWelford sCalibrationGyroYStats;
    ts_FlightControlWelford sCalibrationGyroZStats;
    ts_FmcPidController sRollPid;
    ts_FmcPidController sPitchPid;
    ts_FmcPidController sYawPid;
    uint32_t u32BoostElapsedCycles;
    uint32_t u32ActuatorCommandSequence;
    uint8_t u8PreflightSubState;
    uint8_t u8HasLastImuTimestamp;
    uint8_t u8PreflightReinitIssued;
    uint8_t u8FailsafeReinitIssued;
    uint8_t u8IsInitialized;
} ts_FlightControlContext;

void FlightControl_InitDefaultConfig(ts_FlightControlConfig *psConfig);
te_FlightControlRetCode FlightControl_Init(ts_FlightControlContext *psContext, const ts_FlightControlConfig *psConfig);
te_FlightControlRetCode FlightControl_Step(ts_FlightControlContext *psContext);

#ifdef __cplusplus
}
#endif

#endif /* APP_FLIGHT_CONTROL_FLIGHT_CONTROL_H_ */
