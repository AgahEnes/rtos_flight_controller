#ifndef APP_NAVIGATION_NAVIGATION_SUBSYSTEM_H_
#define APP_NAVIGATION_NAVIGATION_SUBSYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "global_data_space.h"

#define NAV_CFG_DEFAULT_ALPHA               (3.0F)   /* Mahony proportional gain Kp */
#define NAV_CFG_DEFAULT_KP                  (NAV_CFG_DEFAULT_ALPHA)
#define NAV_CFG_DEFAULT_DT_S                (0.01F)
#define NAV_CFG_ZERO_EPSILON                (1.0e-6F)
#define NAV_CFG_STUCK_THRESHOLD_CYCLES      (3U)
#define NAV_ACCEL_MAX_MPS2                  (200.0F)
#define NAV_GYRO_MAX_RADS                   (35.0F)

typedef enum
{
    NAV_RET_OK = 0,
    NAV_RET_ERR_ARG,
    NAV_RET_ERR_STATE
} te_NavigationRetCode;

typedef enum
{
    NAV_STATUS_OK = 0,
    NAV_STATUS_STALE,
    NAV_STATUS_ZERO_FAULT,
    NAV_STATUS_STUCK_FAULT,
    NAV_STATUS_OUT_OF_BOUNDS
} te_NavDataStatus;

typedef struct
{
    float f32RollRad;       /* Range: [-PI, +PI] */
    float f32PitchRad;      /* Range: [-PI, +PI] */
    float f32YawRad;        /* Range: [0, 2*PI) */
    uint8_t u8IsValid;
} ts_NavInitialAttitude;

typedef struct
{
    float f32Alpha; /* Repurposed as Mahony proportional gain (Kp). */
    float f32DtS;
    float f32ZeroEpsilon;
    uint8_t u8StuckThresholdCycles;
    ts_NavInitialAttitude sInitialAttitude;
} ts_NavConfig;

typedef struct
{
    float f32q0;
    float f32q1;
    float f32q2;
    float f32q3;
} ts_NavQuaternion;

typedef struct
{
    ts_NavConfig sConfig;
    ts_NavQuaternion sQuaternion;
    ts_TopicVehicleState sEstimatedState;
    ts_TopicRawImu sLastRawImu;
    uint32_t u32LastProcessedTimestampMs;
    uint8_t u8HasLastRawImu;
    uint8_t u8StuckCycleCount;
    uint8_t u8IsInitialized;
    uint32_t u32LastExecutedCmdSeq;
    te_NavDataStatus eLastDataStatus;
} ts_NavContext;

te_NavigationRetCode Navigation_Init(ts_NavContext *psContext, const ts_NavConfig *psConfig);
te_NavigationRetCode NavigationTask_Step(ts_NavContext *psContext,
                                     const ts_TopicRawImu *psRawImu,
                                     ts_TopicVehicleState *psVehicleState);

#ifdef __cplusplus
}
#endif

#endif /* APP_NAVIGATION_NAVIGATION_SUBSYSTEM_H_ */
