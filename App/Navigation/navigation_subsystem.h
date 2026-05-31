#ifndef APP_NAVIGATION_NAVIGATION_SUBSYSTEM_H_
#define APP_NAVIGATION_NAVIGATION_SUBSYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "global_data_space.h"

#define NAV_CFG_DEFAULT_ALPHA               (0.98F)
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
    float f32RollRad;
    float f32PitchRad;
    float f32YawRad;
    uint8_t u8IsValid;
} ts_NavInitialAttitude;

typedef struct
{
    float f32Alpha;
    float f32DtS;
    float f32ZeroEpsilon;
    uint8_t u8StuckThresholdCycles;
    ts_NavInitialAttitude sInitialAttitude;
} ts_NavConfig;

typedef struct
{
    ts_NavConfig sConfig;
    ts_TopicVehicleState sEstimatedState;
    ts_TopicRawImu sLastRawImu;
    uint32_t u32LastProcessedTimestampMs;
    uint8_t u8HasLastRawImu;
    uint8_t u8StuckCycleCount;
    uint8_t u8IsInitialized;
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
