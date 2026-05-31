#ifndef APP_SENSOR_ACQ_GLOBAL_DATA_SPACE_H_
#define APP_SENSOR_ACQ_GLOBAL_DATA_SPACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float f32X;
    float f32Y;
    float f32Z;
} ts_Vector3d;

typedef struct
{
    ts_Vector3d sAccel;
    ts_Vector3d sGyro;
    float f32TempC;
    uint32_t u32TimestampMs;
    bool bIsValid;
} ts_TopicRawImu;

typedef struct
{
    float f32RollRad;
    float f32PitchRad;
    float f32YawRad;
    float f32RollRateRadS;
    float f32PitchRateRadS;
    float f32YawRateRadS;
    uint32_t u32TimestampMs;
    bool bIsEstimated;
} ts_TopicVehicleState;

typedef struct
{
    ts_Vector3d sAccelBiasMps2;
    ts_Vector3d sGyroBiasRadS;
    uint32_t u32TimestampMs;
    uint32_t u32UpdateCounter;
    bool bIsValid;
} ts_TopicImuCalibration;

typedef enum
{
    NAV_CMD_NONE = 0,
    NAV_CMD_RESET_FILTER,
    NAV_CMD_REINIT
} te_NavCmdType;

typedef struct
{
    te_NavCmdType eCommand;
    uint32_t u32Sequence;
    uint32_t u32TimestampMs;
} ts_TopicNavCommand;

typedef enum
{
    GDS_OK = 0,
    GDS_ERR_ARG,
    GDS_ERR_INCONSISTENT_READ
} te_GdsRetCode;

void Gds_ResetRawImu(void);
te_GdsRetCode Gds_PublishRawImu(const ts_TopicRawImu *psRawImu);
te_GdsRetCode Gds_ReadRawImu(ts_TopicRawImu *psRawImu);
void Gds_ResetVehicleState(void);
te_GdsRetCode Gds_PublishVehicleState(const ts_TopicVehicleState *psVehicleState);
te_GdsRetCode Gds_ReadVehicleState(ts_TopicVehicleState *psVehicleState);
void Gds_ResetImuCalibration(void);
te_GdsRetCode Gds_PublishImuCalibration(const ts_TopicImuCalibration *psCalibration);
te_GdsRetCode Gds_ReadImuCalibration(ts_TopicImuCalibration *psCalibration);
void Gds_ResetNavCommand(void);
te_GdsRetCode Gds_PublishNavCommand(const ts_TopicNavCommand *psCommand);
te_GdsRetCode Gds_ReadNavCommand(ts_TopicNavCommand *psCommand);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_ACQ_GLOBAL_DATA_SPACE_H_ */
