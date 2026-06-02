#ifndef APP_SENSOR_ACQ_DEVICE_DRIVER_INTERFACE_H_
#define APP_SENSOR_ACQ_DEVICE_DRIVER_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "global_data_space.h"

typedef enum
{
    IMU_DRIVER_OK = 0,
    IMU_DRIVER_ERR_ARG,
    IMU_DRIVER_ERR_STATE,
    IMU_DRIVER_ERR_IO
} te_ImuDriverRetCode;

typedef te_ImuDriverRetCode (*tpfn_ImuDriverInit)(void *vpContext);
typedef te_ImuDriverRetCode (*tpfn_ImuDriverReadImu)(void *vpContext, ts_TopicRawImu *psRawImu);
typedef te_ImuDriverRetCode (*tpfn_ImuDriverSetBias)(void *vpContext, const ts_TopicImuCalibration *psCalibration);

typedef struct
{
    tpfn_ImuDriverInit pfnInit;
    tpfn_ImuDriverReadImu pfnReadImu;
    tpfn_ImuDriverSetBias pfnSetBias;
} ts_ImuDriverVTable;

typedef struct
{
    const ts_ImuDriverVTable *psVTable;
    void *vpContext;
} ts_ImuDevice;

typedef enum
{
    BARO_DRIVER_OK = 0,
    BARO_DRIVER_ERR_ARG,
    BARO_DRIVER_ERR_STATE,
    BARO_DRIVER_ERR_IO
} te_BaroDriverRetCode;

typedef te_BaroDriverRetCode (*tpfn_BaroDriverInit)(void *vpContext);
typedef te_BaroDriverRetCode (*tpfn_BaroDriverStartMeasurement)(void *vpContext);
typedef te_BaroDriverRetCode (*tpfn_BaroDriverProcess)(void *vpContext);
typedef te_BaroDriverRetCode (*tpfn_BaroDriverRead)(void *vpContext, ts_TopicBarometer *psBarometer);

typedef struct
{
    tpfn_BaroDriverInit pfnInit;
    tpfn_BaroDriverStartMeasurement pfnStartMeasurement;
    tpfn_BaroDriverProcess pfnProcess;
    tpfn_BaroDriverRead pfnReadBarometer;
} ts_BaroDriverVTable;

typedef struct
{
    const ts_BaroDriverVTable *psVTable;
    void *vpContext;
} ts_BaroDevice;

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_ACQ_DEVICE_DRIVER_INTERFACE_H_ */
