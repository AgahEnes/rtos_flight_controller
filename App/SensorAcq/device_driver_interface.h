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

typedef struct
{
    tpfn_ImuDriverInit pfnInit;
    tpfn_ImuDriverReadImu pfnReadImu;
} ts_ImuDriverVTable;

typedef struct
{
    const ts_ImuDriverVTable *psVTable;
    void *vpContext;
} ts_ImuDevice;

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_ACQ_DEVICE_DRIVER_INTERFACE_H_ */
