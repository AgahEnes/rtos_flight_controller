#ifndef APP_SENSOR_ACQ_SENSOR_MANAGER_H_
#define APP_SENSOR_ACQ_SENSOR_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "device_driver_interface.h"

#define SENSOR_MANAGER_MAX_IMU_DEVICES    (4U)

typedef enum
{
    SENSOR_MANAGER_OK = 0,
    SENSOR_MANAGER_ERR_ARG,
    SENSOR_MANAGER_ERR_STATE,
    SENSOR_MANAGER_ERR_DRIVER,
    SENSOR_MANAGER_ERR_GDS
} te_SensorManagerRetCode;

typedef struct
{
    ts_ImuDevice *psImuDevices;
    uint8_t u8ImuDeviceCount;
} ts_SensorManagerConfig;

typedef struct
{
    ts_SensorManagerConfig sConfig;
    uint32_t u32LastAppliedCalibrationCounter;
    uint8_t u8IsInitialized;
} ts_SensorManagerContext;

te_SensorManagerRetCode SensorManager_Init(ts_SensorManagerContext *psContext,
                                           const ts_SensorManagerConfig *psConfig);
te_SensorManagerRetCode SensorManager_Step(ts_SensorManagerContext *psContext);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_ACQ_SENSOR_MANAGER_H_ */
