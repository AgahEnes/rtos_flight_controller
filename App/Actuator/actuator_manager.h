#ifndef APP_ACTUATOR_ACTUATOR_MANAGER_H_
#define APP_ACTUATOR_ACTUATOR_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "actuator_driver_interface.h"

#define ACTUATOR_MANAGER_MAX_DEVICES    (4U)

typedef enum
{
    ACTUATOR_MANAGER_OK = 0,
    ACTUATOR_MANAGER_ERR_ARG,
    ACTUATOR_MANAGER_ERR_STATE,
    ACTUATOR_MANAGER_ERR_DRIVER,
    ACTUATOR_MANAGER_ERR_GDS
} te_ActuatorManagerRetCode;

typedef struct
{
    ts_ActuatorDevice *psActuatorDevices;
    uint8_t u8ActuatorDeviceCount;
} ts_ActuatorManagerConfig;

typedef struct
{
    ts_ActuatorManagerConfig sConfig;
    uint32_t u32LastAppliedSequence;
    uint8_t u8HasAppliedCommand;
    uint8_t u8IsInitialized;
} ts_ActuatorManagerContext;

te_ActuatorManagerRetCode ActuatorManager_Init(ts_ActuatorManagerContext *psContext,
                                               const ts_ActuatorManagerConfig *psConfig);
te_ActuatorManagerRetCode ActuatorManager_Step(ts_ActuatorManagerContext *psContext);

#ifdef __cplusplus
}
#endif

#endif /* APP_ACTUATOR_ACTUATOR_MANAGER_H_ */
