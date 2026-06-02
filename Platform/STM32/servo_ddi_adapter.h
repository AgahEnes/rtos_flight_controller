#ifndef PLATFORM_STM32_SERVO_DDI_ADAPTER_H_
#define PLATFORM_STM32_SERVO_DDI_ADAPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "actuator_driver_interface.h"

typedef struct
{
    void *vpServoHandle;
} ts_ServoDdiAdapterContext;

void ServoDdiAdapter_Bind(ts_ActuatorDevice *psActuatorDevice,
                          ts_ServoDdiAdapterContext *psAdapterContext,
                          void *vpServoHandle);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_STM32_SERVO_DDI_ADAPTER_H_ */
