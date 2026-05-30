#ifndef PLATFORM_STM32_MPU6050_DDI_ADAPTER_H_
#define PLATFORM_STM32_MPU6050_DDI_ADAPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "device_driver_interface.h"

typedef struct
{
    void *vpMpuHandle;
} ts_Mpu6050DdiAdapterContext;

void Mpu6050DdiAdapter_Bind(ts_ImuDevice *psImuDevice,
                            ts_Mpu6050DdiAdapterContext *psAdapterContext,
                            void *vpMpuHandle);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_STM32_MPU6050_DDI_ADAPTER_H_ */
