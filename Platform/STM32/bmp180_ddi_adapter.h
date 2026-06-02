#ifndef PLATFORM_STM32_BMP180_DDI_ADAPTER_H_
#define PLATFORM_STM32_BMP180_DDI_ADAPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "device_driver_interface.h"

typedef struct
{
    void *vpBmp180Handle;
} ts_Bmp180DdiAdapterContext;

void Bmp180DdiAdapter_Bind(ts_BaroDevice *psBaroDevice,
                           ts_Bmp180DdiAdapterContext *psAdapterContext,
                           void *vpBmp180Handle);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_STM32_BMP180_DDI_ADAPTER_H_ */
