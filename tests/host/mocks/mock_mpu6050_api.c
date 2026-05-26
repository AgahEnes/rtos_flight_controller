#include "mock_mpu6050_api.h"

#include <string.h>

static te_Driver_RetCode geMockReadRet = DRIVER_OK;
static ts_Mpu6050_Data gsMockData;

void MockMpu6050_SetReadResponse(te_Driver_RetCode eRet, const ts_Mpu6050_Data *psData)
{
    geMockReadRet = eRet;
    if (psData == NULL)
    {
        (void)memset(&gsMockData, 0, sizeof(gsMockData));
    }
    else
    {
        gsMockData = *psData;
    }
}

te_Driver_RetCode Mpu6050_Read(ts_Mpu6050_Handle *psHandle, ts_Mpu6050_Data *psOutData)
{
    (void)psHandle;

    if (psOutData == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    *psOutData = gsMockData;
    return geMockReadRet;
}
