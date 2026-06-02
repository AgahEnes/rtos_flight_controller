#include "servo_ddi_adapter.h"
#include <stddef.h>
#include "servo/servo_driver.h"

static te_ActuatorDriverRetCode ServoDdiAdapter_prvMapRetCode(te_Driver_RetCode eDriverRet)
{
    te_ActuatorDriverRetCode eRet;

    switch (eDriverRet)
    {
        case DRIVER_OK:
            eRet = ACTUATOR_DRIVER_OK;
            break;
        case DRIVER_ERR_NULL_PTR:
        case DRIVER_ERR_INVALID_ARG:
            eRet = ACTUATOR_DRIVER_ERR_ARG;
            break;
        case DRIVER_ERR_STATE:
            eRet = ACTUATOR_DRIVER_ERR_STATE;
            break;
        default:
            eRet = ACTUATOR_DRIVER_ERR_IO;
            break;
    }

    return eRet;
}

static te_ActuatorDriverRetCode ServoDdiAdapter_prvInit(void *vpContext)
{
    ts_ServoDdiAdapterContext *psContext;

    psContext = (ts_ServoDdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpServoHandle == NULL))
    {
        return ACTUATOR_DRIVER_ERR_ARG;
    }

    return ACTUATOR_DRIVER_OK;
}

static te_ActuatorDriverRetCode ServoDdiAdapter_prvWriteAngle(void *vpContext, float f32AngleRad)
{
    ts_ServoDdiAdapterContext *psContext;
    ts_Servo_Handle *psServoHandle;
    te_Driver_RetCode eDriverRet;

    psContext = (ts_ServoDdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpServoHandle == NULL))
    {
        return ACTUATOR_DRIVER_ERR_ARG;
    }

    psServoHandle = (ts_Servo_Handle *)psContext->vpServoHandle;
    eDriverRet = Servo_Write(psServoHandle, &f32AngleRad);

    return ServoDdiAdapter_prvMapRetCode(eDriverRet);
}

void ServoDdiAdapter_Bind(ts_ActuatorDevice *psActuatorDevice,
                          ts_ServoDdiAdapterContext *psAdapterContext,
                          void *vpServoHandle)
{
    static const ts_ActuatorDriverVTable ksServoVtable =
    {
        .pfnInit = ServoDdiAdapter_prvInit,
        .pfnWriteAngle = ServoDdiAdapter_prvWriteAngle
    };

    if ((psActuatorDevice == NULL) || (psAdapterContext == NULL))
    {
        return;
    }

    psAdapterContext->vpServoHandle = vpServoHandle;
    psActuatorDevice->psVTable = &ksServoVtable;
    psActuatorDevice->vpContext = psAdapterContext;
}
