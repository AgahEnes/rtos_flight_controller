#include "bmp180_ddi_adapter.h"

#include <string.h>

#include "bmp180_driver.h"

static te_BaroDriverRetCode Bmp180DdiAdapter_prvMapRetCode(te_Driver_RetCode eDriverRet)
{
    te_BaroDriverRetCode eRet;

    switch (eDriverRet)
    {
        case DRIVER_OK:
            eRet = BARO_DRIVER_OK;
            break;
        case DRIVER_ERR_NULL_PTR:
        case DRIVER_ERR_INVALID_ARG:
        case DRIVER_ERR_CONFIG:
            eRet = BARO_DRIVER_ERR_ARG;
            break;
        case DRIVER_ERR_STATE:
            eRet = BARO_DRIVER_ERR_STATE;
            break;
        default:
            eRet = BARO_DRIVER_ERR_IO;
            break;
    }

    return eRet;
}

static te_BaroDriverRetCode Bmp180DdiAdapter_prvInit(void *vpContext)
{
    ts_Bmp180DdiAdapterContext *psContext;

    psContext = (ts_Bmp180DdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpBmp180Handle == NULL))
    {
        return BARO_DRIVER_ERR_ARG;
    }

    return BARO_DRIVER_OK;
}

static te_BaroDriverRetCode Bmp180DdiAdapter_prvStartMeasurement(void *vpContext)
{
    ts_Bmp180DdiAdapterContext *psContext;
    ts_Bmp180_Handle *psBmp180Handle;
    te_Driver_RetCode eDriverRet;

    psContext = (ts_Bmp180DdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpBmp180Handle == NULL))
    {
        return BARO_DRIVER_ERR_ARG;
    }

    psBmp180Handle = (ts_Bmp180_Handle *)psContext->vpBmp180Handle;
    eDriverRet = Bmp180_Ioctl(psBmp180Handle, BMP180_IOCTL_START_MEASUREMENT, NULL);
    return Bmp180DdiAdapter_prvMapRetCode(eDriverRet);
}

static te_BaroDriverRetCode Bmp180DdiAdapter_prvProcess(void *vpContext)
{
    ts_Bmp180DdiAdapterContext *psContext;
    ts_Bmp180_Handle *psBmp180Handle;
    te_Driver_RetCode eDriverRet;

    psContext = (ts_Bmp180DdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpBmp180Handle == NULL))
    {
        return BARO_DRIVER_ERR_ARG;
    }

    psBmp180Handle = (ts_Bmp180_Handle *)psContext->vpBmp180Handle;
    eDriverRet = Bmp180_Ioctl(psBmp180Handle, BMP180_IOCTL_PROCESS_MEASUREMENT, NULL);
    return Bmp180DdiAdapter_prvMapRetCode(eDriverRet);
}

static te_BaroDriverRetCode Bmp180DdiAdapter_prvReadBarometer(void *vpContext,
                                                              ts_TopicBarometer *psBarometer)
{
    ts_Bmp180DdiAdapterContext *psContext;
    ts_Bmp180_Handle *psBmp180Handle;
    ts_Bmp180_Data sBmp180Data;
    te_Driver_RetCode eDriverRet;

    psContext = (ts_Bmp180DdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpBmp180Handle == NULL) || (psBarometer == NULL))
    {
        return BARO_DRIVER_ERR_ARG;
    }

    psBmp180Handle = (ts_Bmp180_Handle *)psContext->vpBmp180Handle;
    (void)memset(&sBmp180Data, 0, sizeof(sBmp180Data));
    eDriverRet = Bmp180_Read(psBmp180Handle, &sBmp180Data);
    if (eDriverRet != DRIVER_OK)
    {
        return Bmp180DdiAdapter_prvMapRetCode(eDriverRet);
    }

    psBarometer->f32TemperatureC = sBmp180Data.f32TemperatureC;
    psBarometer->f32PressurePa = sBmp180Data.f32PressurePa;
    psBarometer->f32AltitudeM = sBmp180Data.f32AltitudeM;
    psBarometer->u32TimestampMs = sBmp180Data.u32TimestampMs;
    psBarometer->bIsValid = sBmp180Data.bValid;

    return BARO_DRIVER_OK;
}

void Bmp180DdiAdapter_Bind(ts_BaroDevice *psBaroDevice,
                           ts_Bmp180DdiAdapterContext *psAdapterContext,
                           void *vpBmp180Handle)
{
    static const ts_BaroDriverVTable ksBmp180Vtable =
    {
        .pfnInit = Bmp180DdiAdapter_prvInit,
        .pfnStartMeasurement = Bmp180DdiAdapter_prvStartMeasurement,
        .pfnProcess = Bmp180DdiAdapter_prvProcess,
        .pfnReadBarometer = Bmp180DdiAdapter_prvReadBarometer
    };

    if ((psBaroDevice == NULL) || (psAdapterContext == NULL))
    {
        return;
    }

    psAdapterContext->vpBmp180Handle = vpBmp180Handle;
    psBaroDevice->psVTable = &ksBmp180Vtable;
    psBaroDevice->vpContext = psAdapterContext;
}
