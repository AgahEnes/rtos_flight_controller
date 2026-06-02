#include "sensor_manager.h"

#include <string.h>

#include "global_data_space.h"

te_SensorManagerRetCode SensorManager_Init(ts_SensorManagerContext *psContext,
                                           const ts_SensorManagerConfig *psConfig)
{
    uint8_t u8Idx;
    te_ImuDriverRetCode eInitRet;
    te_BaroDriverRetCode eBaroInitRet;

    if ((psContext == NULL) || (psConfig == NULL) || (psConfig->psImuDevices == NULL))
    {
        return SENSOR_MANAGER_ERR_ARG;
    }

    if ((psConfig->u8ImuDeviceCount == 0U) || (psConfig->u8ImuDeviceCount > SENSOR_MANAGER_MAX_IMU_DEVICES))
    {
        return SENSOR_MANAGER_ERR_ARG;
    }

    for (u8Idx = 0U; u8Idx < psConfig->u8ImuDeviceCount; u8Idx++)
    {
        if ((psConfig->psImuDevices[u8Idx].psVTable == NULL) ||
            (psConfig->psImuDevices[u8Idx].psVTable->pfnInit == NULL) ||
            (psConfig->psImuDevices[u8Idx].psVTable->pfnReadImu == NULL))
        {
            return SENSOR_MANAGER_ERR_ARG;
        }
    }

    if (psConfig->u8BaroDeviceCount > SENSOR_MANAGER_MAX_BARO_DEVICES)
    {
        return SENSOR_MANAGER_ERR_ARG;
    }

    if ((psConfig->u8BaroDeviceCount > 0U) && (psConfig->psBaroDevices == NULL))
    {
        return SENSOR_MANAGER_ERR_ARG;
    }

    for (u8Idx = 0U; u8Idx < psConfig->u8BaroDeviceCount; u8Idx++)
    {
        if ((psConfig->psBaroDevices[u8Idx].psVTable == NULL) ||
            (psConfig->psBaroDevices[u8Idx].psVTable->pfnInit == NULL) ||
            (psConfig->psBaroDevices[u8Idx].psVTable->pfnStartMeasurement == NULL) ||
            (psConfig->psBaroDevices[u8Idx].psVTable->pfnProcess == NULL) ||
            (psConfig->psBaroDevices[u8Idx].psVTable->pfnReadBarometer == NULL))
        {
            return SENSOR_MANAGER_ERR_ARG;
        }
    }

    (void)memset(psContext, 0, sizeof(*psContext));
    psContext->sConfig = *psConfig;

    for (u8Idx = 0U; u8Idx < psContext->sConfig.u8ImuDeviceCount; u8Idx++)
    {
        eInitRet = psContext->sConfig.psImuDevices[u8Idx].psVTable->pfnInit(
            psContext->sConfig.psImuDevices[u8Idx].vpContext);
        if (eInitRet != IMU_DRIVER_OK)
        {
            return SENSOR_MANAGER_ERR_DRIVER;
        }
    }

    for (u8Idx = 0U; u8Idx < psContext->sConfig.u8BaroDeviceCount; u8Idx++)
    {
        eBaroInitRet = psContext->sConfig.psBaroDevices[u8Idx].psVTable->pfnInit(
            psContext->sConfig.psBaroDevices[u8Idx].vpContext);
        if (eBaroInitRet != BARO_DRIVER_OK)
        {
            return SENSOR_MANAGER_ERR_DRIVER;
        }
    }

    if ((psContext->sConfig.u8BaroDeviceCount > 0U) &&
        (psContext->sConfig.u8BaroReadPeriodTicks > 1U))
    {
        psContext->u8BaroReadTickCounter = (uint8_t)(psContext->sConfig.u8BaroReadPeriodTicks - 1U);
    }

    psContext->u8IsInitialized = 1U;
    return SENSOR_MANAGER_OK;
}

te_SensorManagerRetCode SensorManager_Step(ts_SensorManagerContext *psContext)
{
    uint8_t u8Idx;
    ts_TopicRawImu sRawImu;
    ts_TopicBarometer sBarometer;
    ts_TopicImuCalibration sCalibration;
    te_ImuDriverRetCode eReadRet;
    te_ImuDriverRetCode eSetBiasRet;
    te_BaroDriverRetCode eBaroReadRet;
    te_GdsRetCode eGdsRet;

    if ((psContext == NULL) || (psContext->u8IsInitialized == 0U))
    {
        return SENSOR_MANAGER_ERR_STATE;
    }

    (void)memset(&sCalibration, 0, sizeof(sCalibration));
    eGdsRet = Gds_ReadImuCalibration(&sCalibration);
    if (eGdsRet != GDS_OK)
    {
        return SENSOR_MANAGER_ERR_GDS;
    }
    if ((sCalibration.bIsValid == true) && (sCalibration.u32UpdateCounter != psContext->u32LastAppliedCalibrationCounter))
    {
        for (u8Idx = 0U; u8Idx < psContext->sConfig.u8ImuDeviceCount; u8Idx++)
        {
            if ((psContext->sConfig.psImuDevices[u8Idx].psVTable == NULL) ||
                (psContext->sConfig.psImuDevices[u8Idx].psVTable->pfnSetBias == NULL))
            {
                return SENSOR_MANAGER_ERR_DRIVER;
            }

            eSetBiasRet = psContext->sConfig.psImuDevices[u8Idx].psVTable->pfnSetBias(
                psContext->sConfig.psImuDevices[u8Idx].vpContext,
                &sCalibration);
            if (eSetBiasRet != IMU_DRIVER_OK)
            {
                return SENSOR_MANAGER_ERR_DRIVER;
            }
        }
        psContext->u32LastAppliedCalibrationCounter = sCalibration.u32UpdateCounter;
    }

    for (u8Idx = 0U; u8Idx < psContext->sConfig.u8ImuDeviceCount; u8Idx++)
    {
        (void)memset(&sRawImu, 0, sizeof(sRawImu));
        eReadRet = psContext->sConfig.psImuDevices[u8Idx].psVTable->pfnReadImu(
            psContext->sConfig.psImuDevices[u8Idx].vpContext,
            &sRawImu);
        if (eReadRet != IMU_DRIVER_OK)
        {
            return SENSOR_MANAGER_ERR_DRIVER;
        }

        if ((sRawImu.bIsValid == true) &&
            ((psContext->u8HasLastPublishedImuTimestamp == 0U) ||
             (sRawImu.u32TimestampMs > psContext->u32LastPublishedImuTimestampMs)))
        {
            eGdsRet = Gds_PublishRawImu(&sRawImu);
            if (eGdsRet != GDS_OK)
            {
                return SENSOR_MANAGER_ERR_GDS;
            }

            psContext->u8HasLastPublishedImuTimestamp = 1U;
            psContext->u32LastPublishedImuTimestampMs = sRawImu.u32TimestampMs;
        }
    }

    if (psContext->sConfig.u8BaroDeviceCount == 0U)
    {
        return SENSOR_MANAGER_OK;
    }

    for (u8Idx = 0U; u8Idx < psContext->sConfig.u8BaroDeviceCount; u8Idx++)
    {
        eBaroReadRet = psContext->sConfig.psBaroDevices[u8Idx].psVTable->pfnProcess(
            psContext->sConfig.psBaroDevices[u8Idx].vpContext);
        if (eBaroReadRet != BARO_DRIVER_OK)
        {
            return SENSOR_MANAGER_ERR_DRIVER;
        }

        (void)memset(&sBarometer, 0, sizeof(sBarometer));
        eBaroReadRet = psContext->sConfig.psBaroDevices[u8Idx].psVTable->pfnReadBarometer(
            psContext->sConfig.psBaroDevices[u8Idx].vpContext,
            &sBarometer);
        if (eBaroReadRet != BARO_DRIVER_OK)
        {
            return SENSOR_MANAGER_ERR_DRIVER;
        }

        if ((sBarometer.bIsValid == true) &&
            ((psContext->u8HasLastPublishedBaroTimestamp == 0U) ||
             (sBarometer.u32TimestampMs > psContext->u32LastPublishedBaroTimestampMs)))
        {
            eGdsRet = Gds_PublishBarometer(&sBarometer);
            if (eGdsRet != GDS_OK)
            {
                return SENSOR_MANAGER_ERR_GDS;
            }

            psContext->u8HasLastPublishedBaroTimestamp = 1U;
            psContext->u32LastPublishedBaroTimestampMs = sBarometer.u32TimestampMs;
        }
    }

    psContext->u8BaroReadTickCounter++;
    if ((psContext->sConfig.u8BaroReadPeriodTicks <= 1U) ||
        (psContext->u8BaroReadTickCounter >= psContext->sConfig.u8BaroReadPeriodTicks))
    {
        psContext->u8BaroReadTickCounter = 0U;
        for (u8Idx = 0U; u8Idx < psContext->sConfig.u8BaroDeviceCount; u8Idx++)
        {
            eBaroReadRet = psContext->sConfig.psBaroDevices[u8Idx].psVTable->pfnStartMeasurement(
                psContext->sConfig.psBaroDevices[u8Idx].vpContext);
            if (eBaroReadRet != BARO_DRIVER_OK)
            {
                return SENSOR_MANAGER_ERR_DRIVER;
            }
        }
    }

    return SENSOR_MANAGER_OK;
}
