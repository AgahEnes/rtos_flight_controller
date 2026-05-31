#include "mpu6050_ddi_adapter.h"

#include <string.h>

#include "mpu6050_driver.h"

static te_ImuDriverRetCode Mpu6050DdiAdapter_prvMapRetCode(te_Driver_RetCode eDriverRet)
{
    te_ImuDriverRetCode eRet;

    switch (eDriverRet)
    {
        case DRIVER_OK:
            eRet = IMU_DRIVER_OK;
            break;
        case DRIVER_ERR_NULL_PTR:
        case DRIVER_ERR_INVALID_ARG:
            eRet = IMU_DRIVER_ERR_ARG;
            break;
        case DRIVER_ERR_STATE:
            eRet = IMU_DRIVER_ERR_STATE;
            break;
        default:
            eRet = IMU_DRIVER_ERR_IO;
            break;
    }

    return eRet;
}

static te_ImuDriverRetCode Mpu6050DdiAdapter_prvInit(void *vpContext)
{
    ts_Mpu6050DdiAdapterContext *psContext;

    psContext = (ts_Mpu6050DdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpMpuHandle == NULL))
    {
        return IMU_DRIVER_ERR_ARG;
    }

    return IMU_DRIVER_OK;
}

static te_ImuDriverRetCode Mpu6050DdiAdapter_prvReadImu(void *vpContext, ts_TopicRawImu *psRawImu)
{
    ts_Mpu6050DdiAdapterContext *psContext;
    ts_Mpu6050_Handle *psMpuHandle;
    ts_Mpu6050_Data sMpuData;
    te_Driver_RetCode eDriverRet;

    psContext = (ts_Mpu6050DdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpMpuHandle == NULL) || (psRawImu == NULL))
    {
        return IMU_DRIVER_ERR_ARG;
    }

    psMpuHandle = (ts_Mpu6050_Handle *)psContext->vpMpuHandle;
    (void)memset(&sMpuData, 0, sizeof(sMpuData));
    eDriverRet = Mpu6050_Read(psMpuHandle, &sMpuData);
    if (eDriverRet != DRIVER_OK)
    {
        return Mpu6050DdiAdapter_prvMapRetCode(eDriverRet);
    }

    psRawImu->sAccel.f32X = sMpuData.sAccelMps2.f32X;
    psRawImu->sAccel.f32Y = sMpuData.sAccelMps2.f32Y;
    psRawImu->sAccel.f32Z = sMpuData.sAccelMps2.f32Z;
    psRawImu->sGyro.f32X = sMpuData.sGyroRadS.f32X;
    psRawImu->sGyro.f32Y = sMpuData.sGyroRadS.f32Y;
    psRawImu->sGyro.f32Z = sMpuData.sGyroRadS.f32Z;
    psRawImu->f32TempC = sMpuData.f32TempC;
    psRawImu->u32TimestampMs = sMpuData.u32TimestampMs;
    psRawImu->bIsValid = sMpuData.bValid;

    return IMU_DRIVER_OK;
}

static te_ImuDriverRetCode Mpu6050DdiAdapter_prvSetBias(void *vpContext,
                                                        const ts_TopicImuCalibration *psCalibration)
{
    ts_Mpu6050DdiAdapterContext *psContext;
    ts_Mpu6050_Handle *psMpuHandle;
    ts_Mpu6050_Calibration sCalibration;
    te_Driver_RetCode eDriverRet;

    psContext = (ts_Mpu6050DdiAdapterContext *)vpContext;
    if ((psContext == NULL) || (psContext->vpMpuHandle == NULL) || (psCalibration == NULL))
    {
        return IMU_DRIVER_ERR_ARG;
    }
    if (psCalibration->bIsValid == false)
    {
        return IMU_DRIVER_ERR_ARG;
    }

    psMpuHandle = (ts_Mpu6050_Handle *)psContext->vpMpuHandle;
    (void)memset(&sCalibration, 0, sizeof(sCalibration));

    eDriverRet = Mpu6050_Ioctl(psMpuHandle, MPU6050_IOCTL_GET_CALIBRATION, &sCalibration);
    if (eDriverRet != DRIVER_OK)
    {
        return Mpu6050DdiAdapter_prvMapRetCode(eDriverRet);
    }

    sCalibration.sAccelBiasMps2.f32X = psCalibration->sAccelBiasMps2.f32X;
    sCalibration.sAccelBiasMps2.f32Y = psCalibration->sAccelBiasMps2.f32Y;
    sCalibration.sAccelBiasMps2.f32Z = psCalibration->sAccelBiasMps2.f32Z;
    sCalibration.sGyroBiasRadS.f32X = psCalibration->sGyroBiasRadS.f32X;
    sCalibration.sGyroBiasRadS.f32Y = psCalibration->sGyroBiasRadS.f32Y;
    sCalibration.sGyroBiasRadS.f32Z = psCalibration->sGyroBiasRadS.f32Z;

    eDriverRet = Mpu6050_Ioctl(psMpuHandle, MPU6050_IOCTL_SET_CALIBRATION, &sCalibration);
    if (eDriverRet != DRIVER_OK)
    {
        return Mpu6050DdiAdapter_prvMapRetCode(eDriverRet);
    }

    return IMU_DRIVER_OK;
}

void Mpu6050DdiAdapter_Bind(ts_ImuDevice *psImuDevice,
                            ts_Mpu6050DdiAdapterContext *psAdapterContext,
                            void *vpMpuHandle)
{
    static const ts_ImuDriverVTable ksMpu6050Vtable =
    {
        .pfnInit = Mpu6050DdiAdapter_prvInit,
        .pfnReadImu = Mpu6050DdiAdapter_prvReadImu,
        .pfnSetBias = Mpu6050DdiAdapter_prvSetBias
    };

    if ((psImuDevice == NULL) || (psAdapterContext == NULL))
    {
        return;
    }

    psAdapterContext->vpMpuHandle = vpMpuHandle;
    psImuDevice->psVTable = &ksMpu6050Vtable;
    psImuDevice->vpContext = psAdapterContext;
}
