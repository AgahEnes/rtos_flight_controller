#include "sensor_acq.h"

#include <string.h>

#include "gnc_telemetry.h"

/**
 * @brief Initializes acquisition runtime context with static dependencies.
 * @param psContext Input/Output acquisition runtime context.
 * @param psConfig Input acquisition configuration.
 * @return SENSOR_ACQ_OK on success; otherwise error code.
 * @note Side effects: context memory is reset and telemetry sequence is reset.
 */
te_SensorAcqRetCode SensorAcq_Init(ts_SensorAcqContext *psContext, const ts_SensorAcqConfig *psConfig)
{
    if ((psContext == NULL) || (psConfig == NULL))
    {
        return SENSOR_ACQ_ERR_ARG;
    }
    if ((psConfig->psMpuHandle == NULL) ||
        (psConfig->pfnUartSend == NULL) ||
        (psConfig->pu8TxBuffer == NULL) ||
        (psConfig->u16TxBufferLength < GNC_TELEM_IMU_PACKET_LENGTH))
    {
        return SENSOR_ACQ_ERR_ARG;
    }

    (void)memset(psContext, 0, sizeof(*psContext));
    psContext->sConfig = *psConfig;
    psContext->u8Sequence = 0U;
    psContext->u8IsInitialized = 1U;
    return SENSOR_ACQ_OK;
}

/**
 * @brief Executes one deterministic acquisition cycle.
 * @param psContext Input/Output acquisition runtime context.
 * @return SENSOR_ACQ_OK on success; otherwise error code.
 * @note Side effects: reads IMU via driver and transmits one telemetry frame when data is valid.
 */
te_SensorAcqRetCode SensorAcq_Step(ts_SensorAcqContext *psContext)
{
    ts_Mpu6050_Data sMpuData;
    ts_GncTelemImuPayload sImuPayload;
    uint16_t u16PacketLength;
    te_Driver_RetCode eMpuRet;

    if ((psContext == NULL) || (psContext->u8IsInitialized == 0U))
    {
        return SENSOR_ACQ_ERR_STATE;
    }

    eMpuRet = Mpu6050_Read(psContext->sConfig.psMpuHandle, &sMpuData);
    if (eMpuRet != DRIVER_OK)
    {
        return SENSOR_ACQ_ERR_DRIVER;
    }
    if (sMpuData.bValid == false)
    {
        return SENSOR_ACQ_OK;
    }

    sImuPayload.u8Sequence = psContext->u8Sequence;
    sImuPayload.u32TimestampMs = sMpuData.u32TimestampMs;
    sImuPayload.f32AccelXMps2 = sMpuData.sAccelMps2.f32X;
    sImuPayload.f32AccelYMps2 = sMpuData.sAccelMps2.f32Y;
    sImuPayload.f32AccelZMps2 = sMpuData.sAccelMps2.f32Z;
    sImuPayload.f32GyroXRadS = sMpuData.sGyroRadS.f32X;
    sImuPayload.f32GyroYRadS = sMpuData.sGyroRadS.f32Y;
    sImuPayload.f32GyroZRadS = sMpuData.sGyroRadS.f32Z;
    sImuPayload.f32TempC = sMpuData.f32TempC;

    u16PacketLength = GncTelemetry_PackImu(&sImuPayload,
                                           psContext->sConfig.pu8TxBuffer,
                                           psContext->sConfig.u16TxBufferLength);
    if (u16PacketLength == 0U)
    {
        return SENSOR_ACQ_ERR_TELEMETRY;
    }

    if (psContext->sConfig.pfnUartSend(psContext->sConfig.pu8TxBuffer,
                                       u16PacketLength,
                                       psContext->sConfig.vpUartContext) == false)
    {
        return SENSOR_ACQ_ERR_TX;
    }

    psContext->u8Sequence++;
    return SENSOR_ACQ_OK;
}
