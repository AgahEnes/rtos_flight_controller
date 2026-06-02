#include "telemetry_task.h"

#include <string.h>

#include "global_data_space.h"

#define TELEMETRY_TASK_CRC16_INIT                (0xFFFFU)
#define TELEMETRY_TASK_CRC16_POLY                (0x1021U)

typedef struct
{
    float f32AccelXMps2;
    float f32AccelYMps2;
    float f32AccelZMps2;
    float f32GyroXRadS;
    float f32GyroYRadS;
    float f32GyroZRadS;
    float f32TempC;
} ts_TelemetryImuPayload;

typedef struct
{
    float f32RollRad;
    float f32PitchRad;
    float f32YawRad;
    float f32RollRateRadS;
    float f32PitchRateRadS;
    float f32YawRateRadS;
    uint8_t u8IsEstimated;
    uint8_t u8FlightMode;
} ts_TelemetryVehicleStatePayload;

typedef struct
{
    uint8_t u8Sequence;
    uint32_t u32TimestampMs;
    ts_TelemetryImuPayload sImu;
    ts_TelemetryVehicleStatePayload sVehicleState;
} ts_TelemetryPayload;

typedef struct
{
    uint8_t u8Sequence;
    uint32_t u32TelemetryTimestampMs;
    ts_TopicImuCalibration sCalibration;
} ts_TelemetryCalibrationPayload;

static uint16_t TelemetryTask_prvCrc16Ccitt(const uint8_t *pu8Data, uint16_t u16Length)
{
    uint16_t u16Crc = TELEMETRY_TASK_CRC16_INIT;
    uint16_t u16Idx;
    uint8_t u8Bit;

    for (u16Idx = 0U; u16Idx < u16Length; ++u16Idx)
    {
        u16Crc ^= ((uint16_t)pu8Data[u16Idx] << 8U);
        for (u8Bit = 0U; u8Bit < 8U; ++u8Bit)
        {
            if ((u16Crc & 0x8000U) != 0U)
            {
                u16Crc = (uint16_t)((u16Crc << 1U) ^ TELEMETRY_TASK_CRC16_POLY);
            }
            else
            {
                u16Crc <<= 1U;
            }
        }
    }

    return u16Crc;
}

static void TelemetryTask_prvWriteU32Le(uint8_t *pu8Dest, uint32_t u32Value)
{
    pu8Dest[0] = (uint8_t)(u32Value & 0xFFU);
    pu8Dest[1] = (uint8_t)((u32Value >> 8U) & 0xFFU);
    pu8Dest[2] = (uint8_t)((u32Value >> 16U) & 0xFFU);
    pu8Dest[3] = (uint8_t)((u32Value >> 24U) & 0xFFU);
}

static void TelemetryTask_prvWriteF32Le(uint8_t *pu8Dest, float f32Value)
{
    uint32_t u32Raw = 0U;

    (void)memcpy(&u32Raw, &f32Value, sizeof(u32Raw));
    TelemetryTask_prvWriteU32Le(pu8Dest, u32Raw);
}

static uint16_t TelemetryTask_prvPackMessage(const ts_TelemetryPayload *psPayload,
                                                     uint8_t *pu8OutBuffer,
                                                     uint16_t u16OutBufferLen)
{
    uint16_t u16Crc;
    uint16_t u16Offset = 0U;

    if ((psPayload == NULL) || (pu8OutBuffer == NULL) || (u16OutBufferLen < TELEMETRY_TASK_FRAME_LENGTH))
    {
        return 0U;
    }

    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_SYNC_BYTE_0;
    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_SYNC_BYTE_1;
    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE;
    pu8OutBuffer[u16Offset++] = psPayload->u8Sequence;

    TelemetryTask_prvWriteU32Le(&pu8OutBuffer[u16Offset], psPayload->u32TimestampMs);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sImu.f32AccelXMps2);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sImu.f32AccelYMps2);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sImu.f32AccelZMps2);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sImu.f32GyroXRadS);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sImu.f32GyroYRadS);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sImu.f32GyroZRadS);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sImu.f32TempC);
    u16Offset += 4U;

    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sVehicleState.f32RollRad);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sVehicleState.f32PitchRad);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sVehicleState.f32YawRad);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sVehicleState.f32RollRateRadS);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sVehicleState.f32PitchRateRadS);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sVehicleState.f32YawRateRadS);
    u16Offset += 4U;
    pu8OutBuffer[u16Offset++] = psPayload->sVehicleState.u8IsEstimated;
    pu8OutBuffer[u16Offset++] = psPayload->sVehicleState.u8FlightMode;

    u16Crc = TelemetryTask_prvCrc16Ccitt(pu8OutBuffer, u16Offset);
    pu8OutBuffer[u16Offset++] = (uint8_t)(u16Crc & 0xFFU);
    pu8OutBuffer[u16Offset++] = (uint8_t)((u16Crc >> 8U) & 0xFFU);

    return u16Offset;
}

static uint16_t TelemetryTask_prvPackCalibrationMessage(const ts_TelemetryCalibrationPayload *psPayload,
                                                        uint8_t *pu8OutBuffer,
                                                        uint16_t u16OutBufferLen)
{
    uint16_t u16Crc;
    uint16_t u16Offset = 0U;

    if ((psPayload == NULL) ||
        (pu8OutBuffer == NULL) ||
        (u16OutBufferLen < TELEMETRY_TASK_CALIBRATION_FRAME_LENGTH))
    {
        return 0U;
    }

    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_SYNC_BYTE_0;
    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_SYNC_BYTE_1;
    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_MSG_ID_IMU_CALIBRATION;
    pu8OutBuffer[u16Offset++] = psPayload->u8Sequence;

    TelemetryTask_prvWriteU32Le(&pu8OutBuffer[u16Offset], psPayload->u32TelemetryTimestampMs);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sCalibration.sAccelBiasMps2.f32X);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sCalibration.sAccelBiasMps2.f32Y);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sCalibration.sAccelBiasMps2.f32Z);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sCalibration.sGyroBiasRadS.f32X);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sCalibration.sGyroBiasRadS.f32Y);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->sCalibration.sGyroBiasRadS.f32Z);
    u16Offset += 4U;
    TelemetryTask_prvWriteU32Le(&pu8OutBuffer[u16Offset], psPayload->sCalibration.u32TimestampMs);
    u16Offset += 4U;
    TelemetryTask_prvWriteU32Le(&pu8OutBuffer[u16Offset], psPayload->sCalibration.u32UpdateCounter);
    u16Offset += 4U;
    pu8OutBuffer[u16Offset++] = (psPayload->sCalibration.bIsValid == true) ? 1U : 0U;

    u16Crc = TelemetryTask_prvCrc16Ccitt(pu8OutBuffer, u16Offset);
    pu8OutBuffer[u16Offset++] = (uint8_t)(u16Crc & 0xFFU);
    pu8OutBuffer[u16Offset++] = (uint8_t)((u16Crc >> 8U) & 0xFFU);

    return u16Offset;
}

static te_TelemetryTaskRetCode TelemetryTask_prvSendPackedFrame(ts_TelemetryTaskContext *psContext,
                                                                uint16_t u16PacketLength)
{
    if (u16PacketLength == 0U)
    {
        return TELEMETRY_TASK_ERR_PACK;
    }

    if (psContext->sConfig.pfnUartSend(psContext->sConfig.pu8TxBuffer,
                                       u16PacketLength,
                                       psContext->sConfig.vpUartContext) == false)
    {
        return TELEMETRY_TASK_ERR_TX;
    }

    psContext->u8Sequence++;
    return TELEMETRY_TASK_OK;
}

static te_TelemetryTaskRetCode TelemetryTask_prvRunSyncSlot(ts_TelemetryTaskContext *psContext)
{
    ts_TopicRawImu sRawImu;
    ts_TopicVehicleState sVehicleState;
    ts_TopicFlightStatus sFlightStatus;
    ts_TelemetryPayload sPayload;
    te_GdsRetCode eGdsRet;
    uint16_t u16PacketLength;

    (void)memset(&sRawImu, 0, sizeof(sRawImu));
    eGdsRet = Gds_ReadRawImu(&sRawImu);
    if (eGdsRet != GDS_OK)
    {
        (void)memset(&sRawImu, 0, sizeof(sRawImu));
    }

    (void)memset(&sVehicleState, 0, sizeof(sVehicleState));
    eGdsRet = Gds_ReadVehicleState(&sVehicleState);
    if (eGdsRet != GDS_OK)
    {
        (void)memset(&sVehicleState, 0, sizeof(sVehicleState));
    }

    (void)memset(&sFlightStatus, 0, sizeof(sFlightStatus));
    eGdsRet = Gds_ReadFlightStatus(&sFlightStatus);
    if (eGdsRet != GDS_OK)
    {
        (void)memset(&sFlightStatus, 0, sizeof(sFlightStatus));
    }

    sPayload.u8Sequence = psContext->u8Sequence;
    sPayload.u32TimestampMs = psContext->sConfig.pfnGetTickMs(psContext->sConfig.vpTickContext);
    sPayload.sImu.f32AccelXMps2 = sRawImu.sAccel.f32X;
    sPayload.sImu.f32AccelYMps2 = sRawImu.sAccel.f32Y;
    sPayload.sImu.f32AccelZMps2 = sRawImu.sAccel.f32Z;
    sPayload.sImu.f32GyroXRadS = sRawImu.sGyro.f32X;
    sPayload.sImu.f32GyroYRadS = sRawImu.sGyro.f32Y;
    sPayload.sImu.f32GyroZRadS = sRawImu.sGyro.f32Z;
    sPayload.sImu.f32TempC = sRawImu.f32TempC;
    sPayload.sVehicleState.f32RollRad = sVehicleState.f32RollRad;
    sPayload.sVehicleState.f32PitchRad = sVehicleState.f32PitchRad;
    sPayload.sVehicleState.f32YawRad = sVehicleState.f32YawRad;
    sPayload.sVehicleState.f32RollRateRadS = sVehicleState.f32RollRateRadS;
    sPayload.sVehicleState.f32PitchRateRadS = sVehicleState.f32PitchRateRadS;
    sPayload.sVehicleState.f32YawRateRadS = sVehicleState.f32YawRateRadS;
    sPayload.sVehicleState.u8IsEstimated = (sVehicleState.bIsEstimated == true) ? 1U : 0U;
    sPayload.sVehicleState.u8FlightMode = sFlightStatus.u8FlightMode;

    u16PacketLength = TelemetryTask_prvPackMessage(&sPayload,
                                                   psContext->sConfig.pu8TxBuffer,
                                                   psContext->sConfig.u16TxBufferLength);
    return TelemetryTask_prvSendPackedFrame(psContext, u16PacketLength);
}

static te_TelemetryTaskRetCode TelemetryTask_prvRunEventSlot(ts_TelemetryTaskContext *psContext)
{
    ts_TopicImuCalibration sCalibration;
    ts_TelemetryCalibrationPayload sPayload;
    te_TelemetryTaskRetCode eRet;
    te_GdsRetCode eGdsRet;
    uint16_t u16PacketLength;

    (void)memset(&sCalibration, 0, sizeof(sCalibration));
    eGdsRet = Gds_ReadImuCalibration(&sCalibration);
    if (eGdsRet != GDS_OK)
    {
        return TELEMETRY_TASK_ERR_GDS;
    }

    if ((sCalibration.bIsValid == false) ||
        (sCalibration.u32UpdateCounter == psContext->u32LastTelemetriedCalibrationCounter))
    {
        return TELEMETRY_TASK_OK;
    }

    sPayload.u8Sequence = psContext->u8Sequence;
    sPayload.u32TelemetryTimestampMs = psContext->sConfig.pfnGetTickMs(psContext->sConfig.vpTickContext);
    sPayload.sCalibration = sCalibration;

    u16PacketLength = TelemetryTask_prvPackCalibrationMessage(&sPayload,
                                                              psContext->sConfig.pu8TxBuffer,
                                                              psContext->sConfig.u16TxBufferLength);
    eRet = TelemetryTask_prvSendPackedFrame(psContext, u16PacketLength);
    if (eRet == TELEMETRY_TASK_OK)
    {
        psContext->u32LastTelemetriedCalibrationCounter = sCalibration.u32UpdateCounter;
    }

    return eRet;
}

te_TelemetryTaskRetCode TelemetryTask_Init(ts_TelemetryTaskContext *psContext,
                                           const ts_TelemetryTaskConfig *psConfig)
{
    if ((psContext == NULL) || (psConfig == NULL))
    {
        return TELEMETRY_TASK_ERR_ARG;
    }

    if ((psConfig->pfnUartSend == NULL) ||
        (psConfig->pfnGetTickMs == NULL) ||
        (psConfig->pu8TxBuffer == NULL) ||
        (psConfig->u16TxBufferLength < TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH))
    {
        return TELEMETRY_TASK_ERR_ARG;
    }

    (void)memset(psContext, 0, sizeof(*psContext));
    psContext->sConfig = *psConfig;
    psContext->u8IsInitialized = 1U;
    psContext->u8Sequence = 0U;
    psContext->u8NextStepIsEvent = 0U;
    psContext->u32LastTelemetriedCalibrationCounter = 0U;

    return TELEMETRY_TASK_OK;
}

te_TelemetryTaskRetCode TelemetryTask_Step(ts_TelemetryTaskContext *psContext)
{
    te_TelemetryTaskRetCode eRet;

    if ((psContext == NULL) || (psContext->u8IsInitialized == 0U))
    {
        return TELEMETRY_TASK_ERR_STATE;
    }

    if (psContext->u8NextStepIsEvent == 0U)
    {
        eRet = TelemetryTask_prvRunSyncSlot(psContext);
        psContext->u8NextStepIsEvent = 1U;
    }
    else
    {
        eRet = TelemetryTask_prvRunEventSlot(psContext);
        psContext->u8NextStepIsEvent = 0U;
    }

    return eRet;
}
