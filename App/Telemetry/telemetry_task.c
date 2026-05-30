#include "telemetry_task.h"

#include <string.h>

#include "global_data_space.h"

#define TELEMETRY_TASK_CRC16_INIT                (0xFFFFU)
#define TELEMETRY_TASK_CRC16_POLY                (0x1021U)

typedef struct
{
    uint8_t u8Sequence;
    uint32_t u32TimestampMs;
    float f32AccelXMps2;
    float f32AccelYMps2;
    float f32AccelZMps2;
    float f32GyroXRadS;
    float f32GyroYRadS;
    float f32GyroZRadS;
    float f32TempC;
} ts_TelemetryImuPayload;

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

static uint16_t TelemetryTask_prvPackImu(const ts_TelemetryImuPayload *psPayload,
                                         uint8_t *pu8OutBuffer,
                                         uint16_t u16OutBufferLen)
{
    uint16_t u16Crc;
    uint16_t u16Offset = 0U;

    if ((psPayload == NULL) || (pu8OutBuffer == NULL) || (u16OutBufferLen < TELEMETRY_TASK_IMU_PACKET_LENGTH))
    {
        return 0U;
    }

    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_SYNC_BYTE_0;
    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_SYNC_BYTE_1;
    pu8OutBuffer[u16Offset++] = TELEMETRY_TASK_MSG_ID_IMU;
    pu8OutBuffer[u16Offset++] = psPayload->u8Sequence;

    TelemetryTask_prvWriteU32Le(&pu8OutBuffer[u16Offset], psPayload->u32TimestampMs);
    u16Offset += 4U;

    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32AccelXMps2);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32AccelYMps2);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32AccelZMps2);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32GyroXRadS);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32GyroYRadS);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32GyroZRadS);
    u16Offset += 4U;
    TelemetryTask_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32TempC);
    u16Offset += 4U;

    u16Crc = TelemetryTask_prvCrc16Ccitt(pu8OutBuffer, u16Offset);
    pu8OutBuffer[u16Offset++] = (uint8_t)(u16Crc & 0xFFU);
    pu8OutBuffer[u16Offset++] = (uint8_t)((u16Crc >> 8U) & 0xFFU);

    return u16Offset;
}

te_TelemetryTaskRetCode TelemetryTask_Init(ts_TelemetryTaskContext *psContext,
                                           const ts_TelemetryTaskConfig *psConfig)
{
    if ((psContext == NULL) || (psConfig == NULL))
    {
        return TELEMETRY_TASK_ERR_ARG;
    }

    if ((psConfig->pfnUartSend == NULL) ||
        (psConfig->pu8TxBuffer == NULL) ||
        (psConfig->u16TxBufferLength < TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH))
    {
        return TELEMETRY_TASK_ERR_ARG;
    }

    (void)memset(psContext, 0, sizeof(*psContext));
    psContext->sConfig = *psConfig;
    psContext->u8IsInitialized = 1U;
    psContext->u8Sequence = 0U;

    return TELEMETRY_TASK_OK;
}

te_TelemetryTaskRetCode TelemetryTask_Step(ts_TelemetryTaskContext *psContext)
{
    ts_TopicRawImu sRawImu;
    ts_TelemetryImuPayload sImuPayload;
    te_GdsRetCode eGdsRet;
    uint16_t u16PacketLength;

    if ((psContext == NULL) || (psContext->u8IsInitialized == 0U))
    {
        return TELEMETRY_TASK_ERR_STATE;
    }

    (void)memset(&sRawImu, 0, sizeof(sRawImu));
    eGdsRet = Gds_ReadRawImu(&sRawImu);
    if (eGdsRet != GDS_OK)
    {
        return TELEMETRY_TASK_ERR_GDS;
    }
    if (sRawImu.bIsValid == false)
    {
        return TELEMETRY_TASK_OK;
    }

    sImuPayload.u8Sequence = psContext->u8Sequence;
    sImuPayload.u32TimestampMs = sRawImu.u32TimestampMs;
    sImuPayload.f32AccelXMps2 = sRawImu.sAccel.f32X;
    sImuPayload.f32AccelYMps2 = sRawImu.sAccel.f32Y;
    sImuPayload.f32AccelZMps2 = sRawImu.sAccel.f32Z;
    sImuPayload.f32GyroXRadS = sRawImu.sGyro.f32X;
    sImuPayload.f32GyroYRadS = sRawImu.sGyro.f32Y;
    sImuPayload.f32GyroZRadS = sRawImu.sGyro.f32Z;
    sImuPayload.f32TempC = 0.0F;

    u16PacketLength = TelemetryTask_prvPackImu(&sImuPayload,
                                               psContext->sConfig.pu8TxBuffer,
                                               psContext->sConfig.u16TxBufferLength);
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
