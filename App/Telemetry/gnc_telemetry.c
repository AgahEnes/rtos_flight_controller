#include "gnc_telemetry.h"

#include <string.h>

#define GNC_TELEM_CRC16_INIT             (0xFFFFU)
#define GNC_TELEM_CRC16_POLY             (0x1021U)

/**
 * @brief Calculates CRC-16/CCITT-FALSE over a byte buffer.
 * @param pu8Data Input data pointer.
 * @param u16Length Number of bytes.
 * @return Calculated CRC16 value.
 */
static uint16_t GncTelemetry_prvCrc16Ccitt(const uint8_t *pu8Data, uint16_t u16Length)
{
    uint16_t u16Crc = GNC_TELEM_CRC16_INIT;
    uint16_t u16Idx;
    uint8_t u8Bit;

    for (u16Idx = 0U; u16Idx < u16Length; ++u16Idx)
    {
        u16Crc ^= ((uint16_t)pu8Data[u16Idx] << 8U);
        for (u8Bit = 0U; u8Bit < 8U; ++u8Bit)
        {
            if ((u16Crc & 0x8000U) != 0U)
            {
                u16Crc = (uint16_t)((u16Crc << 1U) ^ GNC_TELEM_CRC16_POLY);
            }
            else
            {
                u16Crc <<= 1U;
            }
        }
    }

    return u16Crc;
}

/**
 * @brief Writes uint32 value in little-endian format.
 * @param pu8Dest Destination pointer.
 * @param u32Value Input value.
 */
static void GncTelemetry_prvWriteU32Le(uint8_t *pu8Dest, uint32_t u32Value)
{
    pu8Dest[0] = (uint8_t)(u32Value & 0xFFU);
    pu8Dest[1] = (uint8_t)((u32Value >> 8U) & 0xFFU);
    pu8Dest[2] = (uint8_t)((u32Value >> 16U) & 0xFFU);
    pu8Dest[3] = (uint8_t)((u32Value >> 24U) & 0xFFU);
}

/**
 * @brief Writes float value in little-endian format.
 * @param pu8Dest Destination pointer.
 * @param f32Value Input value.
 */
static void GncTelemetry_prvWriteF32Le(uint8_t *pu8Dest, float f32Value)
{
    uint32_t u32Raw = 0U;
    (void)memcpy(&u32Raw, &f32Value, sizeof(u32Raw));
    GncTelemetry_prvWriteU32Le(pu8Dest, u32Raw);
}

uint16_t GncTelemetry_PackImu(const ts_GncTelemImuPayload *psPayload,
                              uint8_t *pu8OutBuffer,
                              uint16_t u16OutBufferLen)
{
    uint16_t u16Crc;
    uint16_t u16Offset = 0U;

    if ((psPayload == NULL) || (pu8OutBuffer == NULL) || (u16OutBufferLen < GNC_TELEM_IMU_PACKET_LENGTH))
    {
        return 0U;
    }

    pu8OutBuffer[u16Offset++] = GNC_TELEM_SYNC_BYTE_0;
    pu8OutBuffer[u16Offset++] = GNC_TELEM_SYNC_BYTE_1;
    pu8OutBuffer[u16Offset++] = GNC_TELEM_MSG_ID_IMU;
    pu8OutBuffer[u16Offset++] = psPayload->u8Sequence;

    GncTelemetry_prvWriteU32Le(&pu8OutBuffer[u16Offset], psPayload->u32TimestampMs);
    u16Offset += 4U;

    GncTelemetry_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32AccelXMps2);
    u16Offset += 4U;
    GncTelemetry_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32AccelYMps2);
    u16Offset += 4U;
    GncTelemetry_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32AccelZMps2);
    u16Offset += 4U;
    GncTelemetry_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32GyroXRadS);
    u16Offset += 4U;
    GncTelemetry_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32GyroYRadS);
    u16Offset += 4U;
    GncTelemetry_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32GyroZRadS);
    u16Offset += 4U;
    GncTelemetry_prvWriteF32Le(&pu8OutBuffer[u16Offset], psPayload->f32TempC);
    u16Offset += 4U;

    u16Crc = GncTelemetry_prvCrc16Ccitt(pu8OutBuffer, u16Offset);
    pu8OutBuffer[u16Offset++] = (uint8_t)(u16Crc & 0xFFU);
    pu8OutBuffer[u16Offset++] = (uint8_t)((u16Crc >> 8U) & 0xFFU);

    return u16Offset;
}
