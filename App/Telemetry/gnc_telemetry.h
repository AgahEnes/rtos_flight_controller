#ifndef APP_TELEMETRY_GNC_TELEMETRY_H_
#define APP_TELEMETRY_GNC_TELEMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define GNC_TELEM_SYNC_BYTE_0            (0xA5U)
#define GNC_TELEM_SYNC_BYTE_1            (0x5AU)
#define GNC_TELEM_MSG_ID_IMU             (0x10U)
#define GNC_TELEM_IMU_PACKET_LENGTH      (38U)

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
} ts_GncTelemImuPayload;

/**
 * @brief Packs IMU payload into a fixed binary telemetry frame.
 * @param psPayload Input payload pointer.
 * @param pu8OutBuffer Output byte buffer pointer.
 * @param u16OutBufferLen Output buffer length.
 * @return Packed frame length on success, zero on validation failure.
 */
uint16_t GncTelemetry_PackImu(const ts_GncTelemImuPayload *psPayload,
                              uint8_t *pu8OutBuffer,
                              uint16_t u16OutBufferLen);

#ifdef __cplusplus
}
#endif

#endif /* APP_TELEMETRY_GNC_TELEMETRY_H_ */
