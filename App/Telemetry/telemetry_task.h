#ifndef APP_TELEMETRY_TELEMETRY_TASK_H_
#define APP_TELEMETRY_TELEMETRY_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define TELEMETRY_TASK_SYNC_BYTE_0               (0xA5U)
#define TELEMETRY_TASK_SYNC_BYTE_1               (0x5AU)
#define TELEMETRY_TASK_MSG_ID_IMU_VEHICLE_STATE  (0x12U)
#define TELEMETRY_TASK_MSG_ID_IMU_CALIBRATION    (0x81U)

#define TELEMETRY_TASK_FRAME_HEADER_LENGTH       (4U)
#define TELEMETRY_TASK_FRAME_CRC_LENGTH          (2U)

#define TELEMETRY_TASK_PACKET_TIMESTAMP_LENGTH   (4U)

/* Payload bytes in combined frame: 7x float (accel/gyro/temp). */
#define TELEMETRY_TASK_IMU_PACKET_LENGTH         (28U)

/* Payload bytes in combined frame: 6x float (attitude/rates) + isEstimated + flightMode. */
#define TELEMETRY_TASK_VEHICLE_PACKET_LENGTH     (26U)

#define TELEMETRY_TASK_FRAME_PAYLOAD_LENGTH      (TELEMETRY_TASK_PACKET_TIMESTAMP_LENGTH + \
                                                    TELEMETRY_TASK_IMU_PACKET_LENGTH + \
                                                    TELEMETRY_TASK_VEHICLE_PACKET_LENGTH)

#define TELEMETRY_TASK_FRAME_LENGTH              (TELEMETRY_TASK_FRAME_HEADER_LENGTH + \
                                                    TELEMETRY_TASK_FRAME_PAYLOAD_LENGTH + \
                                                    TELEMETRY_TASK_FRAME_CRC_LENGTH)

#define TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH      (TELEMETRY_TASK_FRAME_LENGTH)

/* Calibration event frame: telemetry ts + accelBias xyz + gyroBias xyz + cal ts + cal counter + isValid */
#define TELEMETRY_TASK_CALIBRATION_PAYLOAD_LENGTH  (37U)
#define TELEMETRY_TASK_CALIBRATION_FRAME_LENGTH    (TELEMETRY_TASK_FRAME_HEADER_LENGTH + \
                                                    TELEMETRY_TASK_CALIBRATION_PAYLOAD_LENGTH + \
                                                    TELEMETRY_TASK_FRAME_CRC_LENGTH)

/* Max transmit frame length across all telemetry messages. */
#define TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH      (TELEMETRY_TASK_FRAME_LENGTH)

typedef bool (*tpfn_TelemetryTxSend)(const uint8_t *pu8Data, uint16_t u16Length, void *vpContext);
typedef uint32_t (*tpfn_TelemetryGetTickMs)(void *vpContext);

typedef enum
{
    TELEMETRY_TASK_OK = 0,
    TELEMETRY_TASK_ERR_ARG,
    TELEMETRY_TASK_ERR_STATE,
    TELEMETRY_TASK_ERR_GDS,
    TELEMETRY_TASK_ERR_PACK,
    TELEMETRY_TASK_ERR_TX
} te_TelemetryTaskRetCode;

typedef struct
{
    tpfn_TelemetryTxSend pfnUartSend;
    void *vpUartContext;
    tpfn_TelemetryGetTickMs pfnGetTickMs;
    void *vpTickContext;
    uint8_t *pu8TxBuffer;
    uint16_t u16TxBufferLength;
} ts_TelemetryTaskConfig;

typedef struct
{
    ts_TelemetryTaskConfig sConfig;
    uint8_t u8Sequence;
    uint8_t u8NextStepIsEvent;
    uint8_t u8IsInitialized;
    uint32_t u32LastTelemetriedCalibrationCounter;
} ts_TelemetryTaskContext;

te_TelemetryTaskRetCode TelemetryTask_Init(ts_TelemetryTaskContext *psContext,
                                           const ts_TelemetryTaskConfig *psConfig);
te_TelemetryTaskRetCode TelemetryTask_Step(ts_TelemetryTaskContext *psContext);

#ifdef __cplusplus
}
#endif

#endif /* APP_TELEMETRY_TELEMETRY_TASK_H_ */
