#ifndef APP_SENSOR_ACQ_SENSOR_ACQ_H_
#define APP_SENSOR_ACQ_SENSOR_ACQ_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "mpu6050_driver.h"
#include "port_uart.h"

typedef enum
{
    SENSOR_ACQ_OK = 0,
    SENSOR_ACQ_ERR_ARG,
    SENSOR_ACQ_ERR_STATE,
    SENSOR_ACQ_ERR_DRIVER,
    SENSOR_ACQ_ERR_TELEMETRY,
    SENSOR_ACQ_ERR_TX
} te_SensorAcqRetCode;

typedef struct
{
    ts_Mpu6050_Handle *psMpuHandle;
    tpfn_PortUartSend pfnUartSend;
    void *vpUartContext;
    uint8_t *pu8TxBuffer;
    uint16_t u16TxBufferLength;
} ts_SensorAcqConfig;

typedef struct
{
    ts_SensorAcqConfig sConfig;
    uint8_t u8Sequence;
    uint8_t u8IsInitialized;
} ts_SensorAcqContext;

/**
 * @brief Initializes Sensor Acquisition state.
 * @param psContext Acquisition context pointer.
 * @param psConfig Initialization configuration pointer.
 * @return SENSOR_ACQ_OK on success, otherwise error code.
 */
te_SensorAcqRetCode SensorAcq_Init(ts_SensorAcqContext *psContext, const ts_SensorAcqConfig *psConfig);

/**
 * @brief Executes one acquisition cycle and emits one telemetry frame when valid data exists.
 * @param psContext Acquisition context pointer.
 * @return SENSOR_ACQ_OK on success, otherwise error code.
 */
te_SensorAcqRetCode SensorAcq_Step(ts_SensorAcqContext *psContext);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_ACQ_SENSOR_ACQ_H_ */
