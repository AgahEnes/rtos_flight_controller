#include "sensor_acq_port.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "main.h"
#include "mpu6050_hal.h"
#include "mpu6050_driver.h"
#include "mpu6050_stm32_hal_port.h"
#include "global_data_space.h"
#include "mpu6050_ddi_adapter.h"
#include "sensor_manager.h"
#include "telemetry_task.h"

#define SENSOR_ACQ_TASK_STACK_WORDS              (512U)
#define TELEMETRY_TASK_STACK_WORDS               (512U)
#define SENSOR_ACQ_TASK_PERIOD_MS                (10U)
#define TELEMETRY_TASK_PERIOD_MS                 (100U)
#define SENSOR_ACQ_TASK_PRIORITY                 (osPriorityAboveNormal)
#define TELEMETRY_TASK_PRIORITY                  (osPriorityNormal)
#define SENSOR_ACQ_BUS_TIMEOUT_MS                (100U)
#define SENSOR_ACQ_BUS_LOCK_TIMEOUT_MS           (20U)
#define SENSOR_ACQ_UART_TX_TIMEOUT_MS            (5U)
#define SENSOR_IMU_DEVICE_COUNT                  (1U)

static I2C_HandleTypeDef *gpxSensorI2cHandle = NULL;
static UART_HandleTypeDef *gpxSensorUartHandle = NULL;
static osThreadId_t gxSensorAcqTaskHandle = NULL;
static osThreadId_t gxTelemetryTaskHandle = NULL;
static osMutexId_t gxI2cBusMutex = NULL;

static ts_Mpu6050_Handle gsMpuHandle;
static ts_Mpu6050_Stm32BusContext gsMpuBusContext;
static ts_Mpu6050DdiAdapterContext gsMpuDdiContext;
static ts_ImuDevice gasImuDevices[SENSOR_IMU_DEVICE_COUNT];
static ts_SensorManagerContext gsSensorManagerContext;
static ts_TelemetryTaskContext gsTelemetryTaskContext;
static uint8_t gau8TelemetryTxBuffer[TELEMETRY_TASK_MIN_TX_BUFFER_LENGTH];

static StaticTask_t gsSensorAcqTaskCb;
static StackType_t gau32SensorAcqTaskStack[SENSOR_ACQ_TASK_STACK_WORDS];
static StaticTask_t gsTelemetryTaskCb;
static StackType_t gau32TelemetryTaskStack[TELEMETRY_TASK_STACK_WORDS];
static StaticSemaphore_t gsI2cBusMutexCb;

/**
 * @brief Enables or disables EXTI line for MPU6050 interrupt pin.
 * @param vpContext Unused callback context.
 * @param bEnable true enables IRQ, false disables IRQ.
 * @return DRIVER_OK on success.
 */
static te_Driver_RetCode SensorAcqPort_prvInterruptPinControl(void *vpContext, bool bEnable)
{
    (void)vpContext;

    if (bEnable == true)
    {
        HAL_NVIC_EnableIRQ(MPU6050_INT_EXTI_IRQn);
    }
    else
    {
        HAL_NVIC_DisableIRQ(MPU6050_INT_EXTI_IRQn);
    }
    return DRIVER_OK;
}

/**
 * @brief Sends telemetry packet over configured UART.
 * @param pu8Data Binary packet pointer.
 * @param u16Length Packet length in bytes.
 * @param vpContext Unused callback context.
 * @return true on successful UART transmission.
 */
static bool SensorAcqPort_prvUartSend(const uint8_t *pu8Data, uint16_t u16Length, void *vpContext)
{
    (void)vpContext;

    if ((gpxSensorUartHandle == NULL) || (pu8Data == NULL) || (u16Length == 0U))
    {
        return false;
    }

    return (HAL_UART_Transmit(gpxSensorUartHandle,
                              (uint8_t *)pu8Data,
                              u16Length,
                              SENSOR_ACQ_UART_TX_TIMEOUT_MS) == HAL_OK);
}

/**
 * @brief Initializes MPU6050 open configuration and starts async DMA worker.
 * @return true on success, false on failure.
 */
static bool SensorAcqPort_prvInitMpu6050(void)
{
    ts_Mpu6050_OpenConfig sOpenConfig;
    ts_BusInterface sBusIf;
    ts_LockInterface sLockIf;
    ts_Mpu6050_TimingInterface sTimingIf;
    te_Driver_RetCode eRet;
    te_Mpu6050_ReadMode eReadMode;
    uint8_t u8IntEnableMask = 0x01U;

    (void)memset(&gsMpuHandle, 0, sizeof(gsMpuHandle));
    (void)memset(&gsMpuBusContext, 0, sizeof(gsMpuBusContext));
    (void)memset(&gsMpuDdiContext, 0, sizeof(gsMpuDdiContext));
    (void)memset(&gasImuDevices, 0, sizeof(gasImuDevices));
    (void)memset(&gsSensorManagerContext, 0, sizeof(gsSensorManagerContext));
    (void)memset(&gsTelemetryTaskContext, 0, sizeof(gsTelemetryTaskContext));
    (void)memset(&sOpenConfig, 0, sizeof(sOpenConfig));

    gsMpuBusContext.pxI2cHandle = gpxSensorI2cHandle;
    gsMpuBusContext.xBusMutex = gxI2cBusMutex;

    eRet = Mpu6050_Stm32Hal_FillBusInterface(&sBusIf, &gsMpuBusContext);
    if (eRet != DRIVER_OK)
    {
        return false;
    }
    eRet = Mpu6050_Stm32Hal_FillLockInterface(&sLockIf, &gsMpuBusContext);
    if (eRet != DRIVER_OK)
    {
        return false;
    }
    eRet = Mpu6050_Stm32Hal_FillTimingInterface(&sTimingIf);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    sOpenConfig.u8I2cAddress = MPU6050_I2C_ADDR_AD0_LOW;
    sOpenConfig.u32BusTimeoutMs = SENSOR_ACQ_BUS_TIMEOUT_MS;
    sOpenConfig.u32BusLockTimeoutMs = SENSOR_ACQ_BUS_LOCK_TIMEOUT_MS;
    sOpenConfig.sBusInterface = sBusIf;
    sOpenConfig.sLockInterface = sLockIf;
    sOpenConfig.sTimingInterface = sTimingIf;
    sOpenConfig.pfnInterruptPinControl = SensorAcqPort_prvInterruptPinControl;
    sOpenConfig.vpInterruptCtx = NULL;

    eRet = Mpu6050_Open(&gsMpuHandle, &sOpenConfig);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    eRet = Mpu6050_Stm32Hal_InitAsyncWorker(&gsMpuBusContext, &gsMpuHandle);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    eReadMode = MPU6050_READ_MODE_ASYNC_DMA;
    eRet = Mpu6050_Ioctl(&gsMpuHandle, MPU6050_IOCTL_SET_READ_MODE, &eReadMode);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    eRet = Mpu6050_Ioctl(&gsMpuHandle, MPU6050_IOCTL_SET_INT_ENABLE, &u8IntEnableMask);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    eRet = Mpu6050_Ioctl(&gsMpuHandle, MPU6050_IOCTL_ENABLE_INTERRUPT_PIN, NULL);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    return true;
}

/**
 * @brief Initializes app-layer objects: DDI binding, manager, and telemetry task.
 * @return true on success, false on failure.
 */
static bool SensorAcqPort_prvInitAppLayers(void)
{
    ts_SensorManagerConfig sSensorManagerConfig;
    ts_TelemetryTaskConfig sTelemetryConfig;

    Gds_ResetRawImu();
    Mpu6050DdiAdapter_Bind(&gasImuDevices[0], &gsMpuDdiContext, &gsMpuHandle);

    sSensorManagerConfig.psImuDevices = gasImuDevices;
    sSensorManagerConfig.u8ImuDeviceCount = 1U;
    if (SensorManager_Init(&gsSensorManagerContext, &sSensorManagerConfig) != SENSOR_MANAGER_OK)
    {
        return false;
    }

    sTelemetryConfig.pfnUartSend = SensorAcqPort_prvUartSend;
    sTelemetryConfig.vpUartContext = NULL;
    sTelemetryConfig.pu8TxBuffer = gau8TelemetryTxBuffer;
    sTelemetryConfig.u16TxBufferLength = sizeof(gau8TelemetryTxBuffer);
    if (TelemetryTask_Init(&gsTelemetryTaskContext, &sTelemetryConfig) != TELEMETRY_TASK_OK)
    {
        return false;
    }

    return true;
}

/**
 * @brief Sensor manager periodic thread entry.
 * @param pvArgument Unused thread argument.
 * @retval None.
 */
static void SensorAcqPort_prvSensorTask(void *pvArgument)
{
    uint32_t u32NextWakeTick;
    te_SensorManagerRetCode eStepRet;

    (void)pvArgument;

    u32NextWakeTick = osKernelGetTickCount();
    for (;;)
    {
        eStepRet = SensorManager_Step(&gsSensorManagerContext);
        (void)eStepRet;

        u32NextWakeTick += SENSOR_ACQ_TASK_PERIOD_MS;
        (void)osDelayUntil(u32NextWakeTick);
    }
}

/**
 * @brief Telemetry periodic thread entry.
 * @param pvArgument Unused thread argument.
 * @retval None.
 */
static void SensorAcqPort_prvTelemetryTask(void *pvArgument)
{
    uint32_t u32NextWakeTick;
    te_TelemetryTaskRetCode eStepRet;

    (void)pvArgument;

    u32NextWakeTick = osKernelGetTickCount();
    for (;;)
    {
        eStepRet = TelemetryTask_Step(&gsTelemetryTaskContext);
        (void)eStepRet;

        u32NextWakeTick += TELEMETRY_TASK_PERIOD_MS;
        (void)osDelayUntil(u32NextWakeTick);
    }
}

bool SensorAcqPort_Init(I2C_HandleTypeDef *pxI2cHandle, UART_HandleTypeDef *pxUartHandle)
{
    const osMutexAttr_t xI2cMutexAttr =
    {
        .name = "i2c1_bus_mtx",
        .cb_mem = &gsI2cBusMutexCb,
        .cb_size = sizeof(gsI2cBusMutexCb)
    };

    if ((pxI2cHandle == NULL) || (pxUartHandle == NULL))
    {
        return false;
    }

    gpxSensorI2cHandle = pxI2cHandle;
    gpxSensorUartHandle = pxUartHandle;

    if (gxI2cBusMutex == NULL)
    {
        gxI2cBusMutex = osMutexNew(&xI2cMutexAttr);
    }
    if (gxI2cBusMutex == NULL)
    {
        return false;
    }

    if (SensorAcqPort_prvInitMpu6050() == false)
    {
        return false;
    }

    return SensorAcqPort_prvInitAppLayers();
}

/**
 * @brief Creates and starts static periodic Sensor Acquisition task.
 * @return Created task handle; NULL if task creation fails.
 */
osThreadId_t SensorAcqPort_CreateTask(void)
{
    const osThreadAttr_t xSensorTaskAttr =
    {
        .name = "sensor_acq",
        .priority = SENSOR_ACQ_TASK_PRIORITY,
        .stack_mem = gau32SensorAcqTaskStack,
        .stack_size = sizeof(gau32SensorAcqTaskStack),
        .cb_mem = &gsSensorAcqTaskCb,
        .cb_size = sizeof(gsSensorAcqTaskCb)
    };
    const osThreadAttr_t xTelemetryTaskAttr =
    {
        .name = "telemetry_tx",
        .priority = TELEMETRY_TASK_PRIORITY,
        .stack_mem = gau32TelemetryTaskStack,
        .stack_size = sizeof(gau32TelemetryTaskStack),
        .cb_mem = &gsTelemetryTaskCb,
        .cb_size = sizeof(gsTelemetryTaskCb)
    };

    if (gxSensorAcqTaskHandle != NULL)
    {
        return gxSensorAcqTaskHandle;
    }

    gxSensorAcqTaskHandle = osThreadNew(SensorAcqPort_prvSensorTask, NULL, &xSensorTaskAttr);
    if (gxSensorAcqTaskHandle == NULL)
    {
        return NULL;
    }

    gxTelemetryTaskHandle = osThreadNew(SensorAcqPort_prvTelemetryTask, NULL, &xTelemetryTaskAttr);
    if (gxTelemetryTaskHandle == NULL)
    {
        return NULL;
    }

    return gxSensorAcqTaskHandle;
}

/**
 * @brief Forwards HAL EXTI callback to MPU6050 async worker trigger.
 * @param u16GpioPin Input GPIO identifier.
 * @retval None
 */
void SensorAcqPort_OnExtiCallback(uint16_t u16GpioPin)
{
    if (u16GpioPin == MPU6050_INT_Pin)
    {
        Mpu6050_Stm32Hal_OnPinInterrupt(&gsMpuBusContext);
    }
}

/**
 * @brief Forwards HAL I2C memory DMA completion callback.
 * @param pxI2cHandle Input I2C handle.
 * @retval None
 */
void SensorAcqPort_OnI2cMemRxComplete(I2C_HandleTypeDef *pxI2cHandle)
{
    Mpu6050_Stm32Hal_OnDmaComplete(pxI2cHandle, &gsMpuBusContext);
}
