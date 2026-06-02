#include "app_platform_port.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "main.h"
#include "mpu6050_hal.h"
#include "mpu6050_driver.h"
#include "mpu6050_stm32_hal_port.h"
#include "bmp180_hal.h"
#include "bmp180_driver.h"
#include "bmp180_stm32_hal_port.h"
#include "servo_driver.h"
#include "servo_stm32_hal_port.h"
#include "global_data_space.h"
#include "mpu6050_ddi_adapter.h"
#include "bmp180_ddi_adapter.h"
#include "servo_ddi_adapter.h"
#include "sensor_manager.h"
#include "actuator_manager.h"
#include "telemetry_task.h"
#include "navigation_subsystem.h"
#include "flight_control.h"

#define APP_PLATFORM_SENSOR_TASK_STACK_WORDS            (512U)
#define APP_PLATFORM_TELEMETRY_TASK_STACK_WORDS         (512U)
#define APP_PLATFORM_NAV_TASK_STACK_WORDS               (512U)
#define APP_PLATFORM_FLIGHT_CONTROL_TASK_STACK_WORDS    (768U)
#define APP_PLATFORM_ACTUATOR_TASK_STACK_WORDS          (512U)
#define APP_PLATFORM_SENSOR_TASK_PERIOD_MS              (10U)
#define APP_PLATFORM_TELEMETRY_TASK_PERIOD_MS           (50U)
#define APP_PLATFORM_NAV_TASK_PERIOD_MS                 (10U)
#define APP_PLATFORM_FLIGHT_CONTROL_TASK_PERIOD_MS      (10U)
#define APP_PLATFORM_ACTUATOR_TASK_PERIOD_MS            (10U)
#define APP_PLATFORM_SENSOR_TASK_PRIORITY               (osPriorityAboveNormal)
#define APP_PLATFORM_TELEMETRY_TASK_PRIORITY            (osPriorityNormal)
#define APP_PLATFORM_NAV_TASK_PRIORITY                  (osPriorityAboveNormal)
#define APP_PLATFORM_FLIGHT_CONTROL_TASK_PRIORITY       (osPriorityAboveNormal)
#define APP_PLATFORM_ACTUATOR_TASK_PRIORITY             (osPriorityAboveNormal)
#define APP_PLATFORM_BUS_TIMEOUT_MS                     (100U)
#define APP_PLATFORM_BUS_LOCK_TIMEOUT_MS                (20U)
#define APP_PLATFORM_IMU_DEVICE_COUNT                   (1U)
#define APP_PLATFORM_ENABLE_BMP180                      (1)
#define APP_PLATFORM_BARO_DEVICE_COUNT                  (1U)
#define APP_PLATFORM_BARO_READ_PERIOD_TICKS             (10U)
#define APP_PLATFORM_SERVO_DEVICE_COUNT                 (4U)
#define APP_PLATFORM_UART_DMA_TOKEN_MAX_COUNT           (1U)
#define APP_PLATFORM_UART_DMA_TOKEN_INITIAL_COUNT       (1U)

static I2C_HandleTypeDef *gpxPlatformI2cHandle = NULL;
static UART_HandleTypeDef *gpxPlatformUartHandle = NULL;
static TIM_HandleTypeDef *gpxPlatformServoTimHandle = NULL;
static osThreadId_t gxAppPlatformSensorTaskHandle = NULL;
static osThreadId_t gxAppPlatformTelemetryTaskHandle = NULL;
static osThreadId_t gxAppPlatformNavTaskHandle = NULL;
static osThreadId_t gxAppPlatformFlightControlTaskHandle = NULL;
static osThreadId_t gxAppPlatformActuatorTaskHandle = NULL;
static osMutexId_t gxAppPlatformI2cBusMutex = NULL;
static osSemaphoreId_t gxAppPlatformUartTxDmaToken = NULL;

static ts_Mpu6050_Handle gsMpuHandle;
static ts_Mpu6050_Stm32BusContext gsMpuBusContext;
static ts_Mpu6050DdiAdapterContext gsMpuDdiContext;
static ts_ImuDevice gasImuDevices[APP_PLATFORM_IMU_DEVICE_COUNT];
static ts_Bmp180_Handle gsBmp180Handle;
static ts_Bmp180_Stm32BusContext gsBmp180BusContext;
static ts_Bmp180DdiAdapterContext gsBmp180DdiContext;
static ts_BaroDevice gasBaroDevices[APP_PLATFORM_BARO_DEVICE_COUNT];
static ts_SensorManagerContext gsSensorManagerContext;
static ts_Servo_Handle gasServoHandles[APP_PLATFORM_SERVO_DEVICE_COUNT];
static ts_Servo_Stm32PortContext gasServoPortContexts[APP_PLATFORM_SERVO_DEVICE_COUNT];
static ts_ServoDdiAdapterContext gasServoDdiContexts[APP_PLATFORM_SERVO_DEVICE_COUNT];
static ts_ActuatorDevice gasActuatorDevices[APP_PLATFORM_SERVO_DEVICE_COUNT];
static ts_ActuatorManagerContext gsActuatorManagerContext;
static ts_TelemetryTaskContext gsTelemetryTaskContext;
static ts_NavContext gsNavContext;
static ts_FlightControlContext gsFlightControlContext;
static uint8_t gau8TelemetryTxBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH];
static uint8_t gau8TelemetryTxDmaBuffer[TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH];

static StaticTask_t gsAppPlatformSensorTaskCb;
static StackType_t gau32AppPlatformSensorTaskStack[APP_PLATFORM_SENSOR_TASK_STACK_WORDS];
static StaticTask_t gsAppPlatformTelemetryTaskCb;
static StackType_t gau32AppPlatformTelemetryTaskStack[APP_PLATFORM_TELEMETRY_TASK_STACK_WORDS];
static StaticTask_t gsAppPlatformNavTaskCb;
static StackType_t gau32AppPlatformNavTaskStack[APP_PLATFORM_NAV_TASK_STACK_WORDS];
static StaticTask_t gsAppPlatformFlightControlTaskCb;
static StackType_t gau32AppPlatformFlightControlTaskStack[APP_PLATFORM_FLIGHT_CONTROL_TASK_STACK_WORDS];
static StaticTask_t gsAppPlatformActuatorTaskCb;
static StackType_t gau32AppPlatformActuatorTaskStack[APP_PLATFORM_ACTUATOR_TASK_STACK_WORDS];
static StaticSemaphore_t gsAppPlatformI2cBusMutexCb;
static StaticSemaphore_t gsAppPlatformUartTxDmaTokenCb;

static volatile uint32_t gu32TelemetryTxDmaDropCount = 0U;
static volatile uint32_t gu32TelemetryTxDmaStartFailCount = 0U;
static volatile uint32_t gu32TelemetryTxDmaErrorIsrCount = 0U;

/**
 * @brief Returns RTOS kernel tick count in milliseconds for telemetry timestamps.
 * @param vpContext Unused callback context.
 * @return Elapsed time in milliseconds since kernel start.
 */
static uint32_t AppPlatformPort_prvGetTickMs(void *vpContext)
{
    (void)vpContext;

    return osKernelGetTickCount();
}

/**
 * @brief Sends telemetry packet over configured UART.
 * @param pu8Data Binary packet pointer.
 * @param u16Length Packet length in bytes.
 * @param vpContext Unused callback context.
 * @return true on successful UART transmission.
 */
static bool AppPlatformPort_prvUartDmaSend(const uint8_t *pu8Data, uint16_t u16Length, void *vpContext)
{
    HAL_StatusTypeDef eHalRet;

    (void)vpContext;

    if ((gpxPlatformUartHandle == NULL) ||
        (gxAppPlatformUartTxDmaToken == NULL) ||
        (pu8Data == NULL) ||
        (u16Length == 0U) ||
        (u16Length > TELEMETRY_TASK_MAX_TX_BUFFER_LENGTH))
    {
        return false;
    }

    if (osSemaphoreAcquire(gxAppPlatformUartTxDmaToken, 0U) != osOK)
    {
        gu32TelemetryTxDmaDropCount++;
        return false;
    }

    (void)memcpy(gau8TelemetryTxDmaBuffer, pu8Data, (size_t)u16Length);
    eHalRet = HAL_UART_Transmit_DMA(gpxPlatformUartHandle,
                                    gau8TelemetryTxDmaBuffer,
                                    u16Length);
    if (eHalRet != HAL_OK)
    {
        gu32TelemetryTxDmaStartFailCount++;
        (void)osSemaphoreRelease(gxAppPlatformUartTxDmaToken);
        return false;
    }

    return true;
}

/**
 * @brief Initializes MPU6050 open configuration and starts async DMA worker.
 * @return true on success, false on failure.
 */
static bool AppPlatformPort_prvInitMpu6050(void)
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
    (void)memset(&gasServoHandles, 0, sizeof(gasServoHandles));
    (void)memset(&gasServoPortContexts, 0, sizeof(gasServoPortContexts));
    (void)memset(&gasServoDdiContexts, 0, sizeof(gasServoDdiContexts));
    (void)memset(&gasActuatorDevices, 0, sizeof(gasActuatorDevices));
    (void)memset(&gsSensorManagerContext, 0, sizeof(gsSensorManagerContext));
    (void)memset(&gsActuatorManagerContext, 0, sizeof(gsActuatorManagerContext));
    (void)memset(&gsTelemetryTaskContext, 0, sizeof(gsTelemetryTaskContext));
    (void)memset(&gsFlightControlContext, 0, sizeof(gsFlightControlContext));
    (void)memset(&sOpenConfig, 0, sizeof(sOpenConfig));

    gsMpuBusContext.pxI2cHandle = gpxPlatformI2cHandle;
    gsMpuBusContext.xBusMutex = gxAppPlatformI2cBusMutex;

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
    sOpenConfig.u32BusTimeoutMs = APP_PLATFORM_BUS_TIMEOUT_MS;
    sOpenConfig.u32BusLockTimeoutMs = APP_PLATFORM_BUS_LOCK_TIMEOUT_MS;
    sOpenConfig.sBusInterface = sBusIf;
    sOpenConfig.sLockInterface = sLockIf;
    sOpenConfig.sTimingInterface = sTimingIf;

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

    return true;
}

/**
 * @brief Initializes BMP180 open configuration and loads calibration coefficients.
 * @return true on success, false on failure.
 */
static bool AppPlatformPort_prvInitBmp180(void)
{
    ts_Bmp180_OpenConfig sOpenConfig;
    ts_Bmp180_BusInterface sBusIf;
    ts_Bmp180_LockInterface sLockIf;
    ts_Bmp180_TimingInterface sTimingIf;
    te_Driver_RetCode eRet;

    (void)memset(&gsBmp180Handle, 0, sizeof(gsBmp180Handle));
    (void)memset(&gsBmp180BusContext, 0, sizeof(gsBmp180BusContext));
    (void)memset(&gsBmp180DdiContext, 0, sizeof(gsBmp180DdiContext));
    (void)memset(&gasBaroDevices, 0, sizeof(gasBaroDevices));
    (void)memset(&sOpenConfig, 0, sizeof(sOpenConfig));

    gsBmp180BusContext.pxI2cHandle = gpxPlatformI2cHandle;
    gsBmp180BusContext.xBusMutex = gxAppPlatformI2cBusMutex;

    eRet = Bmp180_Stm32Hal_FillBusInterface(&sBusIf, &gsBmp180BusContext);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    eRet = Bmp180_Stm32Hal_FillLockInterface(&sLockIf, &gsBmp180BusContext);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    eRet = Bmp180_Stm32Hal_FillTimingInterface(&sTimingIf);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    sOpenConfig.u8I2cAddress = BMP180_I2C_ADDR_DEFAULT;
    sOpenConfig.u32BusTimeoutMs = APP_PLATFORM_BUS_TIMEOUT_MS;
    sOpenConfig.u32BusLockTimeoutMs = APP_PLATFORM_BUS_LOCK_TIMEOUT_MS;
    sOpenConfig.eOversampling = BMP180_OSS3_ULTRA_HIGH_RESOLUTION;
    sOpenConfig.sBusInterface = sBusIf;
    sOpenConfig.sLockInterface = sLockIf;
    sOpenConfig.sTimingInterface = sTimingIf;
    sOpenConfig.f32SeaLevelPressurePa = BMP180_SEA_LEVEL_PRESSURE_PA;

    eRet = Bmp180_Open(&gsBmp180Handle, &sOpenConfig);
    if (eRet != DRIVER_OK)
    {
        return false;
    }

    return true;
}

/**
 * @brief Initializes and opens four servo instances on one PWM timer.
 * @return true on success, false on failure.
 */
static bool AppPlatformPort_prvInitServos(void)
{
    static const uint32_t kau32ServoChannels[APP_PLATFORM_SERVO_DEVICE_COUNT] =
    {
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        TIM_CHANNEL_3,
        TIM_CHANNEL_4
    };
    uint8_t u8Idx;
    ts_Servo_OpenConfig sOpenConfig;
    te_Driver_RetCode eRet;
    HAL_StatusTypeDef eHalRet;

    if ((gpxPlatformServoTimHandle == NULL) || (gpxPlatformServoTimHandle->Instance == NULL))
    {
        return false;
    }

    for (u8Idx = 0U; u8Idx < APP_PLATFORM_SERVO_DEVICE_COUNT; u8Idx++)
    {
        (void)memset(&sOpenConfig, 0, sizeof(sOpenConfig));

        gasServoPortContexts[u8Idx].pxTimHandle = gpxPlatformServoTimHandle;
        gasServoPortContexts[u8Idx].u32Channel = kau32ServoChannels[u8Idx];
        gasServoPortContexts[u8Idx].xServoMutex = NULL;
        gasServoPortContexts[u8Idx].u32TimerClockHz = 0U;

        eHalRet = HAL_TIM_PWM_Start(gpxPlatformServoTimHandle, kau32ServoChannels[u8Idx]);
        if (eHalRet != HAL_OK)
        {
            return false;
        }

        eRet = Servo_Stm32Hal_FillInterfaces(&sOpenConfig, &gasServoPortContexts[u8Idx]);
        if (eRet != DRIVER_OK)
        {
            return false;
        }

        sOpenConfig.f32MinAngleRad = SERVO_SG90_MIN_ANGLE_RAD;
        sOpenConfig.f32MaxAngleRad = SERVO_SG90_MAX_ANGLE_RAD;
        sOpenConfig.u32MinPulseUs = SERVO_SG90_MIN_PULSE_US;
        sOpenConfig.u32MaxPulseUs = SERVO_SG90_MAX_PULSE_US;
        sOpenConfig.f32AngleOffsetRad = 0.0F;
        sOpenConfig.u32LockTimeoutMs = SERVO_DEFAULT_LOCK_TIMEOUT_MS;
        sOpenConfig.bEnableSlewRate = false;
        sOpenConfig.f32MaxSlewRateRadPerSec = 0.0F;

        eRet = Servo_Open(&gasServoHandles[u8Idx], &sOpenConfig);
        if (eRet != DRIVER_OK)
        {
            return false;
        }
    }

    return true;
}

/**
 * @brief Initializes app-layer objects: DDI binding, manager, and telemetry task.
 * @return true on success, false on failure.
 */
static bool AppPlatformPort_prvInitAppLayers(void)
{
    uint8_t u8Idx;
    ts_SensorManagerConfig sSensorManagerConfig;
    ts_ActuatorManagerConfig sActuatorManagerConfig;
    ts_TelemetryTaskConfig sTelemetryConfig;
    ts_NavConfig sNavConfig;
    ts_FlightControlConfig sFlightControlConfig;

    FlightControl_InitDefaultConfig(&sFlightControlConfig);
    (void)memset(&sNavConfig, 0, sizeof(sNavConfig));
    (void)memset(&sTelemetryConfig, 0, sizeof(sTelemetryConfig));
    (void)memset(&sSensorManagerConfig, 0, sizeof(sSensorManagerConfig));
    (void)memset(&sActuatorManagerConfig, 0, sizeof(sActuatorManagerConfig));
    
    Gds_ResetRawImu();
    Gds_ResetVehicleState();
    Gds_ResetImuCalibration();
    Gds_ResetNavCommand();
    Gds_ResetBarometer();
    Gds_ResetActuatorCmd();
    Mpu6050DdiAdapter_Bind(&gasImuDevices[0], &gsMpuDdiContext, &gsMpuHandle);
#if (APP_PLATFORM_ENABLE_BMP180 != 0)
    Bmp180DdiAdapter_Bind(&gasBaroDevices[0], &gsBmp180DdiContext, &gsBmp180Handle);
#endif

    sSensorManagerConfig.psImuDevices = gasImuDevices;
    sSensorManagerConfig.u8ImuDeviceCount = 1U;
#if (APP_PLATFORM_ENABLE_BMP180 != 0)
    sSensorManagerConfig.psBaroDevices = gasBaroDevices;
    sSensorManagerConfig.u8BaroDeviceCount = APP_PLATFORM_BARO_DEVICE_COUNT;
    sSensorManagerConfig.u8BaroReadPeriodTicks = APP_PLATFORM_BARO_READ_PERIOD_TICKS;
#endif
    if (SensorManager_Init(&gsSensorManagerContext, &sSensorManagerConfig) != SENSOR_MANAGER_OK)
    {
        return false;
    }

    for (u8Idx = 0U; u8Idx < APP_PLATFORM_SERVO_DEVICE_COUNT; u8Idx++)
    {
        ServoDdiAdapter_Bind(&gasActuatorDevices[u8Idx],
                             &gasServoDdiContexts[u8Idx],
                             &gasServoHandles[u8Idx]);
    }

    sActuatorManagerConfig.psActuatorDevices = gasActuatorDevices;
    sActuatorManagerConfig.u8ActuatorDeviceCount = APP_PLATFORM_SERVO_DEVICE_COUNT;
    if (ActuatorManager_Init(&gsActuatorManagerContext, &sActuatorManagerConfig) != ACTUATOR_MANAGER_OK)
    {
        return false;
    }

    sNavConfig.f32Alpha = NAV_CFG_DEFAULT_ALPHA;
    sNavConfig.f32DtS = NAV_CFG_DEFAULT_DT_S;
    sNavConfig.f32ZeroEpsilon = NAV_CFG_ZERO_EPSILON;
    sNavConfig.u8StuckThresholdCycles = NAV_CFG_STUCK_THRESHOLD_CYCLES;
    if (Navigation_Init(&gsNavContext, &sNavConfig) != NAV_RET_OK)
    {
        return false;
    }

    if (FlightControl_Init(&gsFlightControlContext, &sFlightControlConfig) != FLIGHT_CONTROL_OK)
    {
        return false;
    }

    sTelemetryConfig.pfnUartSend = AppPlatformPort_prvUartDmaSend;
    sTelemetryConfig.vpUartContext = NULL;
    sTelemetryConfig.pfnGetTickMs = AppPlatformPort_prvGetTickMs;
    sTelemetryConfig.vpTickContext = NULL;
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
static void AppPlatformPort_prvSensorTask(void *pvArgument)
{
    uint32_t u32NextWakeTick;
    te_SensorManagerRetCode eStepRet;

    (void)pvArgument;

    u32NextWakeTick = osKernelGetTickCount();
    for (;;)
    {
        eStepRet = SensorManager_Step(&gsSensorManagerContext);
        (void)eStepRet;

        u32NextWakeTick += APP_PLATFORM_SENSOR_TASK_PERIOD_MS;
        (void)osDelayUntil(u32NextWakeTick);
    }
}

/**
 * @brief Telemetry periodic thread entry.
 * @param pvArgument Unused thread argument.
 * @retval None.
 */
static void AppPlatformPort_prvTelemetryTask(void *pvArgument)
{
    uint32_t u32NextWakeTick;
    te_TelemetryTaskRetCode eStepRet;

    (void)pvArgument;

    u32NextWakeTick = osKernelGetTickCount();
    for (;;)
    {
        eStepRet = TelemetryTask_Step(&gsTelemetryTaskContext);
        (void)eStepRet;

        u32NextWakeTick += APP_PLATFORM_TELEMETRY_TASK_PERIOD_MS;
        (void)osDelayUntil(u32NextWakeTick);
    }
}

/**
 * @brief Navigation periodic thread entry.
 * @param pvArgument Unused thread argument.
 * @retval None.
 */
static void AppPlatformPort_prvNavTask(void *pvArgument)
{
    uint32_t u32NextWakeTick;
    ts_TopicRawImu sRawImu;
    ts_TopicVehicleState sVehicleState;
    te_GdsRetCode eGdsRet;
    te_NavigationRetCode eNavRet;

    (void)pvArgument;

    u32NextWakeTick = osKernelGetTickCount();
    for (;;)
    {
        eGdsRet = Gds_ReadRawImu(&sRawImu);
        if (eGdsRet == GDS_OK)
        {
            eNavRet = NavigationTask_Step(&gsNavContext, &sRawImu, &sVehicleState);
            if (eNavRet == NAV_RET_OK)
            {
                (void)Gds_PublishVehicleState(&sVehicleState);
            }
        }

        u32NextWakeTick += APP_PLATFORM_NAV_TASK_PERIOD_MS;
        (void)osDelayUntil(u32NextWakeTick);
    }
}

/**
 * @brief Flight control periodic thread entry.
 * @param pvArgument Unused thread argument.
 * @retval None.
 */
static void AppPlatformPort_prvFlightControlTask(void *pvArgument)
{
    uint32_t u32NextWakeTick;
    te_FlightControlRetCode eStepRet;

    (void)pvArgument;

    u32NextWakeTick = osKernelGetTickCount();
    for (;;)
    {
        gsFlightControlContext.u32ModuleTimestampMs = osKernelGetTickCount();
        eStepRet = FlightControl_Step(&gsFlightControlContext);
        (void)eStepRet;

        u32NextWakeTick += APP_PLATFORM_FLIGHT_CONTROL_TASK_PERIOD_MS;
        (void)osDelayUntil(u32NextWakeTick);
    }
}

/**
 * @brief Actuator periodic thread entry.
 * @param pvArgument Unused thread argument.
 * @retval None.
 */
static void AppPlatformPort_prvActuatorTask(void *pvArgument)
{
    uint32_t u32NextWakeTick;
    te_ActuatorManagerRetCode eStepRet;

    (void)pvArgument;

    u32NextWakeTick = osKernelGetTickCount();
    for (;;)
    {
        eStepRet = ActuatorManager_Step(&gsActuatorManagerContext);
        (void)eStepRet;

        u32NextWakeTick += APP_PLATFORM_ACTUATOR_TASK_PERIOD_MS;
        (void)osDelayUntil(u32NextWakeTick);
    }
}

bool AppPlatformPort_Init(I2C_HandleTypeDef *pxI2cHandle,
                          UART_HandleTypeDef *pxUartHandle,
                          TIM_HandleTypeDef *pxServoTimHandle)
{
    const osMutexAttr_t xI2cMutexAttr =
    {
        .name = "i2c1_bus_mtx",
        .cb_mem = &gsAppPlatformI2cBusMutexCb,
        .cb_size = sizeof(gsAppPlatformI2cBusMutexCb)
    };
    const osSemaphoreAttr_t xUartTxDmaTokenAttr =
    {
        .name = "uart2_tx_dma_token",
        .cb_mem = &gsAppPlatformUartTxDmaTokenCb,
        .cb_size = sizeof(gsAppPlatformUartTxDmaTokenCb)
    };

    if ((pxI2cHandle == NULL) || (pxUartHandle == NULL) || (pxServoTimHandle == NULL))
    {
        return false;
    }

    gpxPlatformI2cHandle = pxI2cHandle;
    gpxPlatformUartHandle = pxUartHandle;
    gpxPlatformServoTimHandle = pxServoTimHandle;

    if (gxAppPlatformI2cBusMutex == NULL)
    {
        gxAppPlatformI2cBusMutex = osMutexNew(&xI2cMutexAttr);
    }
    if (gxAppPlatformI2cBusMutex == NULL)
    {
        return false;
    }

    if (gxAppPlatformUartTxDmaToken == NULL)
    {
        gxAppPlatformUartTxDmaToken = osSemaphoreNew(APP_PLATFORM_UART_DMA_TOKEN_MAX_COUNT,
                                                     APP_PLATFORM_UART_DMA_TOKEN_INITIAL_COUNT,
                                                     &xUartTxDmaTokenAttr);
    }
    if (gxAppPlatformUartTxDmaToken == NULL)
    {
        return false;
    }

    if (AppPlatformPort_prvInitMpu6050() == false)
    {
        return false;
    }

#if (APP_PLATFORM_ENABLE_BMP180 != 0)
    if (AppPlatformPort_prvInitBmp180() == false)
    {
        return false;
    }
#endif

    if (AppPlatformPort_prvInitServos() == false)
    {
        return false;
    }

    return AppPlatformPort_prvInitAppLayers();
}

/**
 * @brief Creates and starts static periodic sensor and telemetry tasks.
 * @return Created sensor task handle; NULL if task creation fails.
 */
osThreadId_t AppPlatformPort_CreateTask(void)
{
    const osThreadAttr_t xSensorTaskAttr =
    {
        .name = "app_platform_sensor",
        .priority = APP_PLATFORM_SENSOR_TASK_PRIORITY,
        .stack_mem = gau32AppPlatformSensorTaskStack,
        .stack_size = sizeof(gau32AppPlatformSensorTaskStack),
        .cb_mem = &gsAppPlatformSensorTaskCb,
        .cb_size = sizeof(gsAppPlatformSensorTaskCb)
    };
    const osThreadAttr_t xTelemetryTaskAttr =
    {
        .name = "app_platform_telemetry",
        .priority = APP_PLATFORM_TELEMETRY_TASK_PRIORITY,
        .stack_mem = gau32AppPlatformTelemetryTaskStack,
        .stack_size = sizeof(gau32AppPlatformTelemetryTaskStack),
        .cb_mem = &gsAppPlatformTelemetryTaskCb,
        .cb_size = sizeof(gsAppPlatformTelemetryTaskCb)
    };
    const osThreadAttr_t xNavTaskAttr =
    {
        .name = "app_platform_nav",
        .priority = APP_PLATFORM_NAV_TASK_PRIORITY,
        .stack_mem = gau32AppPlatformNavTaskStack,
        .stack_size = sizeof(gau32AppPlatformNavTaskStack),
        .cb_mem = &gsAppPlatformNavTaskCb,
        .cb_size = sizeof(gsAppPlatformNavTaskCb)
    };
    const osThreadAttr_t xFlightControlTaskAttr =
    {
        .name = "app_platform_flight_control",
        .priority = APP_PLATFORM_FLIGHT_CONTROL_TASK_PRIORITY,
        .stack_mem = gau32AppPlatformFlightControlTaskStack,
        .stack_size = sizeof(gau32AppPlatformFlightControlTaskStack),
        .cb_mem = &gsAppPlatformFlightControlTaskCb,
        .cb_size = sizeof(gsAppPlatformFlightControlTaskCb)
    };
    const osThreadAttr_t xActuatorTaskAttr =
    {
        .name = "app_platform_actuator",
        .priority = APP_PLATFORM_ACTUATOR_TASK_PRIORITY,
        .stack_mem = gau32AppPlatformActuatorTaskStack,
        .stack_size = sizeof(gau32AppPlatformActuatorTaskStack),
        .cb_mem = &gsAppPlatformActuatorTaskCb,
        .cb_size = sizeof(gsAppPlatformActuatorTaskCb)
    };

    if (gxAppPlatformSensorTaskHandle != NULL)
    {
        return gxAppPlatformSensorTaskHandle;
    }

    gxAppPlatformSensorTaskHandle = osThreadNew(AppPlatformPort_prvSensorTask, NULL, &xSensorTaskAttr);
    if (gxAppPlatformSensorTaskHandle == NULL)
    {
        return NULL;
    }

    gxAppPlatformTelemetryTaskHandle = osThreadNew(AppPlatformPort_prvTelemetryTask, NULL, &xTelemetryTaskAttr);
    if (gxAppPlatformTelemetryTaskHandle == NULL)
    {
        return NULL;
    }

    gxAppPlatformNavTaskHandle = osThreadNew(AppPlatformPort_prvNavTask, NULL, &xNavTaskAttr);
    if (gxAppPlatformNavTaskHandle == NULL)
    {
        return NULL;
    }

    gxAppPlatformFlightControlTaskHandle = osThreadNew(AppPlatformPort_prvFlightControlTask, NULL, &xFlightControlTaskAttr);
    if (gxAppPlatformFlightControlTaskHandle == NULL)
    {
        return NULL;
    }

    gxAppPlatformActuatorTaskHandle = osThreadNew(AppPlatformPort_prvActuatorTask, NULL, &xActuatorTaskAttr);
    if (gxAppPlatformActuatorTaskHandle == NULL)
    {
        return NULL;
    }

    return gxAppPlatformSensorTaskHandle;
}

/**
 * @brief Forwards HAL EXTI callback to MPU6050 async worker trigger.
 * @param u16GpioPin Input GPIO identifier.
 * @retval None
 */
void AppPlatformPort_OnExtiCallback(uint16_t u16GpioPin)
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
void AppPlatformPort_OnI2cMemRxComplete(I2C_HandleTypeDef *pxI2cHandle)
{
    Mpu6050_Stm32Hal_OnDmaComplete(pxI2cHandle, &gsMpuBusContext);
}

/**
 * @brief Forwards HAL UART TX complete callback for telemetry DMA path.
 * @param pxUartHandle UART handle from HAL.
 * @retval None
 */
void AppPlatformPort_OnUartTxComplete(UART_HandleTypeDef *pxUartHandle)
{
    if ((pxUartHandle != NULL) &&
        (pxUartHandle == gpxPlatformUartHandle) &&
        (gxAppPlatformUartTxDmaToken != NULL))
    {
        (void)osSemaphoreRelease(gxAppPlatformUartTxDmaToken);
    }
}

/**
 * @brief Forwards HAL UART error callback for telemetry DMA path.
 * @param pxUartHandle UART handle from HAL.
 * @retval None
 */
void AppPlatformPort_OnUartError(UART_HandleTypeDef *pxUartHandle)
{
    if ((pxUartHandle != NULL) &&
        (pxUartHandle == gpxPlatformUartHandle) &&
        (gxAppPlatformUartTxDmaToken != NULL))
    {
        gu32TelemetryTxDmaErrorIsrCount++;
        (void)osSemaphoreRelease(gxAppPlatformUartTxDmaToken);
    }
}
