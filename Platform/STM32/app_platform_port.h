#ifndef PLATFORM_STM32_APP_PLATFORM_PORT_H_
#define PLATFORM_STM32_APP_PLATFORM_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

/**
 * @brief Initializes application platform layer.
 * @param pxI2cHandle I2C handle used by MPU6050 driver.
 * @param pxUartHandle UART handle used for telemetry transmission.
 * @return true on success, false on initialization failure.
 */
bool AppPlatformPort_Init(I2C_HandleTypeDef *pxI2cHandle, UART_HandleTypeDef *pxUartHandle);

/**
 * @brief Creates statically allocated sensor and telemetry threads.
 * @return Sensor thread handle on success, NULL on failure.
 */
osThreadId_t AppPlatformPort_CreateTask(void);

/**
 * @brief EXTI callback bridge for MPU6050 INT pin.
 * @param u16GpioPin GPIO pin identifier passed by HAL.
 */
void AppPlatformPort_OnExtiCallback(uint16_t u16GpioPin);

/**
 * @brief I2C memory RX complete callback bridge for MPU6050 DMA reads.
 * @param pxI2cHandle I2C handle passed by HAL.
 */
void AppPlatformPort_OnI2cMemRxComplete(I2C_HandleTypeDef *pxI2cHandle);

/**
 * @brief UART TX complete callback bridge for telemetry UART DMA.
 * @param pxUartHandle UART handle passed by HAL.
 */
void AppPlatformPort_OnUartTxComplete(UART_HandleTypeDef *pxUartHandle);

/**
 * @brief UART error callback bridge for telemetry UART DMA.
 * @param pxUartHandle UART handle passed by HAL.
 */
void AppPlatformPort_OnUartError(UART_HandleTypeDef *pxUartHandle);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_STM32_APP_PLATFORM_PORT_H_ */
