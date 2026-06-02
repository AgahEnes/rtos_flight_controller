#ifndef BMP180_TEST_H_
#define BMP180_TEST_H_

#include <stdint.h>

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t u32TimeoutMs;                                  /* Test döngüsünün toplam süresidir; örnekler 200 ms aralıklarla alınır. */
    UART_HandleTypeDef *pxUart;                             /* Opsiyonel UART log çıkışıdır; NULL verilirse test sessiz çalışır. */
    I2C_HandleTypeDef *pxI2c;                                /* BMP180'in bağlı olduğu STM32 HAL I2C handle pointer'ıdır. */
    osMutexId_t xBusMutex;                                  /* MPU6050 ile paylaşılan opsiyonel I2C mutex'idir; NULL ise test kendi mutex'ini oluşturur. */
    uint8_t u8I2cAddr7bit;                                  /* BMP180 7-bit I2C adresidir; 0 verilirse 0x77 varsayılan adres kullanılır. */
    uint8_t u8Oversampling;                                 /* 0..3 arası BMP180 OSS değeridir; test bunu te_Bmp180_Oversampling'e çevirir. */
} ts_Bmp180_TestParams;

#if defined(BMP180_ENABLE_TEST)
/**
 * @brief BMP180'i POSIX ilhamlı driver API ile açar, health-check yapar ve UART'a ölçüm satırları basar.
 * @param psX Test parametrelerini taşıyan uygulama struct pointer'ıdır; pxI2c zorunludur.
 * @return 0 başarıyı, negatif değerler açma/konfigürasyon/okuma hatasını bildirir.
 */
int8_t BMP180_TEST(const ts_Bmp180_TestParams *psX);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BMP180_TEST_H_ */
