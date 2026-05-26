#ifndef TESTS_HOST_MOCKS_MOCK_LOCK_TIME_H_
#define TESTS_HOST_MOCKS_MOCK_LOCK_TIME_H_

#include "mpu6050_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

void MockTime_Reset(void);

te_Driver_RetCode MockLock_Lock(uint32_t u32TimeoutMs, void *vpCtx);
te_Driver_RetCode MockLock_Unlock(void *vpCtx);
te_Driver_RetCode MockTime_DelayMs(uint32_t u32DelayMs, void *vpCtx);
uint32_t MockTime_GetTickMs(void *vpCtx);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_HOST_MOCKS_MOCK_LOCK_TIME_H_ */
