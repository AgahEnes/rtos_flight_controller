#ifndef TESTS_HOST_MOCKS_MOCK_MPU6050_API_H_
#define TESTS_HOST_MOCKS_MOCK_MPU6050_API_H_

#include "mpu6050_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

void MockMpu6050_SetReadResponse(te_Driver_RetCode eRet, const ts_Mpu6050_Data *psData);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_HOST_MOCKS_MOCK_MPU6050_API_H_ */
