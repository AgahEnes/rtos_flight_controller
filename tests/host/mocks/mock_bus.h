#ifndef TESTS_HOST_MOCKS_MOCK_BUS_H_
#define TESTS_HOST_MOCKS_MOCK_BUS_H_

#include "mpu6050_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

void MockBus_Reset(void);
void MockBus_SetWhoAmI(uint8_t u8WhoAmI);

te_Driver_RetCode MockBus_Read(uint8_t u8DeviceAddr,
                               uint8_t u8RegisterAddr,
                               uint8_t *pu8ReadData,
                               uint16_t u16ReadLen,
                               uint32_t u32TimeoutMs,
                               void *vpCtx);

te_Driver_RetCode MockBus_Write(uint8_t u8DeviceAddr,
                                uint8_t u8RegisterAddr,
                                const uint8_t *pu8WriteData,
                                uint16_t u16WriteLen,
                                uint32_t u32TimeoutMs,
                                void *vpCtx);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_HOST_MOCKS_MOCK_BUS_H_ */
