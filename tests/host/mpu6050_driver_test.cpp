#include "gtest/gtest.h"

extern "C" {
#include "mock_bus.h"
#include "mock_lock_time.h"
#include "mpu6050_driver.h"
#include "mpu6050_hal.h"
}

TEST(Mpu6050DriverTest, OpenSucceedsWithValidMockInterfaces)
{
    ts_Mpu6050_Handle sHandle {};
    ts_Mpu6050_OpenConfig sConfig {};

    MockBus_Reset();
    MockTime_Reset();

    sConfig.u8I2cAddress = MPU6050_I2C_ADDR_AD0_LOW;
    sConfig.u32BusTimeoutMs = 100U;
    sConfig.u32BusLockTimeoutMs = 10U;
    sConfig.sBusInterface.pfnRead = MockBus_Read;
    sConfig.sBusInterface.pfnWrite = MockBus_Write;
    sConfig.sBusInterface.vpCtx = nullptr;
    sConfig.sLockInterface.pfnLock = MockLock_Lock;
    sConfig.sLockInterface.pfnUnlock = MockLock_Unlock;
    sConfig.sLockInterface.vpCtx = nullptr;
    sConfig.sTimingInterface.pfnDelayMs = MockTime_DelayMs;
    sConfig.sTimingInterface.pfnGetTickMs = MockTime_GetTickMs;
    sConfig.sTimingInterface.vpCtx = nullptr;

    ASSERT_EQ(Mpu6050_Open(&sHandle, &sConfig), DRIVER_OK);
    EXPECT_EQ(sHandle.eState, MPU6050_STATE_READY);
    EXPECT_EQ(Mpu6050_Close(&sHandle), DRIVER_OK);
}
