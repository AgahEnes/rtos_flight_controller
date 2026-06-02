#include "gtest/gtest.h"

extern "C" {
#include "actuator_manager.h"
#include "global_data_space.h"
}

namespace {

struct ts_FakeActuatorContext
{
    te_ActuatorDriverRetCode eInitRet;
    te_ActuatorDriverRetCode eWriteRet;
    float f32LastAngleRad;
    uint32_t u32WriteCallCount;
};

te_ActuatorDriverRetCode FakeActuator_Init(void *vpContext)
{
    ts_FakeActuatorContext *psContext;

    psContext = static_cast<ts_FakeActuatorContext *>(vpContext);
    if (psContext == nullptr)
    {
        return ACTUATOR_DRIVER_ERR_ARG;
    }

    return psContext->eInitRet;
}

te_ActuatorDriverRetCode FakeActuator_WriteAngle(void *vpContext, float f32AngleRad)
{
    ts_FakeActuatorContext *psContext;

    psContext = static_cast<ts_FakeActuatorContext *>(vpContext);
    if (psContext == nullptr)
    {
        return ACTUATOR_DRIVER_ERR_ARG;
    }
    if (psContext->eWriteRet != ACTUATOR_DRIVER_OK)
    {
        return psContext->eWriteRet;
    }

    if (psContext->u32WriteCallCount < UINT32_MAX)
    {
        psContext->u32WriteCallCount++;
    }
    psContext->f32LastAngleRad = f32AngleRad;

    return ACTUATOR_DRIVER_OK;
}

}  // namespace

TEST(ActuatorManagerTest, StepWritesAllAnglesForNewActiveCommand)
{
    static const ts_ActuatorDriverVTable ksVtable = {FakeActuator_Init, FakeActuator_WriteAngle};
    ts_ActuatorManagerContext sContext {};
    ts_ActuatorManagerConfig sConfig {};
    ts_ActuatorDevice asDevices[4] {};
    ts_FakeActuatorContext asFakeContexts[4] {};
    ts_TopicActuatorCmd sCommand {};

    for (uint8_t u8Idx = 0U; u8Idx < 4U; u8Idx++)
    {
        asFakeContexts[u8Idx].eInitRet = ACTUATOR_DRIVER_OK;
        asFakeContexts[u8Idx].eWriteRet = ACTUATOR_DRIVER_OK;
        asDevices[u8Idx].psVTable = &ksVtable;
        asDevices[u8Idx].vpContext = &asFakeContexts[u8Idx];
        sCommand.f32FinAngleRad[u8Idx] = (0.1F * static_cast<float>(u8Idx + 1U));
    }
    sCommand.u32Sequence = 10U;
    sCommand.bIsActive = true;

    sConfig.psActuatorDevices = asDevices;
    sConfig.u8ActuatorDeviceCount = 4U;

    Gds_ResetActuatorCmd();
    ASSERT_EQ(ActuatorManager_Init(&sContext, &sConfig), ACTUATOR_MANAGER_OK);
    ASSERT_EQ(Gds_PublishActuatorCmd(&sCommand), GDS_OK);
    ASSERT_EQ(ActuatorManager_Step(&sContext), ACTUATOR_MANAGER_OK);

    for (uint8_t u8Idx = 0U; u8Idx < 4U; u8Idx++)
    {
        EXPECT_EQ(asFakeContexts[u8Idx].u32WriteCallCount, 1U);
        EXPECT_FLOAT_EQ(asFakeContexts[u8Idx].f32LastAngleRad, sCommand.f32FinAngleRad[u8Idx]);
    }
}

TEST(ActuatorManagerTest, StepDedupesSameSequenceAndRewritesOnNewSequence)
{
    static const ts_ActuatorDriverVTable ksVtable = {FakeActuator_Init, FakeActuator_WriteAngle};
    ts_ActuatorManagerContext sContext {};
    ts_ActuatorManagerConfig sConfig {};
    ts_ActuatorDevice asDevices[4] {};
    ts_FakeActuatorContext asFakeContexts[4] {};
    ts_TopicActuatorCmd sCommand {};

    for (uint8_t u8Idx = 0U; u8Idx < 4U; u8Idx++)
    {
        asFakeContexts[u8Idx].eInitRet = ACTUATOR_DRIVER_OK;
        asFakeContexts[u8Idx].eWriteRet = ACTUATOR_DRIVER_OK;
        asDevices[u8Idx].psVTable = &ksVtable;
        asDevices[u8Idx].vpContext = &asFakeContexts[u8Idx];
    }

    sConfig.psActuatorDevices = asDevices;
    sConfig.u8ActuatorDeviceCount = 4U;

    Gds_ResetActuatorCmd();
    ASSERT_EQ(ActuatorManager_Init(&sContext, &sConfig), ACTUATOR_MANAGER_OK);

    sCommand.f32FinAngleRad[0] = 0.11F;
    sCommand.f32FinAngleRad[1] = 0.22F;
    sCommand.f32FinAngleRad[2] = 0.33F;
    sCommand.f32FinAngleRad[3] = 0.44F;
    sCommand.u32Sequence = 1U;
    sCommand.bIsActive = true;
    ASSERT_EQ(Gds_PublishActuatorCmd(&sCommand), GDS_OK);
    ASSERT_EQ(ActuatorManager_Step(&sContext), ACTUATOR_MANAGER_OK);

    ASSERT_EQ(ActuatorManager_Step(&sContext), ACTUATOR_MANAGER_OK);
    for (uint8_t u8Idx = 0U; u8Idx < 4U; u8Idx++)
    {
        EXPECT_EQ(asFakeContexts[u8Idx].u32WriteCallCount, 1U);
    }

    sCommand.f32FinAngleRad[0] = -0.11F;
    sCommand.f32FinAngleRad[1] = -0.22F;
    sCommand.f32FinAngleRad[2] = -0.33F;
    sCommand.f32FinAngleRad[3] = -0.44F;
    sCommand.u32Sequence = 2U;
    ASSERT_EQ(Gds_PublishActuatorCmd(&sCommand), GDS_OK);
    ASSERT_EQ(ActuatorManager_Step(&sContext), ACTUATOR_MANAGER_OK);

    for (uint8_t u8Idx = 0U; u8Idx < 4U; u8Idx++)
    {
        EXPECT_EQ(asFakeContexts[u8Idx].u32WriteCallCount, 2U);
        EXPECT_FLOAT_EQ(asFakeContexts[u8Idx].f32LastAngleRad, sCommand.f32FinAngleRad[u8Idx]);
    }
}

TEST(ActuatorManagerTest, StepHoldsLastWhenCommandInactive)
{
    static const ts_ActuatorDriverVTable ksVtable = {FakeActuator_Init, FakeActuator_WriteAngle};
    ts_ActuatorManagerContext sContext {};
    ts_ActuatorManagerConfig sConfig {};
    ts_ActuatorDevice asDevices[4] {};
    ts_FakeActuatorContext asFakeContexts[4] {};
    ts_TopicActuatorCmd sCommand {};

    for (uint8_t u8Idx = 0U; u8Idx < 4U; u8Idx++)
    {
        asFakeContexts[u8Idx].eInitRet = ACTUATOR_DRIVER_OK;
        asFakeContexts[u8Idx].eWriteRet = ACTUATOR_DRIVER_OK;
        asDevices[u8Idx].psVTable = &ksVtable;
        asDevices[u8Idx].vpContext = &asFakeContexts[u8Idx];
    }

    sConfig.psActuatorDevices = asDevices;
    sConfig.u8ActuatorDeviceCount = 4U;

    Gds_ResetActuatorCmd();
    ASSERT_EQ(ActuatorManager_Init(&sContext, &sConfig), ACTUATOR_MANAGER_OK);

    sCommand.f32FinAngleRad[0] = 0.7F;
    sCommand.f32FinAngleRad[1] = 0.8F;
    sCommand.f32FinAngleRad[2] = 0.9F;
    sCommand.f32FinAngleRad[3] = 1.0F;
    sCommand.u32Sequence = 50U;
    sCommand.bIsActive = false;
    ASSERT_EQ(Gds_PublishActuatorCmd(&sCommand), GDS_OK);
    ASSERT_EQ(ActuatorManager_Step(&sContext), ACTUATOR_MANAGER_OK);

    for (uint8_t u8Idx = 0U; u8Idx < 4U; u8Idx++)
    {
        EXPECT_EQ(asFakeContexts[u8Idx].u32WriteCallCount, 0U);
    }
}
