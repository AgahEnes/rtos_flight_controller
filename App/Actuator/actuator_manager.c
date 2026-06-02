#include "actuator_manager.h"

#include <string.h>

#include "global_data_space.h"

te_ActuatorManagerRetCode ActuatorManager_Init(ts_ActuatorManagerContext *psContext,
                                               const ts_ActuatorManagerConfig *psConfig)
{
    uint8_t u8Idx;
    te_ActuatorDriverRetCode eInitRet;

    if ((psContext == NULL) || (psConfig == NULL) || (psConfig->psActuatorDevices == NULL))
    {
        return ACTUATOR_MANAGER_ERR_ARG;
    }

    if ((psConfig->u8ActuatorDeviceCount == 0U) || (psConfig->u8ActuatorDeviceCount > ACTUATOR_MANAGER_MAX_DEVICES))
    {
        return ACTUATOR_MANAGER_ERR_ARG;
    }

    for (u8Idx = 0U; u8Idx < psConfig->u8ActuatorDeviceCount; u8Idx++)
    {
        if ((psConfig->psActuatorDevices[u8Idx].psVTable == NULL) ||
            (psConfig->psActuatorDevices[u8Idx].psVTable->pfnInit == NULL) ||
            (psConfig->psActuatorDevices[u8Idx].psVTable->pfnWriteAngle == NULL))
        {
            return ACTUATOR_MANAGER_ERR_ARG;
        }
    }

    (void)memset(psContext, 0, sizeof(*psContext));
    psContext->sConfig = *psConfig;

    for (u8Idx = 0U; u8Idx < psContext->sConfig.u8ActuatorDeviceCount; u8Idx++)
    {
        eInitRet = psContext->sConfig.psActuatorDevices[u8Idx].psVTable->pfnInit(
            psContext->sConfig.psActuatorDevices[u8Idx].vpContext);
        if (eInitRet != ACTUATOR_DRIVER_OK)
        {
            return ACTUATOR_MANAGER_ERR_DRIVER;
        }
    }

    psContext->u8IsInitialized = 1U;
    return ACTUATOR_MANAGER_OK;
}

te_ActuatorManagerRetCode ActuatorManager_Step(ts_ActuatorManagerContext *psContext)
{
    uint8_t u8Idx;
    ts_TopicActuatorCmd sActuatorCmd;
    te_ActuatorDriverRetCode eWriteRet;
    te_GdsRetCode eGdsRet;

    if ((psContext == NULL) || (psContext->u8IsInitialized == 0U))
    {
        return ACTUATOR_MANAGER_ERR_STATE;
    }

    (void)memset(&sActuatorCmd, 0, sizeof(sActuatorCmd));
    eGdsRet = Gds_ReadActuatorCmd(&sActuatorCmd);
    if (eGdsRet != GDS_OK)
    {
        return ACTUATOR_MANAGER_ERR_GDS;
    }

    if ((sActuatorCmd.bIsActive == false) ||
        ((psContext->u8HasAppliedCommand == 1U) &&
         (sActuatorCmd.u32Sequence == psContext->u32LastAppliedSequence)))
    {
        return ACTUATOR_MANAGER_OK;
    }

    for (u8Idx = 0U; u8Idx < psContext->sConfig.u8ActuatorDeviceCount; u8Idx++)
    {
        eWriteRet = psContext->sConfig.psActuatorDevices[u8Idx].psVTable->pfnWriteAngle(
            psContext->sConfig.psActuatorDevices[u8Idx].vpContext,
            sActuatorCmd.f32FinAngleRad[u8Idx]);
        if (eWriteRet != ACTUATOR_DRIVER_OK)
        {
            return ACTUATOR_MANAGER_ERR_DRIVER;
        }
    }

    psContext->u8HasAppliedCommand = 1U;
    psContext->u32LastAppliedSequence = sActuatorCmd.u32Sequence;

    return ACTUATOR_MANAGER_OK;
}
