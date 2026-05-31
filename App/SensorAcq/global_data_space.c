#include "global_data_space.h"

#include <stdatomic.h>
#include <string.h>

typedef struct
{
    ts_TopicRawImu asRawImuBuffers[2];
    atomic_uint_fast8_t u8ActiveBufferIdx;
    atomic_uint_fast32_t u32SeqLock;
} ts_GdsRawImuStorage;

typedef struct
{
    ts_TopicVehicleState asVehicleStateBuffers[2];
    atomic_uint_fast8_t u8ActiveBufferIdx;
    atomic_uint_fast32_t u32SeqLock;
} ts_GdsVehicleStateStorage;

typedef struct
{
    ts_TopicImuCalibration asImuCalibrationBuffers[2];
    atomic_uint_fast8_t u8ActiveBufferIdx;
    atomic_uint_fast32_t u32SeqLock;
} ts_GdsImuCalibrationStorage;

typedef struct
{
    ts_TopicNavCommand asNavCommandBuffers[2];
    atomic_uint_fast8_t u8ActiveBufferIdx;
    atomic_uint_fast32_t u32SeqLock;
} ts_GdsNavCommandStorage;

static ts_GdsRawImuStorage gsRawImuStorage;
static ts_GdsVehicleStateStorage gsVehicleStateStorage;
static ts_GdsImuCalibrationStorage gsImuCalibrationStorage;
static ts_GdsNavCommandStorage gsNavCommandStorage;

void Gds_ResetRawImu(void)
{
    ts_TopicRawImu sZeroTopic;

    (void)memset(&sZeroTopic, 0, sizeof(sZeroTopic));
    (void)memset(&gsRawImuStorage, 0, sizeof(gsRawImuStorage));
    gsRawImuStorage.asRawImuBuffers[0] = sZeroTopic;
    gsRawImuStorage.asRawImuBuffers[1] = sZeroTopic;
    (void)atomic_store_explicit(&gsRawImuStorage.u8ActiveBufferIdx, 0U, memory_order_relaxed);
    (void)atomic_store_explicit(&gsRawImuStorage.u32SeqLock, 0U, memory_order_relaxed);
}

void Gds_ResetVehicleState(void)
{
    ts_TopicVehicleState sZeroTopic;

    (void)memset(&sZeroTopic, 0, sizeof(sZeroTopic));
    (void)memset(&gsVehicleStateStorage, 0, sizeof(gsVehicleStateStorage));
    gsVehicleStateStorage.asVehicleStateBuffers[0] = sZeroTopic;
    gsVehicleStateStorage.asVehicleStateBuffers[1] = sZeroTopic;
    (void)atomic_store_explicit(&gsVehicleStateStorage.u8ActiveBufferIdx, 0U, memory_order_relaxed);
    (void)atomic_store_explicit(&gsVehicleStateStorage.u32SeqLock, 0U, memory_order_relaxed);
}

void Gds_ResetImuCalibration(void)
{
    ts_TopicImuCalibration sZeroTopic;

    (void)memset(&sZeroTopic, 0, sizeof(sZeroTopic));
    (void)memset(&gsImuCalibrationStorage, 0, sizeof(gsImuCalibrationStorage));
    gsImuCalibrationStorage.asImuCalibrationBuffers[0] = sZeroTopic;
    gsImuCalibrationStorage.asImuCalibrationBuffers[1] = sZeroTopic;
    (void)atomic_store_explicit(&gsImuCalibrationStorage.u8ActiveBufferIdx, 0U, memory_order_relaxed);
    (void)atomic_store_explicit(&gsImuCalibrationStorage.u32SeqLock, 0U, memory_order_relaxed);
}

void Gds_ResetNavCommand(void)
{
    ts_TopicNavCommand sZeroTopic;

    (void)memset(&sZeroTopic, 0, sizeof(sZeroTopic));
    sZeroTopic.eCommand = NAV_CMD_NONE;
    (void)memset(&gsNavCommandStorage, 0, sizeof(gsNavCommandStorage));
    gsNavCommandStorage.asNavCommandBuffers[0] = sZeroTopic;
    gsNavCommandStorage.asNavCommandBuffers[1] = sZeroTopic;
    (void)atomic_store_explicit(&gsNavCommandStorage.u8ActiveBufferIdx, 0U, memory_order_relaxed);
    (void)atomic_store_explicit(&gsNavCommandStorage.u32SeqLock, 0U, memory_order_relaxed);
}

te_GdsRetCode Gds_PublishRawImu(const ts_TopicRawImu *psRawImu)
{
    uint8_t u8ActiveIdx;
    uint8_t u8WriteIdx;

    if (psRawImu == NULL)
    {
        return GDS_ERR_ARG;
    }

    (void)atomic_fetch_add_explicit(&gsRawImuStorage.u32SeqLock, 1U, memory_order_acq_rel);

    u8ActiveIdx = (uint8_t)atomic_load_explicit(&gsRawImuStorage.u8ActiveBufferIdx, memory_order_acquire);
    u8WriteIdx = (uint8_t)((uint8_t)1U - u8ActiveIdx);

    gsRawImuStorage.asRawImuBuffers[u8WriteIdx] = *psRawImu;

    (void)atomic_store_explicit(&gsRawImuStorage.u8ActiveBufferIdx, u8WriteIdx, memory_order_release);
    (void)atomic_fetch_add_explicit(&gsRawImuStorage.u32SeqLock, 1U, memory_order_acq_rel);

    return GDS_OK;
}

te_GdsRetCode Gds_ReadRawImu(ts_TopicRawImu *psRawImu)
{
    uint32_t u32SeqStart;
    uint32_t u32SeqEnd;
    uint8_t u8ActiveIdx;
    uint32_t u32RetryCount;

    if (psRawImu == NULL)
    {
        return GDS_ERR_ARG;
    }

    for (u32RetryCount = 0U; u32RetryCount < 3U; u32RetryCount++)
    {
        u32SeqStart = (uint32_t)atomic_load_explicit(&gsRawImuStorage.u32SeqLock, memory_order_acquire);
        if ((u32SeqStart & 1U) != 0U)
        {
            continue;
        }

        u8ActiveIdx = (uint8_t)atomic_load_explicit(&gsRawImuStorage.u8ActiveBufferIdx, memory_order_acquire);
        *psRawImu = gsRawImuStorage.asRawImuBuffers[u8ActiveIdx];

        u32SeqEnd = (uint32_t)atomic_load_explicit(&gsRawImuStorage.u32SeqLock, memory_order_acquire);
        if ((u32SeqStart == u32SeqEnd) && ((u32SeqEnd & 1U) == 0U))
        {
            return GDS_OK;
        }
    }

    return GDS_ERR_INCONSISTENT_READ;
}

te_GdsRetCode Gds_PublishVehicleState(const ts_TopicVehicleState *psVehicleState)
{
    uint8_t u8ActiveIdx;
    uint8_t u8WriteIdx;

    if (psVehicleState == NULL)
    {
        return GDS_ERR_ARG;
    }

    (void)atomic_fetch_add_explicit(&gsVehicleStateStorage.u32SeqLock, 1U, memory_order_acq_rel);

    u8ActiveIdx = (uint8_t)atomic_load_explicit(&gsVehicleStateStorage.u8ActiveBufferIdx, memory_order_acquire);
    u8WriteIdx = (uint8_t)((uint8_t)1U - u8ActiveIdx);

    gsVehicleStateStorage.asVehicleStateBuffers[u8WriteIdx] = *psVehicleState;

    (void)atomic_store_explicit(&gsVehicleStateStorage.u8ActiveBufferIdx, u8WriteIdx, memory_order_release);
    (void)atomic_fetch_add_explicit(&gsVehicleStateStorage.u32SeqLock, 1U, memory_order_acq_rel);

    return GDS_OK;
}

te_GdsRetCode Gds_ReadVehicleState(ts_TopicVehicleState *psVehicleState)
{
    uint32_t u32SeqStart;
    uint32_t u32SeqEnd;
    uint8_t u8ActiveIdx;
    uint32_t u32RetryCount;

    if (psVehicleState == NULL)
    {
        return GDS_ERR_ARG;
    }

    for (u32RetryCount = 0U; u32RetryCount < 3U; u32RetryCount++)
    {
        u32SeqStart = (uint32_t)atomic_load_explicit(&gsVehicleStateStorage.u32SeqLock, memory_order_acquire);
        if ((u32SeqStart & 1U) != 0U)
        {
            continue;
        }

        u8ActiveIdx = (uint8_t)atomic_load_explicit(&gsVehicleStateStorage.u8ActiveBufferIdx, memory_order_acquire);
        *psVehicleState = gsVehicleStateStorage.asVehicleStateBuffers[u8ActiveIdx];

        u32SeqEnd = (uint32_t)atomic_load_explicit(&gsVehicleStateStorage.u32SeqLock, memory_order_acquire);
        if ((u32SeqStart == u32SeqEnd) && ((u32SeqEnd & 1U) == 0U))
        {
            return GDS_OK;
        }
    }

    return GDS_ERR_INCONSISTENT_READ;
}

te_GdsRetCode Gds_PublishImuCalibration(const ts_TopicImuCalibration *psCalibration)
{
    uint8_t u8ActiveIdx;
    uint8_t u8WriteIdx;

    if (psCalibration == NULL)
    {
        return GDS_ERR_ARG;
    }

    (void)atomic_fetch_add_explicit(&gsImuCalibrationStorage.u32SeqLock, 1U, memory_order_acq_rel);

    u8ActiveIdx = (uint8_t)atomic_load_explicit(&gsImuCalibrationStorage.u8ActiveBufferIdx, memory_order_acquire);
    u8WriteIdx = (uint8_t)((uint8_t)1U - u8ActiveIdx);

    gsImuCalibrationStorage.asImuCalibrationBuffers[u8WriteIdx] = *psCalibration;

    (void)atomic_store_explicit(&gsImuCalibrationStorage.u8ActiveBufferIdx, u8WriteIdx, memory_order_release);
    (void)atomic_fetch_add_explicit(&gsImuCalibrationStorage.u32SeqLock, 1U, memory_order_acq_rel);

    return GDS_OK;
}

te_GdsRetCode Gds_ReadImuCalibration(ts_TopicImuCalibration *psCalibration)
{
    uint32_t u32SeqStart;
    uint32_t u32SeqEnd;
    uint8_t u8ActiveIdx;
    uint32_t u32RetryCount;

    if (psCalibration == NULL)
    {
        return GDS_ERR_ARG;
    }

    for (u32RetryCount = 0U; u32RetryCount < 3U; u32RetryCount++)
    {
        u32SeqStart = (uint32_t)atomic_load_explicit(&gsImuCalibrationStorage.u32SeqLock, memory_order_acquire);
        if ((u32SeqStart & 1U) != 0U)
        {
            continue;
        }

        u8ActiveIdx = (uint8_t)atomic_load_explicit(&gsImuCalibrationStorage.u8ActiveBufferIdx, memory_order_acquire);
        *psCalibration = gsImuCalibrationStorage.asImuCalibrationBuffers[u8ActiveIdx];

        u32SeqEnd = (uint32_t)atomic_load_explicit(&gsImuCalibrationStorage.u32SeqLock, memory_order_acquire);
        if ((u32SeqStart == u32SeqEnd) && ((u32SeqEnd & 1U) == 0U))
        {
            return GDS_OK;
        }
    }

    return GDS_ERR_INCONSISTENT_READ;
}

te_GdsRetCode Gds_PublishNavCommand(const ts_TopicNavCommand *psCommand)
{
    uint8_t u8ActiveIdx;
    uint8_t u8WriteIdx;

    if (psCommand == NULL)
    {
        return GDS_ERR_ARG;
    }

    (void)atomic_fetch_add_explicit(&gsNavCommandStorage.u32SeqLock, 1U, memory_order_acq_rel);

    u8ActiveIdx = (uint8_t)atomic_load_explicit(&gsNavCommandStorage.u8ActiveBufferIdx, memory_order_acquire);
    u8WriteIdx = (uint8_t)((uint8_t)1U - u8ActiveIdx);

    gsNavCommandStorage.asNavCommandBuffers[u8WriteIdx] = *psCommand;

    (void)atomic_store_explicit(&gsNavCommandStorage.u8ActiveBufferIdx, u8WriteIdx, memory_order_release);
    (void)atomic_fetch_add_explicit(&gsNavCommandStorage.u32SeqLock, 1U, memory_order_acq_rel);

    return GDS_OK;
}

te_GdsRetCode Gds_ReadNavCommand(ts_TopicNavCommand *psCommand)
{
    uint32_t u32SeqStart;
    uint32_t u32SeqEnd;
    uint8_t u8ActiveIdx;
    uint32_t u32RetryCount;

    if (psCommand == NULL)
    {
        return GDS_ERR_ARG;
    }

    for (u32RetryCount = 0U; u32RetryCount < 3U; u32RetryCount++)
    {
        u32SeqStart = (uint32_t)atomic_load_explicit(&gsNavCommandStorage.u32SeqLock, memory_order_acquire);
        if ((u32SeqStart & 1U) != 0U)
        {
            continue;
        }

        u8ActiveIdx = (uint8_t)atomic_load_explicit(&gsNavCommandStorage.u8ActiveBufferIdx, memory_order_acquire);
        *psCommand = gsNavCommandStorage.asNavCommandBuffers[u8ActiveIdx];

        u32SeqEnd = (uint32_t)atomic_load_explicit(&gsNavCommandStorage.u32SeqLock, memory_order_acquire);
        if ((u32SeqStart == u32SeqEnd) && ((u32SeqEnd & 1U) == 0U))
        {
            return GDS_OK;
        }
    }

    return GDS_ERR_INCONSISTENT_READ;
}
