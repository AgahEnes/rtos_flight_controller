#include "global_data_space.h"

#include <stdatomic.h>
#include <string.h>

typedef struct
{
    ts_TopicRawImu asRawImuBuffers[2];
    atomic_uint_fast8_t u8ActiveBufferIdx;
    atomic_uint_fast32_t u32SeqLock;
} ts_GdsRawImuStorage;

static ts_GdsRawImuStorage gsRawImuStorage;

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
