#include "cave_talk_link.h"

#include <stddef.h>
#include <stdint.h>

#include "cave_talk_types.h"

#define CAVE_TALK_VERSION 1U

#define CAVE_TALK_VERSION_INDEX 0U
#define CAVE_TALK_ID_INDEX      (CAVE_TALK_VERSION_INDEX + sizeof(CaveTalk_Version_t))
#define CAVE_TALK_LENGTH_INDEX  (CAVE_TALK_ID_INDEX + sizeof(CaveTalk_Id_t))

#define CAVE_TALK_HEADER_SIZE (CAVE_TALK_LENGTH_INDEX + 1U)

#define CAVE_TALK_CRC_INDEX_0 0U
#define CAVE_TALK_CRC_INDEX_1 (CAVE_TALK_CRC_INDEX_0 + 1)
#define CAVE_TALK_CRC_INDEX_2 (CAVE_TALK_CRC_INDEX_0 + 2)
#define CAVE_TALK_CRC_INDEX_3 (CAVE_TALK_CRC_INDEX_0 + 3)

#define CAVE_TALK_BYTE_MASK        0xFFU
#define CAVE_TALK_UINT16_MASK      0xFFFFU
#define CAVE_TALK_BYTE_BIT_SHIFT   8U
#define CAVE_TALK_UINT16_BIT_SHIFT 16U

static CaveTalk_Error_t CaveTalk_Receive(CaveTalk_LinkHandle_t *const handle,
                                         CaveTalk_Id_t *const id,
                                         void *const data,
                                         const size_t size,
                                         CaveTalk_Length_t *const length);
static CaveTalk_Error_t CaveTalk_ReceiveHeader(CaveTalk_LinkHandle_t *const handle, void *const data, const size_t size);
static CaveTalk_Error_t CaveTalk_ReceivePayload(CaveTalk_LinkHandle_t *const handle, void *const data, const size_t size);
static CaveTalk_Error_t CaveTalk_ReceiveCrc(CaveTalk_LinkHandle_t *const handle);

/* TODO SD-164 consider portability and sending CRC as byte array, remove functions if unused */
static inline uint8_t CaveTalk_GetUpperByte(const uint16_t value);
static inline uint8_t CaveTalk_GetLowerByte(const uint16_t value);
static inline uint16_t CaveTalk_GetUpperUint16(const uint32_t value);
static inline uint16_t CaveTalk_GetLowerUint16(const uint32_t value);

CaveTalk_Error_t CaveTalk_Speak(const CaveTalk_LinkHandle_t *const handle,
                                const CaveTalk_Id_t id,
                                const void *const data,
                                const CaveTalk_Length_t length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->send) || (NULL == data))
    {
    }
    else
    {
        uint8_t header[CAVE_TALK_HEADER_SIZE];
        header[CAVE_TALK_VERSION_INDEX] = CAVE_TALK_VERSION;
        header[CAVE_TALK_ID_INDEX]      = id;
        header[CAVE_TALK_LENGTH_INDEX]  = length;

        /* TODO SD-164 calculate CRC */
        CaveTalk_Crc_t crc = 0U;

        /* TODO SD-182 determine error behavior */
        /* Send header */
        error = handle->send(header, sizeof(header));

        /* Send payload */
        if (CAVE_TALK_ERROR_NONE == error)
        {
            error = handle->send(data, length);
        }

        /* Send CRC */
        if (CAVE_TALK_ERROR_NONE == error)
        {
            error = handle->send(&crc, sizeof(crc));
        }
    }

    return error;
}

CaveTalk_Error_t CaveTalk_Listen(CaveTalk_LinkHandle_t *const handle,
                                 CaveTalk_Id_t *const id,
                                 void *const data,
                                 const size_t size,
                                 CaveTalk_Length_t *const length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) ||
        (NULL == id) ||
        (NULL == data) ||
        (NULL == length))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        *id     = CAVE_TALK_ID_NONE;
        *length = 0U;

        while (CAVE_TALK_ERROR_NONE == error)
        {
            CaveTalk_LinkState_t previous_state = handle->receive_state;
            error = CaveTalk_Receive(handle, id, data, size, length);

            if ((previous_state == handle->receive_state) || (CAVE_TALK_LINK_STATE_RESET == handle->receive_state))
            {
                break;
            }
        }

        /* TODO SD-182 drop packet on size error (flush remaining bytes to be received) */
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_Receive(CaveTalk_LinkHandle_t *const handle,
                                         CaveTalk_Id_t *const id,
                                         void *const data,
                                         const size_t size,
                                         CaveTalk_Length_t *const length)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

    if ((NULL == handle) ||
        (NULL == id) ||
        (NULL == data) ||
        (NULL == length))
    {
        error = CAVE_TALK_ERROR_NULL;
    }
    else
    {
        switch (handle->receive_state)
        {
        case CAVE_TALK_LINK_STATE_RESET:
            handle->receive_state  = CAVE_TALK_LINK_STATE_HEADER;
            handle->receive_id     = CAVE_TALK_ID_NONE;
            handle->receive_length = 0U;
            handle->crc            = 0U;
            handle->bytes_received = 0U;
            break;
        case CAVE_TALK_LINK_STATE_HEADER:
            error = CaveTalk_ReceiveHeader(handle, data, size);
            break;
        case CAVE_TALK_LINK_STATE_PAYLOAD:
            error = CaveTalk_ReceivePayload(handle, data, size);
            break;
        case CAVE_TALK_LINK_STATE_CRC:
            error = CaveTalk_ReceiveCrc(handle);
            break;
        case CAVE_TALK_LINK_STATE_COMPLETE:
            *id                   = handle->receive_id;
            *length               = handle->receive_length;
            handle->receive_state = CAVE_TALK_LINK_STATE_RESET;
            break;
        default:
            break;
        }
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_ReceiveHeader(CaveTalk_LinkHandle_t *const handle, void *const data, const size_t size)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->receive) || (NULL == data))
    {
    }
    else if (size < CAVE_TALK_HEADER_SIZE)
    {
        error = CAVE_TALK_ERROR_SIZE;
    }
    else
    {
        size_t bytes_received = 0U;
        error = handle->receive(((uint8_t*)data + handle->bytes_received), (CAVE_TALK_HEADER_SIZE - handle->bytes_received), &bytes_received);

        if (CAVE_TALK_ERROR_NONE == error)
        {
            handle->bytes_received += bytes_received;

            if (CAVE_TALK_HEADER_SIZE == handle->bytes_received)
            {
                /* TODO SD-184 check version */
                handle->receive_state  = CAVE_TALK_LINK_STATE_PAYLOAD;
                handle->receive_id     = *(uint8_t *)((uint8_t *)data + CAVE_TALK_ID_INDEX);
                handle->receive_length = *(uint8_t *)((uint8_t *)data + CAVE_TALK_LENGTH_INDEX);
                handle->bytes_received = 0U;
            }
        }
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_ReceivePayload(CaveTalk_LinkHandle_t *const handle, void *const data, const size_t size)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->receive) || (NULL == data))
    {
    }
    else if (size < handle->receive_length)
    {
        error = CAVE_TALK_ERROR_SIZE;
    }
    else
    {
        size_t bytes_received = 0U;
        error = handle->receive(((uint8_t*)data + handle->bytes_received), (handle->receive_length - handle->bytes_received), &bytes_received);

        if (CAVE_TALK_ERROR_NONE == error)
        {
            handle->bytes_received += bytes_received;

            if (handle->receive_length == handle->bytes_received)
            {
                handle->receive_state  = CAVE_TALK_LINK_STATE_CRC;
                handle->bytes_received = 0U;
            }
        }
    }

    return error;
}

static CaveTalk_Error_t CaveTalk_ReceiveCrc(CaveTalk_LinkHandle_t *const handle)
{
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) || (NULL == handle->receive))
    {
    }
    else
    {
        size_t bytes_received = 0U;
        error = handle->receive(((uint8_t*)&handle->crc + handle->bytes_received), (sizeof(CaveTalk_Crc_t) - handle->bytes_received), &bytes_received);

        if (CAVE_TALK_ERROR_NONE == error)
        {
            handle->bytes_received += bytes_received;

            if (sizeof(CaveTalk_Crc_t) == handle->bytes_received)
            {
                /* TODO SD-164 check CRC */
                handle->receive_state  = CAVE_TALK_LINK_STATE_COMPLETE;
                handle->bytes_received = 0U;
            }
        }
    }

    return error;
}

static inline uint8_t CaveTalk_GetUpperByte(const uint16_t value)
{
    return (uint8_t)((value >> CAVE_TALK_BYTE_BIT_SHIFT) & CAVE_TALK_BYTE_MASK);
}

static inline uint8_t CaveTalk_GetLowerByte(const uint16_t value)
{
    return (uint8_t)(value & CAVE_TALK_BYTE_MASK);
}

static inline uint16_t CaveTalk_GetUpperUint16(const uint32_t value)
{
    return (uint16_t)((value >> CAVE_TALK_UINT16_BIT_SHIFT) & CAVE_TALK_UINT16_MASK);
}

static inline uint16_t CaveTalk_GetLowerUint16(const uint32_t value)
{
    return (uint16_t)(value & CAVE_TALK_UINT16_MASK);
}