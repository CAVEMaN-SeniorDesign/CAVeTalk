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

/* TODO consider portability and sending CRC as byte array, remove functions if unused */
static inline uint8_t CaveTalk_GetUpperByte(const uint16_t value);
static inline uint8_t CaveTalk_GetLowerByte(const uint16_t value);
static inline uint16_t CaveTalk_GetUpperUint16(const uint32_t value);
static inline uint16_t CaveTalk_GetLowerUint16(const uint32_t value);

CaveTalk_Error_t CaveTalk_Speak(CaveTalk_LinkHandle_t *const handle,
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

        /* TODO calculate CRC */
        CaveTalk_Crc_t crc = 0U;

        /* TODO determine error behavior */
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
    CaveTalk_Error_t error = CAVE_TALK_ERROR_NULL;

    if ((NULL == handle) ||
        (NULL == handle->receive) ||
        (NULL == handle->available) ||
        (NULL == id) ||
        (NULL == data) ||
        (NULL == length))
    {
    }
    else if (handle->available < CAVE_TALK_HEADER_SIZE)
    {
        *id     = 0U; /* TODO set to message ID none */
        *length = 0U;

        error = CAVE_TALK_ERROR_NONE;
    }
    else
    {
        size_t         bytes_received = 0U;
        uint8_t        header[CAVE_TALK_HEADER_SIZE];
        CaveTalk_Crc_t crc = 0U;

        *id     = 0U; /* TODO set to message ID none */
        *length = 0U;

        /* TODO determine error behavior */
        /* TODO check version */
        /* Receive header */
        error   = handle->receive(header, sizeof(header), &bytes_received);
        *id     = header[CAVE_TALK_ID_INDEX];
        *length = header[CAVE_TALK_LENGTH_INDEX];

        /* Receive payload */
        if ((CAVE_TALK_ERROR_NONE != error) || (CAVE_TALK_HEADER_SIZE != bytes_received))
        {
            error = CAVE_TALK_ERROR_INCOMPLETE;
        }
        else if (size < length)
        {
            error = CAVE_TALK_ERROR_SIZE;
        }
        else
        {
            error = handle->receive(data, length, &bytes_received);
        }

        /* Receive CRC */
        if ((CAVE_TALK_ERROR_NONE != error) || (length != bytes_received))
        {
            error = CAVE_TALK_ERROR_INCOMPLETE;
        }
        else
        {
            error = handle->receive(&crc, sizeof(crc), &bytes_received);
        }

        /* Verify CRC */
        if ((CAVE_TALK_ERROR_NONE != error) || (sizeof(crc) != bytes_received))
        {
            error = CAVE_TALK_ERROR_INCOMPLETE;
        }
        else
        {
            /* TODO check CRC */
            error = CAVE_TALK_ERROR_NONE;
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