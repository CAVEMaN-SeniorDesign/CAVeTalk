#ifndef CAVE_TALK_LINK_H
#define CAVE_TALK_LINK_H

#include <stddef.h>

#include "cave_talk_types.h"

#define CAVE_TALK_ID_NONE 0U /* See ids.proto */

typedef enum
{
    CAVE_TALK_LINK_STATE_RESET,
    CAVE_TALK_LINK_STATE_HEADER,
    CAVE_TALK_LINK_STATE_PAYLOAD,
    CAVE_TALK_LINK_STATE_CRC,
    CAVE_TALK_LINK_STATE_COMPLETE
} CaveTalk_LinkState_t;

typedef struct
{
    CaveTalk_Error_t (*send)(const void *const data, const size_t size);
    CaveTalk_Error_t (*receive)(void *const data, const size_t size, size_t *const bytes_received);
    CaveTalk_LinkState_t receive_state;
    CaveTalk_Id_t receive_id;
    CaveTalk_Length_t receive_length;
    CaveTalk_Crc_t crc;
    size_t bytes_received;
} CaveTalk_LinkHandle_t;

static const CaveTalk_LinkHandle_t kCaveTalk_LinkHandleNull = {
    .send           = NULL,
    .receive        = NULL,
    .receive_state  = CAVE_TALK_LINK_STATE_RESET,
    .receive_id     = CAVE_TALK_ID_NONE,
    .receive_length = 0U,
    .crc            = 0U,
    .bytes_received = 0U,
};

#ifdef __cplusplus
extern "C"
{
#endif

CaveTalk_Error_t CaveTalk_Speak(const CaveTalk_LinkHandle_t *const handle,
                                const CaveTalk_Id_t id,
                                const void *const data,
                                const CaveTalk_Length_t length);
CaveTalk_Error_t CaveTalk_Listen(CaveTalk_LinkHandle_t *const handle,
                                 CaveTalk_Id_t *const id,
                                 void *const data,
                                 const size_t size,
                                 CaveTalk_Length_t *const length);

#ifdef __cplusplus
}
#endif

#endif /* CAVE_TALK_LINK_H */