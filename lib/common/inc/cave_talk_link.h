#ifndef CAVE_TALK_LINK_H
#define CAVE_TALK_LINK_H

#include <stddef.h>

#include "cave_talk_types.h"

typedef struct
{
    CaveTalk_Error_t (*send)(const void *const data, const size_t size);
    CaveTalk_Error_t (*receive)(void *const data, const size_t size, size_t *const bytes_received);
    CaveTalk_Error_t (*available)(size_t *const bytes_available);
} CaveTalk_LinkHandle_t;

const CaveTalk_LinkHandle_t kCaveTalk_LinkHandleNull = {
    .send = NULL, .receive = NULL, .available = NULL
};

#ifdef __cplusplus
extern "C"
{
#endif

CaveTalk_Error_t CaveTalk_Speak(const CaveTalk_LinkHandle_t *const handle,
                                const CaveTalk_Id_t id,
                                const void *const data,
                                const CaveTalk_Length_t length);
CaveTalk_Error_t CaveTalk_Listen(const CaveTalk_LinkHandle_t *const handle,
                                 CaveTalk_Id_t *const id,
                                 void *const data,
                                 const size_t size,
                                 CaveTalk_Length_t *const length);

#ifdef __cplusplus
}
#endif

#endif /* CAVE_TALK_LINK_H */