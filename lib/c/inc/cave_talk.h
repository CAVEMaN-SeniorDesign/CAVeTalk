#ifndef CAVE_TALK_H
#define CAVE_TALK_H

#include <stdbool.h>
#include <stdint.h>

#include "ooga_booga.pb.h"

#include "cave_talk_link.h"
#include "cave_talk_types.h"

typedef struct
{
    void (*hear_ooga_booga)(const cave_talk_Say ooga_booga);
    void (*hear_movement)(const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate);
    void (*hear_camera_movement)(const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt);
    void (*hear_lights)(const bool headlights);
    void (*hear_mode)(const bool manual);
} CaveTalk_ListenCallbacks_t;

typedef struct
{
    CaveTalk_LinkHandle_t link_handle;
    uint8_t *buffer;
    size_t buffer_size;
    CaveTalk_ListenCallbacks_t listen_callbacks;
} CaveTalk_Handle_t;

const CaveTalk_ListenCallbacks_t kCaveTalk_ListenCallbacksNull = {
    .hear_ooga_booga      = NULL,
    .hear_movement        = NULL,
    .hear_camera_movement = NULL,
    .hear_lights          = NULL,
    .hear_mode            = NULL,
};

const CaveTalk_Handle_t kCaveTalk_HandleNull = {
    .link_handle      = kCaveTalk_LinkHandleNull,
    .buffer           = NULL,
    .buffer_size      = 0U,
    .listen_callbacks = kCaveTalk_ListenCallbacksNull,
};

#ifdef __cplusplus
extern "C"
{
#endif

CaveTalk_Error_t CaveTalk_Hear(const CaveTalk_Handle_t *const handle);
CaveTalk_Error_t CaveTalk_SpeakOogaBooga(const CaveTalk_Handle_t *const handle, const cave_talk_Say ooga_booga);
CaveTalk_Error_t CaveTalk_SpeakMovement(const CaveTalk_Handle_t *const handle, const CaveTalk_MetersPerSecond_t speed, const CaveTalk_RadiansPerSecond_t turn_rate);
CaveTalk_Error_t CaveTalk_SpeakCameraMovement(const CaveTalk_Handle_t *const handle, const CaveTalk_Radian_t pan, const CaveTalk_Radian_t tilt);
CaveTalk_Error_t CaveTalk_SpeakLights(const CaveTalk_Handle_t *const handle, const bool headlights);
CaveTalk_Error_t CaveTalk_SpeakMode(const CaveTalk_Handle_t *const handle, const bool manual);

#ifdef __cplusplus
}
#endif

#endif /* CAVE_TALK_H */