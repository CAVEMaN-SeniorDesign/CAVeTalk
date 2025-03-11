#ifndef CAVE_TALK_TYPES_H
#define CAVE_TALK_TYPES_H

#include <stdint.h>

#define CAVE_TALK_UNUSED(arg) (void)(arg)

typedef uint8_t  CaveTalk_Version_t;
typedef uint8_t  CaveTalk_Id_t;
typedef uint8_t  CaveTalk_Length_t;
typedef uint32_t CaveTalk_Crc_t;
typedef double   CaveTalk_MetersPerSecond_t;
typedef double   CaveTalk_MetersPerSecondSquared_t;
typedef double   CaveTalk_Radian_t;
typedef double   CaveTalk_RadiansPerSecond_t;
typedef char*    CaveTalk_Message_t;


typedef enum
{
    CAVE_TALK_ERROR_NONE,
    CAVE_TALK_ERROR_NULL,
    CAVE_TALK_ERROR_SIZE,
    CAVE_TALK_ERROR_SOCKET_CLOSED,
    CAVE_TALK_ERROR_INCOMPLETE,
    CAVE_TALK_ERROR_CRC,
    CAVE_TALK_ERROR_VERSION,
    CAVE_TALK_ERROR_ID,
    CAVE_TALK_ERROR_PARSE
} CaveTalk_Error_t;

#endif /* CAVE_TALK_TYPES_H */