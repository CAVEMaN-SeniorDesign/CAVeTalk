#include <gtest/gtest.h>

#include "ooga_booga.pb.h"

#include "cave_talk.h"
#include "cave_talk_types.h"

// TODO SD-155

TEST(CaveTalkCTests, Placeholder)
{
    ASSERT_EQ(CAVE_TALK_ERROR_NULL, CaveTalk_SpeakOogaBooga(NULL, cave_talk_Say_SAY_OOGA));
}