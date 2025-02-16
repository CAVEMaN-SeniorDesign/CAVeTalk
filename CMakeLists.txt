cmake_minimum_required(VERSION 3.30)
project(CavemanController LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always")
# Add global flags for other compilers here
endif()

set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/external)
set(LIB_DIR ${CMAKE_SOURCE_DIR}/lib)

################################################################################
# Common sources
################################################################################
set(COMMON_DIR ${LIB_DIR}/common)
set(COMMON_INC_DIR ${COMMON_DIR}/inc)
set(COMMON_SRC_DIR ${COMMON_DIR}/src)
set(COMMON_SRCS ${COMMON_SRC_DIR}/cave_talk_link.c)

################################################################################
# Messages sources
################################################################################
set(CAVE_TALK_MESSAGES_DIR ${CMAKE_SOURCE_DIR}/messages)
file(GLOB CAVE_TALK_MESSAGE_SRCS LIST_DIRECTORIES false CONFIGURE_DEPENDS
    "${CAVE_TALK_MESSAGES_DIR}/*.proto"
)

################################################################################
# C++ messages
################################################################################
option(ABSL_PROPAGATE_CXX_STD "Propagate C++ standard to abseil-cpp" ON)
set(PROTOBUF_INSTALL_DIR ${EXTERNAL_DIR}/protobuf/_build/protobuf-install)
add_subdirectory(${CMAKE_SOURCE_DIR}/external/protobuf/third_party/abseil-cpp)
find_package(utf8_range CONFIG REQUIRED HINTS ${PROTOBUF_INSTALL_DIR})
find_package(Protobuf CONFIG REQUIRED HINTS ${PROTOBUF_INSTALL_DIR})
add_library(CAVeTalk_cpp_messages)
protobuf_generate(
	TARGET CAVeTalk_cpp_messages
	LANGUAGE cpp
	PROTOS ${CAVE_TALK_MESSAGE_SRCS}
	IMPORT_DIRS ${CAVE_TALK_MESSAGES_DIR}
)
target_link_libraries(CAVeTalk_cpp_messages PRIVATE protobuf::libprotobuf)
target_include_directories(CAVeTalk_cpp_messages
    PUBLIC
        ${PROTOBUF_INSTALL_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}
)
target_compile_options(CAVeTalk_cpp_messages PRIVATE -Wno-missing-requires)

################################################################################
# C++ library
################################################################################
set(CPP_DIR ${LIB_DIR}/c++)
set(CPP_INC_DIR ${CPP_DIR}/inc)
set(CPP_SRC_DIR ${CPP_DIR}/src)
set(CPP_SRCS ${CPP_SRC_DIR}/cave_talk.cc)
add_library(CAVeTalk_cpp)
target_sources(CAVeTalk_cpp
    PRIVATE
        ${COMMON_SRC}
        ${CPP_SRCS}
)
target_include_directories(CAVeTalk_cpp
    PUBLIC
        ${COMMON_INC_DIR}
        ${CPP_INC_DIR}
)
target_link_libraries(CAVeTalk_cpp PRIVATE CAVeTalk_cpp_messages)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(CAVeTalk_cpp PRIVATE -Wall -Wextra -Werror -Wno-missing-requires)
# Add flags for other compilers here
endif()

################################################################################
# CI and tools
################################################################################
set(TOOLS_DIR ${CMAKE_SOURCE_DIR}/tools)
set(CPPCHECK_SOURCES ${COMMON_SRCS} ${CPP_SRCS})
include(${TOOLS_DIR}/cppcheck/cppcheck.cmake)
set(UNCRUSTIFY_SOURCES ${CPPCHECK_SOURCES})
set(UNCRUSTIFY_INC_DIRS ${COMMON_INC_DIR} ${CPP_INC_DIR})
include(${TOOLS_DIR}/uncrustify/uncrustify.cmake)