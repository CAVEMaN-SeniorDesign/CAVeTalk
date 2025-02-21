cmake_minimum_required(VERSION 3.30)
project(CAVeTalk-tests LANGUAGES C CXX)

string(COMPARE EQUAL "${CMAKE_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}" IS_TOP_LEVEL)

if(IS_TOP_LEVEL)
    enable_testing()
endif()

################################################################################
# GoogleTest
################################################################################
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/6910c9d9165801d8827d628cb72eb7ea9dd538c5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

################################################################################
# CAVeTalk library
################################################################################
if(IS_TOP_LEVEL)
    find_package(CAVeTalk-common REQUIRED)
    find_package(CAVeTalk-c REQUIRED)
    find_package(CAVeTalk-cpp REQUIRED)
endif()

################################################################################
# Common tests
################################################################################
set(${PROJECT_NAME}_COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/common/common_tests.cc
)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/common" FILES ${${PROJECT_NAME}_COMMON_SOURCES})
set(COMMON_TEST_TARGET ${PROJECT_NAME}-common)
add_executable(${COMMON_TEST_TARGET})
target_sources(${COMMON_TEST_TARGET}
    PRIVATE
        ${${PROJECT_NAME}_COMMON_SOURCES}
)
target_include_directories(${COMMON_TEST_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(${COMMON_TEST_TARGET}
    PUBLIC
        CAVeTalk-common
        GTest::gtest_main
        GTest::gmock
)
target_link_options(${COMMON_TEST_TARGET}
    PRIVATE
        --coverage
)
gtest_discover_tests(${COMMON_TEST_TARGET})

################################################################################
# C tests
################################################################################
set(${PROJECT_NAME}_C_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/c/cave_talk_tests.cc
)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/c" FILES ${${PROJECT_NAME}_C_SOURCES})
set(C_TEST_TARGET ${PROJECT_NAME}-c)
add_executable(${C_TEST_TARGET})
target_sources(${C_TEST_TARGET}
    PRIVATE
        ${${PROJECT_NAME}_C_SOURCES}
)
target_include_directories(${C_TEST_TARGET}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(${C_TEST_TARGET}
    PUBLIC
        CAVeTalk-c
        GTest::gtest_main
        GTest::gmock
)
target_link_options(${C_TEST_TARGET}
    PRIVATE
        --coverage
)
gtest_discover_tests(${C_TEST_TARGET})

################################################################################
# C++ tests
################################################################################
set(${PROJECT_NAME}_CPP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/c++/cave_talk_tests.cc
)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/c++" FILES ${${PROJECT_NAME}_CPP_SOURCES})
add_executable(${PROJECT_NAME}-cpp)
target_sources(${PROJECT_NAME}-cpp
    PRIVATE
        ${${PROJECT_NAME}_CPP_SOURCES}
)
target_include_directories(${PROJECT_NAME}-cpp
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(${PROJECT_NAME}-cpp
    PUBLIC
        CAVeTalk-cpp
        GTest::gtest_main
        GTest::gmock
)
target_link_options(${PROJECT_NAME}-cpp
    PRIVATE
        --coverage
)
gtest_discover_tests(${PROJECT_NAME}-cpp)