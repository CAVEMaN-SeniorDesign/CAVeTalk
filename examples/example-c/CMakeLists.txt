cmake_minimum_required(VERSION 3.30)
project(example-c)
set(CMAKE_PREFIX_PATH /exampleLocation)  #change this /testLocation to the location you installed the exported files to
set(HEADER_DIR ${CMAKE_PREFIX_PATH}/include/c)
set(COMMON_DIR ${CMAKE_PREFIX_PATH}/include/common)

find_package(CAVeTalk-c REQUIRED HINTS ${CMAKE_PREFIX_PATH})

add_executable(example-c "example-c.c" "example-c.h")

target_include_directories(example-c
    PUBLIC
        ${COMMON_DIR}
        ${HEADER_DIR}
)

target_link_libraries(example-c CAVeTalk-c)