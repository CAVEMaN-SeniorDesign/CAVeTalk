cmake_minimum_required(VERSION 3.30)
project(example-cpp)
set(CMAKE_PREFIX_PATH /exampleLocation)    #change this /testLocation to the location you installed the exported files to
set(HEADER_DIR ${CMAKE_PREFIX_PATH}/include/c++)
set(COMMON_DIR ${CMAKE_PREFIX_PATH}/include/common)
set(PROTOBUF_DIR ${CMAKE_PREFIX_PATH}/include/c++/include)
find_package(CAVeTalk-cpp REQUIRED HINTS ${CMAKE_PREFIX_PATH})

add_executable(example-cpp "example-cpp.cpp" "example-cpp.h")

target_include_directories(example-cpp
    PUBLIC
        ${COMMON_DIR}
		${HEADER_DIR}
		${PROTOBUF_DIR}
)


set_property(TARGET example-cpp PROPERTY CXX_STANDARD 20)

target_link_libraries(example-cpp cave_talk::CAVeTalk-cpp)