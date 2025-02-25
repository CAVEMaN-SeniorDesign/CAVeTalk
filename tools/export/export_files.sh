#!/bin/sh

SCRIPT_DIR=$(dirname "$0")
C_MESSAGES_DIR=$SCRIPT_DIR/../../build/CAVeTalk-c_protos
CPP_MESSAGES_DIR=$SCRIPT_DIR/../../build/CAVeTalk-cpp_protos
PROTOBUF_INSTALL=$SCRIPT_DIR/../../external/protobuf/_build/protobuf-install
PROJ_DIR=$SCRIPT_DIR/../.. 

default_path="/usr/local"
read -p "Please enter path (default: $default_path): " input
path="${input:-$default_path}"


# Generate path
if [ ! -d "$path" ]; then
    mkdir -p $path
fi 

# Generate c path
if [ ! -d "$path/include/c" ]; then
    mkdir -p $$path/include/c
fi 

# Generate c++ path
if [ ! -d "$path/include/c++" ]; then
    mkdir -p $$path/include/c++
fi 

# Generate lib path
if [ ! -d "$path/lib" ]; then
    mkdir -p $$path/lib
fi 

cd $PROJ_DIR
cmake --install build --prefix $path

cp -r ${C_MESSAGES_DIR}/. "$path/include/c"
cp -r ${CPP_MESSAGES_DIR}/. "$path/include/c++"
cp -r ${PROTOBUF_INSTALL}/. "$path/include/c++"

#cmake -DCMAKE_PREFIX_PATH="$path"
#list(APPEND CMAKE_PREFIX_PATH $path)
# Create a list by replacing colon with semicolon
#string(REPLACE ":" ";" CMAKE_PREFIX_PATH "$ENV{CMAKE_PREFIX_PATH}")
# Append to the newly created list
#list(APPEND CMAKE_PREFIX_PATH "$path")




