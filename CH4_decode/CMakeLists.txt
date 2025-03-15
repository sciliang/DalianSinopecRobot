
cmake_minimum_required(VERSION 3.21)
project(CH4_decode)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED 11) # MUST BE C++ 14 FOR {XTENSOR} !!!
set(CMAKE_BUILD_TYPE "Debug")
include_directories(
    "include"
)
message("project_dir:${PROJECT_SOURCE_DIR}")

file(GLOB HEADER
    "include/*.h"
    "include/*.hpp"
)

file(GLOB SOURCES
    "src/*.c"
    "src/*.cpp"
)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/log)
ADD_DEFINITIONS(-DLOG_PATH="${CMAKE_BINARY_DIR}/log")

set(PROJECT_LIBARARIES
    # ${CMAKE_SOURCE_DIR}/lib/libglog.so
    pthread
    rt
    )

message("----- Libraries Paths -----") 
message("libs:${PROJECT_LIBARARIES}")
message("-------------------------------")
add_executable(${PROJECT_NAME} ${HEADER} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBARARIES})