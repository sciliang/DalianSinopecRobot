cmake_minimum_required(VERSION 3.22)
project(Dove_E4)
message("project_dir:${PROJECT_SOURCE_DIR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE DEBUG)
find_package(Boost REQUIRED )
find_package(GFlags REQUIRED)
MESSAGE(STATUS "Current path : ${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${Boost_INCLUDE_DIRS})
MESSAGE(STATUS "Cmake path : ${CMAKE_CURRENT_LIST_DIR}")
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/nmea)
include_directories(${CMAKE_CURRENT_LIST_DIR}/nmea/parsers)
link_directories(${PROJECT_SOURCE_DIR}/lib)
file(GLOB HEADER
    "include/*.h"
    "include/*.hpp"
    "nmea/*.hpp"
    "nmea/*.h"
    "nmea/parsers/*.hpp"
    "nmea/parsers/*.h"
    "rs232/*.hpp"
    "rs232/*.h"
)
file(GLOB SOURCES
    "src/*.cpp"
    "src/*.c"
    "nmea/*.cpp"
    "nmea/*.c"
    "nmea/parsers/*.cpp"
    "nmea/parsers/*.c"
    "rs232/*.cpp"
    "rs232/*.c"
)
set(RobotLib_DIR ${CMAKE_SOURCE_DIR}/lib)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/log)
ADD_DEFINITIONS(-DLOG_PATH="${CMAKE_BINARY_DIR}/log")
set(PROJECT_LIBARARIES
    /usr/lib/x86_64-linux-gnu/libglog.so.0
    pthread
    rt
    m
    )
message("----- Libraries Paths -----") 
message("libs:${PROJECT_LIBARARIES}")
message("-------------------------------")
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
MESSAGE(STATUS "projectname : ${PROJECT_NAME}")
add_executable(${PROJECT_NAME} ${HEADER} ${SOURCES})#告诉cmake 我的构建目标，构建可执行文件
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBARARIES} boost_thread)#告诉cmake我要链接的库文件
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBARARIES})#告诉cmake我要链接的库文件




