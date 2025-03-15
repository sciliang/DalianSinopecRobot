
cmake_minimum_required(VERSION 3.22)
# project(PatrolRobot LANGUAGES CXX)
project(ZSHRobot)
message("project_dir:${PROJECT_SOURCE_DIR}")

#将一个CMAKE变量设置为给定值
# set(CMAKE_BUILD_TYPE DEBUG)#传递FLAGS给C++编译器
# set(CMAKE_C_FLAGS "-std=c99")
#Configure Complilation parameters
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)

#将一个CMAKE变量设置为给定值
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE DEBUG)#传递FLAGS给C++编译器

##
find_package(Python2 COMPONENTS Interpreter Development REQUIRED)
# target_link_libraries(PatrolRobot INTERFACE
#   Python3::Python
#   Python3::Module
# )
find_package(Python2 COMPONENTS Interpreter NumPy)
# if(Python3_NumPy_FOUND)
#   target_link_libraries(PatrolRobot INTERFACE
#     Python3::NumPy
#   )
# else()
#   target_compile_definitions(PatrolRobot INTERFACE WITHOUT_NUMPY)
# endif()
# install(
#   TARGETS PatrolRobot
#   EXPORT install_targets
# )
##
find_package(Boost REQUIRED )
find_package(GFlags REQUIRED)
MESSAGE(STATUS "Current path : ${CMAKE_CURRENT_SOURCE_DIR}")
add_library(tinyxml2 SHARED global/tinyxml2.cpp global/tinyxml2.h)
#把当前目录(CMakeLsits.txt所在目录)下的include文件加入到包含路径
include_directories(${Boost_INCLUDE_DIRS})
MESSAGE(STATUS "Cmake path : ${CMAKE_CURRENT_LIST_DIR}")
#将指定目录添加到编译器的头文件头文件搜索路径之下，指定的目录被解释成当前源码路径的相对路径
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/nmea)
include_directories(${CMAKE_CURRENT_LIST_DIR}/nmea/parsers)
link_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories( "/usr/include/eigen3" )
include_directories( "/usr/include/python2.7" )

#文件操作命令，将一则信息写入文件中，文件存在会覆盖，不存在会创建
file(GLOB HEADER
    "include/*.h"
    "include/*.hpp"
)

file(GLOB SOURCES
    "src/*.cpp"
    "src/*.c"
)
message("project_dir:${PROJECT_SOURCE_DIR}")

set(RobotLib_DIR ${CMAKE_SOURCE_DIR}/lib)
#set(RobotControl_DIR ${CMAKE_SOURCE_DIR}/lib/RobotControlLib)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/log)
ADD_DEFINITIONS(-DLOG_PATH="${CMAKE_BINARY_DIR}/log")

set(PROJECT_LIBARARIES
    ${RobotLib_DIR}/libxml2.so 
    ${RobotLib_DIR}/libxml2.so.2 
    ${RobotLib_DIR}/libxml2.so.2.9.12 
    ${RobotLib_DIR}/libtinyxml2.so 
    ${RobotLib_DIR}/libcontrolcan.so 
    ${RobotLib_DIR}/liban_packet_protocol.so 
    ${RobotLib_DIR}/libAvoidAbstacle.so 
    ${RobotLib_DIR}/libDriver.so 
    ${RobotLib_DIR}/libcoppeliaSimMsg.so 
    ${RobotLib_DIR}/libcontrol.so 
    ${RobotLib_DIR}/libglobal.so 
    ${RobotLib_DIR}/libkCurves.so 
    ${RobotLib_DIR}/libfilter.so 
    ${RobotLib_DIR}/librobottrace.so 
    ${RobotLib_DIR}/libdoveE4.so 
    ${RobotLib_DIR}/libcommunication.so 
    /usr/lib/x86_64-linux-gnu/libglog.so.0
    pthread
    rt
    m
    )

message("----- Libraries Paths -----") 
message("libs:${PROJECT_LIBARARIES}")

#编译生成动态链接库  
#--1--
# add_library(an_packet_protocol SHARED src/an_packet_protocol.c)

#--2--
# add_library(AvoidAbstacle SHARED src/AvoidAbstacle.cpp)

#--3--
# add_library(CAN_bus SHARED src/CAN_bus.cpp)
# target_link_libraries(CAN_bus PRIVATE controlcan)

#--4--
# add_library(Driver SHARED src/Driver.cpp)
# target_link_libraries(Driver PRIVATE CAN_bus)
# target_link_libraries(Driver PRIVATE tinyxml2)

#--5--
add_library(communication SHARED src/communication.cpp)
target_link_libraries(communication PRIVATE an_packet_protocol)

#--6--
# add_library(control SHARED src/control.cpp)

#--7--
add_library(coppeliaSimMsg SHARED src/coppeliaSimMsg.cpp)

#--8--
# add_library(DataPlot SHARED src/DataPlot.cpp)

#--9--
# add_library(doveE4 SHARED src/doveE4.cpp)

#--10--
# add_library(filter SHARED src/filter.cpp)

#--11--
# add_library(global SHARED src/global.cpp)

#--12--
# add_library(kCurves SHARED src/kCurves.cpp)

#--13--
add_library(robottrace SHARED src/robottrace.cpp)

message("-------------------------------")
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_executable(${PROJECT_NAME} ${HEADER} ${SOURCES})#告诉cmake 我的构建目标，构建可执行文件
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBARARIES} boost_thread Python2::Python Python2::Module  Python2::NumPy)#告诉cmake我要链接的库文件




