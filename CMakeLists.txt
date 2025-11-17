cmake_minimum_required(VERSION 3.22)
project(CapstoneSocDameon)

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_CXX_STANDARD 20)

set(CMAKE_SYSTEM_NAME Linux)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
set(THIRD_PARTY_API "../ThirdPartyApi")

set(CMAKE_CXX_FLAGS "-g -Wall")
set(CMAKE_CXX_DEBUG_FLAGS -O0 ${CMAKE_CXX_DEBUG_FLAGS})
set(CMAKE_CXX_RELEASE_FLAGS -O2 ${CMAKE_CXX_RELEASE_FLAGS})

set(CMAKE_PROJECT_NAME capstone_soc_dameon)

set(APP_SOURCES
    util.hpp
    main.cpp
    ini.h
    ini.c
    INIReader.h
    INIReader.cpp
    configservice.h
    configservice.cpp
    interface.h
    interface.cpp
    app.h
    app.cpp
    logger.h
    logger.cpp
)

include_directories(
    ${THIRD_PARTY_API}/mqtt/include
    ${THIRD_PARTY_API}/spdlog/include
    ${CURL_INCLUDE_DIRS}
)

link_directories(
    ${THIRD_PARTY_API}/mqtt/lib
    ${THIRD_PARTY_API}/spdlog/lib
)

add_executable(${CMAKE_PROJECT_NAME} ${APP_SOURCES})

target_link_libraries(${CMAKE_PROJECT_NAME}
    pthread
    rt
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
)

if(OpenCV_FOUND)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()
