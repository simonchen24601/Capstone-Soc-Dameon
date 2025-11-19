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

file(GLOB APP_SOURCES
    "*.h"
    "*.cpp"
    "*.c"
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

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    pthread
    rt
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
)

# Link any mqtt/spdlog libraries installed under THIRD_PARTY_API if present.
# file(GLOB MQTT_LIBS
#     "${THIRD_PARTY_API}/mqtt/lib/*.a"
#     "${THIRD_PARTY_API}/mqtt/lib/*.so"
# )
# file(GLOB SPDLOG_LIBS
#     "${THIRD_PARTY_API}/spdlog/lib/*.a"
#     "${THIRD_PARTY_API}/spdlog/lib/*.so"
# )
if(MQTT_LIBS)
    message(STATUS "Found MQTT libs: ${MQTT_LIBS}")
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${MQTT_LIBS})
endif()
if(SPDLOG_LIBS)
    message(STATUS "Found spdlog libs: ${SPDLOG_LIBS}")
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${SPDLOG_LIBS})
endif()

if(OpenCV_FOUND)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()
