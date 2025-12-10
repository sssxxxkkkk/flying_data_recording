cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
add_definitions(-D NOMINMAX)
project(dvsense_hal_sync)

find_package(DvsenseDriver COMPONENTS Driver REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/DsHalSyncSample.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    DvsenseDriver::Driver
    ${OpenCV_LIBS}
)