cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
add_definitions(-D NOMINMAX)
project(dvsense_file_reader)

find_package(DvsenseDriver COMPONENTS Driver REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/DsFileReaderSample.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    DvsenseDriver::Driver
    ${OpenCV_LIBS}
)


add_custom_target(copy_executable
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:dvsense_file_reader>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../launch/
    DEPENDS dvsense_file_reader
)

add_custom_target(default ALL DEPENDS dvsense_file_reader copy_executable)
