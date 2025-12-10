add_executable(dvsense_fusion_reader
    ${CMAKE_CURRENT_SOURCE_DIR}/DsFusionFileReaderSample.cpp
)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries(dvsense_fusion_reader
PRIVATE
    ${OpenCV_LIBS}
    DvsenseDriver::Driver
)

add_custom_target(copy_executable
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:dvsense_fusion_reader>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../launch/
    DEPENDS dvsense_fusion_reader
)

add_custom_target(default ALL DEPENDS dvsense_fusion_reader copy_executable)
