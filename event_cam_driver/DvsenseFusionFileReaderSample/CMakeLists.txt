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