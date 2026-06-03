cmake_minimum_required(VERSION 3.10)

project(MultiProject)

# ==========================================
# Added: Global C++17 standard settings (fixes std::filesystem errors)
# ==========================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Link filesystem library for GCC compiler
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # GCC versions below 11 require separate linking of stdc++fs library
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "11.0")
        link_libraries(stdc++fs)
    endif()
endif()

# ==========================================
# 2. Build type configuration (Debug / Release / RelWithDebInfo)
# ==========================================
# Default to Release if no build type is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")

# ==========================================
# 3. Compiler Flags
# ==========================================
# Debug mode: Disable optimizations, enable debug information
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0" CACHE STRING "Debug C++ flags" FORCE)
set(CMAKE_C_FLAGS_DEBUG "-g -O0" CACHE STRING "Debug C flags" FORCE)

# Release mode: Maximum optimizations, disable debug information
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "Release C++ flags" FORCE)
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "Release C flags" FORCE)

function(add_executable_copy TARGET_NAME)
    # 1. Set destination directory
    set(DEFAULT_DEST_DIR "${CMAKE_SOURCE_DIR}/../../launch")
    set(DEST_DIR "${DEFAULT_DEST_DIR}")
    if(ARGC GREATER 1)
        set(DEST_DIR "${ARGV1}")
    endif()

    # 2. Convert to absolute path
    get_filename_component(DEST_DIR_ABS "${DEST_DIR}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

    # 3. Check if target exists
    if(NOT TARGET ${TARGET_NAME})
        message(WARNING "Target ${TARGET_NAME} does not exist yet. Make sure add_subdirectory is called before add_executable_copy.")
        return()
    endif()

    # 4. Add copy command
    add_custom_target(copy_${TARGET_NAME} ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory "${DEST_DIR_ABS}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${TARGET_NAME}>"
            "${DEST_DIR_ABS}/"
        DEPENDS ${TARGET_NAME}
        COMMENT "Copying ${TARGET_NAME} to ${DEST_DIR_ABS}"
    )
endfunction()

# ==========================================
# 3. Add subprojects
# ==========================================
add_subdirectory(DvsenseDataToRos)
add_subdirectory(DvsenseFileReaderSample)
add_subdirectory(DvsenseFusionFileReaderSample)
add_subdirectory(DvsenseSyncViewerSample)

# ==========================================
# 4. Copy executables to target directories
# ==========================================
add_executable_copy(dvtorosbag ${CMAKE_CURRENT_SOURCE_DIR}/../tools/)
add_executable_copy(dvsense_file_reader ${CMAKE_CURRENT_SOURCE_DIR}/../tools/)
add_executable_copy(dvsense_fusion_reader ${CMAKE_CURRENT_SOURCE_DIR}/../tools/)
add_executable_copy(dvsense_recorder ${CMAKE_CURRENT_SOURCE_DIR}/../launch/)
