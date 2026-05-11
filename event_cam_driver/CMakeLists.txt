cmake_minimum_required(VERSION 3.10)

project(MultiProject)

# ==========================================
# 2. 设置默认编译类型 (Debug / Release / RelWithDebInfo)
# ==========================================
# 如果用户没有指定，默认使用 Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()


message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ==========================================
# 3. 设置全局编译 Flags
# ==========================================
# 针对 Debug 的配置：无优化，生成调试信息
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0" CACHE STRING "Debug C++ flags" FORCE)
set(CMAKE_C_FLAGS_DEBUG "-g -O0" CACHE STRING "Debug C flags" FORCE)

# 针对 Release 的配置：最高优化等级，去除调试信息
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "Release C++ flags" FORCE)
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "Release C flags" FORCE)

function(add_executable_copy TARGET_NAME)
    # 1. 处理参数
    set(DEFAULT_DEST_DIR "${CMAKE_SOURCE_DIR}/../../launch")
    set(DEST_DIR "${DEFAULT_DEST_DIR}")
    if(ARGC GREATER 1)
        set(DEST_DIR "${ARGV1}")
    endif()

    # 2. 转换为绝对路径
    get_filename_component(DEST_DIR_ABS "${DEST_DIR}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

    # 3. 检查 Target 是否存在（防止子目录还没加载就调用）
    if(NOT TARGET ${TARGET_NAME})
        message(WARNING "Target ${TARGET_NAME} does not exist yet. Make sure add_subdirectory is called before add_executable_copy.")
        return()
    endif()

    # 4. 核心逻辑：创建拷贝命令
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
# 3. 添加子目录
# ==========================================
add_subdirectory(DvsenseDataToRos)
add_subdirectory(DvsenseFileReaderSample)
add_subdirectory(DvsenseFusionFileReaderSample)
add_subdirectory(DvsenseSyncViewerSample)

# ==========================================
# 4. 调用拷贝函数
# ==========================================
add_executable_copy(dvtorosbag ${CMAKE_CURRENT_SOURCE_DIR}/../tools/)
add_executable_copy(dvsense_file_reader ${CMAKE_CURRENT_SOURCE_DIR}/../tools/)
add_executable_copy(dvsense_fusion_reader ${CMAKE_CURRENT_SOURCE_DIR}/../tools/)
add_executable_copy(dvsense_recorder ${CMAKE_CURRENT_SOURCE_DIR}/../launch/)
