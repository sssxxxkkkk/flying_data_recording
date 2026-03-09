cmake_minimum_required(VERSION 3.10)
project(MultiProject)


function(add_executable_copy TARGET_NAME)
    # 默认目标目录：项目根目录下的 tools
    set(DEFAULT_DEST_DIR "${CMAKE_SOURCE_DIR}/../../launch")
    
    # 如果提供了第二个参数，则使用用户指定的目录
    set(DEST_DIR "${DEFAULT_DEST_DIR}")
    if(ARGC GREATER 1)
        set(DEST_DIR "${ARGV1}")
    endif()
    
    # 将目标目录转换为绝对路径（相对于当前源码目录）
    get_filename_component(DEST_DIR_ABS "${DEST_DIR}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    
    # 检查目标是否存在
    if(NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "Target ${TARGET_NAME} does not exist.")
    endif()
    
    # 创建自定义目标：先确保目录存在，再复制文件
    add_custom_target(copy_${TARGET_NAME}
        COMMAND ${CMAKE_COMMAND} -E make_directory "${DEST_DIR_ABS}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${TARGET_NAME}>"
            "${DEST_DIR_ABS}/"   # 末尾加 / 确保复制到目录内
        DEPENDS ${TARGET_NAME}
        COMMENT "Copying ${TARGET_NAME} to ${DEST_DIR_ABS}"
    )
    
    # 全局依赖目标（ALL 确保默认构建时执行）
    if(NOT TARGET copy_all)
        add_custom_target(copy_all ALL)
    endif()
    add_dependencies(copy_all copy_${TARGET_NAME})
endfunction()

# 添加子目录，每个子目录包含独立的 CMakeLists.txt
add_subdirectory(DvsenseDataToRos)
add_subdirectory(DvsenseFileReaderSample)
add_subdirectory(DvsenseFusionFileReaderSample)
add_subdirectory(DvsenseSyncViewerSample)
