cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)
project(dvsense_fusion_sync_recorder)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -O3")
# set(CMAKE_BUILD_TYPE Release)  # 改为Release模式以启用优化
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG") 

# 添加OpenMP支持检查
find_package(OpenMP QUIET)
find_package(CURL REQUIRED)

# 打印OpenMP状态信息
if(OpenMP_FOUND)
    message(STATUS "OpenMP found: YES")
    message(STATUS "OpenMP CXX Flags: ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP CXX Libraries: ${OpenMP_CXX_LIBRARIES}")
else()
    message(STATUS "OpenMP found: NO")
endif()

add_definitions(-D NOMINMAX)


# 告诉 CMake 优先查找 /usr/local 下的库（避免找到系统高版本）
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

# 查找 fmt v6.2.1（必须 exact，防止找到 v8/v9）
find_package(fmt 6.2.1 EXACT REQUIRED)

# 查找 spdlog（它会自动找到你编译的 v1.8.5 + fmt v6）
find_package(spdlog REQUIRED)

# 查找其他依赖
find_package(DvsenseDriver COMPONENTS Driver REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(dvsense_recorder
    ${CMAKE_CURRENT_SOURCE_DIR}/DVSyncRecorder.cpp
)

# 链接所有依赖（使用 IMPORTED targets，最安全）
target_link_libraries(dvsense_recorder
    PRIVATE
        DvsenseDriver::Driver
        spdlog::spdlog
        fmt::fmt
        ${OpenCV_LIBS}
        CURL::libcurl
)

if(OpenMP_FOUND)
    target_link_libraries(dvsense_recorder PRIVATE OpenMP::OpenMP_CXX)
    target_compile_options(dvsense_recorder PRIVATE ${OpenMP_CXX_FLAGS})
endif()

# 包含 OpenCV 头文件（fmt/spdlog 的头文件由 target 自动传递）
target_include_directories(dvsense_recorder
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${CURL_INCLUDE_DIRS}
        ./
)
