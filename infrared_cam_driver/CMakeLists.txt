# 指定 CMake 的最低版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称
project(save_infrared)

# 设置 C++ 标准为 C++17 (支持 chrono, thread 等)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. 查找 OpenCV 库
# 自动寻找系统中安装的 OpenCV，要求必须找到 (REQUIRED)
find_package(OpenCV REQUIRED)

# 2. 查找线程库 (用于多线程异步存图)
find_package(Threads REQUIRED)

# 打印 OpenCV 找到的信息（可选，用于调试）
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# 3. 指定头文件包含目录
include_directories(${OpenCV_INCLUDE_DIRS})

# 4. 生成可执行文件
# 假设你的源代码文件名为 main.cpp
add_executable(infrared_driver save_infrared_images.cpp)

# 5. 链接库文件
# 链接 OpenCV 库和线程库
target_link_libraries(infrared_driver 
    ${OpenCV_LIBS} 
    Threads::Threads
)

# (可选) 如果你是在 Linux 下编译，有时需要显式链接数学库
target_link_libraries(infrared_driver m)

add_custom_target(copy_executable
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:infrared_driver>
    ${CMAKE_CURRENT_SOURCE_DIR}/../launch/
    DEPENDS infrared_driver
)

add_custom_target(default ALL DEPENDS infrared_driver copy_executable)
