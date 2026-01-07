cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 20)

project(dvsense_to_rosbag)

# 1. 查找 ROS 基础包和相关消息包
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    std_msgs
    sensor_msgs
    cv_bridge
)

find_package(DvsenseDriver COMPONENTS Driver REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. 添加包含目录
include_directories( 
   include
   ${OpenCV_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS} # 必须包含 ROS 的头文件路径
)

# 3. 定义执行文件
add_executable(dvtorosbag
    ${CMAKE_CURRENT_SOURCE_DIR}/DvToRosBag.cpp
)

# 4. 链接库
target_link_libraries(dvtorosbag
PRIVATE
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}     # 必须链接 ROS 的库文件
    DvsenseDriver::Driver
)

# 5. 自定义拷贝逻辑（保持不变）
add_custom_target(copy_executable
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:dvtorosbag>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/
    DEPENDS dvtorosbag
)

add_custom_target(default ALL DEPENDS dvtorosbag copy_executable)