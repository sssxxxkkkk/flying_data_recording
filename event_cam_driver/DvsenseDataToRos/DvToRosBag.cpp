#include <opencv2/opencv.hpp>
#include "DvsenseDriver/camera/FusionCamera.hpp"
#include "DvsenseDriver/FileReader/DvsFileReader.h"
#include <condition_variable>
#include "DvsenseBase/Utils/Json/JsonUtils.hpp"
#include <filesystem> // C++17
#include <regex>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

using namespace std;

// Helper: check if string is all digits (for timestamp filename)
bool isAllDigits(const std::string& s) {
    if (s.empty()) return false;
    for (size_t i = 0; i < s.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(s[i]))) {
            return false;
        }
    }
    return true;
}


// Helper: parse sync_signal.txt -> map cam_time (us) to system_time (us)
std::map<uint64_t, int64_t> loadSyncSignal(const std::string& path) {
    std::map<uint64_t, int64_t> offset_map; // system_time_us -> delta_t (cam - sys) in microseconds
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open sync_signal.txt: %s", path.c_str());
        return offset_map;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        uint64_t cam_ts, sys_ts;
        double delta_t;
        // cam_timestamp system_time delta_t (全部为微秒单位)
        if (!(iss >> cam_ts >> sys_ts >> delta_t)) {
            continue;
        }
        // 计算时间偏差: cam_ts - sys_ts (单位: 微秒)
        int64_t calculated_delta = static_cast<int64_t>(sys_ts - cam_ts);
        offset_map[cam_ts] = calculated_delta; // system timestamp -> camera-system time offset
    }
    return offset_map;
}

// Helper: linear interpolation for time offset
int64_t interpolateOffset(const std::map<uint64_t, int64_t>& offsets, uint64_t sys_ts_ns) {
    if (offsets.empty()) return 0;
    
    // 静态变量用于缓存上次的迭代器位置，以优化连续时间戳的查找
    static auto last_it = offsets.begin();
    static bool first_call = true;
    
    if (first_call) {
        last_it = offsets.begin();
        first_call = false;
    }
    
    // 如果当前时间戳大于等于上次的位置，从当前位置继续搜索
    auto it = last_it;
    if (sys_ts_ns >= last_it->first) {
        // 向前搜索直到找到合适的位置
        while (std::next(it) != offsets.end() && std::next(it)->first <= sys_ts_ns) {
            ++it;
        }
        // 更新缓存位置
        last_it = it;
    } else {
        // 如果时间戳回退了，使用标准查找方法
        it = offsets.upper_bound(sys_ts_ns);
        if (it != offsets.begin()) {
            last_it = std::prev(it);
        } else {
            last_it = it;
        }
    }
    
    if (it == offsets.begin()) {
        return it->second;
    }
    if (it == offsets.end()) {
        return (--it)->second;
    }
    
    auto it_prev = std::prev(it);
    uint64_t t0 = it_prev->first, t1 = it->first;
    int64_t o0 = it_prev->second, o1 = it->second;
    if (t1 == t0) return o0;
    double ratio = double(sys_ts_ns - t0) / double(t1 - t0);
    return static_cast<int64_t>(o0 + ratio * (o1 - o0));
}

// Helper: load IMU data
std::vector<std::tuple<uint64_t, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector4d>> 
loadImuData(const std::string& path) {
    std::vector<std::tuple<uint64_t, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector4d>> imu_data;
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open inertial_data.txt: %s", path.c_str());
        return imu_data;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        // 格式: num,time_stample_sec,time_stample_nsec,angle_rate.x,angle_rate.y,angle_rate.z,accel.x, accel.y, accel.z, quaternion0, quaternion1, quaternion2, quaternion3
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        int num;
        long sec, nsec;
        double wx, wy, wz, ax, ay, az;
        double quat0, quat1, quat2, quat3;  // 四元数数据 (w, x, y, z)
        
        if (!(iss >> num >> sec >> nsec >> wx >> wy >> wz >> ax >> ay >> az >> quat0 >> quat1 >> quat2 >> quat3)) {
            continue;
        }

        uint64_t imu_ts_ns = static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec);

        Eigen::Vector3d gyro(wx, wy, wz);
        Eigen::Vector3d accel(ax, ay, az);
        Eigen::Vector4d quat(quat0, quat1, quat2, quat3);  // w, x, y, z
        imu_data.emplace_back(imu_ts_ns, gyro, accel, quat);
    }

    // Sort by IMU timestamp
    std::sort(imu_data.begin(), imu_data.end(),
        [](const auto& a, const auto& b) {
            return std::get<0>(a) < std::get<0>(b);
        });
    return imu_data;
}

// Helper: uint64_t (us or ns) -> ros::Time
ros::Time toRosTime(uint64_t ts_us_or_ns, bool is_ns = true) {
    if (is_ns) {
        return ros::Time(ts_us_or_ns / 1000000000ULL, ts_us_or_ns % 1000000000ULL);
    } else {
        return ros::Time(ts_us_or_ns / 1000000ULL, (ts_us_or_ns % 1000000ULL) * 1000);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "data_to_rosbag");
    ros::NodeHandle nh;

    // ----------------- 路径配置 -----------------
    std::string base_path = "/media/songxiaokai/E1/svn/flying_data_recording/save_data";
    std::string image_folder = base_path + "/image_data";
    std::string dvs_file_path = base_path + "/event_data/events.raw";
    std::string sync_file = base_path + "/event_data/sync_signal.txt";
    std::string imu_file = base_path + "/inertial_data/inertial_data.txt"; // 修正路径
    std::string output_bag = base_path + "/output.bag";

    // ----------------- 加载同步信号 -----------------
    auto sync_offsets = loadSyncSignal(sync_file);
    if (sync_offsets.empty()) {
        ROS_WARN("No sync signal loaded! IMU timestamps will not be corrected.");
    }

    // ----------------- 打开 ROS bag -----------------
    rosbag::Bag bag;
    bag.open(output_bag, rosbag::bagmode::Write);

    // ----------------- 1. 写入 ALL IMU DATA（系统时间） -----------------
    ROS_INFO("Loading and writing all IMU data...");
    auto imu_data = loadImuData(imu_file);
    for (const auto& [imu_ts_ns, gyro, accel, quat] : imu_data) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time(imu_ts_ns / 1000000000ULL, imu_ts_ns % 1000000000ULL);
        imu_msg.header.frame_id = "imu";

        imu_msg.angular_velocity.x = gyro.x();
        imu_msg.angular_velocity.y = gyro.y();
        imu_msg.angular_velocity.z = gyro.z();

        imu_msg.linear_acceleration.x = accel.x();
        imu_msg.linear_acceleration.y = accel.y();
        imu_msg.linear_acceleration.z = accel.z();

        // 设置姿态四元数 (w, x, y, z)
        imu_msg.orientation.w = quat.x();  // w
        imu_msg.orientation.x = quat.y();  // x
        imu_msg.orientation.y = quat.z();  // y
        imu_msg.orientation.z = quat.w();  // z

        imu_msg.angular_velocity_covariance.fill(0.0);
        imu_msg.linear_acceleration_covariance.fill(0.0);

        imu_msg.orientation_covariance.fill(0.0); 

        bag.write("/imu/", imu_msg.header.stamp, imu_msg);
    }
    ROS_INFO("Wrote %zu IMU messages.", imu_data.size());

    // ----------------- 2. 写入 ALL DVS EVENTS -----------------
    ROS_INFO("Loading DVS file...");
    std::shared_ptr<dvsense::DvsFileReader> dvs_reader = dvsense::DvsFileReader::createFileReader(dvs_file_path);
    if (!dvs_reader->loadFile()) {
        ROS_ERROR("Failed to load DVS file: %s", dvs_file_path.c_str());
        bag.close();
        return -1;
    }

    dvsense::TimeStamp dvs_start, dvs_end;
    dvs_reader->getStartTimeStamp(dvs_start);
    dvs_reader->getEndTimeStamp(dvs_end);
    ROS_INFO("DVS time range: %lu ~ %lu (us)", dvs_start, dvs_end);

    // 假设事件时间戳单位是 **微秒（μs）** —— DVSense 默认
    const bool EVENT_TS_IN_MICROSECONDS = true;

    // 分块读取所有事件（避免内存爆炸）
    const size_t CHUNK_SIZE = 1000000; // 每次读 1M 事件
    dvsense::TimeStamp current_time = dvs_start;
    size_t total_event_count = 0;

    // 全局变量用于优化连续时间戳的查找
    static uint64_t g_last_search_key = 0;
    static std::map<uint64_t, int64_t>::const_iterator g_last_it;

    while (current_time < dvs_end) {
        auto events_chunk = dvs_reader->getNTimeEventsGivenStartTimeStamp(current_time, CHUNK_SIZE);
        if (!events_chunk || events_chunk->empty()) break;

        dvs_msgs::EventArray event_array_msg;
        event_array_msg.header.frame_id = "dvs";
        event_array_msg.width = 1280;
        event_array_msg.height = 720;

        for (const auto& ev : *events_chunk) {
            dvs_msgs::Event e;
            e.x = ev.x;
            e.y = ev.y;
            e.polarity = ev.polarity;
            // 应用时间偏移：将相机时间戳转换为系统时间戳
            uint64_t cam_ts_us = ev.timestamp;
            int64_t offset = interpolateOffset(sync_offsets, cam_ts_us); // 获取系统时间与相机时间的偏移
            uint64_t sys_ts_us = cam_ts_us + offset; // 系统时间 = 相机时间 + 偏移
            
            if (EVENT_TS_IN_MICROSECONDS) {
                e.ts = ros::Time(sys_ts_us / 1000000ULL, (sys_ts_us % 1000000ULL) * 1000);
            } else {
                e.ts = ros::Time(sys_ts_us / 1000000000ULL, sys_ts_us % 1000000000ULL);
            }
            event_array_msg.events.push_back(e);
        }

        // 使用最后一个事件的时间作为 chunk 时间戳（或第一个）
        if (!event_array_msg.events.empty()) {
            event_array_msg.header.stamp = event_array_msg.events.back().ts;
            bag.write("/dvs/events", event_array_msg.header.stamp, event_array_msg);
            total_event_count += event_array_msg.events.size();
        }

        // 更新 current_time：取最后一个事件时间 + 1
        current_time = events_chunk->back().timestamp + 1;
    }
    ROS_INFO("Wrote %zu DVS events in chunks.", total_event_count);

    // ----------------- 3. 写入 ALL IMAGES -----------------
    ROS_INFO("Loading image files...");
    std::vector<cv::String> bmp_files;
    cv::glob(image_folder + "/*.bmp", bmp_files, false);
    std::vector<std::pair<uint64_t, std::string>> timestamped_images;
    for (const auto& filepath : bmp_files) {
        size_t last_slash = filepath.find_last_of("/\\");
        std::string filename = (last_slash == std::string::npos) ? filepath : filepath.substr(last_slash + 1);
        if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".bmp") {
            std::string stem = filename.substr(0, filename.size() - 4);
            if (isAllDigits(stem)) {
                uint64_t cam_img_ts = std::stoull(stem); // 图像的相机时间戳
                int64_t img_offset = interpolateOffset(sync_offsets, cam_img_ts); // 获取时间偏移
                uint64_t sys_img_ts = cam_img_ts + img_offset; // 转换为系统时间戳
                timestamped_images.emplace_back(sys_img_ts, filepath); // 存储系统时间戳
            }
        }
    }
    std::sort(timestamped_images.begin(), timestamped_images.end());

    const bool IMG_TS_IN_NANOSECONDS = false; 

    for (const auto& [img_ts, img_path] : timestamped_images) {
        cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
        if (img.empty()) {
            ROS_WARN_STREAM("Failed to read image: " << img_path);
            continue;
        }

        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(
            std_msgs::Header(), "bgr8", img
        ).toImageMsg();
        img_msg->header.stamp = toRosTime(img_ts, IMG_TS_IN_NANOSECONDS);
        img_msg->header.frame_id = "camera";
        bag.write("/camera/image_raw", img_msg->header.stamp, img_msg);
    }
    ROS_INFO("Wrote %zu images.", timestamped_images.size());

    // ----------------- 完成 -----------------
    bag.close();
    ROS_INFO_STREAM("✅ Full dataset converted! Bag saved as: " << output_bag);
    return 0;
}