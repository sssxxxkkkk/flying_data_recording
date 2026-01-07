#include <opencv2/opencv.hpp>
#include "DvsenseDriver/camera/FusionCamera.hpp"
#include "DvsenseDriver/FileReader/DvsFileReader.h"
#include <condition_variable>
#include"DvsenseBase/Utils/Json/JsonUtils.hpp"
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


// Helper: parse sync_signal.txt -> map system_time (ns) to cam_time (ns)
std::map<uint64_t, int64_t> loadSyncSignal(const std::string& path) {
    std::map<uint64_t, int64_t> offset_map; // system_time_ns -> delta_t (cam - sys)
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open sync_signal.txt: %s", path.c_str());
        return offset_map;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        uint64_t cam_ts, sys_sec, sys_nsec;
        int64_t delta_t;
        char comma;
        // 格式: cam_timestamp system_time delta_t
        // 注意：system_time 可能是 sec.nsec 或直接 ns，根据你的实际格式调整
        // 假设 sync_signal.txt 是：cam_ts sys_sec sys_nsec delta_t
        if (!(iss >> cam_ts >> sys_sec >> sys_nsec >> delta_t)) {
            continue;
        }
        uint64_t sys_ts_ns = sys_sec * 1000000000ULL + sys_nsec;
        offset_map[sys_ts_ns] = delta_t; // cam_ts - sys_ts = delta_t
    }
    return offset_map;
}

// Helper: linear interpolation for time offset
int64_t interpolateOffset(const std::map<uint64_t, int64_t>& offsets, uint64_t sys_ts_ns) {
    if (offsets.empty()) return 0;
    auto it = offsets.upper_bound(sys_ts_ns);
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

// Helper: load IMU data and convert to camera time
std::vector<std::tuple<uint64_t, Eigen::Vector3d, Eigen::Vector3d>> 
loadImuData(const std::string& path, const std::map<uint64_t, int64_t>& sync_offsets) {
    std::vector<std::tuple<uint64_t, Eigen::Vector3d, Eigen::Vector3d>> imu_data;
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open inertial_data.txt: %s", path.c_str());
        return imu_data;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        // 格式: num,sec,nsec,wx,wy,wz,ax,ay,az,
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        int num;
        long sec; long nsec;
        double wx, wy, wz, ax, ay, az;
        if (!(iss >> num >> sec >> nsec >> wx >> wy >> wz >> ax >> ay >> az)) {
            continue;
        }

        uint64_t sys_ts_ns = static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec);
        int64_t offset = interpolateOffset(sync_offsets, sys_ts_ns);
        uint64_t cam_ts_ns = sys_ts_ns + offset;

        Eigen::Vector3d gyro(wx, wy, wz);
        Eigen::Vector3d accel(ax, ay, az);
        imu_data.emplace_back(cam_ts_ns, gyro, accel);
    }

    // Sort by camera timestamp
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
    std::string image_folder = "../save_data/image_data";
    std::string dvs_file_path = "../save_data/event_data/events.raw";
    std::string sync_file = "../save_data/event_data/sync_signal.txt";
    std::string imu_file = "../save_data/inertial_data/inertial_data.txt"; // 修正路径
    std::string output_bag = "output.bag";

    // ----------------- 加载同步信号 -----------------
    auto sync_offsets = loadSyncSignal(sync_file);
    if (sync_offsets.empty()) {
        ROS_WARN("No sync signal loaded! IMU timestamps will not be corrected.");
    }

    // ----------------- 打开 ROS bag -----------------
    rosbag::Bag bag;
    bag.open(output_bag, rosbag::bagmode::Write);

    // ----------------- 1. 写入 ALL IMU DATA（转为相机时间） -----------------
    ROS_INFO("Loading and writing all IMU data...");
    auto imu_data = loadImuData(imu_file, sync_offsets);
    for (const auto& [cam_ts_ns, gyro, accel] : imu_data) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time(cam_ts_ns / 1000000000ULL, cam_ts_ns % 1000000000ULL);
        imu_msg.header.frame_id = "imu";

        imu_msg.angular_velocity.x = gyro.x();
        imu_msg.angular_velocity.y = gyro.y();
        imu_msg.angular_velocity.z = gyro.z();

        imu_msg.linear_acceleration.x = accel.x();
        imu_msg.linear_acceleration.y = accel.y();
        imu_msg.linear_acceleration.z = accel.z();

        imu_msg.angular_velocity_covariance.fill(0.0);
        imu_msg.linear_acceleration_covariance.fill(0.0);

        imu_msg.orientation_covariance.fill(0.0); 
        imu_msg.orientation_covariance[0] = -1.0; 

        bag.write("/imu/data", imu_msg.header.stamp, imu_msg);
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
            // 事件时间戳已经是相机域，直接转换为 ros::Time
            if (EVENT_TS_IN_MICROSECONDS) {
                e.ts = ros::Time(ev.timestamp / 1000000ULL, (ev.timestamp % 1000000ULL) * 1000);
            } else {
                e.ts = ros::Time(ev.timestamp / 1000000000ULL, ev.timestamp % 1000000000ULL);
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
                uint64_t ts = std::stoull(stem);
                timestamped_images.emplace_back(ts, filepath);
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