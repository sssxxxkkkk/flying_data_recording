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
std::map<uint64_t, int64_t> loadSyncSignal(const std::string& path, int64_t &out_avg_offset, double &out_std_dev) {
    std::map<uint64_t, int64_t> offset_map; 
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open sync_signal.txt: %s", path.c_str());
        return offset_map;
    }

    std::vector<int64_t> all_offsets; // 用于统计
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        uint64_t cam_ts, sys_ts;
        double delta_t;
        if (!(iss >> cam_ts >> sys_ts >> delta_t)) {
            continue;
        }
        int64_t calculated_delta = static_cast<int64_t>(delta_t);
        offset_map[cam_ts] = calculated_delta;
        all_offsets.push_back(calculated_delta);
    }

    // ========== 改进 1/2: 计算均值和标准差 ==========
    if (!all_offsets.empty()) {
        // 1. 计算均值
        int64_t sum = 0;
        for (auto v : all_offsets) sum += v;
        out_avg_offset = sum / static_cast<int64_t>(all_offsets.size());

        // 2. 计算标准差
        double sum_sq = 0.0;
        for (auto v : all_offsets) {
            double diff = static_cast<double>(v - out_avg_offset);
            sum_sq += diff * diff;
        }
        out_std_dev = std::sqrt(sum_sq / static_cast<double>(all_offsets.size()));
        
        ROS_INFO("========================================");
        ROS_INFO("Sync Signal Statistics:");
        ROS_INFO("  Count:     %zu", all_offsets.size());
        ROS_INFO("  Mean:      %ld us", out_avg_offset);
        ROS_INFO("  Std Dev:   %.2f us", out_std_dev);
        ROS_INFO("========================================");
    } else {
        out_avg_offset = 0;
        out_std_dev = 0;
    }

    return offset_map;
}

// Helper: linear interpolation (保持原样，或者这里我们直接用均值简化，视需求而定)
// 为了代码简洁，这里我们主要使用 avg_offset，如果需要插值可以再启用

// Helper: load IMU data
using ImuTuple = std::tuple<uint64_t, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector4d>;
std::vector<ImuTuple> loadImuData(const std::string& path) {
    std::vector<ImuTuple> imu_data;
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open inertial_data.txt: %s", path.c_str());
        return imu_data;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        int num;
        long sec, nsec;
        double wx, wy, wz, ax, ay, az;
        double quat0, quat1, quat2, quat3; 
        
        if (!(iss >> num >> sec >> nsec >> wx >> wy >> wz >> ax >> ay >> az >> quat0 >> quat1 >> quat2 >> quat3)) {
            continue;
        }

        uint64_t imu_ts_ns = static_cast<uint64_t>(sec) * 1000000000ULL + static_cast<uint64_t>(nsec);
        Eigen::Vector3d gyro(wx, wy, wz);
        Eigen::Vector3d accel(ax, ay, az);
        Eigen::Vector4d quat(quat0, quat1, quat2, quat3);
        imu_data.emplace_back(imu_ts_ns, gyro, accel, quat);
    }

    std::sort(imu_data.begin(), imu_data.end(),
        [](const auto& a, const auto& b) {
            return std::get<0>(a) < std::get<0>(b);
        });
    return imu_data;
}

// Helper: uint64_t -> ros::Time
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
    std::string base_path = "../save_data";
    std::string image_folder = base_path + "/image_data";
    std::string dvs_file_path = base_path + "/event_data/events.raw";
    std::string sync_file = base_path + "/event_data/sync_signal.txt";
    std::string imu_file = base_path + "/inertial_data/inertial_data.txt";
    std::string output_bag = base_path + "/output.bag";

    // ----------------- 1. 加载同步信号 & 统计 -----------------
    int64_t avg_offset = 0;
    double std_dev_offset = 0.0;
    auto sync_offsets = loadSyncSignal(sync_file, avg_offset, std_dev_offset);

    if (sync_offsets.empty()) {
        ROS_WARN("No sync signal loaded! Using offset = 0.");
    }

    // ----------------- 2. 预加载所有数据索引，确定时间范围 -----------------
    ROS_INFO("Loading data indices for time range calculation...");

    // 2.1 IMU 时间范围
    auto imu_data = loadImuData(imu_file);
    uint64_t imu_t_min_ns = 0, imu_t_max_ns = 0;
    if (!imu_data.empty()) {
        imu_t_min_ns = std::get<0>(imu_data.front());
        imu_t_max_ns = std::get<0>(imu_data.back());
    }

    // 2.2 图像 时间范围
    std::vector<std::pair<uint64_t, std::string>> timestamped_images;
    uint64_t img_t_min_ns = 0, img_t_max_ns = 0;
    {
        std::vector<cv::String> bmp_files;
        cv::glob(image_folder + "/*.bmp", bmp_files, false);
        for (const auto& filepath : bmp_files) {
            size_t last_slash = filepath.find_last_of("/\\");
            std::string filename = (last_slash == std::string::npos) ? filepath : filepath.substr(last_slash + 1);
            if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".bmp") {
                std::string stem = filename.substr(0, filename.size() - 4);
                if (isAllDigits(stem)) {
                    uint64_t cam_img_ts_us = std::stoull(stem);
                    uint64_t sys_img_ts_ns = (cam_img_ts_us + avg_offset) * 1000ULL; // 统一转为纳秒
                    timestamped_images.emplace_back(sys_img_ts_ns, filepath);
                }
            }
        }
        std::sort(timestamped_images.begin(), timestamped_images.end());
        if (!timestamped_images.empty()) {
            img_t_min_ns = timestamped_images.front().first;
            img_t_max_ns = timestamped_images.back().first;
        }
    }

    // 2.3 DVS 时间范围
    uint64_t dvs_t_min_ns = 0, dvs_t_max_ns = 0;
    dvsense::TimeStamp dvs_start_us = 0, dvs_end_us = 0;
    std::shared_ptr<dvsense::DvsFileReader> dvs_reader = nullptr;
    
    // 先尝试加载DVS以获取时间范围
    dvs_reader = dvsense::DvsFileReader::createFileReader(dvs_file_path);
    if (dvs_reader && dvs_reader->loadFile()) {
        dvs_reader->getStartTimeStamp(dvs_start_us);
        dvs_reader->getEndTimeStamp(dvs_end_us);
        // 转换为系统时间纳秒
        dvs_t_min_ns = (dvs_start_us + avg_offset) * 1000ULL;
        dvs_t_max_ns = (dvs_end_us + avg_offset) * 1000ULL;
    }

    // ========== 改进 2/2: 计算时间交集 ==========
    // 逻辑：取所有传感器最大的起始时间 和 最小的结束时间
    std::vector<uint64_t> all_starts;
    std::vector<uint64_t> all_ends;

    if (!imu_data.empty()) { all_starts.push_back(imu_t_min_ns); all_ends.push_back(imu_t_max_ns); }
    if (!timestamped_images.empty()) { all_starts.push_back(img_t_min_ns); all_ends.push_back(img_t_max_ns); }
    if (dvs_reader) { all_starts.push_back(dvs_t_min_ns); all_ends.push_back(dvs_t_max_ns); }

    if (all_starts.empty() || all_ends.empty()) {
        ROS_ERROR("No valid data found to compute intersection!");
        return -1;
    }

    uint64_t global_start_ns = *std::max_element(all_starts.begin(), all_starts.end());
    uint64_t global_end_ns = *std::min_element(all_ends.begin(), all_ends.end());

    ROS_INFO("========================================");
    ROS_INFO("Data Time Ranges (System time, ns):");
    if (!imu_data.empty()) ROS_INFO("  IMU:   [%lu, %lu]", imu_t_min_ns, imu_t_max_ns);
    if (!timestamped_images.empty()) ROS_INFO("  Img:   [%lu, %lu]", img_t_min_ns, img_t_max_ns);
    if (dvs_reader) ROS_INFO("  DVS:   [%lu, %lu]", dvs_t_min_ns, dvs_t_max_ns);
    ROS_INFO("----------------------------------------");
    ROS_INFO("Final Intersection: [%lu, %lu]", global_start_ns, global_end_ns);
    ROS_INFO("Duration: %.2f s", (global_end_ns - global_start_ns) / 1e9);
    ROS_INFO("========================================");

    if (global_start_ns >= global_end_ns) {
        ROS_FATAL("Time intersection is empty! Data cannot be aligned.");
        return -1;
    }

    // ----------------- 3. 打开 ROS bag 并开始写入 -----------------
    rosbag::Bag bag;
    bag.open(output_bag, rosbag::bagmode::Write);

    // 3.1 写入 IMU (过滤)
    size_t imu_written = 0;
    if (!imu_data.empty()) {
        ROS_INFO("Writing filtered IMU data...");
        for (const auto& [imu_ts_ns, gyro, accel, quat] : imu_data) {
            // 时间过滤
            if (imu_ts_ns < global_start_ns || imu_ts_ns > global_end_ns) continue;

            sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = toRosTime(imu_ts_ns, true);
            imu_msg.header.frame_id = "imu";

            imu_msg.angular_velocity.x = gyro.x();
            imu_msg.angular_velocity.y = gyro.y();
            imu_msg.angular_velocity.z = gyro.z();
            imu_msg.linear_acceleration.x = accel.x();
            imu_msg.linear_acceleration.y = accel.y();
            imu_msg.linear_acceleration.z = accel.z();

            // 注意：这里之前的代码似乎把 quat 的顺序弄反了 (w,x,y,z vs x,y,z,w)
            // 这里保持你原来的逻辑，但建议确认一下
            imu_msg.orientation.w = quat.x(); 
            imu_msg.orientation.x = quat.y();
            imu_msg.orientation.y = quat.z();
            imu_msg.orientation.z = quat.w();

            imu_msg.angular_velocity_covariance.fill(0.0);
            imu_msg.linear_acceleration_covariance.fill(0.0);
            imu_msg.orientation_covariance.fill(0.0); 

            bag.write("/imu", imu_msg.header.stamp, imu_msg);
            imu_written++;
        }
        ROS_INFO("IMU: Wrote %zu (Filtered out %zu)", imu_written, imu_data.size() - imu_written);
    }

    // 3.2 写入 DVS Events (过滤)
    if (dvs_reader) {
        ROS_INFO("Writing filtered DVS events...");
        const std::string EVENT_TOPIC = "/dvs/events";
        const size_t time_step = 50000; // 50 ms
        
        size_t total_event_count = 0;
        dvsense::TimeStamp current_time = dvs_start_us;

        while (current_time < dvs_end_us) {
            auto events_chunk = dvs_reader->getNTimeEventsGivenStartTimeStamp(current_time, time_step);
            if (!events_chunk || events_chunk->empty()) break;

            dvs_msgs::EventArray event_array_msg;
            event_array_msg.header.frame_id = "dvs";
            event_array_msg.width = 1280;
            event_array_msg.height = 720;
            event_array_msg.events.reserve(events_chunk->size());

            for (const auto& ev : *events_chunk) {
                // 计算系统时间戳 (ns)
                uint64_t cam_ts_us = ev.timestamp;
                uint64_t sys_ts_ns = (cam_ts_us + avg_offset) * 1000ULL;

                // 时间过滤
                if (sys_ts_ns < global_start_ns || sys_ts_ns > global_end_ns) continue;

                dvs_msgs::Event e;
                e.x = event_array_msg.width - ev.x;
                e.y = ev.y;
                e.polarity = ev.polarity == 1 ? false : true;
                e.ts = toRosTime(sys_ts_ns, true);
                
                event_array_msg.events.push_back(e);
            }

            if (!event_array_msg.events.empty()) {
                event_array_msg.header.stamp = event_array_msg.events.front().ts;
                bag.write(EVENT_TOPIC, event_array_msg.header.stamp, event_array_msg);
                total_event_count += event_array_msg.events.size();
            }

            dvsense::TimeStamp last_ts_in_chunk = events_chunk->back().timestamp;
            current_time = last_ts_in_chunk + 1;
        }
        ROS_INFO("DVS: Wrote %zu events in intersection.", total_event_count);
    }

    // 3.3 写入 Images (过滤)
    size_t img_written = 0;
    if (!timestamped_images.empty()) {
        ROS_INFO("Writing filtered images...");
        for (const auto& [img_ts_ns, img_path] : timestamped_images) {
            // 时间过滤
            if (img_ts_ns < global_start_ns || img_ts_ns > global_end_ns) continue;

            cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
            if (img.empty()) continue;

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(
                std_msgs::Header(), "bgr8", img
            ).toImageMsg();
            img_msg->header.stamp = toRosTime(img_ts_ns, true);
            img_msg->header.frame_id = "camera";
            bag.write("/camera/image_raw", img_msg->header.stamp, img_msg);
            img_written++;
        }
        ROS_INFO("Images: Wrote %zu (Filtered out %zu)", img_written, timestamped_images.size() - img_written);
    }

    // ----------------- 完成 -----------------
    bag.close();
    ROS_INFO_STREAM("✅ All Done! Bag saved to: " << output_bag);
    return 0;
}

