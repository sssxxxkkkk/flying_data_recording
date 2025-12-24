#ifndef DVSYNC_RECORDER_HPP
#define DVSYNC_RECORDER_HPP

#include <condition_variable>
#include "DvsenseDriver/camera/FusionCamera.hpp"
#include "DvsenseDriver/camera/DvsCameraManager.hpp"
#include <opencv2/opencv.hpp>
#include "DvsenseBase/logging/logger.hh"
#include "DvsenseDriver/DataProcess/DvsApsFusionProccessor.hpp"
#include "DvsenseDriver/Calibration/Calibrator.hpp"
#include "DvsenseBase/Utils/Json/JsonUtils.hpp"
#include <thread>
#include <chrono>
#include <fstream>
#include <queue>
#include <filesystem>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class VideoSender {
public:
    // 构造函数：初始化UDP连接
    VideoSender(const std::string& dest_ip, int dest_port, int jpeg_quality = 85)
        : dest_ip_(dest_ip), dest_port_(dest_port), jpeg_quality_(jpeg_quality) {
        
        // 创建UDP Socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            throw std::runtime_error("Failed to create socket");
        }

        // 配置目标地址
        memset(&servaddr_, 0, sizeof(servaddr_));
        servaddr_.sin_family = AF_INET;
        servaddr_.sin_port = htons(dest_port_);
        if (inet_pton(AF_INET, dest_ip_.c_str(), &servaddr_.sin_addr) <= 0) {
            throw std::runtime_error("Invalid destination IP");
        }

        std::cout << "VideoSender initialized: " << dest_ip_ << ":" << dest_port_ 
                  << " (JPEG quality: " << jpeg_quality_ << ")" << std::endl;
    }

    // 发送图像
    void sendFrame(const cv::Mat& frame) {
        // 1. 检查输入图像是否为空
        if (frame.empty()) {
            std::cerr << "Error: Empty frame (size=" << frame.cols << "x" << frame.rows << ")" << std::endl;
            return;
        }
    
        // 2. 确保尺寸为偶数（避免缩放错误）
        int new_width = (frame.cols / 2) & ~1; // 保证偶数
        int new_height = (frame.rows / 2) & ~1;
        
        cv::Mat resized_frame;
        if (new_width < 1 || new_height < 1) {
            resized_frame = frame; // 退化到原图
        } else {
            cv::resize(frame, resized_frame, cv::Size(new_width, new_height), 
                      0, 0, cv::INTER_LINEAR);
        }
    
        // 3. 压缩为JPEG（关键：避免每次创建vector）
        buffer_.clear();
        cv::imencode(".jpg", resized_frame, buffer_, 
                     {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});
    
        // 4. 添加发送数据的代码
        struct timeval timeout;
        timeout.tv_sec = 0.3;
        timeout.tv_usec = 0;
        setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    
        // 4. 添加发送数据的代码
        int bytes_sent = sendto(sockfd_, buffer_.data(), buffer_.size(), 0, 
                              (struct sockaddr*)&servaddr_, sizeof(servaddr_));
        
        // 5. 添加发送状态日志
        if (bytes_sent < 0) {
            std::cerr << "UDP send failed (errno=" << errno << "): " << strerror(errno) 
                      << " | Frame size: " << resized_frame.cols << "x" << resized_frame.rows 
                      << " | JPEG quality: " << jpeg_quality_ << std::endl;
        } else {
            std::cout << "Sent frame: " << resized_frame.cols << "x" << resized_frame.rows 
                      << " (JPG quality: " << jpeg_quality_ << ") | Size: " << buffer_.size() << " bytes" << std::endl;
        }
    }


    // 析构函数：关闭Socket
    ~VideoSender() {
        close(sockfd_);
    }

private:
    int sockfd_;
    struct sockaddr_in servaddr_;
    std::string dest_ip_;
    int dest_port_;
    int jpeg_quality_;
    std::vector<uchar> buffer_; // 重用缓冲区，避免频繁内存分配
};


class DvsenseRecorder
{
private:
    std::queue<dvsense::ApsFrame> frame_queue_;
    std::queue<dvsense::EventTriggerIn> trigger_queue_;  // 新增触发信号队列
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    bool stop_worker_ = false;
    bool is_calibrated_ = false;
    cv::Mat map_x_, map_y_;
    
    //初始化远程发送服务
    std::string dest_ip = "192.168.10.1";
    int dest_port = 5000;
    VideoSender v_sender;
    
    // 同步文件流
    std::ofstream sync_file_stream_;

    void process_data() {
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { 
                return !frame_queue_.empty() || !trigger_queue_.empty() || stop_worker_; 
            });
            
            // 检查是否需要退出
            if (stop_worker_) {
                // 处理剩余的触发信号数据
                while (!trigger_queue_.empty()) {
                    std::cout << "处理剩余同步信号数据..." << std::endl;
                    auto trigger_in = std::move(trigger_queue_.front());
                    trigger_queue_.pop();
                    lock.unlock();
                    
                    // 记录同步信号到文件
                    if (trigger_in.polarity == 0 && sync_file_stream_.is_open()) {
                        uint64_t cam_timestamp = trigger_in.timestamp;
                        auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                        delta_t = (double)(system_time - cam_timestamp);
                        
                        // 写入同步信号信息：相机时间戳 系统时间戳 差值
                        sync_file_stream_ << cam_timestamp << " " << system_time << " " << delta_t << std::endl;
                        sync_file_stream_.flush();
                    }
                    
                    lock.lock();
                }
                
                // 处理剩余的图像数据
                while (!frame_queue_.empty()) {
                    std::cout << "处理剩余图像数据..." << std::endl;
                    auto frame = std::move(frame_queue_.front());
                    frame_queue_.pop();
                    lock.unlock();
                    
                    // 保存图像到文件
                    uint64_t exposure_timestamp = frame.exposure_start_timestamp;
                    std::string image_filename = file_path_images_ + "/" + std::to_string(exposure_timestamp) + ".bmp";
                    
                    cv::Mat image(frame.height(), frame.width(), CV_8UC3, (void*)frame.data());
                    cv::Mat image_bgr;
    
                    if (is_calibrated_) {
                        cv::Mat image_rgb;
                        cv::remap(image, image_rgb, map_x_, map_y_, cv::INTER_LINEAR);
                        cv::cvtColor(image_rgb, image_bgr, cv::COLOR_RGB2BGR);
                        cv::imwrite(image_filename, image_bgr);
                        v_sender.sendFrame(image_bgr);  
                    } else {
                        cv::cvtColor(image, image_bgr, cv::COLOR_RGB2BGR);
                        cv::imwrite(image_filename, image_bgr);
                        v_sender.sendFrame(image);  
                    }

                    lock.lock();
                    }

                // 关闭同步信号文件流
                if (sync_file_stream_.is_open()) {
                    sync_file_stream_.close();
                }
                
                break;
            }
            
            // 处理触发信号数据
            int triggers_processed = 0;
            const int max_triggers_per_batch = 10;
            
            while (!trigger_queue_.empty() && triggers_processed < max_triggers_per_batch) {
                std::cout << "处理同步信号数据..." << std::endl;
                auto trigger_in = std::move(trigger_queue_.front());
                trigger_queue_.pop();
                triggers_processed++;
                lock.unlock();
                
                // 记录同步信号到文件
                if (trigger_in.polarity == 0 && sync_file_stream_.is_open()) {
                    uint64_t cam_timestamp = trigger_in.timestamp;
                    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                    delta_t = (double)(system_time - cam_timestamp);
                    
                    // 写入同步信号信息：相机时间戳 系统时间戳 差值
                    sync_file_stream_ << cam_timestamp << " " << system_time << " " << delta_t << std::endl;
                }
                
                lock.lock();
            }
            
            // 处理图像数据（保持原有逻辑）
            int images_processed = 0;
            const int max_images_per_batch = 5;
            
            while (!frame_queue_.empty() && images_processed < max_images_per_batch) {
                std::cout << "处理图像数据..." << std::endl;
                auto frame = std::move(frame_queue_.front());
                frame_queue_.pop();
                images_processed++;
                lock.unlock();
                
                // 保存图像到文件
                uint64_t exposure_timestamp = frame.exposure_start_timestamp;
                std::string image_filename = file_path_images_ + "/" + std::to_string(exposure_timestamp) + ".bmp";
                
                cv::Mat image(frame.height(), frame.width(), CV_8UC3, (void*)frame.data());
                cv::Mat image_bgr;

                if (is_calibrated_) {
                    cv::Mat image_rgb;
                    cv::remap(image, image_rgb, map_x_, map_y_, cv::INTER_LINEAR);
                    cv::cvtColor(image_rgb, image_bgr, cv::COLOR_RGB2BGR);
                    cv::imwrite(image_filename, image_bgr);
                    v_sender.sendFrame(image_bgr);  
                } else {
                    cv::cvtColor(image, image_bgr, cv::COLOR_RGB2BGR);
                    cv::imwrite(image_filename, image_bgr);
                    v_sender.sendFrame(image);  
                }
                                              
                lock.lock();
            }
        }
        
        std::cout << "工作线程退出" << std::endl;
    }

public:
    double delta_t = 0;      // 相机时间 + delta_t = 系统时间
    std::string file_path_;
    std::string file_path_images_;
    std::string file_path_events_;
    std::string sync_filename_;
    
    DvsenseRecorder(std::string file_path, std::string& dest_ip, int dest_port ): dest_ip(dest_ip), dest_port(dest_port), 
      v_sender(dest_ip, dest_port)
    {
        file_path_ = file_path;
        file_path_images_ = file_path + "/image_data";
        file_path_events_ = file_path + "/event_data";
        sync_filename_ = file_path_events_ + "/sync_signal.txt";

        //1. 如果文件夹不存在，则创建
        if (!std::filesystem::exists(file_path_))
        {
            std::filesystem::create_directory(file_path_);
        }

        if (!std::filesystem::exists(file_path_events_))
        {
            std::filesystem::create_directory(file_path_events_);
        }

        if (!std::filesystem::exists(file_path_images_))
        {
            std::filesystem::create_directory(file_path_images_);
        }
        else 
        {
            // 如果文件夹存在，清空文件夹中的所有内容
            for (const auto& entry : std::filesystem::directory_iterator(file_path_images_)) 
            {
                std::filesystem::remove_all(entry.path());
            }
        }
        
        if (std::filesystem::exists(sync_filename_)) {
            std::filesystem::remove(sync_filename_);
        }
        
        // 创建并打开新的同步信号文件
        sync_file_stream_.open(sync_filename_, std::ios::out | std::ios::app);
        if(sync_file_stream_.is_open())
        {
             sync_file_stream_ << "cam_timestamp" << " " << "system_time" << " " << "delta_t" << std::endl;
        }

        // 启动工作线程
        worker_thread_ = std::thread(&DvsenseRecorder::process_data, this);
    }
    
    // 析构函数，清理资源
    ~DvsenseRecorder() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_worker_ = true;
        }
        queue_cv_.notify_all();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }
    

    // 保存图像
    void save_images(const dvsense::ApsFrame frame)
    {
        // 图像名称为 file_path_images_ + "/" + frame.exposure_end_timestamp + delta_t + ".png"

        // 寻找高效的保存方式
        // 将图像帧放入队列，由工作线程处理
        std::lock_guard<std::mutex> lock(queue_mutex_);
        frame_queue_.push(frame);
        queue_cv_.notify_one();
    }
   
    // 更新delta_t并记录同步信号
    void update_delta_t(const dvsense::EventTriggerIn &trigger_in)
    {
        // 将触发信号放入队列，由工作线程处理
        std::lock_guard<std::mutex> lock(queue_mutex_);
        trigger_queue_.push(trigger_in);
        queue_cv_.notify_one();
    }

    void compute_remap(dvsense::CalibratorParameters cali_param)
	{
	   is_calibrated_ = true;
	   int dvs_rows = cali_param.dvs_rows;
	   int dvs_cols = cali_param.dvs_cols;
	   int aps_rows = cali_param.aps_rows;
	   int aps_cols = cali_param.aps_cols;
       std::vector<double> dvs_to_aps_affine_matrix = cali_param.affine_matrix["1"].data;
	   float offset_x = 0.f;
       float offset_y = 0.f;

	   //
	   map_x_.create(dvs_rows, dvs_cols, CV_32FC1); 
	   map_y_.create(dvs_rows, dvs_cols, CV_32FC1);

	   //
       for(int y = 0; y < dvs_rows; ++y)
        {
            for(int x = 0; x < dvs_cols; ++x)
            {
                // Apply affine transformation to map APS pixels to DVS events
                float aps_x = static_cast<float>(x * dvs_to_aps_affine_matrix[0] + y * dvs_to_aps_affine_matrix[1] + dvs_to_aps_affine_matrix[2] + offset_x);
                float aps_y = static_cast<float>(x * dvs_to_aps_affine_matrix[3] + y * dvs_to_aps_affine_matrix[4] + dvs_to_aps_affine_matrix[5] + offset_y);
                if (aps_x < 0 || aps_x >= cali_param.aps_cols || aps_y < 0 || aps_y >= cali_param.aps_rows)
                {
                    continue;
                }

				map_x_.at<float>(y, x) = aps_x;
				map_y_.at<float>(y, x) = aps_y;
            }
        }
	}
   

};

#endif // DVSYNC_RECORDER_HPP
