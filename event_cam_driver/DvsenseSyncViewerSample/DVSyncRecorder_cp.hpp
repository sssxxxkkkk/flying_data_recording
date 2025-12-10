#ifndef DVSYNC_RECORDER_HPP


#include <condition_variable>
#include "DvsenseDriver/camera/FusionCamera.hpp"
#include <curl/curl.h>
#include <sstream>
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


// ... existing code ...
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
                        double delta_t = (double)(system_time - cam_timestamp);
                        
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
                    uint64_t adjusted_timestamp = frame.exposure_start_timestamp + (uint64_t)delta_t;
                    std::string image_filename = file_path_images_ + "/" + std::to_string(adjusted_timestamp) + ".png";
                    
                    cv::Mat image(frame.height(), frame.width(), CV_8UC3, (void*)frame.data());
                    cv::Mat corrected_image;

                    if (is_calibrated_) {
                        cv::remap(image, corrected_image, map_x_, map_y_, cv::INTER_LINEAR);
                        cv::imwrite(image_filename, corrected_image);
                    } else {
                        cv::imwrite(image_filename, image);
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
                    double delta_t = (double)(system_time - cam_timestamp);
                    
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
                uint64_t adjusted_timestamp = frame.exposure_start_timestamp + (uint64_t)delta_t;
                std::string image_filename = file_path_images_ + "/" + std::to_string(adjusted_timestamp) + ".png";
                
                cv::Mat image(frame.height(), frame.width(), CV_8UC3, (void*)frame.data());
                cv::Mat corrected_image;

                if (is_calibrated_) {
                    cv::remap(image, corrected_image, map_x_, map_y_, cv::INTER_LINEAR);
                    cv::imwrite(image_filename, corrected_image);
                } else {
                    cv::imwrite(image_filename, image);
                }
                
                lock.lock();
            }
        }
        
        std::cout << "工作线程退出" << std::endl;
    }

public:
    DvsenseRecorder(std::string file_path)
    {
        file_path_ = file_path;
        file_path_images_ = file_path + "/image";
        sync_filename_ = file_path_ + "/sync_signal.txt";

        //1. 如果文件夹不存在，则创建
        if (!std::filesystem::exists(file_path_))
        {
            std::filesystem::create_directory(file_path_);
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
   
    double delta_t = 0;      // 相机时间 + delta_t = 系统时间
    int image_count_ = 0;
    std::string file_path_;
    std::string file_path_images_;
    std::string sync_filename_;
};

#endif // DVSYNC_RECORDER_HPP
