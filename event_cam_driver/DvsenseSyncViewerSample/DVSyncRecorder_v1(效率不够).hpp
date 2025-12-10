#ifndef DVSYNC_RECORDER_HPP
#define DVSYNC_RECORDER_HPP

#include <hdf5.h>
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

// 事件数据结构
struct EventData {
    uint16_t x;
    uint16_t y;
    uint8_t polarity;
    uint64_t timestamp;
};

class HDF5EventWriter {
private:
    hid_t file_id_;
    hid_t dataset_id_;
    hid_t dataspace_id_;
    std::vector<EventData> buffer_;
    size_t buffer_size_;
    hsize_t current_size_;
    
public:
    HDF5EventWriter() : buffer_size_(1000000), current_size_(0) {  // 增加缓冲区大小与批量处理一致
        file_id_ = H5I_INVALID_HID;
        dataset_id_ = H5I_INVALID_HID;
        dataspace_id_ = H5I_INVALID_HID;
    }
    
    bool initialize(const std::string& filename) {
        // 如果文件已存在，则删除它
        std::filesystem::path filepath(filename);
        if (std::filesystem::exists(filepath)) {
            std::filesystem::remove(filepath);
        }
        
        // 创建新的 HDF5 文件
        file_id_ = H5Fcreate(filename.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
        if (file_id_ < 0) {
            return false;
        }
        
        // 定义复合数据类型
        hid_t event_type = H5Tcreate(H5T_COMPOUND, sizeof(EventData));
        H5Tinsert(event_type, "x", HOFFSET(EventData, x), H5T_NATIVE_UINT16);
        H5Tinsert(event_type, "y", HOFFSET(EventData, y), H5T_NATIVE_UINT16);
        H5Tinsert(event_type, "polarity", HOFFSET(EventData, polarity), H5T_NATIVE_UINT8);
        H5Tinsert(event_type, "timestamp", HOFFSET(EventData, timestamp), H5T_NATIVE_UINT64);
        
        // 创建可扩展的数据集
        hsize_t dims[1] = {0};
        hsize_t maxdims[1] = {H5S_UNLIMITED};
        dataspace_id_ = H5Screate_simple(1, dims, maxdims);
        
        // 创建属性列表以启用 chunking 和压缩
        hid_t plist_id = H5Pcreate(H5P_DATASET_CREATE);
        hsize_t chunk_dims[1] = {10000};  // 增加chunk大小以适应更大的批次
        H5Pset_chunk(plist_id, 1, chunk_dims);
        H5Pset_deflate(plist_id, 6); // 启用 gzip 压缩
        
        // 创建数据集
        dataset_id_ = H5Dcreate2(file_id_, "events", event_type, dataspace_id_,
                                H5P_DEFAULT, plist_id, H5P_DEFAULT);
        
        // 清理
        H5Pclose(plist_id);
        H5Tclose(event_type);
        
        return dataset_id_ >= 0;
    }
    
    void writeEvents(const std::vector<EventData>& events) {
        buffer_.insert(buffer_.end(), events.begin(), events.end());
        
        // 根据批量大小调整写入策略
        if (buffer_.size() >= buffer_size_) {
            flush();
        }
    }
    
    void flush() {
        if (buffer_.empty() || dataset_id_ < 0) return;
        
        hsize_t old_size[1] = {current_size_};
        hsize_t new_size[1] = {current_size_ + buffer_.size()};
        
        // 扩展数据集
        H5Dset_extent(dataset_id_, new_size);
        
        // 选择要写入的 hyperslab
        hid_t filespace = H5Dget_space(dataset_id_);
        hsize_t offset[1] = {old_size[0]};
        hsize_t count[1] = {buffer_.size()};
        H5Sselect_hyperslab(filespace, H5S_SELECT_SET, offset, NULL, count, NULL);
        
        // 创建内存数据空间
        hid_t memspace = H5Screate_simple(1, count, NULL);
        
        // 写入数据
        H5Dwrite(dataset_id_, getEventType(), memspace, filespace, H5P_DEFAULT, buffer_.data());
        
        // 清理
        H5Sclose(memspace);
        H5Sclose(filespace);
        
        current_size_ = new_size[0];
        buffer_.clear();
    }
    
    ~HDF5EventWriter() {
        flush();
        if (dataset_id_ >= 0) H5Dclose(dataset_id_);
        if (dataspace_id_ >= 0) H5Sclose(dataspace_id_);
        if (file_id_ >= 0) H5Fclose(file_id_);
    }
    
private:
    hid_t getEventType() {
        static hid_t event_type = H5I_INVALID_HID;
        if (event_type < 0) {
            event_type = H5Tcreate(H5T_COMPOUND, sizeof(EventData));
            H5Tinsert(event_type, "x", HOFFSET(EventData, x), H5T_NATIVE_UINT16);
            H5Tinsert(event_type, "y", HOFFSET(EventData, y), H5T_NATIVE_UINT16);
            H5Tinsert(event_type, "polarity", HOFFSET(EventData, polarity), H5T_NATIVE_UINT8);
            H5Tinsert(event_type, "timestamp", HOFFSET(EventData, timestamp), H5T_NATIVE_UINT64);
        }
        return event_type;
    }
};


class DvsenseRecorder
{
private:
    // 添加事件队列和相关变量以支持异步处理
    std::queue<std::vector<dvsense::Event2D>> event_queue_;
    std::queue<dvsense::ApsFrame> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread worker_thread_;
    bool stop_worker_ = false;
    bool is_calibrated_ = false;
	cv::Mat map_x_, map_y_;
    
    // HDF5事件写入器
    std::unique_ptr<HDF5EventWriter> event_writer_;
    std::string event_filename_;
    
     // 增加批量处理大小
    size_t batch_process_size_ = 1000000;  // 一次处理100万个事件
    
    // 优化的事件处理函数
    void process_events_batch(const std::vector<dvsense::Event2D>& events) {
        // 批量转换事件数据
        std::vector<EventData> hdf5_events;
        hdf5_events.reserve(events.size());

        // 使用并行处理加速大批量事件转换
        hdf5_events.resize(events.size());
        
        // 并行化事件数据转换过程
        #pragma omp parallel for
        for (size_t i = 0; i < events.size(); i++) {
            const auto& event = events[i];
            EventData& hdf5_event = hdf5_events[i];
            hdf5_event.x = event.x;
            hdf5_event.y = event.y;
            hdf5_event.polarity = event.polarity;
            hdf5_event.timestamp = event.timestamp + (uint64_t)delta_t;
        }
        
        event_writer_->writeEvents(hdf5_events);
    }
    
    void process_data() {
        // 初始化HDF5事件文件
        event_filename_ = file_path_events_ + "/events.h5";
        event_writer_ = std::make_unique<HDF5EventWriter>();
        if (!event_writer_->initialize(event_filename_)) {
            std::cerr << "Failed to initialize HDF5 event writer!" << std::endl;
            return;
        }
        
        std::vector<dvsense::Event2D> combined_events;
        combined_events.reserve(batch_process_size_);
        
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] { 
                return !event_queue_.empty() || !frame_queue_.empty() || stop_worker_; 
            });
            
            // 检查是否需要退出
            if (stop_worker_) {
                // 处理合并缓冲区中剩余的事件
                if (!combined_events.empty()) {
                    lock.unlock();
                    process_events_batch(combined_events);
                    lock.lock();
                }
                
                // 处理队列中剩余的事件数据
                while (!event_queue_.empty()) {
                    std::cout << "处理缓存区事件数据..." << std::endl;
                    auto events = std::move(event_queue_.front());
                    event_queue_.pop();
                    lock.unlock();
                    process_events_batch(events);
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

                // 确保所有事件都被写入
                if (event_writer_) {
                    std::cout << "刷新HDF5缓冲区..." << std::endl;
                    lock.unlock();
                    event_writer_->flush();
                    lock.lock();
                }
                
                break;
            }
            
            // 批量收集事件数据
            while (!event_queue_.empty() && combined_events.size() < batch_process_size_) {
                auto events = std::move(event_queue_.front());
                event_queue_.pop();
                // 将事件添加到合并缓冲区
                combined_events.insert(combined_events.end(), events.begin(), events.end());
            }
            
            // 如果收集到了足够的事件，进行批量处理
            if (combined_events.size() >= batch_process_size_) {
                std::cout << "处理事件数据..." << std::endl;
                auto process_events = std::move(combined_events);
                combined_events.clear();
                combined_events.reserve(batch_process_size_);
                lock.unlock();
                
                process_events_batch(process_events);
                
                lock.lock();
            }
            
            // 处理图像数据（保持原有逻辑）
            int images_processed = 0;
            const int max_images_per_batch = 3;
            
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
        file_path_events_ = file_path + "/events";
        file_path_images_ = file_path + "/image";
        
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
    
    // 保存事件
    void save_events(const dvsense::Event2D *begin, const dvsense::Event2D *end)
    {
        // 保存时间时，事件的时间戳要等于 事件时间 + delta_t
        if (begin == end) return;

        // 将事件放入队列，由工作线程处理
        std::lock_guard<std::mutex> lock(queue_mutex_);
        std::vector<dvsense::Event2D> events(begin, end);
        event_queue_.emplace(std::move(events));
        queue_cv_.notify_one();
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
   

    void update_delta_t(const dvsense::EventTriggerIn &trigger_in)
    {
        if(trigger_in.polarity == 0)
        {       
            uint64_t cam_timestamp = trigger_in.timestamp;
            // delta_t_ = 当前系统时间 - cam_timestamp;
            auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            delta_t = (double)(system_time - cam_timestamp);
        }

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
    std::string file_path_events_;
    std::string file_path_images_;
};

#endif // DVSYNC_RECORDER_HPP