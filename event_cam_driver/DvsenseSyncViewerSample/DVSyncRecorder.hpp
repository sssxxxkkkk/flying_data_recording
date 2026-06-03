#ifndef DVSYNC_RECORDER_HPP
#define DVSYNC_RECORDER_HPP

#include <condition_variable>
#include <mutex>
#include <thread>
#include <queue>
#include <vector>
#include <fstream>
#include <filesystem>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// ★ 新增无锁队列头文件
#include <boost/lockfree/spsc_queue.hpp>

#include "DvsenseDriver/camera/FusionCamera.hpp"
#include "DvsenseDriver/camera/DvsCameraManager.hpp"
#include "DvsenseBase/logging/logger.hh"
#include "DvsenseDriver/DataProcess/DvsApsFusionProccessor.hpp"
#include "DvsenseDriver/Calibration/Calibrator.hpp"
#include "DvsenseBase/Utils/Json/JsonUtils.hpp"

// ====================== 异步 UDP 发送线程 ======================
class VideoSender {
public:
    VideoSender(const std::string& dest_ip, int dest_port, int jpeg_quality = 85);
    ~VideoSender();
    void sendFrame(const cv::Mat& frame);

private:
    void sendThreadFunc();

    int sockfd_;
    struct sockaddr_in servaddr_;
    std::string dest_ip_;
    int dest_port_;
    int jpeg_quality_;

    std::thread send_thread_;
    std::mutex queue_mtx_;
    std::condition_variable queue_cv_;
    std::queue<cv::Mat> frame_queue_;
    std::vector<uchar> buffer_;
    std::atomic<bool> stop_{false};
};

// ====================== 图像保存线程池 ======================
class ImageSaveThreadPool {
public:
    static ImageSaveThreadPool& instance();
    void enqueue(const std::string& path, const cv::Mat& img);

private:
    ImageSaveThreadPool();
    ~ImageSaveThreadPool();

    std::vector<std::thread> workers_;
    std::queue<std::pair<std::string, cv::Mat>> tasks_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::atomic<bool> stop_{false};
};

// ====================== 主录制器 ======================
class DvsenseRecorder
{
public:
    uint64_t delta_t = 0;
    std::string file_path_;
    std::string file_path_images_;
    std::string file_path_events_;
    std::string sync_filename_;

    DvsenseRecorder(std::string file_path, bool save_images, bool udp_display,
                    std::string& dest_ip, int dest_port);
    ~DvsenseRecorder();

    void save_images(const dvsense::ApsFrame frame);
    void update_delta_t(const dvsense::EventTriggerIn &trigger_in);
    void compute_remap(dvsense::CalibratorParameters cali_param);

private:
    void processImages();
    void processSyncSignals();

    // 图像队列（仍用锁 + 条件变量）
    std::queue<dvsense::ApsFrame> frame_queue_;
    std::mutex frame_queue_mtx_;
    std::condition_variable frame_queue_cv_;

    // ★ 同步信号改为无锁队列
    struct SyncTimestamp {
        uint64_t cam_timestamp;
        uint64_t system_timestamp;
        int polarity;
    };
    // 单生产者（回调）单消费者（处理线程），容量 4096
    boost::lockfree::spsc_queue<SyncTimestamp, boost::lockfree::capacity<4096>> sync_queue_;

    std::mutex delta_t_mtx_;           
    std::thread image_thread_;
    std::thread sync_thread_;
    std::atomic<bool> stop_worker_{false};

    bool is_calibrated_{false};
    cv::Mat map_x_, map_y_;

    std::string dest_ip_;
    int dest_port_;
    VideoSender v_sender_;
    bool save_images_;
    bool udp_display_;

    std::ofstream sync_file_stream_;
};

#endif