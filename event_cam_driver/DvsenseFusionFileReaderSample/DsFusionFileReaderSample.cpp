#include <opencv2/opencv.hpp>
#include "DvsenseDriver/camera/FusionCamera.hpp"
#include "DvsenseDriver/FileReader/DvsFileReader.h"
#include <condition_variable>
#include"DvsenseBase/Utils/Json/JsonUtils.hpp"
#include <thread>
#include <filesystem> // C++17
#include <regex>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>

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


class SyncDisplayer
{
public:
	dvsense::TimeStamp aps_frame_export_start_time_ = 0;
	dvsense::TimeStamp aps_frame_export_end_time_ = 0;
	std::mutex aps_frame_buffer_mutex_;

	cv::Mat fusion_image_;
	cv::Mat fusion_image_swap_;
	std::mutex fusion_image_mutex_;

	cv::Mat fusion_image_dvs_size_;
	cv::Mat fusion_image_dvs_size_swap_;
	std::mutex fusion_image_dvs_size_mutex_;

	cv::Mat aps_raw_image_;
	cv::Mat aps_raw_image_swap_;
	std::mutex aps_raw_image_mutex_;

	cv::Mat dvs_raw_image_;
	cv::Mat dvs_raw_image_swap_;
	std::mutex dvs_raw_image_mutex_;

	std::mutex event_buffer_mutex_;
	using EventBuffer = std::vector<uint16_t>;
	std::unique_ptr<EventBuffer> event_buffer_;
	int max_count_ = 10;

	int dvs_height_ = 720;
	int dvs_width_ = 1280;

	int aps_height_ = 720;
	int aps_width_ = 1280;

	// Gray
	cv::Vec3b color_bg_ = cv::Vec3b(0xff, 0xff, 0xff);
	cv::Vec3b color_on_ = cv::Vec3b(0xff, 0x00, 0x00);
	cv::Vec3b color_off_ = cv::Vec3b(0x00, 0x00, 0xff);

	SyncDisplayer(int dvs_height, int dvs_width) : dvs_height_(dvs_height), dvs_width_(dvs_width)
	{
		aps_raw_image_ = cv::Mat(aps_height_, aps_width_, CV_8UC3);
		aps_raw_image_swap_ = cv::Mat(aps_height_, aps_width_, CV_8UC3);

		fusion_image_dvs_size_ = cv::Mat(dvs_height_, dvs_width_, CV_8UC3);
		fusion_image_dvs_size_swap_ = cv::Mat(dvs_height_, dvs_width_, CV_8UC3);

		event_buffer_ = std::make_unique<EventBuffer>(dvs_height_ * dvs_width_ * 2, 0);
	}

	void getFusionImage(cv::Mat &display)
	{
		// Swap images
		{
			std::unique_lock<std::mutex> lock(fusion_image_dvs_size_mutex_);
			std::swap(fusion_image_dvs_size_, fusion_image_dvs_size_swap_);
			fusion_image_dvs_size_.setTo(color_bg_);
		}
		fusion_image_dvs_size_swap_.copyTo(display);
	}

	void getApsImage(cv::Mat &display)
	{
		// Swap images
		{
			std::unique_lock<std::mutex> lock(aps_raw_image_mutex_);
			std::swap(aps_raw_image_, aps_raw_image_swap_);
		}
		aps_raw_image_swap_.copyTo(display);
	}

	void processApsFrame(dvsense::ApsFrame &aps_frame)
	{
		std::unique_lock<std::mutex> lock(fusion_image_dvs_size_mutex_);
		std::memcpy(fusion_image_dvs_size_.data, aps_frame.data(), aps_frame.getDataSize());
		aps_frame_export_end_time_ = aps_frame.exposure_end_timestamp;
	}

	void fusionDvsToAps(const dvsense::Event2D *begin, const dvsense::Event2D *end)
	{
		std::unique_lock<std::mutex> lock(fusion_image_dvs_size_mutex_);
		for (auto event = begin; event != end; ++event)
		{
			if (event->y >= dvs_height_ || event->x >= dvs_width_)
			{
				continue;
			}
			fusion_image_dvs_size_.at<cv::Vec3b>(event->y, dvs_width_ - event->x) = (event->polarity) ? color_on_ : color_off_;
		}
	}

	void updateEvents(const dvsense::Event2D *begin, const dvsense::Event2D *end)
	{
		int index = 0;
		std::unique_lock<std::mutex> lock(event_buffer_mutex_);
		for (auto &event = begin; event != end; event++)
		{
			int y = event->y;
			int x = event->x;

			index = (y * dvs_width_ + x) * 2 + event->polarity;
			if (index >= event_buffer_->size())
			{
				continue;
			}
			if (event_buffer_->at(index) < max_count_)
			{
				event_buffer_->at(index)++;
			}
		}
	}

	void visualizeEventsOnAPS(cv::Mat &canvas)
	{
		std::unique_lock<std::mutex> lock(fusion_image_mutex_);
		cv::resize(canvas, fusion_image_, cv::Size(aps_width_, aps_height_));
		for (int y = 0; y < dvs_height_; y++)
		{
			for (int x = 0; x < dvs_width_; x++)
			{
				int base_idx = (y * dvs_width_ + x) * 2;
				int negative_event_cnt = event_buffer_->at(base_idx);
				int positive_event_cnt = event_buffer_->at(base_idx + 1);
				int y_rectified = int(float(y - 360) * 4.86 / 1.45) + 1080;
				int x_rectified = int(float(x - 640) * 4.86 / 1.45) + 1920;
				if (y_rectified < 0 || x_rectified < 0)
					continue;
				if (y_rectified >= aps_height_ || x_rectified >= aps_width_)
					continue;
				if (positive_event_cnt > 0 && positive_event_cnt >= negative_event_cnt)
				{
					fusion_image_.at<cv::Vec3b>(y_rectified, x_rectified) =
						cv::Vec3b(0, 0, uint8_t(255.0 * (positive_event_cnt - negative_event_cnt) / max_count_));
				}
				else if (negative_event_cnt > 0 && negative_event_cnt > positive_event_cnt)
				{
					fusion_image_.at<cv::Vec3b>(y_rectified, x_rectified) =
						cv::Vec3b(uint8_t(255.0 * (negative_event_cnt - positive_event_cnt) / max_count_), 0, 0);
				}
			}
		}
	}

	void mapAPSToDvsSize(const cv::Mat &aps_raw_image, cv::Mat &fusion_image, int aps_width, int aps_height, int dvs_width, int dvs_height)
	{
		// 确保输入图像是三通道的BGR格式
		if (aps_raw_image.empty() || aps_raw_image.channels() != 3)
		{
			throw std::invalid_argument("Input aps_raw_image must be a non-empty 3-channel BGR image.");
		}

		// 如果fusion_image为空或者尺寸不匹配，则创建或调整大小
		if (fusion_image.empty() || fusion_image.size() != cv::Size(dvs_width, dvs_height))
		{
			fusion_image = cv::Mat(dvs_height, dvs_width, aps_raw_image.type());
		}

		for (int y_target = 0; y_target < dvs_height; ++y_target)
		{
			for (int x_target = 0; x_target < dvs_width; ++x_target)
			{

				int y_aps = static_cast<int>((static_cast<float>(y_target - dvs_height / 2) * 4.86 / 1.45) + aps_height / 2);
				int x_aps = static_cast<int>((static_cast<float>(x_target - dvs_width / 2) * 4.86 / 1.45) + aps_width / 2);

				if (y_aps >= 0 && y_aps < aps_raw_image.rows &&
					x_aps >= 0 && x_aps < aps_raw_image.cols)
				{

					fusion_image.at<cv::Vec3b>(y_target, x_target) = aps_raw_image.at<cv::Vec3b>(y_aps, x_aps);
				}
				else
				{
					// 设置背景颜色，需要确保color_bg_是一个有效的cv::Scalar类型变量
					fusion_image.at<cv::Vec3b>(y_target, x_target) = color_bg_;
				}
			}
		}
	}
};

int main(int argc, char *argv[])
{
    const std::string short_program_desc("Simple viewer to stream events and BMP images from folder.\n");
    std::string long_program_desc(short_program_desc + "Press 'q' or Escape key to leave the program.\n");
    std::cout << long_program_desc << std::endl;

    // ----------------- 配置路径 -----------------
    std::string image_folder = "../save_data/image_data";
    std::string dvs_file_path = "../save_data/event_data/events.raw";

    // ----------------- 使用 OpenCV glob 获取所有 .bmp 文件 -----------------
    std::vector<cv::String> bmp_files;
    cv::glob(image_folder + "/*.bmp", bmp_files, false); // false = non-recursive

    if (bmp_files.empty()) {
        std::cerr << "No .bmp files found in: " << image_folder << std::endl;
        return -1;
    }

    // 转为 (timestamp, filepath) 并过滤非数字文件名
    std::vector<std::pair<uint64_t, std::string>> timestamped_images;
    for (const auto& filepath : bmp_files) {
        // 提取文件名（不含路径和扩展名）
        size_t last_slash = filepath.find_last_of("/\\");
        std::string filename = (last_slash == std::string::npos) ? filepath : filepath.substr(last_slash + 1);
        
        if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".bmp") {
            std::string stem = filename.substr(0, filename.size() - 4);
            if (isAllDigits(stem)) {
                uint64_t ts = std::stoull(stem);
                timestamped_images.push_back(std::make_pair(ts, filepath));
            }
        }
    }

    if (timestamped_images.empty()) {
        std::cerr << "No valid timestamp-named .bmp files found." << std::endl;
        return -1;
    }

    // 按时间戳排序
    std::sort(timestamped_images.begin(), timestamped_images.end(),
        [](const std::pair<uint64_t, std::string>& a,
           const std::pair<uint64_t, std::string>& b) {
            return a.first < b.first;
        });

    std::cout << "Found " << timestamped_images.size() << " valid BMP images." << std::endl;

    // ----------------- 初始化 DVS 事件读取器 -----------------
    std::shared_ptr<dvsense::DvsFileReader> dvs_reader = dvsense::DvsFileReader::createFileReader(dvs_file_path);
    if (!dvs_reader->loadFile()) {
        std::cerr << "Failed to load DVS file: " << dvs_file_path << std::endl;
        return -1;
    }

    dvsense::TimeStamp start_ts, end_ts;
    dvs_reader->getStartTimeStamp(start_ts);
    dvs_reader->getEndTimeStamp(end_ts);
    std::cout << "DVS time range: " << start_ts << " ~ " << end_ts << std::endl;

    // ----------------- 显示设置 -----------------
    const std::string window_name = "APS + DVS Fusion Viewer";
    cv::namedWindow(window_name, cv::WINDOW_GUI_NORMAL);

    const int fps = 30;
    const int wait_time_ms = static_cast<int>(1000.0f / fps);

    SyncDisplayer sync_displayer(720, 1280); // DVS 分辨率

    // ----------------- 主播放循环 -----------------
    for (const auto& item : timestamped_images) {
        uint64_t ts = item.first;
        const std::string& img_path = item.second;

        // 1. 读取 BMP 图像
        cv::Mat aps_image = cv::imread(img_path, cv::IMREAD_COLOR);
        if (aps_image.empty()) {
            std::cerr << "Warning: Failed to read image: " << img_path << std::endl;
            continue;
        }

        // 2. 获取该时间戳附近的事件（±10ms）
        dvsense::TimeStamp event_start = ts - 10000; // 10ms = 10,000 μs
        dvsense::TimeStamp event_end = ts + 10000;
        auto dvs_events = dvs_reader->getNTimeEventsGivenStartTimeStamp(event_start, event_end - event_start);

        // 3. 将 APS 图像缩放到 DVS 分辨率（1280x720）
        cv::Mat aps_resized;
        cv::resize(aps_image, aps_resized, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);

        // 4. 复制到 SyncDisplayer
        {
            std::unique_lock<std::mutex> lock(sync_displayer.fusion_image_dvs_size_mutex_);
            aps_resized.copyTo(sync_displayer.fusion_image_dvs_size_);
        }

        // 5. 叠加事件
        if (dvs_events && !dvs_events->empty()) {
            sync_displayer.fusionDvsToAps(dvs_events->data(), dvs_events->data() + dvs_events->size());
        }

        // 6. 显示
        cv::Mat display;
        sync_displayer.getFusionImage(display);
        if (!display.empty()) {
            cv::imshow(window_name, display);
        }

        // 7. 控制播放速度 & 退出
        int key = cv::waitKey(wait_time_ms);
        if (key == 'q' || key == 27) { // ESC
            break;
        }
    }

    cv::destroyAllWindows();
    return 0;
}
