#include <opencv2/opencv.hpp>
#include "DvsenseDriver/FileReader/ApsFileReader.h"
#include "DvsenseDriver/camera/FusionCamera.hpp"
#include "DvsenseDriver/FileReader/DvsFileReader.h"
#include <condition_variable>
#include <thread>
#include "DvsenseDriver/Calibration/Calibrator.hpp"
#include"DvsenseBase/Utils/Json/JsonUtils.hpp"
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

	int aps_height_ = 2160;
	int aps_width_ = 3840;

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

	// ----------------- Program description -----------------

	// if (argc < 2) {
	//	std::cerr << "Usage: " << argv[0] << " <aps_file_path>" << std::endl;
	//	return 1;
	// }

	const std::string short_program_desc(
		"Simple viewer to stream events from an event file, using the SDK driver API.\n");
	std::string long_program_desc(short_program_desc +
								  "Press 'q' or Escape key to leave the program.\n");
	std::cout << long_program_desc << std::endl;

	// ----------------- Event file initialization -----------------

	std::string aps_file_path;
	std::string perfix_path = "C:/DVSense/dvsensehal/build/bin/Release"; 
	// aps_file_path = argv[1];
	aps_file_path = perfix_path + "\\default_file_name.mp4";
	std::cout << "Aps file path: " << aps_file_path << std::endl;

	std::string dvs_file_path;
	dvs_file_path = perfix_path + "/default_file_name.raw";

	std::string calibration_file_path = perfix_path + "/default_file_name.json";
	dvsense::CalibratorParameters calibration_param;
	if(!dvsense::jsonFileToParam(calibration_file_path, calibration_param))
	{
		return -1;
	}
	dvsense::Calibrator calibrator;
	calibrator.loadCalibrationParam(calibration_param);
	std::shared_ptr<dvsense::ApsFileReader> aps_reader = dvsense::ApsFileReader::createFileReader(aps_file_path);
	aps_reader->loadFile();

	std::shared_ptr<dvsense::DvsFileReader> dvs_reader = dvsense::DvsFileReader::createFileReader(dvs_file_path);
	dvs_reader->loadFile();

	dvsense::TimeStamp start_timestamp, end_timestamp;
	dvs_reader->getStartTimeStamp(start_timestamp);
	dvs_reader->getEndTimeStamp(end_timestamp);
	dvsense::TimeStamp get_time = start_timestamp;
	dvsense::DvsFileInfo file_info;
	dvs_reader->getFileInfo(file_info);
	LOG_INFO("aps start ts: %llu", file_info.aps_start_timestamp);
	std::mutex disp_mutex;
	std::condition_variable disp_cond_;

	const std::string window_name = "DVSense APS File Viewer";
	cv::namedWindow(window_name, cv::WINDOW_GUI_NORMAL);

	cv::Mat aps_display;
	const int fps = 30;													  // event-based cameras do not have a frame rate, but we need one for visualization
	const int wait_time = static_cast<int>(std::round(1.f / fps * 1000)); // how long we should wait between two frames
	bool palying = true;
	SyncDisplayer sync_displayer(720, 1280);
	std::thread get_frame_thread = std::thread([&]()
											   {
			while (palying)
			{
				dvsense::ApsFrame aps_frame;
				
				if(aps_reader->getNextFrame(aps_frame))
				{
					std::unique_lock<std::mutex> lock(disp_mutex);
					dvsense::TimeStamp aps_dvs_ts = aps_frame.exposure_end_timestamp + file_info.aps_start_timestamp;
					std::shared_ptr<dvsense::Event2DVector> dvs_events = dvs_reader->getNTimeEventsGivenStartTimeStamp(aps_dvs_ts - 10000, 10000);
					LOG_INFO("timestamp: %llu, dvs ts: %llu", aps_frame.exposure_end_timestamp, aps_dvs_ts);
					dvsense::ApsFrame aps_to_dvs_frame = calibrator.mapApsToDvs(aps_frame);
					sync_displayer.processApsFrame(aps_to_dvs_frame); 
					sync_displayer.fusionDvsToAps(dvs_events->data(), dvs_events->data() + dvs_events->size()); 
					disp_cond_.notify_one();
				}else
				{
					disp_cond_.notify_one();
					palying = false;
				}				
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
			disp_cond_.notify_one();
			palying = false; });
	while (palying)
	{
		{
			std::unique_lock<std::mutex> lock(disp_mutex);
			disp_cond_.wait(lock);
			sync_displayer.getFusionImage(aps_display);
		}

		if (!aps_display.empty())
		{
			cv::imshow(window_name, aps_display);
		}

		cv::waitKey(1);
	}
	// ----------------- Event processing and show -----------------
	get_frame_thread.join();
	cv::destroyAllWindows();
	return 0;
}
