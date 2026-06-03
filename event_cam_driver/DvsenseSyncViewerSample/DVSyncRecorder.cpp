#include "DVSyncRecorder.hpp"
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <chrono>    // 确保 chrono 可用

// ========================= VideoSender =========================
VideoSender::VideoSender(const std::string& dest_ip, int dest_port, int jpeg_quality)
    : dest_ip_(dest_ip), dest_port_(dest_port), jpeg_quality_(jpeg_quality)
{
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) throw std::runtime_error("socket create failed");

    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(dest_port_);
    if (inet_pton(AF_INET, dest_ip.c_str(), &servaddr_.sin_addr) <= 0)
        throw std::runtime_error("invalid IP");

    struct timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 300000;
    setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    send_thread_ = std::thread(&VideoSender::sendThreadFunc, this);
}

VideoSender::~VideoSender() {
    stop_ = true;
    queue_cv_.notify_all();
    if (send_thread_.joinable()) send_thread_.join();
    close(sockfd_);
}

void VideoSender::sendFrame(const cv::Mat& frame) {
    if (frame.empty() || stop_) return;
    std::lock_guard<std::mutex> lock(queue_mtx_);
    if (frame_queue_.size() > 15) return;
    frame_queue_.push(frame.clone());
    queue_cv_.notify_one();
}

void VideoSender::sendThreadFunc() {
    while (!stop_) {
        std::unique_lock<std::mutex> lock(queue_mtx_);
        queue_cv_.wait(lock, [this] { return !frame_queue_.empty() || stop_; });

        if (stop_ && frame_queue_.empty()) break;
        if (frame_queue_.empty()) continue;

        cv::Mat frame = frame_queue_.front();
        frame_queue_.pop();
        lock.unlock();

        int w = (frame.cols / 2) & ~1;
        int h = (frame.rows / 2) & ~1;
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);

        buffer_.clear();
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_, cv::IMWRITE_JPEG_OPTIMIZE, 0};
        cv::imencode(".jpg", resized, buffer_, params);

        sendto(sockfd_, buffer_.data(), buffer_.size(), 0,
               (struct sockaddr*)&servaddr_, sizeof(servaddr_));
    }
}

// ====================== 图像保存线程池 ======================
ImageSaveThreadPool& ImageSaveThreadPool::instance() {
    static ImageSaveThreadPool pool;
    return pool;
}

ImageSaveThreadPool::ImageSaveThreadPool() {
    int num = std::min(std::thread::hardware_concurrency(), 4u);
    for (int i = 0; i < num; ++i) {
        workers_.emplace_back([this] {
            while (!stop_) {
                std::pair<std::string, cv::Mat> task;
                {
                    std::unique_lock<std::mutex> lock(mtx_);
                    cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
                    if (stop_ && tasks_.empty()) return;
                    task = std::move(tasks_.front());
                    tasks_.pop();
                }
                cv::imwrite(task.first, task.second);
            }
        });
    }
}

ImageSaveThreadPool::~ImageSaveThreadPool() {
    stop_ = true;
    cv_.notify_all();
    for (auto& t : workers_) if (t.joinable()) t.join();
}

void ImageSaveThreadPool::enqueue(const std::string& path, const cv::Mat& img) {
    if (stop_) return;
    std::lock_guard<std::mutex> lock(mtx_);
    tasks_.emplace(path, img.clone());
    cv_.notify_one();
}

// ====================== DvsenseRecorder ======================
DvsenseRecorder::DvsenseRecorder(std::string file_path, bool save_images, bool udp_display,
                                 std::string& dest_ip, int dest_port)
    : save_images_(save_images), udp_display_(udp_display),
      dest_ip_(dest_ip), dest_port_(dest_port),
      v_sender_(dest_ip, dest_port, 85)
{
    file_path_ = file_path;
    file_path_images_ = file_path + "/image_data";
    file_path_events_ = file_path + "/event_data";
    sync_filename_ = file_path_events_ + "/sync_signal.txt";

    std::filesystem::create_directories(file_path_images_);
    std::filesystem::create_directories(file_path_events_);

    for (auto& e : std::filesystem::directory_iterator(file_path_images_))
        std::filesystem::remove_all(e.path());

    if (std::filesystem::exists(sync_filename_))
        std::filesystem::remove(sync_filename_);

    sync_file_stream_.open(sync_filename_, std::ios::app);
    if (sync_file_stream_.is_open())
        sync_file_stream_ << "cam_timestamp system_time delta_t\n";

    image_thread_ = std::thread(&DvsenseRecorder::processImages, this);
    sync_thread_ = std::thread(&DvsenseRecorder::processSyncSignals, this);
}

DvsenseRecorder::~DvsenseRecorder() {
    stop_worker_ = true;
    frame_queue_cv_.notify_all();
    // ★ 不再需要 trigger_queue_cv_.notify_all()
    if (image_thread_.joinable()) image_thread_.join();
    if (sync_thread_.joinable()) sync_thread_.join();
}

void DvsenseRecorder::save_images(const dvsense::ApsFrame frame) {
    std::lock_guard<std::mutex> lock(frame_queue_mtx_);
    if (frame_queue_.size() > 30) return;
    frame_queue_.push(frame);
    frame_queue_cv_.notify_one();
}

void DvsenseRecorder::processImages() {
    // 未修改，同原版
    while (true) {
        std::unique_lock<std::mutex> lock(frame_queue_mtx_);
        frame_queue_cv_.wait(lock, [this] { return !frame_queue_.empty() || stop_worker_; });

        if (stop_worker_ && frame_queue_.empty()) break;
        if (frame_queue_.empty()) continue;

        auto frame = std::move(frame_queue_.front());
        frame_queue_.pop();
        lock.unlock();

        cv::Mat img(frame.height(), frame.width(), CV_8UC3, frame.data());
        cv::Mat bgr;
        cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);

        cv::Mat out;
        if (is_calibrated_)
            cv::remap(bgr, out, map_x_, map_y_, cv::INTER_LINEAR);
        else
            out = bgr;

        std::string name = file_path_images_ + "/" + std::to_string(frame.exposure_start_timestamp) + ".bmp";
        if (save_images_)
            ImageSaveThreadPool::instance().enqueue(name, out);

        if (udp_display_)
            v_sender_.sendFrame(out);
    }
}

// ★ 回调：无锁 push
void DvsenseRecorder::update_delta_t(const dvsense::EventTriggerIn &trigger_in) {
    auto sys_ts = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
        
    SyncTimestamp ts;
    ts.cam_timestamp = trigger_in.timestamp;
    ts.system_timestamp = sys_ts;
    ts.polarity = trigger_in.polarity;

    // 非阻塞 push；若队列满则静默丢弃（容量 4096，几乎不会满）
    sync_queue_.push(ts);
}

// ★ 处理线程：无锁 pop + 轮询
void DvsenseRecorder::processSyncSignals() {
    SyncTimestamp ts;
    while (!stop_worker_) {
        // 一次性取出所有可用的同步信号
        while (sync_queue_.pop(ts)) {
            if (ts.polarity == 0 && sync_file_stream_.is_open()) {
                {
                    std::lock_guard<std::mutex> d_lock(delta_t_mtx_);
                    delta_t = ts.system_timestamp - ts.cam_timestamp;
                }
                sync_file_stream_ << ts.cam_timestamp << " "
                                  << ts.system_timestamp << " "
                                  << delta_t << "\n";
            }
        }
        // 短暂休眠，避免空转占满 CPU
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    // 退出前清空队列中剩余数据
    while (sync_queue_.pop(ts)) {
        if (ts.polarity == 0 && sync_file_stream_.is_open()) {
            {
                std::lock_guard<std::mutex> d_lock(delta_t_mtx_);
                delta_t = ts.system_timestamp - ts.cam_timestamp;
            }
            sync_file_stream_ << ts.cam_timestamp << " "
                              << ts.system_timestamp << " "
                              << delta_t << "\n";
        }
    }

    if (sync_file_stream_.is_open()) {
        sync_file_stream_.close();
    }
}

void DvsenseRecorder::compute_remap(dvsense::CalibratorParameters cali_param) {
    is_calibrated_ = true;
    int h = cali_param.dvs_rows;
    int w = cali_param.dvs_cols;
    auto& m = cali_param.affine_matrix["1"].data;

    map_x_.create(h, w, CV_32FC1);
    map_y_.create(h, w, CV_32FC1);
    map_x_.setTo(-1);
    map_y_.setTo(-1);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float px = x * m[0] + y * m[1] + m[2];
            float py = x * m[3] + y * m[4] + m[5];
            if (px < 0 || px >= cali_param.aps_cols || py < 0 || py >= cali_param.aps_rows) continue;
            map_x_.at<float>(y, x) = px;
            map_y_.at<float>(y, x) = py;
        }
    }
}

int main(int argc, char *argv[])
{
    // ----------------- Program description -----------------
    const std::string short_program_desc(
        "Save events and images from device, using the SDK driver API.\n");
    std::string long_program_desc(short_program_desc +
                                  "Press 'q' or Escape key to leave the program.\n");

    // ----------------- Default parameters -----------------
    bool save_images = true;   // 默认存储照片
    bool save_events = true;
    bool udp_display = true;    // 默认进行UDP显示
    std::string dest_ip = "192.168.10.1";
    int dest_port = 5000;
    const std::string out_file_path = "../save_data";  // 固定图片存储路径

    // ----------------- Parse command line arguments -----------------
    if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help"))
    {
        std::cout << "Usage: " << argv[0] << " [save_images] [save_events] [udp_display] [dest_ip] [dest_port]" << std::endl;
        std::cout << "  save_images: 0 or 1 (default: 1 - save images)" << std::endl;
        std::cout << "  save_events: 0 or 1 (default: 1 - save events)" << std::endl;
        std::cout << "  udp_display: 0 or 1 (default: 1 - enable UDP display)" << std::endl;
        std::cout << "  dest_ip: destination IP address (default: 192.168.10.1)" << std::endl;
        std::cout << "  dest_port: destination port (default: 5000)" << std::endl;
        std::cout << long_program_desc << std::endl;
        return 0;
    }

    // Parse save_images flag (argv[1])
    if (argc > 1) {
        try {
            save_images = (std::stoi(argv[1]) != 0);
        } catch (...) {
            std::cerr << "Invalid save_images parameter. Using default: " << save_images << std::endl;
        }
    }

    // Parse save_images flag (argv[2])
    if (argc > 1) {
        try {
            save_events = (std::stoi(argv[2]) != 0);
        } catch (...) {
            std::cerr << "Invalid save_events parameter. Using default: " << save_events << std::endl;
        }
    }

    // Parse udp_display flag (argv[3])
    if (argc > 3) {
        try {
            udp_display = (std::stoi(argv[3]) != 0);
        } catch (...) {
            std::cerr << "Invalid udp_display parameter. Using default: " << udp_display << std::endl;
        }
    }

    // Parse dest_ip (argv[4])
    if (argc > 4) {
        dest_ip = argv[4];
    }

    // Parse dest_port (argv[5])
    if (argc > 5) {
        try {
            dest_port = std::stoi(argv[5]);
        } catch (...) {
            std::cerr << "Invalid port number: " << argv[5] << ". Using default port " << dest_port << std::endl;
        }
    }

    std::cout << "Configuration:" << std::endl;
    std::cout << "  Save images: " << (save_images ? "YES" : "NO") << std::endl;
    std::cout << "  Save events: " << (save_events ? "YES" : "NO") << std::endl;
    std::cout << "  UDP display: " << (udp_display ? "YES" : "NO") << std::endl;
    std::cout << "  Destination IP: " << dest_ip << std::endl;
    std::cout << "  Destination port: " << dest_port << std::endl;
    std::cout << "  Save path: " << out_file_path << std::endl;
    std::cout << long_program_desc << std::endl;

	// ----------------- Camera initialization -----------------
	dvsense::FusionCameraDevice camera = nullptr;
	bool is_recording = false;
	bool stop_application = false;
	dvsense::DsStatisticInfo statistic_info;
  
    //远程服务器的ip
	DvsenseRecorder recorder(out_file_path,save_images, udp_display, dest_ip,dest_port);
 
	dvsense::Calibrator calibrator;
	do
	{
		if (!camera || !camera->isConnected())
		{
			camera.reset();
			// find all cameras
			dvsense::DvsCameraManager cameraManager;
			std::vector<dvsense::CameraDescription> cameraDescs = cameraManager.getCameraDescs();
			if (cameraDescs.size() == 0)
			{
				LOG_INFO("Watting camera connect...");
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				continue;
			}
			std::string open_camera_serial;
			// Print all cameras found
			for (auto &cameraDesc : cameraDescs)
			{
				std::cout << "Camera found: " << cameraDesc.manufacturer << ": " << cameraDesc.serial << std::endl;
				if(cameraDesc.product == "DVSync" )
				{
					open_camera_serial = cameraDesc.serial;
				}
			}
			if(open_camera_serial.empty())
			{
				LOG_ERROR("No DVSync camera found, please connect a DVSync camera.");
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				continue;
			}

			// Open the first camera found
			camera = cameraManager.openFusionCamera(open_camera_serial);

			dvsense::CalibratorParameters cali_param;
			if(camera->readCalibrationParam(cali_param))
			{
				dvsense::paramToJsonFile(cali_param, "./calibration.json");
				calibrator.loadCalibrationParam(cali_param);
				recorder.compute_remap(cali_param);
			}

			camera->addApsFrameCallback([&recorder](const dvsense::ApsFrame frame)
										{ recorder.save_images(frame); });

			// camera->addEventsStreamHandleCallback([&recorder](const dvsense::Event2D *begin, const dvsense::Event2D *end)
			// 									  { recorder.save_events(begin, end); });
			camera->addSyncSignalCallback([&recorder](const dvsense::EventTriggerIn &trigger_in)
					{ recorder.update_delta_t(trigger_in);});
			// camera->setStatisticInfoCallback([&statistic_info](const dvsense::DsStatisticInfo info)
			// 								 { statistic_info = info; });


            // 设置事件触发阈值
			std::shared_ptr<dvsense::CameraTool> bias = camera->getTool(dvsense::ToolType::TOOL_BIAS);
			std::shared_ptr<dvsense::CameraTool> aps = camera->getTool(dvsense::ToolType::TOOL_APS_CTRL);
			const std::vector<dvsense::ToolInfo> tools = camera->getAllToolsInfo();
			bool ret;
			ret = aps->setParam("auto_exposure", false);
			if (!ret) {
				LOG_ERROR("Failed to set auto_exposure");
			}
			ret = aps->setParam("auto_gain", true);
			if (!ret) {
				LOG_ERROR("Failed to set auto_gain");
			}


			camera->start();
            
			std::this_thread::sleep_for(std::chrono::milliseconds(1500));

			
			ret = aps->setParam("exposure_time", 5000);
			if (!ret) {
				LOG_ERROR("Failed to set exposure_time");
			}

			int value;
			ret = aps->getParam("exposure_time", value);
			LOG_INFO("exposure_time: %d", value);

			float gain;
			ret = aps->getParam("gain", gain);
			LOG_INFO("gain: %f", gain);

			bool auto_exposure;
			ret = aps->getParam("auto_exposure", auto_exposure);
			LOG_INFO("auto_exposure: %d", auto_exposure);

			bool auto_gain;
			ret = aps->getParam("auto_gain", auto_gain);
			LOG_INFO("auto_gain: %d", auto_gain);
		}
        

		if(save_events)
		{
			camera->startRecording(recorder.file_path_events_, "events", dvsense::DVS_STREAM);
		}
		
	    //camera->startRecording(recorder.file_path_events_, "events");

		std::string input_command;
		std::getline(std::cin, input_command);
		
		if (input_command == "q" || input_command == "quit")
		{
			stop_application = true;
			camera->stopRecording(); 
			camera->stop();
			std::cout << "Quit command received, exiting." << std::endl;
		}
		else if (!input_command.empty()) 
		{
			std::cout << "Unknown command. Please enter 'q' to quit or ' ' to record: ";
		}

	} while (!stop_application);
	return 0;
 }