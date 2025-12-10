/*
 * 版权所有 (c) 2024 Dvsense
 *
 * 本软件及其相关文档文件（以下简称“软件”）的所有权利均归 Dvsense 所有。
 * 未经 Dvsense 明确书面许可，任何人不得以任何形式使用、复制、修改、合并、发布、分发、转授权及/或销售本软件的副本。
 *
 * 本软件按“现状”提供，不附带任何形式的明示或暗示保证，包括但不限于适销性、适合特定目的和不侵权的保证。在任何情况下，Dvsense 均不对任何索赔、损害或其他责任负责，无论是合同行为、侵权行为还是其他行为，均与软件或使用软件有关。
 */

/*
 * Copyright (c) 2024 Dvsense
 *
 * All rights to this software and associated documentation files (hereinafter referred to as "the Software") are owned by Dvsense.
 * No one may use, reproduce, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software in any form without explicit written permission from Dvsense.
 *
 * The Software is provided "as is," without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular purpose, and non-infringement. In no event shall Dvsense be liable for any claim, damages, or other liability, whether in an action of contract, tort, or otherwise, arising from, out of, or in connection with the Software or the use or other dealings in the Software.
 */

#include <thread>
#include <opencv2/opencv.hpp>
#include "DvsenseDriver/camera/DvsCameraManager.hpp"
#include "DvsenseBase/logging/logger.hh"

class EventAnalyzer
{
public:
	cv::Mat img, img_swap;
	std::mutex m;

	// Gray
	cv::Vec3b color_bg = cv::Vec3b(0x70, 0x70, 0x70);
	cv::Vec3b color_on = cv::Vec3b(0xbf, 0xbc, 0xb4);
	cv::Vec3b color_off = cv::Vec3b(0x40, 0x3d, 0x33);

	void setup_display(const int width, const int height)
	{
		img = cv::Mat(height, width, CV_8UC3);
		img_swap = cv::Mat(height, width, CV_8UC3);
		img.setTo(color_bg);
	}

	// Called from main Thread
	void get_display_frame(cv::Mat &display)
	{
		// Swap images
		{
			std::unique_lock<std::mutex> lock(m);
			std::swap(img, img_swap);
			img.setTo(color_bg);
		}
		img_swap.copyTo(display);
	}

	// Called from decoding Thread
	void process_events(const dvsense::Event2D *begin, const dvsense::Event2D *end)
	{
		std::unique_lock<std::mutex> lock(m);

		for (auto it = begin; it != end; ++it)
		{
			img.at<cv::Vec3b>(it->y, it->x) = (it->polarity) ? color_on : color_off;
		}
	}
};

int main(int argc, char *argv[])
{
	bool sync_mode_master = false;
	bool sync_mode_slave = false;
	std::string open_camera_sn = "";
	// ----------------- Program description -----------------

	const std::string short_program_desc(
		"Simple viewer to stream events from device, using the SDK driver API.\n");
	std::string long_program_desc(short_program_desc +
								  "Press 'q' or Escape key to leave the program.\n"
								  "Press 'Space' key to start/stop recording events to a raw file.\n");

	if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help"))
	{
		std::cout << "Usage: " << argv[0] << " [output_file_path]" << std::endl;
		std::cout << "Default output file path: ./default_file_name.raw" << std::endl
				  << std::endl;
		std::cout << long_program_desc << std::endl;
		return 0;
	}

	if (argc == 3)
	{
		if (std::string(argv[1]) == "-s")
		{
			sync_mode_slave = true;
		}
		else if (std::string(argv[1]) == "-m")
		{
			sync_mode_master = true;
		}
		open_camera_sn = std::string(argv[2]);
	}
	else
	{
		std::cout << "Please specify master/slave mode and camera SN." << std::endl;
		return 0;
	}

	std::cout << long_program_desc << std::endl;

	// ----------------- Camera initialization -----------------
	dvsense::DvsCameraManager cameraManager;
	dvsense::CameraDevice camera;

	const int fps = 25;													  // event-based cameras do not have a frame rate, but we need one for visualization
	const int wait_time = static_cast<int>(std::round(1.f / fps * 1000)); // how much we should wait between two frames
	cv::Mat display;													  // frame where events will be accumulated
	const std::string window_name = "DVSense Camera Viewer";
	cv::namedWindow(window_name, cv::WINDOW_GUI_EXPANDED);

	EventAnalyzer event_analyzer;
	bool is_recording = false;
	bool stop_application = false;
	// If the camera is not connected, reconnect it.
	const std::vector<dvsense::CameraDescription> camera_descs = cameraManager.getCameraDescs();
	// Print all cameras found
	for (auto& cameraDesc : camera_descs) {
		LOG_INFO("Camera found: %s : %s", cameraDesc.manufacturer.c_str(), cameraDesc.serial.c_str());
	}
	// Open the first camera found
	camera = cameraManager.openCamera(open_camera_sn);
	if (camera)
	{
		LOG_INFO("Camera open success.");
		std::shared_ptr<dvsense::CameraTool> hal_sync = camera->getTool(dvsense::ToolType::TOOL_SYNC);
		

		if(sync_mode_slave)
		{
			hal_sync->setParam("mode", std::string("SLAVE"));
		}else if(sync_mode_master)
		{
			hal_sync->setParam("mode", std::string("MASTER"));
		}
		else
		{
			LOG_INFO("Sync mode not set.");
		}
		event_analyzer.setup_display(camera->getWidth(), camera->getHeight());

		// Start a thread to get events from the camera
		camera->setBatchEventsNum(10000);
		camera->addEventsStreamHandleCallback([&event_analyzer](const dvsense::Event2D *begin, const dvsense::Event2D *end)
											  { event_analyzer.process_events(begin, end); });
		camera->addTriggerInCallback([](const dvsense::EventTriggerIn trigger)
									 { LOG_INFO("Trigger info: id: %d, p: %d, time: %d", trigger.id, trigger.polarity, trigger.timestamp); });
		cv::resizeWindow(window_name, camera->getWidth(), camera->getHeight());

		camera->start();
	}
	else
	{
		LOG_INFO("Open camera failed.");
		return 0;
	}

	while (!stop_application)
	{
		event_analyzer.get_display_frame(display);
		if (!display.empty())
		{
			cv::imshow(window_name, display);
		}

		// If user presses `q` key, exit loop and stop application
		int key = cv::waitKey(wait_time);
		if ((key & 0xff) == 'q' || (key & 0xff) == 27)
		{
			stop_application = true;
			std::cout << "q pressed, exiting." << std::endl;
			camera->stop();
		}
		
	}
}