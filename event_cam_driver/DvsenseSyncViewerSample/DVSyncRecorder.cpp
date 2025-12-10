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
#include "DVSyncRecorder.hpp"
#include <map>
#include "DvsenseDriver/camera/DvsCamera.hpp"

int main(int argc, char *argv[])
{
	// ----------------- Program description -----------------

	const std::string short_program_desc(
		"Save events and images from device, using the SDK driver API.\n");
	std::string long_program_desc(short_program_desc +
								  "Press 'q' or Escape key to leave the program.\n");

	if (argc > 1 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help"))
	{
		std::cout << "Usage: " << argv[0] << " [output_file_path]" << std::endl;
		std::cout << "Default output file path: ./\n Default output file name: default_file_name" << std::endl
				  << std::endl;
		std::cout << long_program_desc << std::endl;
		return 0;
	}

	std::string out_file_path = "./save_data";
	if (argc == 3 && std::string(argv[1]) != "-h" && std::string(argv[1]) != "--help")
	{
		out_file_path = argv[1];
	}
	else
	{
		std::cout << "The current file path is not entered, "
				  << "If you need to set the file path,"
				  << "use the input parameters to set it." << std::endl
				  << std::endl;
	}

	std::cout << long_program_desc << std::endl;

	// ----------------- Camera initialization -----------------
	dvsense::FusionCameraDevice camera = nullptr;
	bool is_recording = false;
	bool stop_application = false;
	dvsense::DsStatisticInfo statistic_info;
  
  //远程服务器的ip
  std::string dest_ip = "192.168.247.31";
  int dest_port = 5000;
	DvsenseRecorder recorder(out_file_path,dest_ip,dest_port);
 
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
			//std::vector<dvsense::ToolInfo> tools_info = camera->getAllToolsInfo();
			// Open the first camera found
			camera = cameraManager.openFusionCamera(open_camera_serial);
			std::shared_ptr<dvsense::CameraTool> bias = camera->getTool(dvsense::ToolType::TOOL_BIAS);
			int val;
			bool ret = bias->getParam("bias_diff_on", val);
			//std::cout << ret << std::endl;
			
			// const std::map<std::string, dvsense::BasicParameterInfo> param_info = bias->getAllParamInfo();
			// int value;
			// bool ret = bias->getParam("bias_diff_on", 50);
			// bool ret = bias->setParam("bias_diff_off", 10);

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
										 { recorder.update_delta_t(trigger_in); });
			camera->setStatisticInfoCallback([&statistic_info](const dvsense::DsStatisticInfo info)
											 { statistic_info = info; });

			camera->start();
		}

	
    // 仅记录事件，通过相机的触发信号来同步
    camera->startRecording(out_file_path, "events", dvsense::DVS_STREAM);
               //camera->startRecording(out_file_path, "events");

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
