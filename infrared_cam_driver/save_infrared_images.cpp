#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <filesystem>
#include <signal.h>
#include <atomic>

using namespace std;
using namespace cv;

// Structure for image save tasks
struct SaveTask {
    Mat frame;
    string filename;
};

// Structure for UDP send tasks
struct UdpSendTask {
    Mat frame;
};

// Global synchronization variables
queue<SaveTask> save_queue;
mutex save_mtx;
condition_variable save_cv;
atomic<bool> stop_save_thread{false};

queue<UdpSendTask> udp_queue;
mutex udp_mtx;
condition_variable udp_cv;
atomic<bool> stop_udp_thread{false};

string base_path = "../save_data/infrared_data/";
const int MAX_QUEUE_SIZE = 60; // Prevent memory overflow
const int JPEG_QUALITY = 50;   // Balance between quality and bandwidth

// Get microsecond timestamp immediately after frame capture
std::string getCurrentTimestamp() {
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return std::to_string(micros);
}

// Background thread for writing images to disk
void imageWriterThread() {
    while (true) {
        SaveTask task;
        {
            unique_lock<mutex> lock(save_mtx);
            save_cv.wait(lock, [] { return !save_queue.empty() || stop_save_thread.load(); });
            
            if (stop_save_thread.load() && save_queue.empty()) break;
            if (save_queue.empty()) continue;

            task = std::move(save_queue.front());
            save_queue.pop();
        }

        if (!task.frame.empty()) {
            imwrite(base_path + task.filename, task.frame);
        }
    }
    cout << "Image writer thread exited gracefully" << endl;
}

// Background thread for JPEG encoding and UDP transmission
void udpSenderThread(int sockfd, const struct sockaddr_in& servaddr) {
    vector<uchar> jpeg_buffer; // Reuse buffer to avoid memory allocation
    jpeg_buffer.reserve(65507); // Preallocate maximum UDP packet size

    while (true) {
        UdpSendTask task;
        {
            unique_lock<mutex> lock(udp_mtx);
            udp_cv.wait(lock, [] { return !udp_queue.empty() || stop_udp_thread.load(); });
            
            if (stop_udp_thread.load() && udp_queue.empty()) break;
            if (udp_queue.empty()) continue;

            task = std::move(udp_queue.front());
            udp_queue.pop();
        }

        if (!task.frame.empty()) {
            jpeg_buffer.clear();
            imencode(".jpg", task.frame, jpeg_buffer, 
                    {IMWRITE_JPEG_QUALITY, JPEG_QUALITY, IMWRITE_JPEG_OPTIMIZE, 0});
            
            if (jpeg_buffer.size() <= 65507) { // Maximum UDP payload size
                sendto(sockfd, jpeg_buffer.data(), jpeg_buffer.size(), 0,
                       (struct sockaddr*)&servaddr, sizeof(servaddr));
            } else {
                cerr << "Warning: Frame too large for UDP (" << jpeg_buffer.size() 
                     << " bytes), skipped" << endl;
            }
        }
    }

    close(sockfd);
    cout << "UDP sender thread exited gracefully" << endl;
}

// Signal handler for Ctrl+C graceful exit
volatile sig_atomic_t flag_exit = 0;
void sigint_handler(int) {
    flag_exit = 1;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Usage: " << argv[0] << " <save image? 0/1> <use udp for display? 0/1> [IP] [port]" << endl;
        return -1;
    }

    bool enable_save = (stoi(argv[1]) == 1);
    bool enable_udp  = (stoi(argv[2]) == 1);
    string dest_ip = (argc >= 4) ? argv[3] : "192.168.10.1";
    int dest_port   = (argc >= 5) ? stoi(argv[4]) : 5000;

    // Prepare save directory
    if (!std::filesystem::exists(base_path)) {
        std::filesystem::create_directories(base_path);
    } else {
        // Clear old files in directory
        for (const auto& entry : std::filesystem::directory_iterator(base_path)) {
            std::filesystem::remove_all(entry.path());
        }
    }

    // Find thermal camera device (prefer video-index0 for raw stream)
    string video_device;
    for (const auto& entry : std::filesystem::directory_iterator("/dev/v4l/by-id/")) {
        string path = entry.path().string();
        if (path.find("Thermal_Thermal_Camera") != string::npos &&
            path.find("video-index0") != string::npos) {
            video_device = path;
            break;
        }
    }

    // Fallback to any thermal camera device if index0 not found
    if (video_device.empty()) {
        for (const auto& entry : std::filesystem::directory_iterator("/dev/v4l/by-id/")) {
            string path = entry.path().string();
            if (path.find("Thermal_Thermal_Camera") != string::npos) {
                video_device = path;
                break;
            }
        }
    }

    if (video_device.empty()) {
        cerr << "Error: No thermal camera found in /dev/v4l/by-id/" << endl;
        return -1;
    }

    // Open camera with V4L2 backend
    VideoCapture cap(video_device, CAP_V4L2);
    if (!cap.isOpened()) {
        cerr << "Error: Failed to open thermal camera: " << video_device << endl;
        // Final fallback to hardcoded device path
        video_device = "/dev/v4l/by-id/usb-Thermal_Thermal_Camera_202505161451-video-index0";
        cap.open(video_device, CAP_V4L2);
        if (!cap.isOpened()) {
            cerr << "Error: Failed to open fallback camera: " << video_device << endl;
            return -1;
        }
    }

    cout << "Successfully opened thermal camera: " << video_device << endl;

    // Optimize V4L2 settings for low latency
    cap.set(CAP_PROP_BUFFERSIZE, 1); // Minimize internal buffer to reduce latency
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G')); // Use hardware MJPG
    // Uncomment and adjust if your camera supports specific resolution/framerate
    // cap.set(CAP_PROP_FRAME_WIDTH, 320);
    // cap.set(CAP_PROP_FRAME_HEIGHT, 240);
    // cap.set(CAP_PROP_FPS, 30);

    cout << "Camera buffer size: " << cap.get(CAP_PROP_BUFFERSIZE) << endl;
    cout << "Camera resolution: " << cap.get(CAP_PROP_FRAME_WIDTH) << "x" 
         << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "Camera FPS: " << cap.get(CAP_PROP_FPS) << endl;

    // Initialize UDP socket
    int sockfd = -1;
    struct sockaddr_in servaddr;
    thread udp_thread;

    if (enable_udp) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            cerr << "Error: Failed to create UDP socket" << endl;
            enable_udp = false;
        } else {
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_port = htons(dest_port);
            if (inet_pton(AF_INET, dest_ip.c_str(), &servaddr.sin_addr) <= 0) {
                cerr << "Error: Invalid destination IP address: " << dest_ip << endl;
                close(sockfd);
                enable_udp = false;
            } else {
                // Start UDP sender thread
                udp_thread = thread(udpSenderThread, sockfd, servaddr);
                cout << "UDP sender initialized: " << dest_ip << ":" << dest_port << endl;
            }
        }
    }

    // Start image writer thread
    thread writer_thread;
    if (enable_save) {
        writer_thread = thread(imageWriterThread);
        cout << "Image writer initialized, saving to: " << base_path << endl;
    }

    // Register Ctrl+C signal handler
    signal(SIGINT, sigint_handler);

    Mat frame;
    int frame_count = 0;
    auto start_time = chrono::steady_clock::now();

    cout << "\nCapture started. Press Ctrl+C to stop." << endl;
    cout << "----------------------------------------" << endl;

    // Main capture loop (only reads frames and enqueues tasks)
    while (!flag_exit) {
        if (!cap.read(frame)) {
            cerr << "Error: Failed to read frame from camera" << endl;
            break;
        }

        // Get timestamp immediately after frame capture (most accurate)
        string timestamp = getCurrentTimestamp() + ".bmp";

        // Enqueue save task
        if (enable_save) {
            lock_guard<mutex> lock(save_mtx);
            if (save_queue.size() < MAX_QUEUE_SIZE) {
                save_queue.push({frame.clone(), timestamp});
                save_cv.notify_one();
            } else {
                static int drop_count = 0;
                if (++drop_count % 10 == 0) {
                    cerr << "Warning: Save queue full, dropped " << drop_count << " frames" << endl;
                }
            }
        }

        // Enqueue UDP send task
        if (enable_udp) {
            lock_guard<mutex> lock(udp_mtx);
            if (udp_queue.size() < MAX_QUEUE_SIZE) {
                udp_queue.push({frame.clone()});
                udp_cv.notify_one();
            } else {
                static int drop_count = 0;
                if (++drop_count % 10 == 0) {
                    cerr << "Warning: UDP queue full, dropped " << drop_count << " frames" << endl;
                }
            }
        }

        // FPS statistics (print every 100 frames)
        if (++frame_count % 100 == 0) {
            auto end_time = chrono::steady_clock::now();
            double elapsed = chrono::duration_cast<chrono::seconds>(end_time - start_time).count();
            double fps = frame_count / elapsed;
        }
    }

    // Graceful shutdown
    cout << "\n----------------------------------------" << endl;
    cout << "Shutting down... Processing remaining tasks" << endl;

    // Stop all threads and wait for them to finish
    stop_save_thread = true;
    stop_udp_thread = true;
    save_cv.notify_all();
    udp_cv.notify_all();

    if (writer_thread.joinable()) writer_thread.join();
    if (udp_thread.joinable()) udp_thread.join();

    // Calculate final statistics
    auto end_time = chrono::steady_clock::now();
    double total_elapsed = chrono::duration_cast<chrono::seconds>(end_time - start_time).count();
    double avg_fps = frame_count / total_elapsed;

    cout << "Total frames captured: " << frame_count << endl;
    cout << "Total runtime: " << fixed << setprecision(1) << total_elapsed << " seconds" << endl;
    cout << "Average FPS: " << fixed << setprecision(1) << avg_fps << endl;
    cout << "Program exited cleanly" << endl;

    return 0;
}
