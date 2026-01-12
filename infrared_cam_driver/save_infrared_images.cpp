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

using namespace std;
using namespace cv;

// 图像任务结构体体
struct SaveTask {
    Mat frame;
    string filename;
};

// 全局变量用于线程同步
queue<SaveTask> saveQueue;
mutex mtx;
condition_variable cv_cond;
bool stop_thread = false;
string base_path = "../save_data/infrared_data/";

// 获取高精度时间戳字符串
string getCurrentTimestamp() {
    auto now = chrono::system_clock::now();
    auto in_time_t = chrono::system_clock::to_time_t(now);
    auto ms = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    char buf[64];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", localtime(&in_time_t));
    sprintf(buf + strlen(buf), "_%03ld", ms.count());
    return string(buf);
}

// 消费者线程：专门负责存图，降低主线程延迟
void imageWriterThread() {
    while (true) {
        SaveTask task;
        {
            unique_lock<mutex> lock(mtx);
            cv_cond.wait(lock, [] { return !saveQueue.empty() || stop_thread; });
            
            if (stop_thread && saveQueue.empty()) break;
            
            task = std::move(saveQueue.front());
            saveQueue.pop();
        }
        
        // 执行耗时的写盘操作
        if (!task.frame.empty()) {
            imwrite(base_path + task.filename, task.frame);
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "method: " << argv[0] << " <save image? 0/1> <use udp for display? 0/1> [IP] [port]" << endl;
        return -1;
    }

    bool enableSave = (stoi(argv[1]) == 1);
    bool enableUDP = (stoi(argv[2]) == 1);
    string ip = (argc >= 4) ? argv[3] : "192.168.10.1";
    int port = (argc >= 5) ? stoi(argv[4]) : 5000;


    if (!std::filesystem::exists(base_path))
	{
	    std::filesystem::create_directory(base_path);
	}
    else 
    {
	  // 如果文件夹存在，清空文件夹中的所有内容
  	for (const auto& entry : std::filesystem::directory_iterator(base_path)) 
  	{
    	   std::filesystem::remove_all(entry.path());
  	}
    }
    
    // 1. 初始化摄像头
    VideoCapture cap(0);
    if (!cap.isOpened()) return -1;

    // 优化摄像头属性：减少缓冲区延迟
    cap.set(CAP_PROP_BUFFERSIZE, 1); 

    // 2. 初始化 UDP
    int sockfd = -1;
    struct sockaddr_in servaddr;
    if (enableUDP) {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);
        servaddr.sin_addr.s_addr = inet_addr(ip.c_str());
    }

    // 3. 启动异步存图线程
    thread writer;
    if (enableSave) {
        writer = thread(imageWriterThread);
    }

    Mat frame;
    cout << "运行中... 按 ESC 退出" << endl;

    while (true) {
        // A. 抓取图像 (尽量快)
        if (!cap.read(frame)) break;

        // B. 如果需要存图，将任务丢入队列，立即返回
        if (enableSave) {
            string ts = getCurrentTimestamp() + ".bmp";
            {
                lock_guard<mutex> lock(mtx);
                // 限制队列长度避免内存溢出，如果写磁盘太慢，丢弃旧帧保证实时性
                if (saveQueue.size() < 30) { 
                    saveQueue.push({frame.clone(), ts});
                    cv_cond.notify_one();
                }
            }
        }

        // C. 处理 UDP 发送流 (使用编码压缩，减小带宽瓶颈)
        if (enableUDP) {
            vector<uchar> buf;
            imencode(".jpg", frame, buf, {IMWRITE_JPEG_QUALITY, 50});
            if (buf.size() <= 65507) {
                sendto(sockfd, buf.data(), buf.size(), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));
            }
        }

        //imshow("XSimple Stream", frame);
        if (waitKey(1) == 27) break;
    }

    // 清理并退出
    stop_thread = true;
    cv_cond.notify_all();
    if (writer.joinable()) writer.join();
    if (sockfd != -1) close(sockfd);
    
    return 0;
}
