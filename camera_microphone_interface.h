#pragma once
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <functional>
#include "logger.h"

namespace cv { class VideoCapture; }
namespace cv { class Mat; }

class CameraMicrophoneInterface {
public:
    CameraMicrophoneInterface();
    ~CameraMicrophoneInterface();
    // initialize camera device (e.g. "/dev/video0" or "0")
    int init(const std::string& device, int camera_width, int camera_height, int camera_framerate);
    void get_image();
    void start_stream_async();
    void stop_stream_async();

private:
    const char* LOGGER_NAME_ = "camera_microphone";
    std::shared_ptr<spdlog::logger> logger_;
    std::unique_ptr<cv::VideoCapture> cap_;
    int camera_width_;
    int camera_height_;
    int camera_framerate_;
    std::string device_;
    // streaming control
    std::thread stream_thread_;
    std::atomic_bool streaming_{false};
    std::mutex stream_mutex_;
    std::condition_variable stream_cv_;
    std::function<void(const cv::Mat&)> frame_callback_;
    int stream_fps_ = 0;
};
