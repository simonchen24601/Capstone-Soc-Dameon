#include "camera_microphone_interface.h"
#include "configservice.h"

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <chrono>
#include <iostream>

CameraMicrophoneInterface::CameraMicrophoneInterface()
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
    , cap_{nullptr}
    , device_{}
{}

CameraMicrophoneInterface::~CameraMicrophoneInterface()
{
    stop_stream_async();
    if (cap_ && cap_->isOpened()) cap_->release();
}

int CameraMicrophoneInterface::init(const std::string& device, int camera_width, int camera_height, int camera_framerate)
{
    device_ = device;
    camera_width_ = camera_width;
    camera_height_ = camera_height;
    camera_framerate_ = camera_framerate;
    try {
        if (device.empty()) {
            return -1;
        }

        // If device looks like an integer index, open by index
        bool opened = false;
        if (device.size() == 1 && isdigit(device[0])) {
            int idx = device[0] - '0';
            cap_ = std::make_unique<cv::VideoCapture>(idx, cv::CAP_ANY);
            opened = cap_->isOpened();
        } else {
            // Try to open device path using V4L2 backend
            cap_ = std::make_unique<cv::VideoCapture>(device, cv::CAP_V4L2);
            opened = cap_->isOpened();
            if (!opened) {
                // try generic open
                cap_ = std::make_unique<cv::VideoCapture>(device);
                opened = cap_->isOpened();
            }
        }

        if (!opened) {
            logger_->error("Camera init failed for device {}", device);
            cap_.reset();
            return -1;
        }

        // apply requested resolution and framerate if provided
        if (camera_width_ > 0) cap_->set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
        if (camera_height_ > 0) cap_->set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);
        if (camera_framerate_ > 0) cap_->set(cv::CAP_PROP_FPS, camera_framerate_);

        logger_->info("Camera initialized: {}", device);
        return 0;
    }
    catch (const std::exception& e) {
        logger_->error("Camera init exception: {}", e.what());
        cap_.reset();
        return -1;
    }
}

void CameraMicrophoneInterface::get_image()
{
    if (!cap_ || !cap_->isOpened()) return;

    cv::Mat frame;
    if (!cap_->read(frame)) {
        logger_->error("Camera read failed");
        return;
    }

    std::filesystem::create_directories("./logs");
    std::string out = "./logs/capture.jpg";
    try {
        if (!cv::imwrite(out, frame)) {
            logger_->error("Failed to write image to {}", out);
        } else {
            logger_->info("Saved camera image to {}", out);
        }
    } catch (const std::exception& e) {
        logger_->error("Exception writing image: {}", e.what());
    }
}

void CameraMicrophoneInterface::start_stream_async()
{
    if (!cap_ || !cap_->isOpened()) {
        logger_->error("start_stream_async: camera not initialized");
        return;
    }

    if (streaming_.load()) {
        logger_->info("start_stream_async: already streaming");
        return;
    }

    stream_fps_ = (camera_framerate_ > 0) ? camera_framerate_ : 10;
    streaming_.store(true);

    stream_thread_ = std::thread([this]() {
        const auto frame_period = std::chrono::milliseconds(1000 / std::max(1, stream_fps_));
        size_t frame_idx = 0;
        logger_->info("camera stream thread started (fps={})", stream_fps_);
        while (streaming_.load()) {
            cv::Mat frame;
            bool ok = false;
            try {
                ok = cap_->read(frame);
            } catch (const std::exception& e) {
                logger_->error("stream read exception: {}", e.what());
            }

            if (!ok || frame.empty()) {
                std::unique_lock<std::mutex> lk(stream_mutex_);
                stream_cv_.wait_for(lk, frame_period, [this]{ return !streaming_.load(); });
                continue;
            }

            if (frame_callback_) {
                try { frame_callback_(frame); } catch (const std::exception& e) { logger_->error("frame callback exception: {}", e.what()); }
            } else {
                if ((frame_idx % std::max(1, stream_fps_)) == 0) {
                    std::filesystem::create_directories("./logs");
                    std::string out = "./logs/stream_" + std::to_string(frame_idx) + ".jpg";
                    try {
                        if (cv::imwrite(out, frame)) {
                            logger_->debug("stream: saved {}", out);
                        } else {
                            logger_->warn("stream: failed to save {}", out);
                        }
                    } catch (const std::exception& e) {
                        logger_->error("stream: save failed: {}", e.what());
                    }
                }
            }

            frame_idx++;
            std::unique_lock<std::mutex> lk(stream_mutex_);
            stream_cv_.wait_for(lk, frame_period, [this]{ return !streaming_.load(); });
        }
        logger_->info("camera stream thread exiting");
    });
}

void CameraMicrophoneInterface::stop_stream_async()
{
    if (!streaming_.load()) return;
    streaming_.store(false);
    stream_cv_.notify_all();
    if (stream_thread_.joinable()) stream_thread_.join();
    logger_->info("stopped camera stream");
}
