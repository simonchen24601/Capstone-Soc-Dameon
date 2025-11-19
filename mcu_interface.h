#pragma once
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <functional>
#include <atomic>
#include <termios.h>    // for the baudrate constants
#include "logger.h"

class MCUInterface {
public:
    MCUInterface();
    ~MCUInterface();
    int init(const std::string& device, int baudrate);
    // enqueue data to be written asynchronously by the write thread
    int write_bytes_async(const std::vector<uint8_t>& data);
    // set a callback invoked when bytes are received by the read thread
    void set_read_callback(const std::function<void(const std::vector<uint8_t>&)>& cb);
    // stop background threads (safe to call multiple times)
    void stop_background();

private:
    int open_uart(const std::string& dev, int baudrate);
    ssize_t write_bytes(const void* data, size_t len);
    ssize_t read_bytes(void* data, size_t len);

private:
    const char* LOGGER_NAME_ = "MCU";
    std::shared_ptr<spdlog::logger> logger_;
    int dev_fd_;    // file descriptor for the MCU
    // background read/write threads and structures
    std::thread read_thread_;
    std::thread write_thread_;
    std::mutex write_mutex_;
    std::condition_variable write_cv_;
    std::queue<std::vector<uint8_t>> write_queue_;
    std::atomic_bool running_{false};
    std::function<void(const std::vector<uint8_t>&)> read_callback_;
};
