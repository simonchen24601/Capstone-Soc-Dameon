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
#include <map>
#include <termios.h>    // for the baudrate constants
#include "logger.h"

class MCUInterface {
public:
    struct DecodedMessage {
        std::string dev;
        std::map<std::string, std::string> fields; // includes temperature, humidity, etc.
    };

public:
    MCUInterface();
    ~MCUInterface();
    // Initialize and set decoded-message callback
    int init(const std::string& device, int baudrate, const std::function<void(const std::vector<DecodedMessage>&)>& cb);
    // enqueue data to be written asynchronously by the write thread
    int write_bytes_async(const std::vector<uint8_t>& data);
    // set a callback invoked when bytes are received by the read thread
    
    void stop_background();

    // Decode messages from MCU where messages are whitespace-separated and each message
    // is of form: "dev=temp_sensor;temperature=25.0;humidity=60".
    // Returns one decoded item per message.

    static std::vector<DecodedMessage> decode_messages(const std::string& input);

private:
    int open_uart(const std::string& dev, int baudrate);
    ssize_t write_bytes(const void* data, size_t len);
    ssize_t read_bytes(void* data, size_t len);

    void set_read_callback(const std::function<void(const std::vector<DecodedMessage>&)>& cb);
    // stop background threads (safe to call multiple times)

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
    std::function<void(const std::vector<DecodedMessage>&)> read_callback_;
};
