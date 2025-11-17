#pragma once
#include <string>
#include <memory>
#include "logger.h"
#include <termios.h>    // for the baudrate constants

class MCUInterface {
public:
    MCUInterface();
    ~MCUInterface();
    int init();

private:
    int open_uart(const std::string& dev, int baudrate);
    ssize_t write_bytes(const void* data, size_t len);
    ssize_t read_bytes(void* data, size_t len);

private:
    const char* LOGGER_NAME_ = "MCU";
    // defaults (overridden from ConfigService when available)
    const std::string HARDWARE_PATH_DEFAULT_ = "/dev/ttyAMA0";
    static constexpr int BAUDRATE_DEFAULT_ = B115200;
    std::shared_ptr<spdlog::logger> logger_;
    int dev_fd_;
};
