#pragma once
#include <cstdint>
#include <variant>
#include <termios.h>
#include <fcntl.h>
// #include <opencv2/opencv.hpp>
#include <mqtt/async_client.h>
#include "logger.h"

enum peripheral_status_t {
    PERIPHERAL_STATUS_OK,
    PERIPHERAL_STATUS_DISCONNECTED,
    PERIPHERAL_STATUS_ERROR,
    PERIPHERAL_STATUS_NOT_SUPPORTED
};

enum MCU_msg_t {
    MCU_MSG_HEARTBEAT,
};

class MsgCallback {
public:
    virtual void on_msg_callback(const void* msg, std::size_t msg_len) = 0;
};

class MotorDriverInterface {

};

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
    const char* HARDWARE_PATH_ = "/dev/ttyAMA0";
    static constexpr int BAUDRATE_ = B115200;
    std::shared_ptr<spdlog::logger> logger_;
    int dev_fd_;
};

class CameraMicrophoneInterface {
public:
    CameraMicrophoneInterface();
    ~CameraMicrophoneInterface();

    void get_image();
    void start_stream_async();
    void stop_stream_async();

private:

};

class PeripheralBroker : public SingletonT<PeripheralBroker>
{
    friend class SingletonT<PeripheralBroker>;

public:
    enum device_t {
        DEVICE_UNKOWN,
        DEVICE_CAMERA,
        DEVICE_MCU
    };

    typedef std::variant<
    std::shared_ptr<MCUInterface>,
    std::shared_ptr<CameraMicrophoneInterface>,
    std::shared_ptr<MotorDriverInterface>>
    device_variant_t;

    PeripheralBroker();
    int init_all();
    int get_camera_status();
    // int get_micophone_status();
    int get_mcu_status();
    int get_motor_driver_status();

private:
    int init_camera();
    // int init_microphone();
    int init_mcu();
    int init_motor_driver();

private:
    const char* LOGGER_NAME_ = "peripheral_broker";
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<CameraMicrophoneInterface> dev_camera_;
    std::shared_ptr<MCUInterface> dev_mcu_;
    std::shared_ptr<MotorDriverInterface> dev_motor_driver_;
};
