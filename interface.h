#pragma once
#include <cstdint>
#include <variant>
#include <string>
#include <termios.h>
#include <fcntl.h>
// #include <opencv2/opencv.hpp>
#include <mqtt/async_client.h>
#include "logger.h"
#include <memory>

namespace cv { class VideoCapture; }
namespace cv { class Mat; }

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
    // defaults (overridden from ConfigService when available)
    const std::string HARDWARE_PATH_DEFAULT_ = "/dev/ttyAMA0";
    static constexpr int BAUDRATE_DEFAULT_ = B115200;
    std::shared_ptr<spdlog::logger> logger_;
    int dev_fd_;
};

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

// Simple MQTT interface using Paho MQTT C++ (async_client)
class MQTTInterface {
public:
    MQTTInterface();
    ~MQTTInterface();

    // Initialize from explicit parameters (endpoint like "ssl://host:8883")
    int init(const std::string& broker_uri,
             const std::string& client_id,
             const std::string& ca_file,
             const std::string& cert_file,
             const std::string& key_file);

    int connect();
    int disconnect();

    // publish a payload (blocking)
    int publish(const std::string& topic, const std::string& payload, int qos = 1);

    // Subscribe to request_topic and automatically publish ACKs to response_topic
    int subscribe_ack(const std::string& request_topic, const std::string& response_topic);

    // Read GPS (NMEA) lines from file and publish parsed JSON messages periodically
    int publish_gps_file(const std::string& topic, const std::string& file_path, double publish_period_sec);
public:
    // expose a minimal getter so callback objects can log via the same logger
    std::shared_ptr<spdlog::logger> get_logger() { return logger_; }

    // message handler called from callback (made public so the callback can invoke it)
    void handle_incoming(const std::string& topic, const std::string& payload);

private:
    std::shared_ptr<spdlog::logger> logger_;
    std::unique_ptr<mqtt::async_client> client_;
    mqtt::connect_options conn_opts_;
    std::string response_topic_;
    std::atomic_bool connected_{false};
    // parse minimal NMEA (GGA or RMC) and return pair lat,lon as strings or empty
    static bool parse_nmea_latlon(const std::string& line, std::string &out_lat, std::string &out_lon);
};
