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

// peripheral class declarations are moved to dedicated headers under peripherals/
#include "msg_callback.h"
#include "motor_driver_interface.h"
#include "mcu_interface.h"
#include "camera_microphone_interface.h"

enum peripheral_status_t {
    PERIPHERAL_STATUS_OK,
    PERIPHERAL_STATUS_DISCONNECTED,
    PERIPHERAL_STATUS_ERROR,
    PERIPHERAL_STATUS_NOT_SUPPORTED
};

enum MCU_msg_t {
    MCU_MSG_HEARTBEAT,
};

// moved MsgCallback, MotorDriverInterface, MCUInterface and CameraMicrophoneInterface
// into separate headers under peripherals/ to keep this file focused on the
// PeripheralBroker and higher-level compositions.

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

    // camera interfaces
    int get_camera_status();
    int start_camera_streaming();
    int stop_camera_streaming();
    int get_camera_image();

    // mcu interfaces
    int get_micophone_status() { return PERIPHERAL_STATUS_NOT_SUPPORTED; };
    int get_mcu_status();
    int get_motor_driver_status() { return PERIPHERAL_STATUS_NOT_SUPPORTED; };

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

// // Simple MQTT interface using Paho MQTT C++ (async_client)
// class MQTTInterface {
// public:
//     MQTTInterface();
//     ~MQTTInterface();

//     // Initialize from explicit parameters (endpoint like "ssl://host:8883")
//     int init(const std::string& broker_uri,
//              const std::string& client_id,
//              const std::string& ca_file,
//              const std::string& cert_file,
//              const std::string& key_file);

//     int connect();
//     int disconnect();

//     // publish a payload (blocking)
//     int publish(const std::string& topic, const std::string& payload, int qos = 1);

//     // Subscribe to request_topic and automatically publish ACKs to response_topic
//     int subscribe_ack(const std::string& request_topic, const std::string& response_topic);

//     // Read GPS (NMEA) lines from file and publish parsed JSON messages periodically
//     int publish_gps_file(const std::string& topic, const std::string& file_path, double publish_period_sec);
// public:
//     // expose a minimal getter so callback objects can log via the same logger
//     std::shared_ptr<spdlog::logger> get_logger() { return logger_; }

//     // message handler called from callback (made public so the callback can invoke it)
//     void handle_incoming(const std::string& topic, const std::string& payload);

// private:
//     std::shared_ptr<spdlog::logger> logger_;
//     std::unique_ptr<mqtt::async_client> client_;
//     mqtt::connect_options conn_opts_;
//     std::string response_topic_;
//     std::atomic_bool connected_{false};
//     // parse minimal NMEA (GGA or RMC) and return pair lat,lon as strings or empty
//     static bool parse_nmea_latlon(const std::string& line, std::string &out_lat, std::string &out_lon);
// };
