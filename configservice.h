#pragma once
#include <string>
#include "util.hpp"
#include "INIReader.h"

class ConfigService : public SingletonT<ConfigService>
{
    friend class SingletonT<ConfigService>;
public:
    int load_config(const std::string& config_path);

    /**** remote ****/
    // server backend
    std::string server_address_;
    int server_port_;
    // AWS IoT Core
    std::string aws_iot_core_endpoint_;
    /**** local ****/
    // Camera settings
    bool enable_camera_;
    std::string camera_device_;
    // STM32 MCU settings
    bool enable_mcu_;
    std::string mcu_device_;
    int mcu_baudrate_;

    /**** logging ****/
    // enable writing logs to file (LoggerFactory still controls sinks)
    bool enable_file_logging_;
    // directory for log files, e.g. ./logs
    std::string log_directory_;
    // log filename, e.g. soc_deamon.log
    std::string log_file_name_;
    // log level string: trace, debug, info, warn, err, critical, off
    std::string log_level_;

private:
    // path to current config file (empty if not set)
    std::string config_path_ = "";

};
