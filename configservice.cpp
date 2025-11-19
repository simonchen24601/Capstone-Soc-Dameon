#include "configservice.h"
#include <algorithm>
#include <cctype>

int ConfigService::load_config(const std::string& config_path)
{
	INIReader reader(config_path);
	int err = reader.ParseError();
	if (err != 0) {
		// parse error or file open error
		return -1;
	}

	/**** remote ****/
    // server backend
	server_address_ = reader.Get("remote", "server_address", "");
	server_port_ = static_cast<int>(reader.GetInteger("remote", "server_port", 443));
	server_ssl_enabled_ = reader.GetBoolean("local", "server_ssl_enabled", true);
	// AWS IoT Core
	aws_iot_core_endpoint_ = reader.Get("remote", "aws_iot_core_endpoint", "");
	aws_iot_core_serial_number_ = reader.Get("remote", "aws_iot_core_serial_number", "");
	aws_iot_ca_filepath_ = reader.Get("remote", "aws_iot_ca_filepath", "");
	aws_iot_cert_filepath_ = reader.Get("remote", "aws_iot_cert_filepath", "");
	aws_iot_key_filepath_ = reader.Get("remote", "aws_iot_key_filepath", "");

	/**** local ****/
    // Camera settings
	enable_camera_ = reader.GetBoolean("local", "enable_camera", false);
	camera_device_ = reader.Get("local", "camera_device", "/dev/video0");
	camera_width_ = static_cast<int>(reader.GetInteger("local", "camera_width", 1280));
	camera_height_ = static_cast<int>(reader.GetInteger("local", "camera_height", 720));
	camera_framerate_ = static_cast<int>(reader.GetInteger("local", "camera_framerate", 30));
	// STM32 MCU settings
	enable_mcu_ = reader.GetBoolean("local", "enable_mcu", false);
	mcu_device_ = reader.Get("local", "mcu_device", "/dev/ttyAMA0");
	mcu_baudrate_ = static_cast<int>(reader.GetInteger("local", "mcu_baudrate", 115200));
	// mcu_mode: USB or UART
	{
	    std::string raw_mode = reader.Get("local", "mcu_mode", "USB");
	    // strip inline comments (starting with # or ;)
	    size_t pos = raw_mode.find_first_of("#;");
	    if (pos != std::string::npos) raw_mode = raw_mode.substr(0, pos);
	    // trim whitespace
	    auto ltrim = [](std::string &s) {
	        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) { return !std::isspace(ch); }));
	    };
	    auto rtrim = [](std::string &s) {
	        s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
	    };
	    ltrim(raw_mode);
	    rtrim(raw_mode);
	    mcu_mode_ = raw_mode;
	}

	/**** logging ****/
	enable_file_logging_ = reader.GetBoolean("log", "enable_file", true);
	log_directory_ = reader.Get("log", "log_directory", "./logs");
	log_file_name_ = reader.Get("log", "log_file", "soc_deamon.log");
	log_level_ = reader.Get("log", "log_level", "debug");

	return 0;
}
