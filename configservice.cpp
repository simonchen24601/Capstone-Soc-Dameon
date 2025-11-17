#include "configservice.h"

int ConfigService::load_config(const std::string& config_path)
{
	INIReader reader(config_path);
	int err = reader.ParseError();
	if (err != 0) {
		// parse error or file open error
		return -1;
	}

	// remote section
	server_address_ = reader.Get("remote", "server_address", "");
	server_port_ = static_cast<int>(reader.GetInteger("remote", "server_port", 443));
	aws_iot_core_endpoint_ = reader.Get("remote", "aws_iot_core_endpoint", "");

	// local section
	enable_camera_ = reader.GetBoolean("local", "enable_camera", false);
	camera_device_ = reader.Get("local", "camera_device", "/dev/video0");

	enable_mcu_ = reader.GetBoolean("local", "enable_mcu", false);
	mcu_device_ = reader.Get("local", "mcu_device", "/dev/ttyAMA0");
	mcu_baudrate_ = static_cast<int>(reader.GetInteger("local", "mcu_baudrate", 115200));

	// log section
	enable_file_logging_ = reader.GetBoolean("log", "enable_file", true);
	log_directory_ = reader.Get("log", "log_directory", "./logs");
	log_file_name_ = reader.Get("log", "log_file", "soc_deamon.log");
	log_level_ = reader.Get("log", "log_level", "debug");

	return 0;
}
