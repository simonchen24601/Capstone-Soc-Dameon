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
	aws_iot_core_serial_number_ = reader.Get("remote", "aws_iot_core_serial_number", "");
	aws_iot_ca_filepath_ = reader.Get("remote", "aws_iot_ca_filepath", "");
	aws_iot_cert_filepath_ = reader.Get("remote", "aws_iot_cert_filepath", "");
	aws_iot_key_filepath_ = reader.Get("remote", "aws_iot_key_filepath", "");

	// local section
	enable_camera_ = reader.GetBoolean("local", "enable_camera", false);
	camera_device_ = reader.Get("local", "camera_device", "/dev/video0");

	camera_width_ = static_cast<int>(reader.GetInteger("local", "camera_width", 1280));
	camera_height_ = static_cast<int>(reader.GetInteger("local", "camera_height", 720));
	camera_framerate_ = static_cast<int>(reader.GetInteger("local", "camera_framerate", 30));

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
