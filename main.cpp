#include <iostream>
#include <string>
#include "logger.h"
#include "configservice.h"
#include "peripheralbroker.h"
#include "app.h"

const char* BOOTSTRAP_LOGGER_NAME = "bootstrap";
const char* DEFAULT_CONFIG_FILE = "./config.ini";

void print_config(std::shared_ptr<spdlog::logger> logger, const ConfigService* cfg)
{
    logger->info("CONFIG: remote.server_address = {}", cfg->server_address_);
    logger->info("CONFIG: remote.server_port = {}", cfg->server_port_);
    logger->info("CONFIG: remote.aws_iot_core_endpoint = {}", cfg->aws_iot_core_endpoint_);
    logger->info("CONFIG: remote.aws_iot_core_serial_number = {}", cfg->aws_iot_core_serial_number_);
    logger->info("CONFIG: remote.aws_iot_ca_filepath = {}", cfg->aws_iot_ca_filepath_);
    logger->info("CONFIG: remote.aws_iot_cert_filepath = {}", cfg->aws_iot_cert_filepath_);
    logger->info("CONFIG: remote.aws_iot_key_filepath = {}", cfg->aws_iot_key_filepath_);

    logger->info("CONFIG: local.enable_camera = {}", cfg->enable_camera_ ? "true" : "false");
    logger->info("CONFIG: local.camera_device = {}", cfg->camera_device_);
    logger->info("CONFIG: local.camera_width = {}", cfg->camera_width_);
    logger->info("CONFIG: local.camera_height = {}", cfg->camera_height_);
    logger->info("CONFIG: local.camera_framerate = {}", cfg->camera_framerate_);

    logger->info("CONFIG: local.enable_mcu = {}", cfg->enable_mcu_ ? "true" : "false");
    logger->info("CONFIG: local.mcu_device = {}", cfg->mcu_device_);
    logger->info("CONFIG: local.mcu_baudrate = {}", cfg->mcu_baudrate_);
    logger->info("CONFIG: local.mcu_mode = {}", cfg->mcu_mode_);

    logger->info("CONFIG: log.enable_file = {}", cfg->enable_file_logging_ ? "true" : "false");
    logger->info("CONFIG: log.log_directory = {}", cfg->log_directory_);
    logger->info("CONFIG: log.log_file = {}", cfg->log_file_name_);
    logger->info("CONFIG: log.log_level = {}", cfg->log_level_);
}

int main(int argc, char** argv)
{
    // load config from INI
    std::string config_path = (argc > 1) ? argv[1] : DEFAULT_CONFIG_FILE;
    ConfigService* p_config = ConfigService::get_instance();
    if(p_config->load_config(config_path) != 0 ) {
        std::cerr << "failed to load config from " << config_path << std::endl;
        return -1;
    }

    // init logger
    LoggerFactory* p_logger_factory = LoggerFactory::get_instance();
    if (!p_logger_factory) {
        return -1;
    }
    if (p_logger_factory->init() != 0) {
        std::cerr << "logger init failed" << std::endl;
		return -1;
    }
    auto logger = p_logger_factory->get_logger(BOOTSTRAP_LOGGER_NAME);
    print_config(logger, p_config);

    // init peripherals
    auto peripheral_broker = PeripheralBroker::get_instance();
    peripheral_broker->init_all();

    App app{};
    logger->info("bootstrap completed, launching app");
    app.run();
    return 0;
}
