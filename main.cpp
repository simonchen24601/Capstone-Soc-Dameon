#include "logger.h"
#include "interface.h"
#include "app.h"
#include <iostream>

const char* BOOTSTRAP_LOGGER_NAME = "bootstrap";

int main(int argc, char** argv)
{
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
    logger->info("logger inited, bootstrap commence...");

    // todo: load config from INI
    logger->info("config service not implemented");

    // init peripherals
    auto peripheral_broker = PeripheralBroker::get_instance();

    App app{};
    logger->info("bootstrap completed");
    app.run();
    return 0;
}
