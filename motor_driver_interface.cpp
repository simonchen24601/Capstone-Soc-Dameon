#include "motor_driver_interface.h"

MotorDriverInterface::MotorDriverInterface()
    : logger_{LoggerFactory::get_instance()->get_logger(LOGGER_NAME_)}
{}

void MotorDriverInterface::init()
{
    logger_->info("MotorDriverInterface not implemented yet");
}
